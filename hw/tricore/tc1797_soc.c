/*
 * Infineon TC1797 SoC + board emulation.
 *
 * Target: BMW MEVD17.2.G (Bosch) DME firmware on SAK-TC1797-512F180E.
 * Loads the raw firmware image (-kernel) into PFLASH (0x80000000) and the real
 * Boot ROM dump (-bios) into BROM (0x8FFFC000), then resets to the BROM entry —
 * mirroring the bridge's BROM-mode cold-init boot, but fully native in QEMU.
 *
 * Peripherals (0xF0000000+) are stubbed as an unimplemented-device for now (it
 * logs every SFR access); the real TC1797 device models (STM, interrupt router,
 * MultiCAN, ADC, ...) are layered on in a later step (Path B P2).
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/core/sysbus.h"
#include "hw/core/loader.h"
#include "hw/core/boards.h"
#include "hw/misc/unimp.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "hw/core/cpu.h"
#include "exec/cpu-interrupt.h"
#include "qom/object.h"
#include "target/tricore/cpu.h"
#include "pcp.h"
#include "tc1797_can.h"
#include "tc1797_adc.h"
#include "tc1797_gpta.h"
#include "tc1797_eray.h"
#include "tc1797_dma.h"
#include "tc1797_asc.h"
#include "tc1797_fadc.h"
#include "tc1797_mli.h"
#include "tc1797_jtag.h"
#include "tricore_soc.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "system/system.h"
#include "net/can_emu.h"
#include "qom/object_interfaces.h"

/* ---- TC1797 memory map (from processors/tc1797.pspec.yaml) ---- */
#define TC1797_PFLASH_BASE   0x80000000UL
#define TC1797_PFLASH_SIZE   (4 * MiB)
#define TC1797_PFLASH_UBASE  0xA0000000UL    /* uncached alias */
#define TC1797_DSPR_BASE     0xD0000000UL
#define TC1797_DSPR_SIZE     (128 * KiB)   /* DMI LDRAM (UM addressable 124K; firmware self-test walks full window) */
#define TC1797_PSPR_BASE     0xC0000000UL
#define TC1797_PSPR_SIZE     (64 * KiB)    /* PMI SPRAM (UM addressable 24K; firmware self-test walks full window) */
#define TC1797_BROM_BASE     0x8FFFC000UL
#define TC1797_BROM_SIZE     (16 * KiB)
#define TC1797_BROM_UBASE    0xAFFFC000UL    /* uncached alias */
#define TC1797_SFR_BASE      0xF0000000UL    /* peripheral/SFR space (stub) */
#define TC1797_SFR_SIZE      0x10000000UL    /* 0xF0000000..0xFFFFFFFF */

/* ---- SoC device ---- */
#define TYPE_TC1797_SOC "tc1797-soc"
OBJECT_DECLARE_SIMPLE_TYPE(TC1797SoCState, TC1797_SOC)

struct TC1797SoCState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/
    TriCoreCPU cpu;
    MemoryRegion pflash_c, pflash_u;
    MemoryRegion brom_c, brom_u;
    MemoryRegion dspr, pspr;
    MemoryRegion sfr;          /* peripheral/SFR space (native model below) */
    MemoryRegion eray_ram;     /* E-Ray (FlexRay) message RAM, plain-RAM backed */
    MemoryRegion eray2_ram;    /* 0xF0060000 buffer in the firmware RAM-init list */
    MemoryRegion dflash0, dflash0_a;  /* DFLASH bank0 @0xAFE00000 + cached alias */
    MemoryRegion dflash1, dflash1_a;  /* DFLASH bank1 @0xAFE10000 + cached alias */
    MemoryRegion ramtest, ramtest_a;  /* OVRAM @0xAFE80000 (uncached) + 0x8FE80000 cached alias */
    MemoryRegion seg7;         /* 0x7FFF0000 segment-7 RAM-test pattern source */
    uint32_t scu_rststat;      /* SCU_RSTSTAT shadow (write-1-to-clear) */
    uint32_t hwcfg_ststat;     /* SCU_STSTAT: HWCFG[7:0] boot-config pins (P0 latch) */
    uint32_t scu_stcon;        /* SCU_STCON startup-config shadow */
    uint32_t fce_crc;          /* FCE CRC0 value register (CRC32-Ethernet) */
    uint32_t wdt_con0;         /* WDT_CON0 shadow (ENDINIT password writes) */
    uint32_t wdt_con1;         /* WDT_CON1 shadow */
    bool wdt_endinit;          /* decoded ENDINIT state (CON0 bit0) */
    bool wdt_modeled;          /* env TC1797_WDT: enforce watchdog timeout->reset */
    uint64_t wdt_service_count;/* CON0 password servicing writes (telemetry) */
    QEMUTimer *wdt_timer;      /* watchdog down-counter (timeout -> SoC reset) */
    /* STM (System Timer Module) compare/control shadows. TIM0..6/CAP are
     * derived live from the virtual clock (see tc1797_stm_count56). */
    uint32_t stm_cmp0, stm_cmp1; /* CMP0/CMP1 compare registers */
    uint32_t stm_cmcon;          /* CMCON (compare-match size/offset) */
    uint32_t stm_icr;            /* ICR: CMP0EN[0],CMP0IR[1],CMP0OS[2],CMP1EN[4],... */
    uint32_t stm_isrr;           /* compare-match service-request pending bits */
    uint32_t stm_src0, stm_src1; /* STM SRC0/SRC1 nodes @0xF00002F8/FC (SRPN/SRE) */
    QEMUTimer *stm_timer;        /* drives the CMP0/CMP1 systick interrupts */
    int64_t stm_period_ns;       /* derived OSEK tick period */
    unsigned stm_rr;             /* round-robin across the enabled STM compares */
    int64_t stm_cmp_deadline[2]; /* edge-on-match: abs virtual-ns when CMP0/CMP1 next fire */
    /* Clock tree (Hz) -- the SoC timing substrate. Every peripheral timer
     * derives its period from one of these rather than a hardcoded constant.
     * Defaults reproduce the previously-hardcoded rates exactly (behaviour-
     * neutral); override via TC1797_F{SYS,CPU,PCP,STM}_MHZ. f_pcp models the
     * PCP co-processor's own clock (TC1797 UM fPCP, derived from fSYS). */
    uint64_t f_sys;              /* PLL system clock */
    uint64_t f_cpu;              /* TriCore CPU clock (180 MHz on -512F180E) */
    uint64_t f_pcp;             /* PCP co-processor clock */
    uint64_t f_stm;              /* System Timer Module clock */
    uint32_t stm_ns_per_tick;    /* cached 1e9 / f_stm (= 10 at the 100 MHz default) */
    uint32_t ssc_tb[2];          /* SSC0/SSC1 transmit-buffer shadow (loopback) */
    uint32_t ssc_src[2][2];      /* SSC0/1 service-request nodes [unit][0]=+0xF8 (xfer), [1]=+0xFC (queue) */
    uint32_t adc_src[TC1797_ADC_NKERN][16]; /* ADC0/1/2 kernel SRNs (top of each 0x400 kernel, off>=0x3C0): firmware pulses SETR to trigger PCP/CPU completion (e.g. ADC0 SRC@0x3F0 -> PCP ch 0x1d -> sets 0xD000416F, DTC 0x3006 gate) */
    QEMUTimer *ssc_done_timer[2];/* SPI transfer-latency: defer completion SRN raise */
    int64_t ssc_xfer_ns;         /* modelled per-transfer time (env TC1797_SSC_XFER_NS) */
    bool sw_reset_pending;       /* set when 0xF0000560=2 requested a SW reset */
    /* Forced OSEK systick. The ERCOSEK kernel promotes itself from phase 3 to
     * phase 4 only after it starts receiving periodic system ticks, but it never
     * arms the STM CMP0 compare itself pre-promotion (chicken-and-egg), so on
     * bare QEMU the RTOS sits in phase 3 forever. We drive the tick via the
     * native interrupt path -- the QEMU analogue of bridge_server.py's injected
     * SRPN 1/7/8 round-robin systick. See tc1797_osek_tick. */
    QEMUTimer *osek_timer;       /* forced OSEK systick timer (always-on) */
    int64_t osek_tick_ns;        /* inter-fire spacing (~1 ms / #tick-prios) */
    int64_t osek_t0_ns;          /* virtual-time origin (phase-4 detect floor) */
    int64_t osek_floor_ns;       /* min virtual time before delivering any tick */
    unsigned osek_rr;            /* round-robin index across the tick priorities */
    uint8_t osek_minphase;       /* min RTOS phase (0xD0003643) before delivery */
    bool osek_enabled;           /* forced systick armed (env TC1797_NO_SYSTICK off) */
    bool osek_phase4;            /* latched once RTOS reaches phase 4 (diagnostic) */
    uint32_t cpu_src;            /* CPU service-request node 0xF7E0FFFC (OSEK dispatch) */
    QEMUBH *irq_bh;              /* defers MMIO-context IRQ raises out of TB context */
    uint8_t irq_pending_srpn;    /* highest SRPN awaiting the bottom-half raise */
    /* Faithful ICU: the set of pending CPU-routed service requests (one bit per
     * SRPN). The arbiter presents the highest as ICR.PIPN; on service-entry the
     * taken SRPN is cleared and it re-arbitrates to the next-highest -- so a
     * busy high source no longer clobbers/starves the lower OSEK ticks + CAN. */
    uint32_t icu_pending[8];
    PcpEngine pcp;               /* PCP2 co-processor, runs on its own thread */
    bool pcp_enabled;            /* env TC1797_PCP: route TOS=1 SRNs to the PCP */
    Tc1797Can can;               /* MultiCAN controller (TX/RX + message objects) */
    CanBusState *canbus;         /* optional QEMU CAN bus backend (-object can-bus) */
    CanBusClientState can_client;/* this SoC's client on the CAN bus */
    CharFrontend can_tun;        /* optional CAN tunnel to host (-chardev ...,id=can0) */
    char can_tun_rx[160];        /* line-accumulation buffer for inbound tunnel frames */
    unsigned can_tun_rxlen;
    CharFrontend eray_tun;       /* optional E-Ray/CP tunnel to host (-chardev ...,id=fray0) */
    char eray_tun_rx[320];       /* line-accumulation buffer for inbound CP frames */
    unsigned eray_tun_rxlen;
    CharFrontend mem_tun;        /* optional memory peek/poke tunnel (-chardev ...,id=mem0) */
    char mem_tun_rx[1088];       /* line-accumulation buffer (W cmd: addr + up to 512B hex) */
    unsigned mem_tun_rxlen;
    Tc1797Asc asc0, asc1;        /* ASC0/ASC1 UARTs (chardev-backed) */
    uint32_t scu_nmisr;          /* SCU NMI status (ESR1/NMI request) */
    uint32_t dts_stat;           /* DTS status (RDY + RESULT = die temperature) */
    uint32_t dts_con;            /* DTS control (PWD/START/CAL) */
    MemoryRegion ebu_ram, ebu_ram_a; /* optional EBU external RAM + cached alias */
    uint32_t *sfr_shadow;        /* config read-back for unmodeled SFRs (0xF0000000..0xF0200000) */
    uint32_t *ebu_shadow;        /* config read-back for the 0xF8000000 EBU/PMU/overlay region */
    Tc1797Adc adc;               /* ADC0/1/2 kernels (result registers) */
    Tc1797Fadc fadc;             /* Fast ADC (FADC) — knock/fast-conversion results */
    Tc1797Mli mli0, mli1;        /* Micro Link Interface (inter-chip serial link) */
    Tc1797Jtag jtag;             /* JTAG TAP + Cerberus/OCDS debug interface */
    uint32_t ocds_ostate;        /* OSCU OSTATE (0xF0000480): bit0=OCDS counter running.
                                  * The ERCOSEK software-counter handler fn_800bfaac writes
                                  * OCNTRL(0xF0000478)=0x5e to start the OCDS run-time counter,
                                  * then polls OSTATE.bit0; until set it early-returns and
                                  * disables the OS counter (-> periodic diag task never armed). */
    Tc1797Gpta gpta;             /* GPTA0/1 + LTCA2 timer arrays (capture/compare) */
    bool en_can, en_adc, en_gpta; /* per-peripheral dispatch gates (bisect/safety) */
    uint32_t port_out[20];       /* GPIO output latch: P0-P11 (idx 0-11) + Port10/11 block @0xF0300000 (idx 12-19) */
    ErayCC eray;                 /* E-Ray (FlexRay CC) POC state machine */
    Tc1797Dma dma;               /* DMA controller (block-move engine) */
    QEMUTimer *bootstrap_timer;  /* coding-marker bootstrap (env TC1797_BOOTSTRAP) */
    bool bootstrap_enabled;      /* seed the coding marker so the boot stabilises */
    bool bootstrap_done;         /* one-shot info log latch */
};

/*
 * TC1797 SFR/peripheral reads. The boot ROM + early firmware poll the SCU
 * (System Control Unit) clock/PLL/reset status registers; these fixed READY
 * values are the RE'd behavioural spec from the Python bridge (bridge_server.py
 * _periph_read), which gets the BROM past its PLL-lock / boot-config polls.
 * Unmodelled offsets read as 0 and are logged (the P2 work list). This is the
 * first of the native peripheral device models replacing the Python hooks.
 */
/*
 * TC1797 STM (System Timer Module), base 0xF0000200.
 *
 * The STM is a 56-bit free-running up-counter. The firmware uses it for timed
 * busy-wait delays (e.g. fn_80018e18's loop at 0x80018eb0: target = TIM0 + d4;
 * while (TIM0 < target);) and, later, as the OSEK system tick (CMP0 compare
 * interrupt). With an unmodelled (constant-0) TIM0 those delay loops never
 * terminate -- the first native stall after the 0x80018eac WDT-spin patch.
 *
 * We drive the counter from QEMU_CLOCK_VIRTUAL (monotonic, advances with run
 * time even inside a tight TCG loop in non-icount mode), modelled at fSTM =
 * 100 MHz (10 ns/tick). The absolute rate only sets how quickly firmware delays
 * elapse; the structure -- 56-bit width, the seven 4-bit-stepped TIMx prescaler
 * views, and the CAP latch -- is faithful to the TC1797 UM and matches
 * bridge_server.py's STM model. The CMP0/CMP1 compare-match interrupt is not
 * wired yet (returns no-pending); that lands with the STM->IR->CPU path.
 */
static uint64_t tc1797_stm_count56(uint32_t ns_per_tick)
{
    uint64_t ns = (uint64_t)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    /* 56-bit free-running counter at f_stm (ns_per_tick = 1e9 / f_stm;
     * = 10 at the 100 MHz default, reproducing the former hardcoded /10). */
    return (ns / ns_per_tick) & 0x00FFFFFFFFFFFFFFULL;
}

/*
 * STM CMP0 compare interrupt = the OSEK system tick (RE'd from FUN_80001160,
 * the firmware's tick-arm). The real TC1797 wiring (NOT bridge_server.py's
 * conflated model) is two-stage:
 *   - STM ICR (0xF000023C): CMP0EN[0] enables the CMP0 compare to raise a
 *     service request (CMP0IR[1] is the match flag, CMP0OS[2] output select;
 *     CMP1 uses [4]/[5]/[6]). FUN_80001160 writes 0x51 = CMP0EN|CMP1EN|CMP1OS.
 *   - STM SRC0 (0xF00002F8): the service-request node carrying SRPN[7:0] +
 *     SRE[12] + TOS[10]. FUN_80001160 writes 0x1001 = SRPN 1, SRE -> the SRPN-1
 *     OSEK counter-0 tick. (SRC1 @0xF00002FC = 0x1002 = SRPN 2 for CMP1.)
 * It also sets CMCON=0x1f001f (32-bit compares) and CMP0 = TIM0 + 9000.
 *
 * When TIM reaches CMP0 we raise SRPN = SRC0[7:0] via the native interrupt path;
 * the CPU takes it if ICR.IE && SRPN > CCPN and vectors to BIV + SRPN*32. The
 * firmware's tick ISR advances the OSEK time base/counter and re-arms CMP0.
 */
/* A compare channel's tick is live when: compares configured (CMCON), the
 * channel match is enabled (ICR.CMP0EN bit0 / CMP1EN bit4), and its SRC node is
 * CPU-routed (TOS bit10 == 0) + enabled (SRE bit12). ch=0 -> CMP0/SRC0,
 * ch=1 -> CMP1/SRC1. */
static bool tc1797_stm_cmp_enabled(TC1797SoCState *s, int ch)
{
    uint32_t en  = ch ? (s->stm_icr & 0x10u) : (s->stm_icr & 1u);
    uint32_t src = ch ? s->stm_src1 : s->stm_src0;
    return (s->stm_cmcon != 0) && en
        && ((src >> 12) & 1u) && !((src >> 10) & 1u);
}

static void tc1797_icu_set(TC1797SoCState *s, uint8_t srpn);
static void tc1797_icu_arbitrate(TC1797SoCState *s);

/*
 * Windowed compare-match helper (TC1797 UM 11.3): ticks from `tim_now` to the
 * next rising edge where the (MSIZE+1)-bit field of the 56-bit counter starting
 * at bit MSTART equals CMP[ch] (CMCON.MSIZE0/MSTART0 for ch0, MSIZE1/MSTART1 for
 * ch1). Used to time the faithful CMP0/CMP1 service requests. 0 -> a full
 * period (so we don't busy-refire the match we are already sitting on).
 */
static uint64_t tc1797_stm_match_ticks(TC1797SoCState *s, int ch, uint64_t tim_now)
{
    uint32_t cmcon  = s->stm_cmcon;
    uint32_t msize  = ch ? ((cmcon >> 16) & 0x1Fu) : (cmcon & 0x1Fu);
    uint32_t mstart = ch ? ((cmcon >> 24) & 0x1Fu) : ((cmcon >> 8) & 0x1Fu);
    uint32_t cmp    = ch ? s->stm_cmp1 : s->stm_cmp0;
    unsigned nbits  = msize + 1;                          /* 1..32 compared bits */
    uint64_t cmask  = (nbits >= 32) ? 0xFFFFFFFFull : ((1ull << nbits) - 1u);
    uint64_t period = (cmask + 1u) << mstart;             /* TIM period of the field */
    uint64_t start  = ((uint64_t)cmp & cmask) << mstart;  /* TIM value at the edge */
    uint64_t pos    = tim_now & (period - 1u);
    uint64_t delta  = (start - pos) & (period - 1u);
    return delta ? delta : period;
}

/*
 * Faithful edge-on-match arm (TC1797 UM 11.3): program the QEMUTimer for the
 * earliest enabled CMP0/CMP1 windowed match. If no compare is enabled the STM
 * raises nothing (the firmware has not started a compare). Re-invoked from the
 * CMP/CMCON/ICR/SRC writes so the cadence tracks the firmware's programmed
 * compare values exactly, as on silicon.
 */
static void tc1797_stm_arm(TC1797SoCState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t tim_now = tc1797_stm_count56(s->stm_ns_per_tick);
    int64_t best = INT64_MAX;
    for (int ch = 0; ch < 2; ch++) {
        if (!tc1797_stm_cmp_enabled(s, ch)) {
            s->stm_cmp_deadline[ch] = INT64_MAX;
            continue;
        }
        int64_t when = now + (int64_t)(tc1797_stm_match_ticks(s, ch, tim_now)
                                       * s->stm_ns_per_tick);
        s->stm_cmp_deadline[ch] = when;
        if (when < best) {
            best = when;
        }
    }
    s->stm_period_ns = (best == INT64_MAX) ? 0 : (best - now);
    if (best == INT64_MAX) {
        timer_del(s->stm_timer);
    } else {
        timer_mod(s->stm_timer, best);
    }
}

static void tc1797_stm_ack(TC1797SoCState *s)
{
    /* Source acknowledged: drop the request line + pending priority. */
    CPUTriCoreState *env = &s->cpu.env;
    env->ICR = FIELD_DP32(env->ICR, ICR, PIPN, 0);
    cpu_reset_interrupt(CPU(&s->cpu), CPU_INTERRUPT_HARD);
}

/*
 * Highest-priority CPU-routed CAN RX service request currently pending, or 0.
 * Re-derived from the MOs' rx_pending flags (the source of truth) each tick:
 * a receive MO with a frame waiting routes via MOIPR.RXINP to a CAN interrupt
 * SRC node (0xF00040C0+node*4); if that node targets the CPU (TOS==0, SRE) its
 * SRPN is a pending request. This makes CAN RX a *level* source (it persists
 * until the firmware reads the MO and rx_pending clears) so the single-slot ICU
 * can't drop it under a busier higher-priority source. Cheap: the common
 * no-pending case is a 128-bool scan; SRC lookups happen only for live frames.
 */
/*
 * Resolve a receive MO's CPU service-request priority from its MOIPR. RXINP[12:8]
 * is NOT the SRPN -- it selects one of the 16 MultiCAN interrupt nodes, whose
 * SRC register (0xF00040C0 + node*4) carries the real SRPN[7:0] + routing
 * (TOS[10]: 0=CPU/1=PCP, SRE[12]=enable). Return that SRPN iff the node is
 * CPU-routed and enabled; 0 otherwise (PCP-routed/disabled -> no CPU raise).
 * The old code used the node index as the SRPN, so e.g. the FEM RX MOs (node 0
 * -> SRPN 29, the firmware's unified CAN-RX ISR) raised SRPN 0 = nothing, and
 * the firmware never drained any RX frame nor answered diagnostics.
 */
static uint8_t tc1797_can_rx_srpn_of(Tc1797Can *c, uint32_t moipr)
{
    uint8_t node = (moipr >> 8) & 0x1Fu;          /* MOIPR.RXINP */
    uint32_t src = tc1797_can_read(c, TC1797_CAN_BASE + 0xC0u + (node & 0xFu) * 4u);
    if (((src >> 12) & 1u) && !((src >> 10) & 1u)) {   /* SRE && TOS==CPU */
        return src & 0xFFu;
    }
    return 0;
}

/*
 * Faithful STM compare-match service (TC1797 UM 11.3, edge-on-match). The
 * 56-bit counter (tc1797_stm_count56), the TIM0..TIM6/CAP reads and the windowed
 * CMP0/CMP1 compare (CMCON.MSIZE/MSTART) are all faithful. Fire CMP0/CMP1 only
 * when their windowed compare actually reaches the programmed value -- a rising
 * edge, once per match -- raising SRC0/SRC1 at the configured SRPN. The firmware's
 * ISR re-arms CMP[x] (the CMP write recomputes the next match via
 * tc1797_stm_arm). No fire-all, no round-robin, no floor/cap, no forced poll: the
 * cadence is exactly the firmware's programmed compare values, as on silicon.
 * The OSEK dispatch SRN (0xF7E0FFFC) and CAN-RX SRNs raise themselves from their
 * own write/receive paths.
 *
 * NOTE: with this faithful STM the firmware does NOT currently reach phase 4 --
 * it never configures the STM CMP0 OSEK system tick in the cold-init path we can
 * reach, stalling at phase 3 behind the same unresolved gate that leaves the
 * software-counter object pointer DAT_d00009a4 unwritten (task #124). That
 * cold-init gate -- not the STM model -- is the real blocker.
 */
static void tc1797_stm_tick(void *opaque)
{
    TC1797SoCState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool raised = false;

    /* CONSENSUS FIX (3-agent tie-break, HIGH confidence): the OSEK schedule-tick ISR
     * (FUN_80081870, raised by the SRPN-7 STM compare) dispatches *(*cursor) as a
     * function pointer, where cursor = DAT_d0012498 (DSPR 0xD0012498). If the
     * schedule table has not been started yet -- FUN_8012529c (OSEK StartScheduleTable)
     * sets DAT_d0012498 = (*(DAT_d0000978+4))[1] and only then arms this tick -- the
     * cursor is NULL, so the ISR does `calli 0` -> jumps to NULL -> the CPU marches
     * NOPs through unmapped segment 0 forever (the phase 3->4 hang). Real silicon's OS
     * tick does not dispatch before the schedule table exists. Faithful accommodation:
     * defer the OS tick until the cursor is non-NULL. Env-gated (TC1797_TICK_GATE);
     * default build unchanged. */
    {
        static int gate = -1;
        if (gate < 0) {
            gate = getenv("TC1797_TICK_GATE") ? 1 : 0;
        }
        if (gate) {
            uint32_t cursor = 0;
            cpu_physical_memory_read(0xD0012498u, &cursor, 4);
            if (cursor == 0) {
                timer_mod(s->stm_timer, now + 1000000);   /* recheck in 1 ms virtual */
                return;
            }
        }
    }

    for (int ch = 0; ch < 2; ch++) {
        if (!tc1797_stm_cmp_enabled(s, ch)) {
            s->stm_cmp_deadline[ch] = INT64_MAX;
            continue;
        }
        if (now < s->stm_cmp_deadline[ch]) {
            continue;                       /* this channel's match is still future */
        }
        if (ch == 0) {
            s->stm_isrr |= 1u; s->stm_icr |= 2u;          /* CMP0 request + CMP0IR */
        } else {
            s->stm_isrr |= 2u; s->stm_icr |= 0x20u;       /* CMP1 request + CMP1IR */
        }
        uint8_t srpn = (ch ? s->stm_src1 : s->stm_src0) & 0xFFu;
        if (srpn) {
            tc1797_icu_set(s, srpn);
            raised = true;
        }
        /* Advance one full window-period; the ISR's CMP write re-arms sooner. */
        uint64_t tim_now = tc1797_stm_count56(s->stm_ns_per_tick);
        s->stm_cmp_deadline[ch] = now +
            (int64_t)(tc1797_stm_match_ticks(s, ch, tim_now) * s->stm_ns_per_tick);
    }

    if (raised) {
        tc1797_icu_arbitrate(s);
    }

    int64_t best = INT64_MAX;
    for (int ch = 0; ch < 2; ch++) {
        if (tc1797_stm_cmp_enabled(s, ch) && s->stm_cmp_deadline[ch] < best) {
            best = s->stm_cmp_deadline[ch];
        }
    }
    if (best != INT64_MAX) {
        timer_mod(s->stm_timer, best);
    }
}

static uint64_t tc1797_stm_read(TC1797SoCState *s, uint32_t addr)
{
    if (addr >= 0xF0000210 && addr <= 0xF000022C) {
        /* TC1797 UM 11.x: TIMk = bits[k*4+31 : k*4] (k=0..6); +0x2C = CAP =
         * bits[63:32] of the 56-bit counter. */
        uint64_t v56 = tc1797_stm_count56(s->stm_ns_per_tick);
        unsigned idx = (addr - 0xF0000210u) >> 2;
        if (idx <= 6) {
            return (uint32_t)(v56 >> (idx * 4));
        }
        return (uint32_t)(v56 >> 32);            /* CAP */
    }
    switch (addr) {
    case 0xF0000208: return 0x0000C000;          /* STM_ID: MODTYPE 0xC0 (was leaking count56) */
    /* STM_CLC (0xF0000200) is intentionally NOT intercepted: the firmware's early
     * cold-init reads 0xF0000200 and depends on a TIME-VARYING value (the count56
     * the default below leaks). Returning the faithful CLC reset (0x200) stalls the
     * boot before DSPR block-init. Flagged for RE -- what the firmware wants at
     * 0xF0000200 -- before STM_CLC can be modelled faithfully. */
    case 0xF0000230: return s->stm_cmp0;
    case 0xF0000234: return s->stm_cmp1;
    case 0xF0000238: return s->stm_cmcon;
    case 0xF000023C:                             /* ICR: raw register (CMP0EN/IR/OS...) */
        /* Return the stored register so the firmware's read-modify-write (e.g.
         * FUN_80001160's `ICR & ~bit2 | 0x51`) preserves the enable bits; overlay
         * the live match flags CMP0IR[1]/CMP1IR[5] from the pending bits. */
        return (s->stm_icr & ~0x22u)
             | ((s->stm_isrr & 1u) << 1)
             | (((s->stm_isrr >> 1) & 1u) << 5);
    case 0xF0000240: return s->stm_isrr;         /* ISRR: pending bits (tuned baseline) */
    case 0xF00002F8:                             /* SRC0 node (SRPN/SRE/TOS + SRR) */
        return (s->stm_src0 & ~0x2000u) | ((s->stm_isrr & 1u) << 13);
    case 0xF00002FC:                             /* SRC1 node */
        return (s->stm_src1 & ~0x2000u) | (((s->stm_isrr >> 1) & 1u) << 13);
    default:         return (uint32_t)tc1797_stm_count56(s->stm_ns_per_tick);
    }
}

static void tc1797_stm_write(TC1797SoCState *s, uint32_t addr, uint32_t val)
{
    switch (addr) {
    case 0xF0000230:                            /* CMP0 */
        s->stm_cmp0 = val;
        tc1797_stm_arm(s);
        break;
    case 0xF0000234:                            /* CMP1 */
        s->stm_cmp1 = val;
        tc1797_stm_arm(s);
        break;
    case 0xF0000238:                            /* CMCON: compare window/enable */
        s->stm_cmcon = val;
        tc1797_stm_arm(s);
        break;
    case 0xF000023C:                            /* ICR: CMP0EN[0]/CMP1EN[4]/... */
        s->stm_icr = val;
        tc1797_stm_arm(s);
        break;
    case 0xF0000240:                            /* ISRR: write-1-to-clear pending */
        if (val & 1u) {
            s->stm_isrr &= ~1u;
            s->stm_icr &= ~2u;                  /* clear CMP0IR match flag */
        }
        if (val & 2u) {
            s->stm_isrr &= ~2u;
            s->stm_icr &= ~0x20u;               /* clear CMP1IR */
        }
        if (s->stm_isrr == 0) {
            tc1797_stm_ack(s);
        }
        break;
    case 0xF00002F8:                            /* SRC0 node: SRPN/SRE/TOS + SETR/CLRR */
        if (val & 0x4000u) {                    /* CLRR: ack the CMP0 request */
            s->stm_isrr &= ~1u;
            s->stm_icr &= ~2u;
            if (s->stm_isrr == 0) {
                tc1797_stm_ack(s);
            }
        }
        s->stm_src0 = val & 0x00001FFFu;        /* keep SRPN/TOS/SRE config bits */
        tc1797_stm_arm(s);                      /* enabling SRE arms the tick */
        break;
    case 0xF00002FC:                            /* SRC1 node */
        if (val & 0x4000u) {                    /* CLRR: ack the CMP1 request */
            s->stm_isrr &= ~2u;
            s->stm_icr &= ~0x20u;
            if (s->stm_isrr == 0) {
                tc1797_stm_ack(s);
            }
        }
        s->stm_src1 = val & 0x00001FFFu;        /* keep SRPN/TOS/SRE config bits */
        tc1797_stm_arm(s);                      /* enabling CMP1's SRE arms the tick */
        break;
    default: break;                             /* TIM/CAP read-only; rest ignored */
    }
}

/*
 * Forced OSEK system tick -- the QEMU-native analogue of bridge_server.py's
 * injected systick (tasks #21/#22/#26).
 *
 * The MEVD17 ERCOSEK kernel advances its time base and expires its alarms from
 * periodic timer interrupts; the phase 3 -> phase 4 promotion (master-task
 * activation + periodic-task arming) only happens once those ticks arrive.
 * The firmware DOES arm its tick sources -- STM CMP0 (SRC0 -> SRPN 8), STM CMP1
 * (SRC1 -> SRPN 7), and the cpu_src dispatch node (0xF7E0FFFC -> SRPN 1) -- and
 * the STM engine above now round-robins all three, which drives the phase
 * 3 -> phase 4 promotion ORGANICALLY by default (verified: phase byte
 * 0xD0003643 reaches 4 with the full DME periodic CAN-TX set). This forced
 * systick is therefore now a LEGACY opt-in fallback (TC1797_SYSTICK, OFF by
 * default), kept only for bring-up experiments; the STM path supersedes it.
 *
 * The bridge drives it by injecting interrupts at SRPN 1, 7, 8 round-robin at
 * ~1 ms (three independent OSEK counters; one fire == one counter advance). We
 * do the same through QEMU's real interrupt path: set ICR.PIPN and raise
 * CPU_INTERRUPT_HARD, and the native tricore_cpu_do_interrupt performs the
 * canonical entry (CSA save, vector to BIV + SRPN*32) with a native rfe on the
 * way out -- none of the manual CSA juggling the Unicorn bridge needs.
 *
 * Faithful gating (mirrors the bridge's check_priority=True cold-init path): a
 * real timer tick is MASKED, not latched, while the firmware has interrupts
 * disabled (ICR.IE=0) or is already running at >= this priority (critical
 * sections, the RAM self-test). So we evaluate the same gate the silicon would
 * (IE==1 && SRPN > CCPN) and simply DROP a tick that can't be delivered now,
 * rather than leaving a stale request latched to fire later out of context. A
 * request that was raised but never taken within one tick period (IE stayed 0
 * the whole time) is likewise dropped on the next fire.
 */
static const uint8_t tc1797_osek_tick_prios[] = { 1, 7, 8 };

/*
 * Bottom-half that performs a deferred interrupt raise. env->ICR is a TCG global
 * (cpu_ICR), so writing it from inside an MMIO store helper (TB context) is
 * unsafe: the stale cached global is written back at the end of the translation
 * block and wipes the PIPN we set. Device-register writes that need to raise an
 * interrupt therefore stage the SRPN and schedule this BH, which runs between
 * TBs (globals synced) where setting ICR.PIPN sticks. The native
 * tricore_cpu_exec_interrupt gate (IE && PIPN>CCPN) then takes it and clears
 * PIPN+HARD on service-entry (the ack in target/tricore/cpu.c), so the
 * software-triggered request is a clean one-shot.
 */
/*
 * Faithful ICU arbiter. icu_pending is the set of CPU-routed service requests
 * currently asserting (one bit per SRPN). Present the HIGHEST as ICR.PIPN and
 * drive HARD; the CPU's exec_interrupt gate (IE && PIPN>CCPN) takes it, and the
 * service-entry ack (tc1797_icu_service_ack) clears that SRPN so we re-arbitrate
 * to the next-highest. This replaces the old single-PIPN slot, which a busy high
 * source (e.g. SSC) clobbered -- starving the lower OSEK ticks (SRPN 7/8) and
 * CAN RX so the OS time base froze and UDS never answered.
 *
 * MUST run between TBs (BH / timer / service-entry), never from MMIO/TB context:
 * a PIPN write there is wiped by the TCG global write-back. MMIO-context raises
 * set the bit and schedule the BH.
 */
static void tc1797_icu_arbitrate(TC1797SoCState *s)
{
    CPUTriCoreState *env = &s->cpu.env;
    uint8_t best = 0;
    for (int w = 7; w >= 0; w--) {
        uint32_t v = s->icu_pending[w];
        if (v) {
            int b = 31;
            while (!(v & (1u << b))) {
                b--;
            }
            best = (uint8_t)(w * 32 + b);
            break;
        }
    }
    env->ICR = FIELD_DP32(env->ICR, ICR, PIPN, best);
    if (best) {
        cpu_interrupt(CPU(&s->cpu), CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(CPU(&s->cpu), CPU_INTERRUPT_HARD);
    }
}

static void tc1797_icu_set(TC1797SoCState *s, uint8_t srpn)
{
    if (srpn) {
        s->icu_pending[srpn >> 5] |= 1u << (srpn & 31u);
    }
}

static void tc1797_icu_clr(TC1797SoCState *s, uint8_t srpn)
{
    s->icu_pending[srpn >> 5] &= ~(1u << (srpn & 31u));
}

/*
 * Service-entry acknowledge, called from target/tricore/cpu.c when the CPU takes
 * the interrupt: clear the taken SRPN (one-shot, mirroring the SRN's SRR
 * auto-clear on service entry) and present the next-highest pending. Runs at the
 * TB boundary, so the PIPN write sticks. Level sources (STM compare match still
 * pending, CAN MO still NEWDAT) re-assert from their own model on the next tick.
 */
static void tc1797_icu_service_ack(void *opaque, uint32_t taken)
{
    TC1797SoCState *s = opaque;
    tc1797_icu_clr(s, (uint8_t)taken);
    /* SRR auto-clears on service entry for edge nodes the firmware SETRs (the
     * CPU service-request/dispatch node). Without this its SRR stays set and the
     * tick re-asserts it every interval (over-dispatch). Level sources (STM
     * compare, CAN MO NEWDAT) intentionally re-assert from their own model. */
    if ((uint8_t)taken && (s->cpu_src & 0xFFu) == (uint8_t)taken) {
        s->cpu_src &= ~(1u << 13);          /* clear SRR */
    }
    tc1797_icu_arbitrate(s);
}

static void tc1797_irq_bh(void *opaque)
{
    tc1797_icu_arbitrate((TC1797SoCState *)opaque);
}

/*
 * Raise a CPU-bound service request at priority srpn. Callers run in MMIO/TB
 * context (a peripheral SRN SETR write), so set the pending bit now and defer
 * the ICR.PIPN write + HARD to tc1797_irq_bh (between TBs). The bit persists in
 * icu_pending until the source is serviced -- real arbitration, not one slot.
 */
static void tc1797_raise_srpn(TC1797SoCState *s, uint8_t srpn)
{
    if (srpn == 0) {
        return;
    }
    tc1797_icu_set(s, srpn);
    qemu_bh_schedule(s->irq_bh);
    /* Kick the vCPU out of its current TB so the BH runs promptly; the HARD line
     * persists (it lives in CPUState, not the clobbered TCG global). */
    cpu_interrupt(CPU(&s->cpu), CPU_INTERRUPT_HARD);
}

/*
 * PCP -> CPU: a channel program that EXITs with INT=1 raises the matching CPU
 * service request. Invoked from the PCP worker thread while it holds the BQL,
 * so it may poke the SoC IRQ path directly (tc1797_raise_srpn defers the actual
 * ICR write to a bottom half).
 */
static void tc1797_pcp_irq(void *opaque, uint8_t srpn)
{
    tc1797_raise_srpn((TC1797SoCState *)opaque, srpn);
}

/*
 * MultiCAN RX -> CPU service request. A frame was accepted into receive MO
 * `mo`; the faithful TC1797 MultiCAN pulses that MO's interrupt output. Route
 * MOIPR.RXINP (bits[11:8]) to one of the 16 CAN interrupt SRC nodes at
 * 0xF00040C0 + node*4; if that node is enabled (SRE) and targets the CPU
 * (TOS==0), raise its SRPN. Without this the firmware's interrupt-driven CAN
 * RX never wakes -- NEWDAT just piles up unread, so it never consumes RX
 * frames nor answers UDS/diagnostic requests. (PCP-routed nodes (TOS==1) are
 * left for the PCP path; the diag + FEM RX objects here target the CPU.)
 */
static void tc1797_can_rx_irq(void *opaque, int mo)
{
    TC1797SoCState *s = opaque;
    /* Diagnostic experiment (env TC1797_NORXINT): suppress the CPU RX interrupt
     * so receive MOs are left for the firmware's *polled* consumers (the diag
     * task polls MO84 via FUN_800dc44c). Tests whether the SRPN-29 unified RX ISR
     * is draining the diag MO before the diag task can. */
    static int norx = -1;
    if (norx < 0) {
        norx = getenv("TC1797_NORXINT") ? 1 : 0;
    }
    if (norx) {
        return;
    }
    uint32_t moipr = tc1797_can_read(&s->can,
                                     TC1797_CAN_MO_BASE + mo * 0x20 + 0x08);
    uint8_t srpn = tc1797_can_rx_srpn_of(&s->can, moipr);  /* node -> real SRPN */
    if (srpn) {
        tc1797_raise_srpn(s, srpn);                    /* prompt best-effort raise */
    }
}

/*
 * MultiCAN TX: the firmware committed a frame (MOCTR SET-TXRQ). Surface it
 * (rate-limited). A QEMU CAN-bus backend can be attached here later so the
 * frame leaves the model; for now it is logged + counted in s->can.
 */
static void tc1797_can_tx(void *opaque, uint32_t can_id,
                          const uint8_t *data, unsigned len)
{
    TC1797SoCState *s = opaque;
    static unsigned once;
    if (once < 64) {
        once++;
        info_report("tc1797: CAN TX id=0x%03x data=%02x%02x%02x%02x%02x%02x%02x%02x",
                    can_id, data[0], data[1], data[2], data[3],
                    data[4], data[5], data[6], data[7]);
    }
    /* Emit onto the QEMU CAN bus so the DME's output leaves the model. */
    if (s->canbus) {
        qemu_can_frame f = { 0 };
        f.can_id = can_id & QEMU_CAN_SFF_MASK;
        f.can_dlc = len > 8 ? 8 : len;
        memcpy(f.data, data, f.can_dlc);
        can_bus_client_send(&s->can_client, &f, 1);
    }
    /* Stream out the host CAN tunnel (-chardev ...,id=can0) as one ASCII line
     * "<id_hex>#<data_hex>\n", so a front-end can observe DME TX directly. */
    {
        char line[64];
        unsigned n = len > 8 ? 8 : len;
        int o = snprintf(line, sizeof(line), "%x#", can_id);
        for (unsigned i = 0; i < n && o < (int)sizeof(line) - 3; i++) {
            o += snprintf(line + o, sizeof(line) - o, "%02x", data[i]);
        }
        line[o++] = '\n';
        qemu_chr_fe_write_all(&s->can_tun, (const uint8_t *)line, o);
    }
}

/* ── CAN bus client: inbound frames from the bus -> matching receive MO ── */
static bool tc1797_can_bus_can_receive(CanBusClientState *client)
{
    return true;                  /* always ready to accept */
}

static ssize_t tc1797_can_bus_receive(CanBusClientState *client,
                                      const qemu_can_frame *frames, size_t cnt)
{
    TC1797SoCState *s = container_of(client, TC1797SoCState, can_client);
    for (size_t i = 0; i < cnt; i++) {
        const qemu_can_frame *f = &frames[i];
        if (f->can_id & (QEMU_CAN_RTR_FLAG | QEMU_CAN_ERR_FLAG)) {
            continue;             /* ignore RTR / error frames */
        }
        tc1797_can_rx_inject(&s->can, f->can_id & QEMU_CAN_EFF_MASK,
                             f->data, f->can_dlc);
    }
    return cnt;
}

static CanBusClientInfo tc1797_can_bus_info = {
    .can_receive = tc1797_can_bus_can_receive,
    .receive = tc1797_can_bus_receive,
};

/* ── Host CAN tunnel (chardev id=can0): inject inbound frames into the DME ──
 * Line protocol: "<id_hex>#<data_hex>\n", e.g. "135#0102030405060708". Each
 * complete line is parsed and pushed through the normal MultiCAN RX acceptance
 * path (tc1797_can_rx_inject), exactly as an on-bus frame would be. */
static void tc1797_can_tun_parse(TC1797SoCState *s, const char *line)
{
    char *p = NULL;
    unsigned long id = strtoul(line, &p, 16);
    uint8_t data[8] = { 0 };
    unsigned dlc = 0;
    if (p && *p == '#') {
        p++;
        while (dlc < 8 && p[0] && p[1]) {
            char b[3] = { p[0], p[1], 0 };
            data[dlc++] = (uint8_t)strtoul(b, NULL, 16);
            p += 2;
        }
    }
    {
        static unsigned rn;
        if (rn < 8) {
            rn++;
            info_report("tc1797: CAN tunnel RX inject id=0x%x dlc=%u",
                        (uint32_t)id, dlc);
        }
    }
    tc1797_can_rx_inject(&s->can, (uint32_t)id, data, dlc);
}

static int tc1797_can_tun_can_receive(void *opaque)
{
    return 64;                       /* accept up to this many bytes per call */
}

static void tc1797_can_tun_receive(void *opaque, const uint8_t *buf, int size)
{
    TC1797SoCState *s = opaque;
    for (int i = 0; i < size; i++) {
        char c = (char)buf[i];
        if (c == '\n' || c == '\r') {
            if (s->can_tun_rxlen) {
                s->can_tun_rx[s->can_tun_rxlen] = 0;
                tc1797_can_tun_parse(s, s->can_tun_rx);
                s->can_tun_rxlen = 0;
            }
        } else if (s->can_tun_rxlen < sizeof(s->can_tun_rx) - 1) {
            s->can_tun_rx[s->can_tun_rxlen++] = c;
        } else {
            s->can_tun_rxlen = 0;    /* overrun: drop the malformed line */
        }
    }
}

/* ── Host E-Ray/CP tunnel (chardev id=fray0): the CAS/immobilizer challenge-
 * response runs over the E-Ray transport, which this firmware drives through a
 * DSPR CP message struct + a PRAM command buffer + a doorbell SFR (not the
 * E-Ray register block). The tunnel:
 *   - TX (DME->host): on the doorbell write (0xF00027B1 bit7) snapshot the CP
 *     command/type words, mark the controller "done" so the firmware's post-TX
 *     poll proceeds, and emit "ERAYTX <be0> <be8>\n". The front-end then reads
 *     the full outbound CP buffer over gdb and computes the response.
 *   - RX (host->DME): a line "<cmd>:<type>:<payloadhex>" is written into the CP
 *     RX struct at DSPR 0xD0012FAE; the firmware's CP sequencer consumes it on
 *     its next task dispatch (the path is task-scheduled, not IRQ-driven). */
#define TC1797_CP_RX_CMD     0xD0012FAEu   /* cmd byte                        */
#define TC1797_CP_RX_PAYLOAD 0xD0012FAFu   /* payload[0..128]                 */
#define TC1797_CP_RX_LEN     0xD0013030u   /* length byte                     */
#define TC1797_CP_RX_TYPE    0xD0013031u   /* message type byte               */
#define TC1797_ERAY_TX_CMD   0xF0050BE0u   /* outbound CP command word        */
#define TC1797_ERAY_TX_TYPE  0xF0050BE8u   /* outbound CP type word           */
#define TC1797_ERAY_TX_REQ   0xF0050BECu   /* request/status (bit0 ready/1 busy) */
#define TC1797_ERAY_RDY      0xF0050A98u   /* ready flag (bit0)               */
#define TC1797_ERAY_DOORBELL 0xF00027B1u   /* TX doorbell SFR (bit7)          */

static void tc1797_eray_doorbell(TC1797SoCState *s)
{
    uint32_t be0 = 0, be8 = 0, w = 0;
    cpu_physical_memory_read(TC1797_ERAY_TX_CMD, &be0, 4);
    cpu_physical_memory_read(TC1797_ERAY_TX_TYPE, &be8, 4);
    /* Mark the (unmodelled) controller "done, not busy" so the firmware's
     * post-dispatch poll of the request/ready words completes instead of
     * spinning forever waiting for hardware that isn't there. */
    cpu_physical_memory_read(TC1797_ERAY_TX_REQ, &w, 4);
    w = (w | 1u) & ~2u;
    cpu_physical_memory_write(TC1797_ERAY_TX_REQ, &w, 4);
    cpu_physical_memory_read(TC1797_ERAY_RDY, &w, 4);
    w |= 1u;
    cpu_physical_memory_write(TC1797_ERAY_RDY, &w, 4);
    if (qemu_chr_fe_backend_connected(&s->eray_tun)) {
        char line[64];
        int o = snprintf(line, sizeof(line), "ERAYTX %08x %08x\n", be0, be8);
        qemu_chr_fe_write_all(&s->eray_tun, (const uint8_t *)line, o);
    }
    {
        static unsigned dn;
        if (dn < 8) {
            dn++;
            info_report("tc1797: E-Ray CP doorbell cmd=0x%08x type=0x%08x", be0, be8);
        }
    }
}

static void tc1797_eray_tun_parse(TC1797SoCState *s, const char *line)
{
    /* format: <cmd_hex>:<type_hex>:<payload_hex>  e.g. 84:02:aabbcc... */
    char *p = NULL;
    unsigned long cmd = strtoul(line, &p, 16);
    unsigned long type = 0;
    uint8_t payload[129] = { 0 };
    unsigned plen = 0;
    if (p && *p == ':') {
        p++;
        type = strtoul(p, &p, 16);
        if (p && *p == ':') {
            p++;
            while (plen < sizeof(payload) && p[0] && p[1]) {
                char b[3] = { p[0], p[1], 0 };
                payload[plen++] = (uint8_t)strtoul(b, NULL, 16);
                p += 2;
            }
        }
    }
    uint8_t cb = (uint8_t)cmd, tb = (uint8_t)type, lb = (uint8_t)plen;
    cpu_physical_memory_write(TC1797_CP_RX_CMD, &cb, 1);
    cpu_physical_memory_write(TC1797_CP_RX_PAYLOAD, payload, plen);
    cpu_physical_memory_write(TC1797_CP_RX_LEN, &lb, 1);
    cpu_physical_memory_write(TC1797_CP_RX_TYPE, &tb, 1);
    {
        static unsigned rn;
        if (rn < 8) {
            rn++;
            info_report("tc1797: E-Ray CP RX inject cmd=0x%x type=0x%x len=%u",
                        (unsigned)cmd, (unsigned)type, plen);
        }
    }
}

static int tc1797_eray_tun_can_receive(void *opaque)
{
    return 128;
}

static void tc1797_eray_tun_receive(void *opaque, const uint8_t *buf, int size)
{
    TC1797SoCState *s = opaque;
    for (int i = 0; i < size; i++) {
        char c = (char)buf[i];
        if (c == '\n' || c == '\r') {
            if (s->eray_tun_rxlen) {
                s->eray_tun_rx[s->eray_tun_rxlen] = 0;
                tc1797_eray_tun_parse(s, s->eray_tun_rx);
                s->eray_tun_rxlen = 0;
            }
        } else if (s->eray_tun_rxlen < sizeof(s->eray_tun_rx) - 1) {
            s->eray_tun_rx[s->eray_tun_rxlen++] = c;
        } else {
            s->eray_tun_rxlen = 0;
        }
    }
}

/*
 * Host memory peek/poke tunnel (-chardev socket,id=mem0,...). This is the
 * clean, zero-perturbation replacement for the gdb stub's run-control dance:
 * memory is served from the SoC side via cpu_physical_memory_read/write (in the
 * iothread under the BQL), so the vCPU keeps running at full real-time speed.
 *
 * ASCII line protocol (the front-end's QemuLink mem channel speaks it):
 *   "R <addr_hex> <len_dec>\n"      -> "<hex>\n"   (len bytes, max 512)
 *   "W <addr_hex> <payload_hex>\n"  -> "OK\n"
 * malformed / out-of-range -> "E\n".
 */
#define TC1797_MEM_TUN_MAX 512
static void tc1797_mem_tun_parse(TC1797SoCState *s, const char *line)
{
    char op = line[0];
    char *p = NULL;

    if (op == 'R' || op == 'r') {
        unsigned long addr = strtoul(line + 1, &p, 16);
        unsigned long len = (p && *p) ? strtoul(p, NULL, 0) : 0;
        if (len == 0 || len > TC1797_MEM_TUN_MAX) {
            qemu_chr_fe_write_all(&s->mem_tun, (const uint8_t *)"E\n", 2);
            return;
        }
        uint8_t buf[TC1797_MEM_TUN_MAX];
        cpu_physical_memory_read(addr, buf, len);
        char out[TC1797_MEM_TUN_MAX * 2 + 2];
        int o = 0;
        for (unsigned i = 0; i < len; i++) {
            o += snprintf(out + o, sizeof(out) - o, "%02x", buf[i]);
        }
        out[o++] = '\n';
        qemu_chr_fe_write_all(&s->mem_tun, (const uint8_t *)out, o);
    } else if (op == 'W' || op == 'w') {
        unsigned long addr = strtoul(line + 1, &p, 16);
        uint8_t buf[TC1797_MEM_TUN_MAX];
        unsigned n = 0;
        if (p) {
            while (*p == ' ') {
                p++;
            }
            while (n < sizeof(buf) && p[0] && p[1] && p[0] != ' ') {
                char b[3] = { p[0], p[1], 0 };
                buf[n++] = (uint8_t)strtoul(b, NULL, 16);
                p += 2;
            }
        }
        if (n) {
            cpu_physical_memory_write(addr, buf, n);
        }
        qemu_chr_fe_write_all(&s->mem_tun, (const uint8_t *)"OK\n", 3);
    } else {
        qemu_chr_fe_write_all(&s->mem_tun, (const uint8_t *)"E\n", 2);
    }
}

static int tc1797_mem_tun_can_receive(void *opaque)
{
    return 512;
}

static void tc1797_mem_tun_receive(void *opaque, const uint8_t *buf, int size)
{
    TC1797SoCState *s = opaque;
    for (int i = 0; i < size; i++) {
        char c = (char)buf[i];
        if (c == '\n' || c == '\r') {
            if (s->mem_tun_rxlen) {
                s->mem_tun_rx[s->mem_tun_rxlen] = 0;
                tc1797_mem_tun_parse(s, s->mem_tun_rx);
                s->mem_tun_rxlen = 0;
            }
        } else if (s->mem_tun_rxlen < sizeof(s->mem_tun_rx) - 1) {
            s->mem_tun_rx[s->mem_tun_rxlen++] = c;
        } else {
            s->mem_tun_rxlen = 0;
        }
    }
}

/*
 * Watchdog timeout. Armed only when modelled (env TC1797_WDT); the firmware
 * reloads it via the WDT_CON0 password/ENDINIT servicing sequence. If servicing
 * stops (a hang) the window expires and the SoC resets -- the faithful WDT
 * behaviour the fail-safe self-loops rely on. Default OFF so the bare-shadow
 * boot is unaffected.
 */
#define TC1797_WDT_WINDOW_NS (50 * 1000 * 1000ll)   /* 50 ms service window */
static void tc1797_wdt_tick(void *opaque)
{
    (void)opaque;
    info_report("tc1797: WDT timeout (no servicing within %lld ms) -> reset",
                (long long)(TC1797_WDT_WINDOW_NS / 1000000));
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void tc1797_wdt_service(TC1797SoCState *s, uint32_t con0)
{
    /* WDT_CON0 password/ENDINIT servicing write: track ENDINIT (bit0) and, when
     * the watchdog is modelled, reload the service window. */
    s->wdt_con0 = con0;
    s->wdt_endinit = con0 & 1u;
    s->wdt_service_count++;
    if (s->wdt_modeled && s->wdt_timer) {
        timer_mod(s->wdt_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TC1797_WDT_WINDOW_NS);
    }
}

/*
 * External reset / NMI request pins (ESR0, ESR1, HDRST), exposed as named GPIO
 * input lines so a board or the monitor can drive them -- genuine CPU-external
 * control pins (no internal source on this closed ECU, hence unconnected by
 * default). ESR0/HDRST trigger a system reset; ESR1 latches an NMI request.
 */
static void tc1797_esr_irq(void *opaque, int line, int level)
{
    TC1797SoCState *s = opaque;
    if (!level) {
        return;                         /* assertion (rising) edge only */
    }
    switch (line) {
    case 0:                             /* ESR0: external reset pin */
    case 2:                             /* HDRST: hardware (warm) reset pin */
        info_report("tc1797: %s asserted -> system reset",
                    line == 0 ? "ESR0" : "HDRST");
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        break;
    case 1:                             /* ESR1: NMI request */
        s->scu_nmisr |= 1u;
        info_report("tc1797: ESR1 NMI request latched (SCU_NMISR)");
        cpu_interrupt(CPU(&s->cpu), CPU_INTERRUPT_HARD);
        break;
    default:
        break;
    }
}

/*
 * Service Request Node register write (TC1.3.1 SRC layout: SRPN[7:0], TOS[10],
 * SRE[12], SRR[13], CLRR[14] write-1-to-clear, SETR[15] write-1-to-set). The
 * firmware drives several peripheral SRNs purely as software service requests
 * (it programs SRPN/SRE, then pokes SETR to make the ISR run). Model that: keep
 * the config bits, apply SETR/CLRR to SRR, and when SRR becomes set on an
 * enabled (SRE=1) node, route by TOS: TOS=0 -> CPU interrupt at this SRPN;
 * TOS=1 -> trigger the PCP channel for this SRPN (it runs on the PCP thread).
 */
static void tc1797_src_node_write(TC1797SoCState *s, uint32_t *node,
                                  uint32_t val, uint32_t addr)
{
    uint32_t srr = (*node >> 13) & 1u;
    /* SETR[15]/CLRR[14] set/clear the request bit. Per the UM SRC-register note,
     * when BOTH are written in the same access SRR is left UNCHANGED (neither wins)
     * -- the old code applied CLRR after SETR, so CLRR incorrectly won a tie. */
    if ((val & 0x8000u) && !(val & 0x4000u)) {
        srr = 1;                          /* SETR only */
    } else if ((val & 0x4000u) && !(val & 0x8000u)) {
        srr = 0;                          /* CLRR only */
    }
    *node = (val & 0x00001FFFu) | (srr << 13);
    if (!(srr && ((*node >> 12) & 1u))) {
        return;                           /* not a set, enabled request */
    }
    if ((*node >> 10) & 1u) {             /* TOS=1: peripheral control processor */
        if (getenv("TC1797_PCPLOG")) {
            static unsigned pc;
            if (pc++ < 64) {
                fprintf(stderr, "PCPLOG: TOS=1 SRN 0x%08x val=0x%08x SRPN=%u "
                        "pcp_en=%d -> trigger ch %u\n", addr, *node,
                        *node & 0xFFu, s->pcp_enabled, *node & 0xFFu);
                fflush(stderr);
            }
        }
        if (s->pcp_enabled) {
            pcp_engine_trigger(&s->pcp, *node & 0xFFu);
        }
        return;
    }
    {                                     /* TOS=0: CPU interrupt */
        static int once;
        if (once < 24) {
            once++;
            info_report("tc1797: SRN 0x%08x service-request raised "
                        "(val=0x%08x SRPN=%u)", addr, *node, *node & 0xFFu);
        }
        tc1797_raise_srpn(s, *node & 0xFFu);
    }
}

static void tc1797_osek_tick(void *opaque)
{
    TC1797SoCState *s = opaque;
    CPUTriCoreState *env = &s->cpu.env;
    CPUState *cs = CPU(&s->cpu);
    uint32_t ie   = FIELD_EX32(env->ICR, ICR, IE_13);
    uint32_t ccpn = FIELD_EX32(env->ICR, ICR, CCPN);
    bool raised = false;

    /* Phase-4 observability: latch + report once the RTOS phase byte reaches 4.
     * The byte reads garbage before the DSPR block-init (a transient >=4 can
     * appear early), so floor it on elapsed virtual time the way the bridge
     * floors on insn count. Purely diagnostic -- it does not gate the tick. */
    if (!s->osek_phase4
        && (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->osek_t0_ns) > 150 * 1000000LL) {
        uint8_t phase = 0, osrun = 0;
        cpu_physical_memory_read(0xD0003643, &phase, 1);
        /* Real RTOS phase is a small ordinal (4..~8); 0xFF is the DSPR power-on
         * prefill (uninitialised / just-reset), so exclude it as a false latch. */
        if (phase >= 4 && phase < 0x40) {
            cpu_physical_memory_read(0xD000400F, &osrun, 1);
            s->osek_phase4 = true;
            info_report("tc1797: OSEK reached phase 4 (0xD0003643=%u, "
                        "OS-running 0xD000400F=0x%02x)", phase, osrun);
        }
    }

    /*
     * Readiness gate. Deliver OS ticks only once (a) the kernel has installed
     * its interrupt-vector base (BIV != 0) and globally enabled interrupts
     * (ICR.IE), AND (b) the RTOS has progressed far enough that its tick-ISR
     * vectors are real -- the phase byte (0xD0003643) has reached osek_minphase.
     * Injecting a tick before the kernel is ready vectors into half-initialised
     * handler state and never returns (CCPN sticks at the tick priority and the
     * boot wedges). This mirrors bridge_server.py throttling boot ticks until
     * the OS phase byte >= 3. A virtual-time floor guards the garbage the phase
     * byte reads before the DSPR block-init.
     */
    bool ready = false;
    if (env->BIV != 0 && ie
        && (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->osek_t0_ns) > s->osek_floor_ns) {
        uint8_t phase = 0;
        cpu_physical_memory_read(0xD0003643, &phase, 1);
        ready = (phase >= s->osek_minphase);
    }

    /* Deliver one round-robin systick when ready. */
    if (ready) {
        uint8_t srpn = tc1797_osek_tick_prios[s->osek_rr
                                              % ARRAY_SIZE(tc1797_osek_tick_prios)];
        s->osek_rr++;
        if (srpn > ccpn) {
            env->ICR = FIELD_DP32(env->ICR, ICR, PIPN, srpn);
            cpu_interrupt(cs, CPU_INTERRUPT_HARD);
            raised = true;
        }
    }

    /* CPU service-request-node dispatch (0xF7E0FFFC): once the RTOS is up the
     * ERCOSEK scheduler requests a task switch by setting SRR; model that node
     * as a real interrupt source so the dispatcher actually runs (otherwise the
     * activated periodic tasks are starved -- bridge task #22). Only when we did
     * not already raise a systick this fire (keep one source in flight at a
     * time), under the same readiness + IE && SRPN > CCPN rule. */
    if (!raised && ready && ((s->cpu_src >> 13) & 1u)) {
        uint8_t srpn = s->cpu_src & 0xFFu;
        if (srpn > ccpn) {
            env->ICR = FIELD_DP32(env->ICR, ICR, PIPN, srpn);
            cpu_interrupt(cs, CPU_INTERRUPT_HARD);
        }
    }

    timer_mod(s->osek_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->osek_tick_ns);
}

/*
 * Coding-marker bootstrap (env TC1797_BOOTSTRAP=1, default OFF).
 *
 * With the CPU-SRC dispatch fix the ERCOSEK dispatcher runs and the firmware
 * advances into the coding self-test selftest_80148da4, which fatals (DTC 0x3026
 * -> SCU sw-reset @0x800BFA76, reset loop) because the "coding valid" marker byte
 * 0xD0018A8C reads blank (0xFF) -- the donor's variant-coding that would carry it
 * is stripped from this firmware image. The marker's source is StartCounter
 * copying counter_obj[+0xA], which never runs (autostart, circular bootstrap).
 *
 * This optional, env-gated bootstrap seeds a self-consistent valid marker once
 * the boot is past the RAM-test sweep (BIV installed + small phase ordinal), so
 * the self-test takes its pass branch instead of resetting. The boot then settles
 * into the live, stable phase-3 RTOS (scheduler dispatching) instead of the reset
 * loop. The seeded bytes are exactly the value+inverse pairs the self-test checks
 * (a70/a71, a76/a77, a90/a91), the marker a8c=0xFF00 (==-0x100), and a75!=0 to
 * select the integrity branch -- i.e. what valid factory coding would have left.
 * Default OFF preserves the faithful, undriven behaviour.
 */
static void tc1797_bootstrap_tick(void *opaque)
{
    TC1797SoCState *s = opaque;
    CPUTriCoreState *env = &s->cpu.env;
    uint8_t phase = 0, a8c = 0;
    static const struct { uint32_t addr; uint8_t val[4]; int n; } seed[] = {
        { 0xD0018A70, { 0x09, 0xf6 }, 2 }, { 0xD0018A75, { 0x01 }, 1 },
        { 0xD0018A76, { 0x00, 0xff }, 2 },
        { 0xD0018A8C, { 0x00, 0xff, 0x7f, 0x80 }, 4 },
        { 0xD0018A90, { 0x00, 0xff }, 2 },
    };

    timer_mod(s->bootstrap_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 5 * 1000000LL); /* ~5 ms */
    if (env->BIV == 0) {
        return;                              /* wait until the kernel vectors exist */
    }
    cpu_physical_memory_read(0xD0003643, &phase, 1);
    if (phase < 1 || phase >= 0x40) {
        return;                              /* not past the RAM-test sweep yet */
    }
    cpu_physical_memory_read(0xD0018A8C, &a8c, 1);
    if (a8c != 0x00) {                       /* (1) keep the coding marker valid */
        for (int i = 0; i < (int)ARRAY_SIZE(seed); i++) {
            cpu_physical_memory_write(seed[i].addr, seed[i].val, seed[i].n);
        }
    }
    /*
     * (2) Inject StartCounter's OS-state bootstrap so the RTOS promotes to full
     * operation (phase 4): set 0xD000400B bit5 (the gate fn_800c0252 bails on) +
     * the OS-running flag 0xD000400F. StartCounter -- the only writer of these --
     * is gated behind the circular autostart and never runs in this stripped
     * image (its source, the counter descriptor's +0xA byte, is also blank). We
     * drive its effect directly. Re-asserted each tick because fn_800c0252 clears
     * 0xD000400F when its own (now-unmet) task-identity gate fails.
     */
    {
        uint8_t b = 0, osr = 0;
        cpu_physical_memory_read(0xD000400B, &b, 1);
        if (!(b & 0x20)) {
            b |= 0x20;
            cpu_physical_memory_write(0xD000400B, &b, 1);
        }
        cpu_physical_memory_read(0xD000400F, &osr, 1);
        if (osr != 1) {
            osr = 1;
            cpu_physical_memory_write(0xD000400F, &osr, 1);
        }
    }
    if (!s->bootstrap_done) {
        s->bootstrap_done = true;
        info_report("tc1797: phase-4 bootstrap active (TC1797_BOOTSTRAP) -- coding "
                    "marker seeded + OS-running flag 0xD000400F driven; RTOS reports "
                    "full operation instead of the phase-3 stall / reset loop");
    }
}

/*
 * TC1797 SSC0/SSC1 (Synchronous Serial Channel = SPI), bases 0xF0100100 /
 * 0xF0100200. The firmware's "trigger transfer + poll done" routine
 * (fn_8001cb82) sets CON.bit14, writes the command word to TB (+0x20), then
 * spins on CON.bit13 (ready) and STAT.bit12 (busy). With an unmodelled block
 * CON reads 0, so the ready-poll never exits -- the native stall at 0x8001cb96.
 *
 * Minimal faithful model: report ready/idle (CON bit13=1, STAT BSY=0) and loop
 * TB back into RB (+0x24) so MISO reads return plausible non-zero data instead
 * of a "no device" zero. Per-slave protocol simulation (SC900685 MSDI switch
 * monitor on SSC0; TLE7183F + L9959 drivers on SSC1) is layered later if a
 * downstream check needs specific response bits. Mirrors bridge_server.py.
 */
static bool tc1797_ssc_addr(uint32_t addr, int *unit, uint32_t *off)
{
    if (addr >= 0xF0100100u && addr <= 0xF01001FFu) {
        *unit = 0; *off = addr - 0xF0100100u; return true;
    }
    if (addr >= 0xF0100200u && addr <= 0xF01002FFu) {
        *unit = 1; *off = addr - 0xF0100200u; return true;
    }
    return false;
}

static uint64_t tc1797_ssc_read(TC1797SoCState *s, int unit, uint32_t off)
{
    switch (off) {
    case 0x08: return 0x00004500;            /* ID: MOD=0x45 (SSC) */
    case 0x0C: return 0x10000000;            /* FDR reset-ish (baud) */
    case 0x20: return s->ssc_tb[unit];       /* TB readback */
    case 0x24: {                             /* RB: loopback echo of last TB */
        uint32_t tb = s->ssc_tb[unit];
        return (tb & 0xFF) == 0 ? 0u : (tb & 0xFFFF);
    }
    case 0x28: return 0x00000000;            /* STAT: ready, BSY=0 (bit12 clear) */
    /*
     * +0xF4 / +0xF8 are the SSC TX/RX ready flags. The firmware's byte-by-byte
     * SPI transmit (FUN_8011a800) writes 0x4000 to trigger, then spins on
     * `while (*(SSC+0xF4) == 0)` (and bit13 of +0xF8) waiting for the slot to
     * drain. SPI here is instantaneous (loopback), so the slot is always ready:
     * report bit13 set (nonzero) for both so each byte's wait resolves at once.
     */
    case 0xF4: return 0x00002000;            /* per-byte TX/RX ready (mode-2 poll) */
    /*
     * +0xF8 transfer SRN: return the stored config (SRPN/SRE) so the firmware's
     * read-modify-write SETR/CLRR keeps the programmed bits, OR'd with SRR
     * (bit13) which doubles as the "transfer ready/done" flag the polled SPI path
     * (FUN_8001cb82 @0x8001cb96) waits on -- loopback transfers complete at once,
     * so it always reads ready. (Before modelling the SRN this returned a bare
     * 0x2000; keep that ready semantics while now also carrying the config.)
     */
    case 0xF8: return s->ssc_src[unit][0] | 0x00002000;
    case 0xFC: return s->ssc_src[unit][1];   /* queue SRN (SRPN/SRE/SRR) */
    default:   return 0;
    }
}

/* SPI transfer-latency expiry: the modelled transfer finished, so raise the
 * SSC completion (queue-node) service request now -- the deferred analogue of
 * the PCP signalling transfer-done. (See tc1797_ssc_write +0xF8.) */
static void tc1797_ssc0_done(void *opaque)
{
    TC1797SoCState *s = opaque;
    tc1797_raise_srpn(s, s->ssc_src[0][1] & 0xFFu);
}
static void tc1797_ssc1_done(void *opaque)
{
    TC1797SoCState *s = opaque;
    tc1797_raise_srpn(s, s->ssc_src[1][1] & 0xFFu);
}

static void tc1797_ssc_write(TC1797SoCState *s, int unit, uint32_t off,
                             uint32_t val)
{
    if (off == 0x20) {
        s->ssc_tb[unit] = val;               /* TB: latch for loopback */
        return;
    }
    /*
     * SSC service-request nodes at +0xF8 (per-transfer) and +0xFC (queue start).
     * The queued SPI driver FUN_8011a800 enqueues a transfer, marks the
     * per-device ready byte = 2 (pending), then writes SETR to +0xFC -- a
     * software-triggered service request that on real silicon raises the SSC
     * ISR (fn_80083610). That ISR programs the transfer and writes SETR to +0xF8
     * to chain the completion stage, which (re-entering the ISR) drains the
     * queue, sets the ready byte's bit0 = done, and re-arms +0xFC if more is
     * queued. Without modelling BOTH nodes the boot wedges at fn_800dd384
     * polling the ready byte (phase 2). Model them as real CPU service requests.
     */
    if (off == 0xF8) {
        /*
         * SSC transfer SRN, configured TOS=1 -- routed to the PCP (Peripheral
         * Control Processor), NOT the CPU. The SSC ISR (fn_80083610) programs a
         * transfer then SETRs this node so the PCP performs the SPI shift and, on
         * completion, signals the CPU to run the ISR's completion stage (which
         * sets the per-device ready byte = 1). We do not model the PCP and the
         * loopback transfer is instantaneous, so simulate the PCP completing by
         * re-raising the partner CPU SSC interrupt -- the +0xFC node's SRPN --
         * which re-enters fn_80083610 on the completion path (*(saved entry) = 1).
         * Without this the boot wedges at fn_800dd384 polling that byte (phase 2).
         */
        uint32_t srr = (s->ssc_src[unit][0] >> 13) & 1u;
        if (val & 0x8000u) {
            srr = 1;                          /* SETR */
        }
        if (val & 0x4000u) {
            srr = 0;                          /* CLRR */
        }
        s->ssc_src[unit][0] = (val & 0x00001FFFu) | (srr << 13);
        if (srr) {
            /* Model the SPI transfer LATENCY. On silicon the PCP shifts the
             * frame over the transfer time (tens of us at the SSC baud) and only
             * THEN signals completion. Raising the completion SRN immediately
             * makes the SSC ISR (fn_80083610) re-fire back-to-back at full CPU
             * speed -- ~90% of all interrupts -- starving the lower-priority OSEK
             * counter ticks (SRPN 7/8) so the OS time base freezes (no alarms ->
             * no periodic tasks -> no os_running). Defer the completion raise by
             * a transfer time so the OSEK ticks run in the gaps. */
            timer_mod(s->ssc_done_timer[unit],
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ssc_xfer_ns);
        }
        return;
    }
    if (off == 0xFC) {
        tc1797_src_node_write(s, &s->ssc_src[unit][1], val,
                              0xF0100100u + (uint32_t)unit * 0x100u + 0xFCu);
        return;
    }
    /* CON trigger, baud, SSOC chip-select writes are accepted silently.
     * SPI-bus map (RE'd via SSOC chip-select trace, task #83): SSC0 cs=0x4000
     * = SC900685 MSDI switch monitor; SSC1 cs=0x0400 = TLE7183F, cs=0x0800 =
     * L9959 (throttle/VANOS/wastegate, 0xBEEF unlock). The external M95512 SPI
     * EEPROM (DME coding/identity store) is NOT accessed before the boot RAM/
     * self-test cluster -- it is read downstream, so it does not gate 0x3027. */
}

/*
 * ── Spec-driven peripheral dispatch ──────────────────────────────────────────
 * The relocatable peripheral IP for this part, as a table of instances each
 * naming an IP kind (the registry) + the base it sits at. The MMIO read/write
 * routers iterate this table; a different part supplies a different table (and,
 * across generations, different kinds). This is the TC1797 instance; bases are
 * the part's own constants (see the #defines above) so a sibling part is just a
 * different table. (Part-specific SCU/clock/DTS/flash register quirks are NOT
 * here -- they stay in the per-part switch below.)
 */
static const TriCorePeriphCfg tc1797_periphs[] = {
    { TC_IP_MULTICAN, TC1797_CAN_BASE,  TC1797_CAN_END  - TC1797_CAN_BASE, 0, 0 },
    { TC_IP_ADC,      TC1797_ADC_LO,    TC1797_ADC_HI   - TC1797_ADC_LO,   0, 0 },
    { TC_IP_FADC,     TC1797_FADC_BASE, TC1797_FADC_SIZE,                  0, 0 },
    { TC_IP_GPTA,     TC1797_GPTA_LO,   TC1797_GPTA_HI  - TC1797_GPTA_LO,  0, 0 },
    { TC_IP_DMA,      TC1797_DMA_BASE,  TC1797_DMA_END  - TC1797_DMA_BASE, 0, 0 },
    { TC_IP_ERAY,     TC1797_ERAY_BASE, TC1797_ERAY_SIZE,                  0, 0 },
    { TC_IP_ASC,      TC1797_ASC0_BASE, TC1797_ASC_SIZE,                   0, 0 },
    { TC_IP_ASC,      TC1797_ASC1_BASE, TC1797_ASC_SIZE,                   1, 0 },
    { TC_IP_MLI,      TC1797_MLI0_BASE, TC1797_MLI_SIZE,                   0, 0 },
    { TC_IP_MLI,      TC1797_MLI1_BASE, TC1797_MLI_SIZE,                   1, 0 },
};

/* Per-peripheral dispatch gates kept from the original SoC (CAN/ADC/GPTA can be
 * disabled for bring-up bisection/safety); other kinds are always on. */
static bool tc1797_periph_enabled(TC1797SoCState *s, TriCoreIpKind kind)
{
    switch (kind) {
    case TC_IP_MULTICAN: return s->en_can;
    case TC_IP_ADC:      return s->en_adc;
    case TC_IP_GPTA:     return s->en_gpta;
    default:             return true;
    }
}

/* Route an MMIO read to the modeled peripheral whose instance window contains
 * `addr`, via the IP-kind registry. Returns false (fall through to the
 * part-specific handler) if nothing matches or the matched kind is gated off. */
static bool tc1797_periph_read(TC1797SoCState *s, uint32_t addr, uint32_t *out)
{
    for (size_t i = 0; i < ARRAY_SIZE(tc1797_periphs); i++) {
        const TriCorePeriphCfg *p = &tc1797_periphs[i];
        if (addr < p->base || addr >= p->base + p->size) {
            continue;
        }
        if (!tc1797_periph_enabled(s, p->kind)) {
            return false;
        }
        switch (p->kind) {
        case TC_IP_MULTICAN: *out = tc1797_can_read(&s->can, addr); return true;
        case TC_IP_ADC:      *out = tc1797_adc_read(&s->adc, addr); return true;
        case TC_IP_FADC:     *out = tc1797_fadc_read(&s->fadc, addr); return true;
        case TC_IP_GPTA:     *out = tc1797_gpta_read(&s->gpta, addr,
                                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)); return true;
        case TC_IP_DMA:      *out = tc1797_dma_read(&s->dma, addr); return true;
        case TC_IP_ERAY:     *out = tc1797_eray_read(&s->eray, addr); return true;
        case TC_IP_ASC:      *out = tc1797_asc_read(p->unit ? &s->asc1 : &s->asc0, addr);
                             return true;
        case TC_IP_MLI:      *out = tc1797_mli_read(p->unit ? &s->mli1 : &s->mli0, addr);
                             return true;
        default:             return false;
        }
    }
    return false;
}

static bool tc1797_periph_write(TC1797SoCState *s, uint32_t addr, uint32_t val)
{
    for (size_t i = 0; i < ARRAY_SIZE(tc1797_periphs); i++) {
        const TriCorePeriphCfg *p = &tc1797_periphs[i];
        if (addr < p->base || addr >= p->base + p->size) {
            continue;
        }
        if (!tc1797_periph_enabled(s, p->kind)) {
            return false;
        }
        switch (p->kind) {
        case TC_IP_MULTICAN:
            {   /* DIAGNOSTIC (env TC1797_MOLOG): who resets MO84 NEWDAT (the diag
                 * request MO, channel 110)? A MOCTR (0xF0005A9C) write with bit3
                 * (RESET NEWDAT) is the only thing that can clear the request. */
                static int mol = -1;
                if (mol < 0) {
                    mol = getenv("TC1797_MOLOG") ? 1 : 0;
                }
                if (mol && addr == 0xF0005A9Cu && (val & (1u << 3))) {
                    static unsigned c;
                    if (c++ < 50) {
                        fprintf(stderr, "MO84 MOCTR-RESET NEWDAT pc=%08x val=%08x\n",
                                (uint32_t)s->cpu.env.PC, val);
                        fflush(stderr);
                    }
                }
            }
            tc1797_can_write(&s->can, addr, val); return true;
        case TC_IP_ADC:
            {   /* ADC kernel service-request nodes sit in the top of each 0x400
                 * kernel (TC1797 UM ch.23/24). The firmware programs SRPN/SRE/TOS
                 * then pulses SETR to fire the conversion-complete handshake:
                 * ADC0 SRC@0x3F0 = 0x941d (SRPN 0x1d, SRE, TOS=1, SETR) triggers
                 * PCP channel 0x1d which sets 0xD000416F (the GPTA/ADC init-done
                 * flag the DTC-0x3006 gate FUN_800fbafe polls). Route SRE-shaped
                 * writes in that region through the generic SRN handler so the
                 * request actually fires (TOS=1 -> PCP, TOS=0 -> CPU); keep the
                 * shadow for the firmware's read-modify-write of the SRC reg. */
                uint32_t aoff = addr - TC1797_ADC0_BASE;
                uint32_t koff = aoff & (TC1797_ADC_KSIZE - 1);
                int ki = (int)(aoff / TC1797_ADC_KSIZE);
                if (ki >= 0 && ki < TC1797_ADC_NKERN
                    && koff >= 0x3C0u && (val & 0x1000u)) {
                    tc1797_adc_write(&s->adc, addr, val);  /* shadow read-back */
                    tc1797_src_node_write(s, &s->adc_src[ki][(koff - 0x3C0u) >> 2],
                                          val, addr);
                    return true;
                }
            }
            tc1797_adc_write(&s->adc, addr, val); return true;
        case TC_IP_FADC:     tc1797_fadc_write(&s->fadc, addr, val); return true;
        case TC_IP_GPTA:     tc1797_gpta_write(&s->gpta, addr, val); return true;
        case TC_IP_DMA:      tc1797_dma_write(&s->dma, addr, val); return true;
        case TC_IP_ERAY:     tc1797_eray_write(&s->eray, addr, val); return true;
        case TC_IP_ASC:      tc1797_asc_write(p->unit ? &s->asc1 : &s->asc0, addr, val);
                             return true;
        case TC_IP_MLI:      tc1797_mli_write(p->unit ? &s->mli1 : &s->mli0, addr, val);
                             return true;
        default:             return false;
        }
    }
    return false;
}

static uint64_t tc1797_sfr_read(void *opaque, hwaddr offset, unsigned size)
{
    TC1797SoCState *s = opaque;
    uint32_t addr = 0xF0000000u + (uint32_t)offset;
    int ssc_u;
    uint32_t ssc_off;

    /* DIAGNOSTIC (env TC1797_MOLOG): trace the firmware's reads of MO84 (the diag
     * request MO, 0xF0005A80..0xF0005A9F) -- which register/offset it polls and
     * what value it actually gets, vs the rx_pending state. Definitively shows
     * whether the firmware's CPU read sees NEWDAT. */
    {
        static int mol = -1;
        if (mol < 0) { mol = getenv("TC1797_MOLOG") ? 1 : 0; }
        if (mol && addr >= 0xF0005A80u && addr < 0xF0005AA0u && s->en_can) {
            static unsigned rc;
            uint32_t v = tc1797_can_read(&s->can, addr & ~3u);
            if (rc++ < 50000) {
                fprintf(stderr, "MO84-READ off=%02x size=%u word=%08x pc=%08x\n",
                        (addr & 0x1Fu), size, v, (uint32_t)s->cpu.env.PC);
                fflush(stderr);
            }
        }
    }

    /* CAN MO data registers (MODR0/MODR1 at MO+0x10/0x14) are read byte-by-byte
     * by the firmware's diag/ISO-TP drain (FUN_800dc44c reads bytes at
     * 0xF0005010+). The MultiCAN model returns the word at the word-aligned
     * index, so serve sub-word reads by extracting the requested byte(s). */
    if (size < 4 && addr >= TC1797_CAN_MO_BASE && addr < TC1797_CAN_END
        && s->en_can) {
        uint32_t word = tc1797_can_read(&s->can, addr & ~3u);
        return (word >> ((addr & 3u) * 8)) & (size == 1 ? 0xFFu : 0xFFFFu);
    }

    /* STM occupies a small contiguous block; dispatch by range. */
    if (addr >= 0xF0000200u && addr <= 0xF00002FFu) {
        return tc1797_stm_read(s, addr);
    }
    if (tc1797_ssc_addr(addr, &ssc_u, &ssc_off)) {
        return tc1797_ssc_read(s, ssc_u, ssc_off);
    }

    /*
     * GPIO ports P0..P11 at 0xF0000C00 + 0x100*n; Pn_IN (input data) is at
     * +0x24. The firmware's port wait-loop (fn_80018214) spins while
     * (Pn_IN & mask) == 0, i.e. until an input pin goes high (first native
     * stall here: P2_IN @ 0xF0000E24). Report all inputs high -- pull-ups /
     * KL15 ignition present -- so the wait resolves. Matches bridge_server.py;
     * other port registers fall through to the default (read 0, logged).
     */
    if (addr >= 0xF0000C00u && addr < 0xF0001800u) {
        unsigned pn = (addr - 0xF0000C00u) >> 8;     /* port index 0..11 */
        unsigned po = addr & 0xFFu;
        if (po == 0x24u) {
            return 0xFFFFFFFFu;                      /* Pn_IN: inputs high (KL15/pull-ups) */
        }
        if (po == 0x00u && pn < 12) {
            return s->port_out[pn];                  /* Pn_OUT: observable output latch */
        }
        /* other port registers fall through (read 0, logged) */
    }

    /* Second port block @0xF0300000 (Port 10/11 controller): reads fall through
     * (return 0, as silicon-reset). Pn_IN is NOT forced high here -- unlike the
     * P0..P11 block (where Pn_IN=0xFFFFFFFF models KL15/strap pins the firmware
     * needs high), these inputs feed boot-config branches and forcing them high
     * diverts the boot. The output latch is still tracked on WRITE (see below)
     * for observability. */

    /* PCP control registers (0xF0043F00..0xF0043FFF): module ID; the rest read
     * 0 (firmware tolerates) -- PCP_CS config is captured on write. */
    if (addr >= 0xF0043F00u && addr <= 0xF0043FFFu) {
        return (addr == 0xF0043F08u) ? 0x0000C001u : 0;   /* PCP_ID */
    }

    /* Relocatable peripheral IP (MultiCAN/ADC/FADC/GPTA/DMA/E-Ray/ASC/MLI),
     * routed by the per-part instance table via the IP-kind registry. */
    {
        uint32_t v;
        if (tc1797_periph_read(s, addr, &v)) {
            return v;
        }
    }

    switch (addr) {
    /* ── SCU clock / PLL / reset status ── */
    case 0xF0000510: return 0x00000F00;   /* OSCCON: PLLLV/PLLHV/OSC2L/OSC2H */
    case 0xF0000514: return 0xFFFFFFFF;   /* PLLSTAT: all lock flags ready */
    case 0xF0000550: return s->scu_rststat; /* RSTSTAT (PORST sticky shadow) */
    case 0xF0000558: return 0x00000000;   /* RSTCON: no pending reset */
    case 0xF0000560: return 0x00000007;   /* PLL-lock OK */
    case 0xF0000640:
        /*
         * SCU clock-state register. func_0x80019268 (called from the clock-init
         * driver fn_8001815a) reads bits[15:8] and compares to 0x91 ("clock
         * already configured"); only if NOT 0x91 does it run FUN_80018e18, which
         * writes 0xF0000560=2 (SCU reset) and halts. With this unmodelled (0),
         * the firmware re-configures the clock on every boot -> reset loop.
         * Report the configured state (bits[15:8]=0x91) so the clock-init is
         * skipped and the boot proceeds without the reset cycle.
         */
        return 0x00009100;
    case 0xF0000648: return 0x00000001;
    case 0xF0000680: return 0x00000003;
    case 0xF00005C0: return s->hwcfg_ststat; /* STSTAT: HWCFG[7:0] boot-config (configurable) */
    case 0xF00005C4: return s->scu_stcon;    /* STCON startup config (configurable) */
    case 0xF00005E0: return 0x00004000;   /* clock-ready: bit 14 set */
    case 0xF00005F0: return s->wdt_con0;  /* WDT_CON0 (ENDINIT shadow) */
    case 0xF00005F4: return s->wdt_con1;  /* WDT_CON1 shadow */
    /* ── MSC0/MSC1 status: command/data ready (bit 0) ── */
    case 0xF0000844: case 0xF0000944: return 0x00000001;
    /*
     * ── PMU / PFlash0/1 FSR (Flash Status Register, +0x10) ──
     * Bits: 0 PROG, 1 ERASE, 2 PFOPER, 3 DFOPER, 4 FABUSY, 7 PFBUSY,
     * 8-15/31 error, plus operation-complete result bits 5/6. The bridge
     * returns 0 ("idle") because it boots mid-firmware and never runs the
     * flash-operation phase; QEMU runs the full BROM->firmware path and hits
     * the OSEK phase handler FUN_80122744, which waits for an op to COMPLETE:
     * state 0x0f needs bit5 set, state 0x10 needs bit4 set, with all busy bits
     * (0,2,3) and error bits (8-15,31) clear. Since we don't physically program
     * flash, the operation is instantly complete: report bit5 (op done), no
     * busy, no error. We deliberately leave bit4 (FABUSY) clear -- 0x20 and
     * 0x30 behave identically for this boot, and a permanently-asserted FABUSY
     * would hang any future "wait until not busy" poll. (A stateful command/busy
     * model can be layered later if a state-0x10 op needs bit4.)
     */
    case 0xF8002010: case 0xF8004010: return 0x00000020; /* FSR: op complete (bit5), no busy/error */
    case 0xF8002014: case 0xF8004014: return 0x00070606; /* FCON reset value */
    /* ── ADC0/1/2 kernel status (+0x38) ──
     * The OSEK cold-init ADC bring-up (init routine 0x800D36CC) enables the
     * three ADC kernels (writes 0x8712 to +0x30) then polls +0x38 for
     * bits[9:8]==3 ("kernel running"); a second pass polls bit2 ("conversion
     * pending"). With an unmodelled 0 the poll at 0x800D36F6 spins forever and
     * the OSEK init dispatch (0xD0000980) eventually fatals -> reset. Report
     * state=3 (running) with bit2 clear (no pending conv) so both passes
     * complete. (Mirrors bridge_server.py; ADC samples reach the firmware via
     * signal DSPR addresses, not a modelled result register.) */
    case 0xF0101038: case 0xF0101438: case 0xF0101838: return 0x00000300;
    case 0xF8002028: case 0xF8004028: return 0x00000000; /* PROCON2 */
    /* ── FCE (CRC engine): BMI-validation bypass. The BROM reads the CRC
     *    result at PC 0xAFFFCFE6 then compares to the expected CRC in D4;
     *    return D4 so validation passes for a validly-flashed image (we don't
     *    have Bosch's exact CRC32 variant). ── */
    case 0xF010C214:
        return (s->cpu.env.PC == 0xAFFFCFE6) ? s->cpu.env.gpr_d[4] : s->fce_crc;
    case 0xF010C210: return 0x00000000;   /* FCE input register */
    /* ── Silicon identification chain (lets chip-ID checks pass) ── */
    case 0xF0000408: return 0x000DC001;   /* JDPID (Cerberus/JTAG) */
    case 0xF0000464: return 0x101701C7;   /* JTAGID (TC1797 IDCODE) */
    case 0xF0000480: return s->ocds_ostate; /* OSCU OSTATE: bit0=OCDS counter running */
    case 0xF0000508: return 0x000DC001;   /* SCU_ID */
    /* ── Die Temperature Sensor (DTS, in the SCU; UM 3.7.1) ──
     * DTSSTAT (0xF00000E0): RESULT[9:0] (significant [9:2]) = die temperature,
     * RDY=bit14, BUSY=bit15. We report a fixed, configurable die temperature
     * (TC1797_DTS_TEMP, default 30 C; also processors/tc1797.pspec.yaml
     * die_temp_c) with RDY=1/BUSY=0, so a read returns a valid temp immediately.
     * DTSCON (0xF00000E4): PWD/START/CAL; START (bit1) auto-clears (reads 0). */
    case 0xF00000E0: return s->dts_stat;
    case 0xF00000E4: return s->dts_con & ~0x2u;
    case 0xF8000508: return 0x0050C001;   /* PMU0_ID */
    case 0xF8000608: return 0x0051C001;   /* PMU1_ID (TC1797 marker) */
    case 0xF8002008: return 0x00C0C001;   /* FLASH0_ID */
    case 0xF8004008: return 0x00C1C001;   /* FLASH1_ID */
    case 0xF010C208: return 0x002CC001;   /* MCHK_ID */
    /* ── CPU service-request node (OSEK task-dispatch trigger) ──
     * The ERCOSEK dispatcher polls SRR (bit13) on this node to find pending
     * task-switch work (FUN_801255da / FUN_80124512). Reflect the shadow so the
     * poll sees the request the scheduler set; the systick timer injects it. */
    case 0xF7E0FFFC: return s->cpu_src;
    default:
        /* EBU / PMU / overlay-control region (0xF8000000+): coherent config
         * read-back. The boot-critical PMU IDs and flash status registers are
         * explicit cases above (they take priority); every other EBU bus-config,
         * overlay-control (OVC) and PMU register reads back what was written
         * instead of a dumb 0, so configuration sequences are self-consistent. */
        if (addr >= 0xF8000000u && addr < 0xF8010000u) {
            return s->ebu_shadow[(addr - 0xF8000000u) >> 2];
        }
        /* Main peripheral cluster (0xF0000000..0xF0200000): coherent config
         * read-back for every register not handled above. Unwritten registers
         * still read 0 (the shadow is zero-init), so this only makes registers
         * the firmware actually wrote self-consistent on read-back -- the
         * faithful behavior vs. a blanket 0 -- without changing any modeled or
         * never-touched register. */
        if (addr >= 0xF0000000u && addr < 0xF0200000u) {
            return s->sfr_shadow[(addr - 0xF0000000u) >> 2];
        }
        /* PMI register block (0xF87Fxxxx): instruction-memory-interface control.
         * Read 0 (NOT read-back): these include self-clearing command/status bits
         * (e.g. cache invalidate) the firmware polls during early boot -- reading
         * back the written value hangs that poll. QEMU fetches directly, so 0 is
         * correct. Quiet (the firmware writes these on every init). */
        if (addr >= 0xF87F0000u && addr < 0xF8800000u) {
            return 0;
        }
        /* Top-of-SFR (0xFFFF0000+): the firmware issues a per-tick write-0 burst
         * to 0xFFFFC000 with no read-back dependency -- return 0 quietly. */
        if (addr >= 0xFFFF0000u) {
            return 0;
        }
        qemu_log_mask(LOG_UNIMP,
                      "tc1797.sfr: read  0x%08x (size %u) -> 0\n", addr, size);
        return 0;
    }
}

static void tc1797_sfr_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    TC1797SoCState *s = opaque;
    uint32_t addr = 0xF0000000u + (uint32_t)offset;
    int ssc_u;
    uint32_t ssc_off;

    /* E-Ray/CP TX doorbell (bit7): snapshot the outbound CP frame to the host
     * tunnel and mark the (unmodelled) controller done; clear bit7 so the
     * firmware's post-dispatch poll proceeds instead of spinning. */
    if (addr == TC1797_ERAY_DOORBELL && (val & 0x80u)) {
        tc1797_eray_doorbell(s);
        val &= ~0x80u;
    }

    if (addr >= 0xF0000200u && addr <= 0xF00002FFu) {
        tc1797_stm_write(s, addr, (uint32_t)val);
        return;
    }
    if (tc1797_ssc_addr(addr, &ssc_u, &ssc_off)) {
        tc1797_ssc_write(s, ssc_u, ssc_off, (uint32_t)val);
        return;
    }
    /* PCP control registers: capture PCP_CS (0xF0043F10) so firmware config
     * drives the engine (RCB bit4 -> entry-table vectoring; CS[7:6] context
     * model). Other PCP regs accept writes (no read-back, stays inert). */
    if (addr >= 0xF0043F00u && addr <= 0xF0043FFFu) {
        if (addr == 0xF0043F10u) {
            s->pcp.rcb = ((uint32_t)val >> 4) & 1u;
            s->pcp.context_model = ((uint32_t)val >> 6) & 3u;
        }
        return;
    }

    /* Relocatable peripheral IP, routed by the per-part instance table via the
     * IP-kind registry (same table as the read path). */
    if (tc1797_periph_write(s, addr, (uint32_t)val)) {
        return;
    }
    /* GPIO ports P0..P11: latch Pn_OUT (+0x00) and apply Pn_OMR (+0x04, PS[15:0]
     * sets / PCL[31:16] clears) so the firmware's driven output pins are
     * observable. Pin-config / IN reads are handled in the read path. */
    if (addr >= 0xF0000C00u && addr < 0xF0001800u) {
        unsigned pn = (addr - 0xF0000C00u) >> 8;
        unsigned po = addr & 0xFFu;
        if (pn < 12 && po == 0x00u) {
            s->port_out[pn] = (uint32_t)val;
        } else if (pn < 12 && po == 0x04u) {
            uint32_t v = (uint32_t)val;
            s->port_out[pn] = (s->port_out[pn] | (v & 0xFFFFu)) & ~((v >> 16) & 0xFFFFu);
        }
        return;
    }
    /* Second port block @0xF0300000 (Port 10/11): latch Pn_OUT (+0x00) / Pn_OMR
     * (+0x04) at port_out[12 + n], mirroring the P0..P11 block above. */
    if (addr >= 0xF0300000u && addr < 0xF0300800u) {
        unsigned pn = (addr - 0xF0300000u) >> 8;
        unsigned po = addr & 0xFFu;
        if (po == 0x00u) {
            s->port_out[12 + pn] = (uint32_t)val;
        } else if (po == 0x04u) {
            uint32_t v = (uint32_t)val;
            s->port_out[12 + pn] =
                (s->port_out[12 + pn] | (v & 0xFFFFu)) & ~((v >> 16) & 0xFFFFu);
        }
        return;
    }
    /* MSC0/MSC1 (Micro Second Channel) actuator-command capture: the firmware
     * writes downstream command/data words here (e.g. +0x40 CMD). Log them for
     * observability; status reads (+0x44 ready) keep their fixed model below. */
    if ((addr & 0xFFFFFF00u) == 0xF0000800u || (addr & 0xFFFFFF00u) == 0xF0000900u) {
        static unsigned msc_once;
        if (msc_once < 32 && (uint32_t)val) {
            msc_once++;
            info_report("tc1797: MSC%u write +0x%02x = 0x%08x (actuator cmd)",
                        (addr >> 8) & 1, addr & 0xFF, (uint32_t)val);
        }
        /* fall through to the switch for any specific MSC register handling */
    }

    switch (addr) {
    case 0xF0000550:                      /* SCU_RSTSTAT: write-1-to-clear */
        s->scu_rststat &= ~(uint32_t)val;
        break;
    case 0xF00000E4:                      /* DTS control: PWD/START/CAL */
        /* Store config (read-back). START (bit1) self-clears and the result is
         * always ready, so the firmware's "start + poll RDY + read RESULT"
         * returns the fixed configured die temperature immediately. */
        s->dts_con = (uint32_t)val;
        break;
    case 0xF0000560:
        /*
         * SCU software-reset request. The firmware's fatal-error handler
         * FUN_800bf97c writes 0xF0000560=2 to reset the SoC after logging the
         * error (code/value/return-address) to its RAM crash-log at *(0xD0018AC8)
         * -- struct [+0x14]=retaddr [+0x18]=value [+0x1f]=code. We do NOT reset
         * (that just reboots into the same deterministic init failure -> loop);
         * instead we surface the firmware's own crash report, which pinpoints the
         * failing call site for the boot-bringup grind. The boot then continues
         * (into a post-fatal spin) so the pre-fatal state stays analysable.
         * First fatal observed: code=0x11 val=0x3023 from 0x80149134 (a coding/
         * consistency check `d0 != d15` at 0x801490F6 -> DFLASH/coding gate).
         */
        if ((uint32_t)val == 2) {
            if (getenv("TC1797_DTCLOG")) {
                CPUTriCoreState *e = &s->cpu.env;
                info_report("tc1797: SCU-reset@0x%08x dregs d4=%08x d5=%08x "
                            "d6=%08x d15=%08x a11=%08x", (uint32_t)e->PC,
                            e->gpr_d[4], e->gpr_d[5], e->gpr_d[6], e->gpr_d[15],
                            e->gpr_a[11]);
            }
            /*
             * Observe (do not act on) the firmware's self-reset request. The
             * fatal handler FUN_800bf97c writes 0xF0000560=2 to reset the SoC
             * after logging to its DSPR crash-log at *(0xD0018AC8) -- struct
             * [+0x14]=retaddr [+0x18]=value [+0x1f]=code. We surface that report
             * (pinpoints the failing site for the boot-bringup grind) but do NOT
             * reset: a warm reset just reboots into the same deterministic init
             * failure and the firmware's reset-then-skip marker (0xD0018A76=0x96)
             * is not, by itself, enough to make the reboot skip the self-test
             * (see task #83). The boot continues into a post-fatal spin so the
             * pre-fatal state stays analysable.
             */
            uint32_t p = 0, ra = 0, v = 0, code = 0;
            static int once;
            cpu_physical_memory_read(0xD0018AC8, &p, 4);
            /* Only a real fatal has a valid DSPR crash-log pointer; the early
             * clock-config also writes 0xF0000560=2 but with the log ptr still
             * at its power-on value -- skip that so we report the real fatal. */
            if (p >= 0xD0000000u && p < 0xD0020000u && !once++) {
                cpu_physical_memory_read((hwaddr)p + 0x14, &ra, 4);
                cpu_physical_memory_read((hwaddr)p + 0x18, &v, 4);
                cpu_physical_memory_read((hwaddr)p + 0x1f, &code, 1);
                warn_report("tc1797 firmware FATAL: code=0x%02x val=0x%04x "
                            "from 0x%08x", code & 0xff, v & 0xffff, ra);
            }
            /* Honour the firmware's SCU software-reset (clock-switch + self-test
             * reset-then-skip). Warm reset preserves RAM/DFLASH; the reboot
             * should skip the already-done switch. (Option 2: full-BROM.)
             * Capped during bring-up: if the reboot doesn't skip (reset loop),
             * stop after N so we don't peg the CPU -- and the count tells us. */
            {
                static int rstn;
                static int cap = -1;
                if (cap < 0) {
                    /* Real silicon has no reset cap; the firmware's reset-then-skip
                     * init cascade (clock-config, RAM self-test 0x3026, and the
                     * gate-3 0x3006 PCP-completion cascade) reboots as many times as
                     * it needs, warm-resetting with SRAM/DFLASH/PCP-PRAM preserved
                     * and RSTSTAT=SWRST so each reboot skips already-done steps.
                     * We keep only a high safety bound so a genuine (non-skipping)
                     * reset loop can't peg the CPU. Override via TC1797_RESET_CAP. */
                    const char *e = getenv("TC1797_RESET_CAP");
                    cap = e ? atoi(e) : 64;
                }
                if (rstn < cap) {
                    rstn++;
                    info_report("tc1797: SCU sw-reset #%d/%d PC=0x%08x",
                                rstn, cap, (uint32_t)s->cpu.env.PC);
                    s->sw_reset_pending = true;
                    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
                }
            }
        }
        break;
    case 0xF0000478:                      /* OSCU OCNTRL (OCDS run-time counter ctrl) */
        /* The ERCOSEK software-counter start (fn_800bfaac) writes 0x5e here to
         * enable the OCDS run-time counter, then polls OSTATE.bit0 for "running".
         * On silicon the OSCU asserts that status once enabled; model it so the
         * handler doesn't take its disable-and-return path. */
        s->sfr_shadow[(0xF0000478u - 0xF0000000u) >> 2] = (uint32_t)val;
        if ((uint32_t)val != 0) {
            s->ocds_ostate |= 1u;         /* counter now running */
        } else {
            s->ocds_ostate &= ~1u;
        }
        break;
    case 0xF00005F0:                      /* WDT_CON0: ENDINIT password servicing */
        tc1797_wdt_service(s, (uint32_t)val);
        break;
    case 0xF00005F4:                      /* WDT_CON1 */
        s->wdt_con1 = (uint32_t)val;
        break;
    case 0xF010C214:                      /* FCE CRC0 value: write = seed */
        s->fce_crc = (uint32_t)val;
        break;
    case 0xF010C210: {                    /* FCE CRC0 input: CRC32-Ethernet step */
        uint32_t v = (uint32_t)val, crc = s->fce_crc;
        for (int bp = 0; bp < 4; bp++) {
            crc ^= (v >> (bp * 8)) & 0xFF;
            for (int i = 0; i < 8; i++) {
                crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
            }
        }
        s->fce_crc = crc;
        break;
    }
    case 0xF7E0FFFC: {
        /*
         * CPU service-request node = OSEK task-dispatch trigger. The ERCOSEK
         * scheduler sets SRR by writing SRPN|0x9000 (SETR bit15 + SRE bit12),
         * drains it with CLRR (bit14), and polls SRR (bit13) for pending work.
         *
         * On real TC1.3.1 silicon, setting SRR on an enabled (SRE=1), CPU-routed
         * (TOS=0) node immediately requests service at its SRPN. So route this
         * node through the SAME SRN model as every other interrupt source:
         * writing SETR raises the dispatch interrupt (deferred via the bottom
         * half, since we are in TB store context), which vectors the ERCOSEK
         * dispatcher FUN_801255da.
         *
         * Faithful root-cause fix (verified by gdbstub breakpoint: FUN_801255da
         * was never entered, so the autostart task set -- StartCounter fn_800bfaac
         * + the phase-4 promotion task fn_800c0252 -- never ran, leaving osrun
         * (0xD000400F)=0 and the RTOS wedged at phase 3). The previous store-only
         * model needed the forced systick to inject the dispatch; with a real SRN
         * raise the firmware drives its own dispatcher exactly as on hardware.
         */
        tc1797_src_node_write(s, &s->cpu_src, (uint32_t)val, addr);
        break;
    }
    default:
        /* EBU / PMU / overlay-control region (0xF8000000+): store for coherent
         * config read-back (see the matching read default). Boot-critical PMU/
         * flash semantics are handled by explicit cases; this just keeps EBU
         * bus-config and overlay-control (OVC) registers self-consistent. */
        if (addr >= 0xF8000000u && addr < 0xF8010000u) {
            s->ebu_shadow[(addr - 0xF8000000u) >> 2] = (uint32_t)val;
            break;
        }
        /* Main peripheral cluster: record for coherent config read-back (see the
         * matching read default). Modeled registers handled above never reach
         * here, so this only backs the otherwise-unmodeled config registers. */
        if (addr >= 0xF0000000u && addr < 0xF0200000u) {
            s->sfr_shadow[(addr - 0xF0000000u) >> 2] = (uint32_t)val;
            break;
        }
        /* PMI register block (0xF87Fxxxx): instruction-memory-interface control
         * (cache/scratchpad config). No functional effect under QEMU's direct
         * fetch; ignore quietly (reads return 0, see the matching read default). */
        if (addr >= 0xF87F0000u && addr < 0xF8800000u) {
            break;
        }
        /* Top-of-SFR (0xFFFF0000+): benign per-tick write-0 burst -- ignore quietly. */
        if (addr >= 0xFFFF0000u) {
            break;
        }
        qemu_log_mask(LOG_UNIMP,
                      "tc1797.sfr: write 0x%08x = 0x%08x (size %u)\n",
                      addr, (uint32_t)val, size);
        break;
    }
}

static const MemoryRegionOps tc1797_sfr_ops = {
    .read = tc1797_sfr_read,
    .write = tc1797_sfr_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * Build one DFLASH "coding" bank (0x8000 bytes = 256 x 128-byte blocks).
 * Port of bridge/coding_seed.py (the RE'd S55 variant-coding format): each
 * block has halfword[0]=hw0 (FCE CRC seed), [2:4]=low16(CRC over [4:0x80]),
 * [4:8]=word4(=3), payload[8:]=slot index. FCE CRC = reflected CRC-32, poly
 * 0xEDB88320, seeded with hw0. The firmware's coding scanner (~0x80121xxx) and
 * the boot RAM/coding self-test validate these; blank 0xFF DFLASH is rejected.
 */
static void tc1797_seed_dflash_bank(uint8_t *bank, uint16_t hw0)
{
    for (unsigned slot = 0; slot < 0x8000u / 0x80u; slot++) {
        uint8_t *blk = bank + slot * 0x80u;
        memset(blk, 0, 0x80);
        blk[0] = hw0 & 0xFF;
        blk[1] = (hw0 >> 8) & 0xFF;
        blk[4] = 3;                               /* word4 = 3 (little-endian) */
        blk[8] = slot & 0xFF;                     /* payload: slot index */
        blk[9] = (slot >> 8) & 0xFF;
        uint32_t crc = hw0;
        for (unsigned i = 4; i < 0x80u; i++) {
            crc ^= blk[i];
            for (int b = 0; b < 8; b++) {
                crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
            }
        }
        blk[2] = crc & 0xFF;
        blk[3] = (crc >> 8) & 0xFF;
    }
}

static void make_ram(MemoryRegion *mr, const char *name, hwaddr base, hwaddr size)
{
    memory_region_init_ram(mr, NULL, name, size, &error_fatal);
    memory_region_add_subregion(get_system_memory(), base, mr);
}

static void make_alias(MemoryRegion *mr, const char *name,
                       MemoryRegion *orig, hwaddr base)
{
    memory_region_init_alias(mr, NULL, name, orig, 0, memory_region_size(orig));
    memory_region_add_subregion(get_system_memory(), base, mr);
}

static void tc1797_soc_realize(DeviceState *dev, Error **errp)
{
    TC1797SoCState *s = TC1797_SOC(dev);
    Error *err = NULL;

    qdev_realize(DEVICE(&s->cpu), NULL, &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }

    /* Opt-in validation of the TC1.3.1 range-based memory-protection logic. */
    if (getenv("TC1797_MPUTEST")) {
        tricore_mpu_selftest();
    }

    make_ram(&s->pflash_c, "tc1797.pflash", TC1797_PFLASH_BASE, TC1797_PFLASH_SIZE);
    make_alias(&s->pflash_u, "tc1797.pflash.u", &s->pflash_c, TC1797_PFLASH_UBASE);
    make_ram(&s->brom_c, "tc1797.brom", TC1797_BROM_BASE, TC1797_BROM_SIZE);
    make_alias(&s->brom_u, "tc1797.brom.u", &s->brom_c, TC1797_BROM_UBASE);
    make_ram(&s->dspr, "tc1797.dspr", TC1797_DSPR_BASE, TC1797_DSPR_SIZE);
    make_ram(&s->pspr, "tc1797.pspr", TC1797_PSPR_BASE, TC1797_PSPR_SIZE);

    /*
     * SRAM power-on pattern. On real silicon DSPR/PSPR come up indeterminate,
     * and this firmware keys several early branches on 0xFF-vs-0 -- notably the
     * persistent self-test/coding region at 0xD0018xxx, which sits JUST PAST the
     * crt0 .bss-zero range (0xD0000020..0xD0018A70) and therefore retains the
     * power-on value. QEMU zero-fills RAM by default (0x00), flipping those
     * branches vs. silicon. Prefill 0xFF to match the hardware / bridge_server.py
     * (which prefills DSPR with 0xFF); the firmware's own crt0 then zeroes its
     * .bss subset natively, leaving the non-.bss bytes at 0xFF as on silicon.
     */
    memset(memory_region_get_ram_ptr(&s->dspr), 0xFF, TC1797_DSPR_SIZE);
    memset(memory_region_get_ram_ptr(&s->pspr), 0xFF, TC1797_PSPR_SIZE);

    /*
     * DFLASH (Data Flash) coding banks at 0xAFE00000 / 0xAFE10000 (non-cached)
     * with cached aliases at 0x8FE00000 / 0x8FE10000. The firmware reads its
     * variant coding from here; blank/0xFF DFLASH is rejected by the coding
     * scanner and the boot self-tests. Back them with RAM and seed synthetic
     * format-valid coding (port of bridge/coding_seed.py, the RE'd S55 coding).
     */
    make_ram(&s->dflash0, "tc1797.dflash0", 0xAFE00000UL, 0x8000);
    make_alias(&s->dflash0_a, "tc1797.dflash0.cached", &s->dflash0, 0x8FE00000UL);
    make_ram(&s->dflash1, "tc1797.dflash1", 0xAFE10000UL, 0x8000);
    make_alias(&s->dflash1_a, "tc1797.dflash1.cached", &s->dflash1, 0x8FE10000UL);
    tc1797_seed_dflash_bank(memory_region_get_ram_ptr(&s->dflash0), 0x0001);
    tc1797_seed_dflash_bank(memory_region_get_ram_ptr(&s->dflash1), 0x0002);

    /*
     * SRAM at 0xAFE80000..0xAFE81FFF (8 KB). The boot RAM-integrity self-test
     * FUN_80148b4a (called from FUN_80115bb4) walks an (start,end) range list at
     * PFLASH 0x80180BD8 and runs the destructive march-C test FUN_80149352 over
     * each entry; that list is {0xD0018AD0..0xD001EFFF, 0xAFE80000..0xAFE81FFF,
     * 0xD0000018..0xD000001F}. The DSPR entries are already RAM-backed, but
     * 0xAFE80000 was unmapped, so the march read-back mismatched and the firmware
     * raised FATAL(0x11,0x3024) at 0x80148c98 -> SCU reset loop. On silicon this
     * is real SRAM; back it with RAM so the march test passes. (The 2nd test is
     * gated by FUN_800bf722: it runs once when *(u16)(*(0xD0018AB0)+2)==0xA1B2,
     * then FUN_800bf73c rewrites that marker to 0xC396 so reboots skip it.)
     * A destructive test leaves patterns behind by design, so no seeding needed.
     */
    make_ram(&s->ramtest, "tc1797.ramtest", 0xAFE80000UL, 0x2000);
    /* OVRAM, like every PMU memory, is visible both non-cached (segment A,
     * 0xAFE80000) and cached (segment 8, 0x8FE80000); alias the cached view to
     * the same backing so accesses via either segment are coherent (UM Tbl 8-2). */
    make_alias(&s->ramtest_a, "tc1797.ovram.cached", &s->ramtest, 0x8FE80000UL);

    /*
     * Segment-7 RAM-cell self-test pattern source (0x7FFFxxxx). The coding-init
     * RAM-cell test fn_80073e54 (=func_0xa0073e54, called from FUN_8014888e) reads
     * its test-pattern bytes from 0x7FFFB24C (= 0x80000000 - 0x4DB4) then writes
     * them into the target DSPR cell and reads back to verify the cell holds them.
     * That whole segment is unmapped on this firmware dump; QEMU faults the source
     * read -> data-abort trap derails the test -> FUN_8014888e takes its failure
     * branch (DTC 0x301b), so the "coding-valid" marker DAT_d0018a8c is never set
     * to 0xFF00 and the dispatched self-test selftest_80148da4 fatals (DTC 0x3026 ->
     * SCU reset loop). The bridge tolerates this via its UC_HOOK_MEM_UNMAPPED
     * lazy-zero-map; mirror that by backing the segment with RAM. The pattern VALUE
     * is immaterial (the test writes it to the cell then reads the same cell back),
     * so plain zero-RAM makes every RAM-cell test pass -- exactly as on silicon
     * where this address resolves to readable memory. Window covers 0x7FFF0000.. so
     * the sibling testers (func_0xa0073e82/eb4) that read nearby sources also pass.
     */
    make_ram(&s->seg7, "tc1797.seg7", 0x7FFF0000UL, 0x10000);

    /* Peripheral/SFR space: native model (SCU clock/PLL/reset done; the rest
     * read 0 + logged as the P2 work list). Replaces the unimplemented stub. */
    s->scu_rststat = 0x00010000;   /* PORST set at cold reset */
    s->sfr_shadow = g_malloc0(0x200000); /* 2 MB SFR config read-back region */
    s->ebu_shadow = g_malloc0(0x10000);  /* 64 KB EBU/PMU/overlay-config read-back */
    /* Boot-mode config pins (HWCFG[7:0], latched from P0[7:0] at PORST -> STSTAT).
     * Default 0x80 = the internal-flash/BMI path this firmware boots from. Override
     * with TC1797_HWCFG=<byte> to model the other strap modes (UM Table 7-1:
     * 0x40 CAN-BSL, 0xA8 ASC-BSL, 0xC0 internal-flash, 0x90/0x20 external/ABM). */
    {
        const char *hw = getenv("TC1797_HWCFG");
        const char *sc = getenv("TC1797_STCON");
        s->hwcfg_ststat = hw ? (uint32_t)strtoul(hw, NULL, 0) : 0x00000080u;
        s->scu_stcon = sc ? (uint32_t)strtoul(sc, NULL, 0) : 0x00000000u;
    }
    /* ── Clock tree ──────────────────────────────────────────────────────────
     * The SoC timing substrate: every peripheral timer derives its period from
     * one of these rates. Defaults are the SAK-TC1797-512F180E clocks per the
     * TC1797 User's Manual (v1.1) "180 MHz derivative":
     *   fCPU max 180 MHz, fPCP max 180 MHz, fSYS (= FPI/SPB bus) max 90 MHz.
     *   2:1 mode (required for CPU/PCP > 90 MHz): fFPI = fCPU/2 -> fSYS = 90 MHz.
     *   STM: "driven by max 90 MHz (= fSYS); default after reset = fSYS/2" (45).
     * We model the post-clock-init running rates (STM = fSYS = 90 MHz), verified
     * to preserve the phase-4 boot. Override any rate in MHz via
     * TC1797_F{SYS,CPU,PCP,STM}_MHZ. (Pacing execution to f_cpu/f_pcp --
     * determinism via icount -- layers on top of this tree later.) */
    s->f_sys = 90000000ULL;        /* fSYS = fFPI/SPB = fCPU/2 (UM: sys clk max 90 MHz) */
    s->f_cpu = 180000000ULL;       /* fCPU max 180 MHz on -512F180E (2:1 mode)  */
    s->f_pcp = 180000000ULL;       /* fPCP max 180 MHz (2:1 mode), PCP own clock */
    s->f_stm = 90000000ULL;        /* fSTM = fSYS = 90 MHz (UM; reset default 45) */
    {
        const struct { const char *env; uint64_t *f; } clk[] = {
            { "TC1797_FSYS_MHZ", &s->f_sys }, { "TC1797_FCPU_MHZ", &s->f_cpu },
            { "TC1797_FPCP_MHZ", &s->f_pcp }, { "TC1797_FSTM_MHZ", &s->f_stm },
        };
        for (unsigned i = 0; i < ARRAY_SIZE(clk); i++) {
            const char *e = getenv(clk[i].env);
            if (e && *e) {
                uint64_t mhz = strtoull(e, NULL, 0);
                if (mhz >= 1 && mhz <= 1000) {
                    *clk[i].f = mhz * 1000000ULL;
                }
            }
        }
    }
    s->stm_ns_per_tick = (uint32_t)(1000000000ULL / s->f_stm);
    if (s->stm_ns_per_tick == 0) {
        s->stm_ns_per_tick = 1;
    }
    info_report("tc1797: clock tree -- fSYS=%" PRIu64 " fCPU=%" PRIu64
                " fPCP=%" PRIu64 " fSTM=%" PRIu64 " MHz (STM %u ns/tick)",
                s->f_sys / 1000000, s->f_cpu / 1000000, s->f_pcp / 1000000,
                s->f_stm / 1000000, s->stm_ns_per_tick);

    s->stm_cmp0 = 0xFFFFFFFF;      /* STM compare regs default all-ones (bridge) */
    s->stm_cmp1 = 0xFFFFFFFF;
    s->stm_cmcon = 0;
    s->stm_icr = 0;
    s->stm_isrr = 0;
    s->stm_src0 = 0;
    s->stm_src1 = 0;
    s->stm_period_ns = 1000000;    /* 1 ms fallback until firmware arms CMP0 */
    s->stm_cmp_deadline[0] = INT64_MAX;   /* edge-on-match: no compare armed yet */
    s->stm_cmp_deadline[1] = INT64_MAX;
    s->stm_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_stm_tick, s);

    /* SSC (SPI) per-transfer latency: model the PCP shift time so the SSC
     * completion interrupt fires at a bounded rate (instead of storming back-to-
     * back and starving the OSEK time-base ticks). ~25 us is realistic for a
     * multi-byte frame at the SSC baud; env TC1797_SSC_XFER_NS overrides. */
    s->ssc_done_timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_ssc0_done, s);
    s->ssc_done_timer[1] = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_ssc1_done, s);
    {
        const char *e = getenv("TC1797_SSC_XFER_NS");
        s->ssc_xfer_ns = e ? (int64_t)strtoll(e, NULL, 0) : 25000;
        if (s->ssc_xfer_ns < 1000) {
            s->ssc_xfer_ns = 1000;       /* floor 1 us */
        }
    }

    /* Forced OSEK systick: ~1 ms per priority, divided across the round-robin
     * set so one fire == one OSEK-counter advance (see tc1797_osek_tick). Armed
     * here and re-armed on every reset; per-fire IE/priority gating means it is
     * harmlessly masked through the early boot until the RTOS enables it. */
    s->irq_pending_srpn = 0;
    s->irq_bh = qemu_bh_new(tc1797_irq_bh, s);
    /* Register the faithful ICU service-entry ack so the CPU re-arbitrates PIPN
     * to the next-highest pending SRN on every interrupt taken (vs the legacy
     * single-slot clear). */
    tricore_icu_ack = tc1797_icu_service_ack;
    tricore_icu_ack_ctx = s;
    s->cpu_src = 0;
    s->osek_rr = 0;
    s->osek_phase4 = false;
    s->osek_tick_ns = 1000000 / ARRAY_SIZE(tc1797_osek_tick_prios);
    s->osek_t0_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    /* Bring-up knobs (env): the forced OSEK systick is OPT-IN (TC1797_SYSTICK=1)
     * -- with the SSC PCP-completion model the boot now reaches a STABLE phase 3
     * on its own, and firing the tick at the very start of phase 3 runs off
     * (the OSEK counter/alarm structures the tick ISR walks aren't initialised
     * yet). Keeping it off by default preserves the stable phase-3 boot; opt in
     * to continue the phase 3 -> 4 promotion work. TC1797_SYSTICK_MINPHASE sets
     * the RTOS phase gate (default 3); TC1797_SYSTICK_FLOOR_MS the virtual-time
     * floor (default 50 ms). */
    s->osek_enabled = (getenv("TC1797_SYSTICK") != NULL);
    {
        const char *mp = getenv("TC1797_SYSTICK_MINPHASE");
        const char *fl = getenv("TC1797_SYSTICK_FLOOR_MS");
        s->osek_minphase = mp ? (uint8_t)strtoul(mp, NULL, 0) : 3;
        s->osek_floor_ns = (fl ? (int64_t)strtoll(fl, NULL, 0) : 50) * 1000000LL;
    }
    s->osek_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_osek_tick, s);
    if (s->osek_enabled) {
        timer_mod(s->osek_timer, s->osek_t0_ns + s->osek_tick_ns);
        info_report("tc1797: forced OSEK systick armed (minphase=%u, floor=%lldms)",
                    s->osek_minphase, (long long)(s->osek_floor_ns / 1000000));
    } else {
        info_report("tc1797: forced OSEK systick off (set TC1797_SYSTICK=1 to "
                    "drive phase 3->4); boot is stable at phase 3");
    }

    /* Coding-marker bootstrap: opt-in (TC1797_BOOTSTRAP=1). Seeds the blank
     * coding marker so the dispatched coding self-test passes instead of looping
     * on the 0x3026 SCU reset -> the firmware settles into the live phase-3 RTOS.
     * Default OFF leaves the faithful (undriven) boot, which reset-loops at the
     * self-test once the dispatcher is up. See tc1797_bootstrap_tick. */
    s->bootstrap_enabled = (getenv("TC1797_BOOTSTRAP") != NULL);
    s->bootstrap_done = false;
    s->bootstrap_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_bootstrap_tick, s);
    if (s->bootstrap_enabled) {
        timer_mod(s->bootstrap_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20 * 1000000LL);
        info_report("tc1797: coding-marker bootstrap armed (TC1797_BOOTSTRAP=1) "
                    "-- boot will stabilise at the live phase-3 RTOS");
    }

    memory_region_init_io(&s->sfr, OBJECT(s), &tc1797_sfr_ops, s,
                          "tc1797.sfr", TC1797_SFR_SIZE);
    memory_region_add_subregion(get_system_memory(), TC1797_SFR_BASE, &s->sfr);

    /*
     * PCP Data Memory (PRAM) at 0xF0050000, 16 KB (UM Table 10-19 / segment-15
     * map). Holds the PCP2 channel contexts (CSAs) + channel data. The firmware
     * also runs write/read-back self-tests over the low part (0x80149xxx); it
     * must be plain RAM, but it sits inside the SFR IO window where reads return
     * 0, so back it with real RAM overlapping the SFR region at higher priority.
     * (Earlier labelled "eray"; it is the PCP PRAM. Sized to the full 16 KB so
     * the native PCP can read every channel context.)
     */
    memory_region_init_ram(&s->eray_ram, OBJECT(s), "tc1797.pcp.pram",
                           PCP_PRAM_SIZE, &error_fatal);
    memory_region_add_subregion_overlap(get_system_memory(), PCP_PRAM_BASE,
                                        &s->eray_ram, 1);

    /*
     * PCP Code Memory (CMEM) at 0xF0060000, 32 KB. Holds the PCP2 channel
     * programs (the firmware's loader at 0x80115EFC copies them here) + the
     * RCB entry table. The firmware's RAM-init/self-test (FUN_801494bc) also
     * zeroes/verifies the low part as plain RAM. Back the full 32 KB with RAM so
     * the PCP fetches the complete channel code.
     */
    memory_region_init_ram(&s->eray2_ram, OBJECT(s), "tc1797.pcp.cmem",
                           PCP_CMEM_SIZE, &error_fatal);
    memory_region_add_subregion_overlap(get_system_memory(), PCP_CMEM_BASE,
                                        &s->eray2_ram, 1);

    /*
     * PCP2 co-processor. Opt-in (env TC1797_PCP) so the proven CPU-only boot is
     * untouched by default. When enabled, an SRN write with TOS=1 routes the
     * service request to the PCP (see tc1797_src_node_write) instead of the CPU;
     * the engine runs that channel program on its own thread and, on EXIT.INT,
     * raises the matching CPU service request via tc1797_pcp_irq.
     */
    /* MultiCAN controller (always modeled: it only acts on firmware MO writes;
     * unconfigured it is inert, so the default boot is unaffected). */
    s->en_can  = (getenv("TC1797_NO_CAN")  == NULL);
    s->en_adc  = (getenv("TC1797_NO_ADC")  == NULL);
    s->en_gpta = (getenv("TC1797_NO_GPTA") == NULL);
    tc1797_can_init(&s->can, TC1797_CAN_BASE, tc1797_can_tx, s);
    s->can.rx_irq_cb = tc1797_can_rx_irq;      /* RX frames pulse the MO's SRN */
    s->can.rx_irq_opaque = s;
    /* Attach to a QEMU CAN bus if one exists (-object can-bus,id=...): the
     * firmware's CAN frames then leave the model and external frames are
     * delivered into the matching receive MOs. */
    {
        Object *cb = object_resolve_path_type("", TYPE_CAN_BUS, NULL);
        if (cb) {
            s->canbus = CAN_BUS(cb);
            s->can_client.info = &tc1797_can_bus_info;
            if (can_bus_insert_client(s->canbus, &s->can_client) < 0) {
                warn_report("tc1797: failed to attach MultiCAN to the CAN bus");
                s->canbus = NULL;
            } else {
                info_report("tc1797: MultiCAN attached to QEMU CAN bus");
            }
        }
    }
    /* Host CAN tunnel: -chardev socket,id=can0,host=...,port=...,server=on,wait=off.
     * The front-end connects to that socket; DME TX frames stream out as
     * "<id>#<hex>\n" and lines written back are injected as RX. No bus needed. */
    {
        Chardev *cc = qemu_chr_find("can0");
        if (cc) {
            qemu_chr_fe_init(&s->can_tun, cc, &error_abort);
            qemu_chr_fe_set_handlers(&s->can_tun, tc1797_can_tun_can_receive,
                                     tc1797_can_tun_receive, NULL, NULL,
                                     s, NULL, true);
            info_report("tc1797: CAN tunnel attached (chardev id=can0)");
        }
    }
    /* Host E-Ray/CP tunnel: -chardev socket,id=fray0,...  Carries the CAS/
     * immobilizer challenge-response. DME CP TX is announced as "ERAYTX ..";
     * a line "<cmd>:<type>:<hex>" injects a CP frame into the DSPR RX struct. */
    {
        Chardev *fc = qemu_chr_find("fray0");
        if (fc) {
            qemu_chr_fe_init(&s->eray_tun, fc, &error_abort);
            qemu_chr_fe_set_handlers(&s->eray_tun, tc1797_eray_tun_can_receive,
                                     tc1797_eray_tun_receive, NULL, NULL,
                                     s, NULL, true);
            info_report("tc1797: E-Ray/CP tunnel attached (chardev id=fray0)");
        }
    }
    /* Host memory peek/poke tunnel: -chardev socket,id=mem0,...  Zero-perturbation
     * live guest memory R/W from the front-end (phase/os_running reads, register
     * pokes). Replaces the gdb stub for orchestration; the vCPU never stops. */
    {
        Chardev *mc = qemu_chr_find("mem0");
        if (mc) {
            qemu_chr_fe_init(&s->mem_tun, mc, &error_abort);
            qemu_chr_fe_set_handlers(&s->mem_tun, tc1797_mem_tun_can_receive,
                                     tc1797_mem_tun_receive, NULL, NULL,
                                     s, NULL, true);
            info_report("tc1797: memory tunnel attached (chardev id=mem0)");
        }
    }
    if (getenv("TC1797_CANTEST")) {
        tc1797_can_selftest(tc1797_can_tx, s);
    }

    /* ADC0/1/2 result-register model. Defaults to silicon reset (RESRn read 0);
     * set TC1797_ADC_DEFAULT=<0..4095> to give every channel a plausible
     * "sensor present" idle count so the firmware's scale path sees valid
     * inputs once it runs conversions (the faithful alt to DSPR injection). */
    tc1797_adc_init(&s->adc, TC1797_ADC0_BASE);
    {
        const char *d = getenv("TC1797_ADC_DEFAULT");
        if (d) {
            tc1797_adc_set_default(&s->adc, (int)strtol(d, NULL, 0));
        }
    }
    if (getenv("TC1797_ADCTEST")) {
        tc1797_adc_selftest();
    }

    /* Fast ADC (FADC, 0xF0100400): result registers default to silicon reset
     * (read 0, no boot perturbation); TC1797_FADC_DEFAULT=<0..4095> gives every
     * channel a plausible idle count (e.g. quiescent knock level). The firmware
     * uses the FADC for knock sensing -- a set count flows through its own
     * read+scale path, the faithful alternative to DSPR-level injection. */
    tc1797_fadc_init(&s->fadc, TC1797_FADC_BASE);
    {
        const char *d = getenv("TC1797_FADC_DEFAULT");
        if (d) {
            tc1797_fadc_set_default(&s->fadc, (int)strtol(d, NULL, 0));
        }
    }
    if (getenv("TC1797_FADCTEST")) {
        tc1797_fadc_selftest();
    }

    /* MLI0/MLI1 (Micro Link Interface, 0xF010C000 / 0xF010C100): inter-chip
     * serial link. Models the firmware-visible CLC/ID + coherent config
     * read-back (the TX/RX datapath needs a second MLI peer, absent in a
     * single-device emulation). Module IDs follow the 0x00mmC001 SFR-ID shape. */
    tc1797_mli_init(&s->mli0, TC1797_MLI0_BASE, 0x0024C001u);
    tc1797_mli_init(&s->mli1, TC1797_MLI1_BASE, 0x0025C001u);
    if (getenv("TC1797_MLITEST")) {
        tc1797_mli_selftest();
    }

    /* JTAG TAP + Cerberus/OCDS debug port. IDCODE matches the JTAGID SFR
     * (0xF0000464) and the JDPID Cerberus product ID (0xF0000408). An external
     * debugger (OpenOCD remote_bitbang) can drive it if a chardev named "jtag"
     * is supplied, e.g.:
     *   -chardev socket,id=jtag,host=127.0.0.1,port=3335,server=on,wait=off
     * then in OpenOCD: 'adapter driver remote_bitbang;
     *   remote_bitbang host 127.0.0.1; remote_bitbang port 3335'. */
    tc1797_jtag_init(&s->jtag, TC1797_JTAG_IDCODE);
    {
        Chardev *jc = qemu_chr_find("jtag");
        if (jc) {
            tc1797_jtag_attach_chardev(&s->jtag, jc);
        }
    }
    if (getenv("TC1797_JTAGTEST")) {
        tc1797_jtag_selftest();
    }

    /* Die Temperature Sensor (DTS, in the SCU): a fixed, configurable die
     * temperature. TC1797_DTS_TEMP=<degC> overrides (default 30 C; also
     * processors/tc1797.pspec.yaml die_temp_c). Encoded into DTSSTAT.RESULT[9:2]
     * via a linear approximation over the -40..150 C sensor range -- the exact
     * RESULT->C constants are calibration/data-sheet specific (the UM defers the
     * formula), so the raw code is documented and the temperature is settable. */
    {
        int tc = 30;
        const char *dt = getenv("TC1797_DTS_TEMP");
        if (dt) {
            tc = (int)strtol(dt, NULL, 0);
        }
        tc = tc < -40 ? -40 : (tc > 150 ? 150 : tc);
        uint32_t code8 = (uint32_t)(((tc + 40) * 255) / 190);  /* -40..150 -> 0..255 */
        s->dts_stat = ((code8 << 2) & 0x3FFu) | (1u << 14);    /* RESULT[9:2] + RDY */
        s->dts_con  = 0x1u;                                    /* reset: PWD=1 (off) */
        info_report("tc1797: DTS die-temp = %d C (DTSSTAT=0x%08x; "
                    "configurable via TC1797_DTS_TEMP)", tc, s->dts_stat);
    }

    /* GPTA0/GPTA1/LTCA2 timer arrays. tc1797_pcp_irq is a generic SRN-raise
     * (opaque + srpn -> tc1797_raise_srpn), reused here for capture-cell IRQs. */
    tc1797_gpta_init(&s->gpta, TC1797_GPTA_LO, tc1797_pcp_irq, s);
    s->gpta.t0_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->gpta.freerun = (getenv("TC1797_GPTA_FREERUN") != NULL);
    if (getenv("TC1797_GPTATEST")) {
        tc1797_gpta_selftest(tc1797_pcp_irq, s);
    }

    /* E-Ray (FlexRay CC) POC state machine (driven by the firmware at phase 4). */
    tc1797_eray_init(&s->eray, TC1797_ERAY_BASE);
    if (getenv("TC1797_ERAYTEST")) {
        tc1797_eray_selftest();
    }

    /* DMA controller (block-move engine on its own thread; transfers run async
     * to the CPU and raise a completion service request via tc1797_pcp_irq, the
     * generic SRN-raise). Inert until the firmware enables+triggers a channel. */
    tc1797_dma_init(&s->dma, TC1797_DMA_BASE, tc1797_pcp_irq, s);
    if (getenv("TC1797_DMATEST")) {
        tc1797_dma_selftest();
    }
    tc1797_dma_start(&s->dma);

    /* ASC0/ASC1 UARTs, backed by -serial 0/1 (NULL if not supplied). */
    tc1797_asc_init(&s->asc0, TC1797_ASC0_BASE, serial_hd(0), tc1797_pcp_irq, s);
    tc1797_asc_init(&s->asc1, TC1797_ASC1_BASE, serial_hd(1), tc1797_pcp_irq, s);

    /* External reset / NMI request pins as drivable named GPIO inputs. */
    qdev_init_gpio_in_named(DEVICE(s), tc1797_esr_irq, "esr", 3);

    /* EBU external memory: optional RAM in the external-bus window
     * (0xA1000000 non-cached + 0x81000000 cached), sized by TC1797_EBU_RAM_MB.
     * Lets firmware that runs from / uses external RAM over the EBU work. */
    {
        const char *mb = getenv("TC1797_EBU_RAM_MB");
        if (mb) {
            uint64_t sz = (uint64_t)strtoul(mb, NULL, 0) * MiB;
            if (sz) {
                make_ram(&s->ebu_ram, "tc1797.ebu.ram", 0xA1000000UL, sz);
                make_alias(&s->ebu_ram_a, "tc1797.ebu.ram.cached", &s->ebu_ram,
                           0x81000000UL);
                info_report("tc1797: EBU external RAM %s MB @0xA1000000 (+0x81000000 cached)", mb);
            }
        }
    }

    /* Watchdog timer. Register/ENDINIT servicing is always modelled; the
     * timeout->reset enforcement is opt-in (TC1797_WDT) so the default boot
     * (which relies on the NOP-patched fail-safe self-loops) is unaffected. */
    s->wdt_modeled = (getenv("TC1797_WDT") != NULL);
    s->wdt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_wdt_tick, s);
    if (s->wdt_modeled) {
        timer_mod(s->wdt_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TC1797_WDT_WINDOW_NS);
        info_report("tc1797: WDT timeout enforcement enabled (TC1797_WDT, 50ms window)");
    }

    s->pcp_enabled = (getenv("TC1797_PCP") != NULL);
    pcp_engine_init(&s->pcp, PCP_PRAM_BASE, /*context_model=*/0, /*rcb=*/true,
                    tc1797_pcp_irq, s);
    s->pcp.f_pcp_hz = s->f_pcp;   /* PCP runs on its own clock from the SoC tree */
    if (getenv("TC1797_PCPTEST")) {
        pcp_selftest();
    }
    if (getenv("TC1797_PCPTHREADTEST")) {
        pcp_thread_selftest();
    }
    if (s->pcp_enabled) {
        pcp_engine_start(&s->pcp);
        info_report("tc1797: PCP2 co-processor enabled (own thread; "
                    "TOS=1 SRNs -> PCP channels)");
    }
}

static void tc1797_soc_init(Object *obj)
{
    TC1797SoCState *s = TC1797_SOC(obj);
    object_initialize_child(obj, "cpu", &s->cpu, TRICORE_CPU_TYPE_NAME("tc1797"));
}

static void tc1797_soc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = tc1797_soc_realize;
    /* SoC is created by the machine, not user-pluggable. */
    dc->user_creatable = false;
}

static const TypeInfo tc1797_soc_info = {
    .name          = TYPE_TC1797_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TC1797SoCState),
    .instance_init = tc1797_soc_init,
    .class_init    = tc1797_soc_class_init,
};

/* ---- machine ---- */
#define TYPE_TC1797_MACHINE MACHINE_TYPE_NAME("tc1797_evb")
OBJECT_DECLARE_SIMPLE_TYPE(TC1797MachineState, TC1797_MACHINE)

struct TC1797MachineState {
    /*< private >*/
    MachineState parent_obj;
    /*< public >*/
    TC1797SoCState soc;
    hwaddr boot_entry;
};

/*
 * Reset handler: QEMU's startup system-reset re-resets the CPU (PC->0) AFTER
 * machine_init, so set the boot PC here, where it runs as part of every system
 * reset. We do the default cpu_reset then override PC to the boot entry (BROM
 * when -bios is supplied, else the firmware base).
 */
static void tc1797_cpu_reset(void *opaque)
{
    TC1797MachineState *ms = TC1797_MACHINE(opaque);
    CPUState *cs = CPU(&ms->soc.cpu);

    cpu_reset(cs);
    cpu_set_pc(cs, ms->boot_entry);

    /*
     * Re-initialise SoC peripheral register state on every (warm or cold) reset.
     * realize() runs only once, so without this a firmware-triggered warm reset
     * (0xF0000560=2) would leave stale peripheral state -- notably a stale FCE
     * CRC value, which makes the BROM's reboot BMI/CRC validation misbehave and
     * stall. DSPR/PSPR and DFLASH are intentionally NOT touched here (they must
     * persist across the warm reset for the firmware's reset-then-skip).
     */
    {
        TC1797SoCState *s = &ms->soc;
        s->fce_crc = 0;
        s->stm_cmp0 = 0xFFFFFFFF;
        s->stm_cmp1 = 0xFFFFFFFF;
        s->stm_cmcon = 0;
        s->stm_icr = 0;
        s->stm_isrr = 0;
        s->stm_src0 = 0;
        s->stm_src1 = 0;
        s->stm_cmp_deadline[0] = INT64_MAX;   /* edge-on-match: clear armed compares */
        s->stm_cmp_deadline[1] = INT64_MAX;
        s->ssc_tb[0] = 0;
        s->ssc_tb[1] = 0;
        s->ssc_src[0][0] = s->ssc_src[0][1] = 0;
        s->ssc_src[1][0] = s->ssc_src[1][1] = 0;
        if (s->stm_timer) {
            /* Re-arm the edge-on-match poll floor (200us) so the CPU-SRC + CAN-RX
             * polls keep running after the warm reset; no compare is armed yet. */
            timer_mod(s->stm_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 200000);
        }
        /* Forced OSEK systick: re-init round-robin/phase state and re-arm. DSPR
         * persists across a warm reset, so reset the phase-4 detection origin
         * (the firmware re-runs cold-init from phase 0 on the reboot). */
        s->cpu_src = 0;
        s->ocds_ostate = 0;          /* OCDS run-time counter off until re-enabled */
        s->osek_rr = 0;
        s->osek_phase4 = false;
        if (s->osek_timer && s->osek_enabled) {
            s->osek_t0_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            timer_mod(s->osek_timer, s->osek_t0_ns + s->osek_tick_ns);
        }
        /* Re-arm the coding-marker bootstrap: the reboot re-runs cold-init and
         * re-blanks the marker, so re-seed it on the warm path too. */
        if (s->bootstrap_timer && s->bootstrap_enabled) {
            timer_mod(s->bootstrap_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20 * 1000000LL);
        }
        /* RSTSTAT reset-cause: cold/power-on = PORST (bit16); a firmware SW
         * reset = the SW-reset cause (bit3) with PORST clear, so the reboot
         * takes the warm path and skips the already-done clock-config. */
        if (s->sw_reset_pending) {
            s->scu_rststat = 0x00000008;   /* SWRST */
            s->sw_reset_pending = false;
        } else {
            s->scu_rststat = 0x00010000;   /* PORST */
        }
    }
}

static void tc1797_machine_init(MachineState *machine)
{
    TC1797MachineState *ms = TC1797_MACHINE(machine);

    object_initialize_child(OBJECT(machine), "soc", &ms->soc, TYPE_TC1797_SOC);
    sysbus_realize(SYS_BUS_DEVICE(&ms->soc), &error_fatal);

    /*
     * Raw firmware image into PFLASH (-kernel s55_full_rom2.bin).
     *
     * We deliberately do NOT use load_image_targphys() here: that stages the
     * image as a ROM blob written into PFLASH by rom_reset at machine reset,
     * which (a) happens after machine_init and (b) re-applies the pristine
     * bytes on every reset -- both of which fight the in-place patch below.
     * Instead we read the file, patch it, and memcpy it straight into the
     * PFLASH backing RAM (allocated by sysbus_realize above). Guest RAM is not
     * cleared on reset and no ROM blob targets PFLASH, so the patched image
     * persists for the life of the machine.
     *
     * Patch: the Bosch WDT-spin idiom `3C 00 00 90` -> `00 00 00 90`. `3C 00`
     * is TriCore 16-bit `J` with disp8=0 (a jump-to-self); `00 90` is 16-bit
     * `RET`. The firmware plants these self-loops as deliberate fail-safe halts
     * (e.g. 0x80018eac) expecting the on-chip watchdog to reset the SoC. We do
     * not model the WDT-reset, so native TCG would translate the self-jump into
     * a TB that branches to itself and peg the CPU forever. NOP the jump so the
     * firmware falls through to the RET, matching bridge_server.py's proven
     * patch. Exactly 8 such sites exist in this image; the FCE CRC over the
     * patched region is satisfied by the passthrough oracle in tc1797_sfr_read
     * (PC==0xAFFFCFE6).
     */
    if (machine->kernel_filename) {
        gchar *data = NULL;
        gsize len = 0;
        GError *gerr = NULL;

        if (!g_file_get_contents(machine->kernel_filename, &data, &len, &gerr)) {
            error_report("tc1797: cannot read firmware '%s': %s",
                         machine->kernel_filename,
                         gerr ? gerr->message : "unknown error");
            exit(1);
        }
        if (len > TC1797_PFLASH_SIZE) {
            len = TC1797_PFLASH_SIZE;
        }

        uint8_t *fw = memory_region_get_ram_ptr(&ms->soc.pflash_c);
        memcpy(fw, data, len);
        g_free(data);

        int wdt_patches = 0;
        for (size_t off = 0; off + 4 <= len; off += 2) {
            if (fw[off] == 0x3C && fw[off + 1] == 0x00 &&
                fw[off + 2] == 0x00 && fw[off + 3] == 0x90) {
                fw[off] = 0x00;          /* J disp8=0  ->  NOP (16-bit 0x0000) */
                fw[off + 1] = 0x00;
                wdt_patches++;
            }
        }
        /*
         * Coded-ECU coding-validity signatures (clears FATAL 0x3027).
         *
         * The boot self-test FUN_801497c8 (a9=0xA0153C38 ptr table) reads two
         * PFLASH coding-validity objects and requires their signature bytes to
         * carry the *programmed* pattern (0x55 / 0xAA); FUN_801495c6 raises
         * FATAL 0x3027 if they are blank. A coded ECU (one that has been through
         * the factory/dealer coding step -- i.e. any ECU fitted to a running
         * car) has these bytes programmed; this ROM image is from an un-coded
         * dump, so they read blank. We program the coded-ECU pattern, which is
         * the faithful state for emulating a working vehicle (NOT a bypass: the
         * self-test runs in full and passes exactly as on coded silicon).
         *   obj1 @0xA01B984C (file 0x1B984C): [+0]=[+1] = CODING_SIG_A (0x55)
         *   obj2 @0x801BB678 (file 0x1BB678): [+1]=[+2] = CODING_SIG_B (0xAA)
         */
        enum { CODING_SIG_A = 0x55, CODING_SIG_B = 0xAA };
        if (len > 0x1BB680) {
            fw[0x1B984C] = CODING_SIG_A; fw[0x1B984D] = CODING_SIG_A;
            fw[0x1BB679] = CODING_SIG_B; fw[0x1BB67A] = CODING_SIG_B;
            info_report("tc1797: coded-ECU coding-validity signatures programmed "
                        "(obj1 0xA01B984C, obj2 0x801BB678) -- emulating a coded ECU");
        }
        info_report("tc1797: loaded %zu bytes firmware, patched %d WDT-spin sites",
                    (size_t)len, wdt_patches);
    }
    /* Real Boot ROM dump into BROM (-bios bootrom.bin); reset to BROM entry. */
    if (machine->firmware) {
        if (load_image_targphys(machine->firmware, TC1797_BROM_BASE,
                                TC1797_BROM_SIZE, &error_fatal) < 0) {
            error_report("tc1797: failed to load BROM '%s'", machine->firmware);
            exit(1);
        }
        ms->boot_entry = TC1797_BROM_BASE;
    } else {
        /* No BROM supplied: start directly at the firmware base. */
        ms->boot_entry = TC1797_PFLASH_BASE;
    }

    /* Set the boot PC on every system reset (survives QEMU's startup reset,
     * which otherwise re-resets the CPU PC to 0 after machine_init). */
    qemu_register_reset(tc1797_cpu_reset, ms);
}

static void tc1797_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc             = "Infineon TC1797 (BMW MEVD17.2.G Bosch DME)";
    mc->init             = tc1797_machine_init;
    mc->max_cpus         = 1;
    mc->default_cpu_type = TRICORE_CPU_TYPE_NAME("tc1797");
    mc->default_ram_size = 0;
}

static const TypeInfo tc1797_machine_info = {
    .name          = TYPE_TC1797_MACHINE,
    .parent        = TYPE_MACHINE,
    .instance_size = sizeof(TC1797MachineState),
    .class_init    = tc1797_machine_class_init,
};

static void tc1797_register_types(void)
{
    type_register_static(&tc1797_soc_info);
    type_register_static(&tc1797_machine_info);
}

type_init(tc1797_register_types)
