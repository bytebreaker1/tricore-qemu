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
#include "exec/tb-flush.h"
#include "qom/object.h"
#include "target/tricore/cpu.h"
#include "pcp.h"
#include "tc1797_can.h"
#include "tc1797_adc.h"
#include "tc1797_gpta.h"
#include "tc1797_eray.h"
#include "tc1797_dma.h"
#include "tc1797_asc.h"
#include "tc1797_reset_seed.h"  /* faithful power-on SFR reset values (DAVE TC1797_v1_0.dip) */
#include "tc1797_write_mask.h"  /* per-register write-protect masks (RO/hardware bits) */
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
    MemoryRegion modvec;             /* env TC1797_MODVEC: overlay on the diag/module vector @0x800627d0 to log who reads it */
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
    QEMUTimer *freeze_timer;     /* DIAGNOSTIC: wall-clock dump of the phase-3 freeze */
    int64_t stm_period_ns;       /* derived OSEK tick period */
    unsigned stm_rr;             /* round-robin across the enabled STM compares */
    int64_t stm_cmp_deadline[2]; /* edge-on-match: abs virtual-ns when CMP0/CMP1 next fire */
    uint64_t stm_match_period[2];/* last good (non-overdue) CMP match delta = the fw's intended
                                  * tick cadence; used to catch up an overdue compare instead
                                  * of waiting a full ~47s 32-bit wrap (see tc1797_stm_match_ticks) */
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
    uint32_t ssc_prev[2];        /* previous TB (SC900685 MSDI replies 1 frame behind) */
    uint16_t ssc_rbuf[2];        /* RBUF latch: the received frame, held until the next frame
                                  * COMPLETES (ssc_done_timer). Read returns this, so the
                                  * firmware never stores the stale pre-completion frame. */
    uint16_t ssc_pend[2];        /* the reply this in-flight frame will latch at completion */
    bool ssc_rx_ready[2];        /* RBUF mode: frame received (set at done, cleared at TB write) --
                                  * gates +0xF8 so the firmware's poll waits for the REAL frame */
    uint8_t  msdi_chan;          /* SC900685 selected channel: 0=A (bank1, d4-select), 1=B (bank2, f0-select) */
    uint16_t msdi_synca;         /* channel-A e8 read counter: models the device's power-up sync handshake */
    uint8_t  msdi_cfg28;         /* SC900685 switch-config register: the host programs it per channel via the
                                  * 0x28|cfg write (cfg = the channel's switch-config = DAT_d000997a[ch]); the
                                  * device echoes it on the 0x2c config read-back (the bring-up's uVar5 check
                                  * FUN_80111cce: (uVar5 & 0xff) must == DAT_d000997a[ch]). Standard write-config/
                                  * read-back-config SPI behaviour; the per-channel fa/7a/7a values flow from the
                                  * firmware's own 0x28 writes (channels are brought up sequentially). */
    uint32_t ssc_src[2][2];      /* SSC0/1 service-request nodes [unit][0]=+0xF8 (xfer), [1]=+0xFC (queue) */
    uint32_t msc_src[2][2];      /* MSC0/1 SRC0/SRC1 @0xF00008FC/F8 (SRPN0x28): the MSDI ring-completion IRQ */
    uint32_t msc_isr[2];         /* MSC0/1 Interrupt Status Reg @+0x44: DEDI[0]/DECI[1]/DTFI[2]/URDI[3] */
    uint32_t msc_ud[2][4];       /* MSC0/1 Upstream Data Regs @+0x30..3C: DATA[7:0], V[16], P[17], LABF[20:19] */
    uint8_t  msc_rx_idx[2];      /* MSC0/1 SC900685 upstream frame index (LABF slot 0..3, advances per reply) */
    uint16_t msdi_d53ctr;        /* SC900685 free-running scan counter -> reg-0xEA d53 token (firmware polls to sync) */
    uint32_t adc_src[TC1797_ADC_NKERN][16]; /* ADC0/1/2 kernel SRNs (top of each 0x400 kernel, off>=0x3C0): firmware pulses SETR to trigger PCP/CPU completion (e.g. ADC0 SRC@0x3F0 -> PCP ch 0x1d -> sets 0xD000416F, DTC 0x3006 gate) */
    QEMUTimer *ssc_done_timer[2];/* SPI transfer-latency: defer completion SRN raise */
    int64_t ssc_xfer_ns;         /* modelled per-transfer time (env TC1797_SSC_XFER_NS) */
    /* ADC conversion-complete: the firmware ARMS each kernel's result-event SRC
     * (e.g. ADC0 SRC@0x3F0) with SRE=1/TOS=1/SETR=0 and waits for the analog
     * conversion to HW-assert that node's SRR (-> PCP channel = SRPN). On real
     * silicon every armed result node is an INDEPENDENT conversion-complete
     * event: arming a second node does not cancel the first's pending EOC. Model
     * that faithfully with a PER-NODE pending deadline (one entry per
     * [kernel][column] result node) plus a single timer set to the earliest due
     * deadline; on expiry every node whose deadline has arrived is completed and
     * the timer re-arms for the next-earliest. A single global slot (the old
     * model) let a later arm clobber an earlier node's completion -- e.g. the
     * firmware arms ADC0 SRC@0x3F0 (PCP ch 0x1d, the DTC-0x3006 gate driver) and
     * then SRC@0x3FC (ch 0x14) one store later, so ch 0x1d's conversion-complete
     * was destroyed before it could fire and ch 0x1d never serviced in its
     * cold-init (RCB=1) window. */
    QEMUTimer *adc_done_timer;   /* fires at the earliest pending node deadline */
    int64_t adc_done_deadline[TC1797_ADC_NKERN][16]; /* abs ns; 0 = node idle */
    int64_t adc_conv_ns;         /* modelled ADC conversion time (env TC1797_ADC_CONV_NS) */
    QEMUTimer *gpta_cmp_timer;   /* polls/fires GPTA GTC compare-match SRC nodes */
    QEMUTimer *sample_timer;     /* diag: 20us env->PC sampler (TC1797_SAMPLE), no inject */
    bool sw_reset_pending;       /* set when 0xF0000560=2 requested a SW reset */
    /* Forced OSEK systick. The ERCOSEK kernel promotes itself from phase 3 to
     * phase 4 only after it starts receiving periodic system ticks, but it never
     * arms the STM CMP0 compare itself pre-promotion (chicken-and-egg), so on
     * bare QEMU the RTOS sits in phase 3 forever. We drive the tick via the
     * native interrupt path -- the QEMU analogue of bridge_server.py's injected
     * SRPN 1/7/8 round-robin systick. See tc1797_osek_tick. */
    QEMUTimer *osek_timer;       /* forced OSEK systick timer (always-on) */
    QEMUTimer *hb_timer;         /* diagnostic heartbeat (env TC1797_HBLOG) -- observe frozen PC/CCPN */
    int64_t osek_tick_ns;        /* inter-fire spacing (~1 ms / #tick-prios) */
    int64_t osek_t0_ns;          /* virtual-time origin (phase-4 detect floor) */
    int64_t osek_floor_ns;       /* min virtual time before delivering any tick */
    unsigned osek_rr;            /* round-robin index across the tick priorities */
    uint8_t osek_minphase;       /* min RTOS phase (0xD0003643) before delivery */
    bool osek_enabled;           /* forced systick armed -- DEFAULT-OFF, opt-in via env TC1797_SYSTICK=1
                                  * (redundant since the CMP0 cold-init tick pin; see realize) */
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
    /* Engine model (env TC1797_ENGINE): faithful 60-2 crank + cam trigger-wheel generator. Fires
     * GPTA LTCA2 captures (crank cell 2 / cam cell 3 -> SRPN 19 -> PCP ch19 engine-position math)
     * at the tooth rate for a commanded RPM, plus a closed-loop VANOS cam-phaser plant that reads
     * the DME's own cam-solenoid command and slews the cam phase, so the cam-sensor feedback
     * reflects the DME output (emulating the physical VANOS loop). */
    QEMUTimer *engine_timer;
    bool     eng_enabled;
    double   eng_rpm;            /* commanded crankshaft speed (RPM) */
    unsigned eng_tooth;          /* 0..59 position on the 60-2 trigger wheel */
    uint64_t eng_rev;            /* crank revolution counter (cam runs at half = 2 crank revs) */
    int64_t  eng_t0_ns;          /* fallback GTTIM epoch if firmware hasn't seeded GTTIM0 */
    double   cam_phase_deg[4];   /* VANOS actual phase: [0]=B1in [1]=B1exh [2]=B2in [3]=B2exh */
    /* Per-cylinder crank-signal variance (misfire / rough-running / jitter simulation). The 720deg
     * 4-stroke cycle has 6 combustion segments (120deg each); segment k belongs to firing-order[k].
     * A modifier scales that cylinder's segment tooth-period: >1 = slow (no/weak combustion torque =
     * misfire), random jitter = rough running, per-cycle miss probability = intermittent misfire. */
    uint8_t  eng_firing[6];      /* firing order (cylinder #); default S55 I6 = 1-5-3-6-2-4 */
    double   cyl_scale[6];       /* per-cyl steady segment-period scale (1=normal, >1=misfire) */
    double   cyl_jitter[6];      /* per-cyl +/- random tooth-period jitter fraction (0..) */
    double   cyl_miss_prob[6];   /* per-cyl per-720-cycle random-misfire probability (0..1) */
    uint8_t  cyl_miss_now[6];    /* per-cycle latch: this cyl is misfiring this cycle */
    uint64_t eng_rng;            /* xorshift64 PRNG for jitter/misfire (non-deterministic input) */
    /* Closed-loop VANOS plant. The DME commands the cam-phaser oil-control solenoid via the L9959
     * SPI driver on SSC1 (16-bit frames: high byte = register, low byte = data/duty); the physical
     * phaser slews the cam, and the cam-position sensor (NW, GPTA cap cell 3) feeds the angle back.
     * We capture the L9959 config, model the phaser as a 1st-order lag (duty -> target advance), and
     * fire the cam capture at (crank reference + cam phase) so the sensor reflects the DME command. */
    uint8_t  l9959_reg[256];     /* SSC1 captured config: reg(high byte) -> data(low byte) */
    unsigned vanos_reg;          /* L9959 register holding the intake-VANOS duty (TC1797_VANOS_REG) */
    double   vanos_max_deg;      /* full duty -> this much cam advance, crank deg (TC1797_VANOS_MAX) */
    double   vanos_tau_s;        /* phaser hydraulic time constant, s (TC1797_VANOS_TAU) */
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
/* STM-FREEZE (env TC1797_STM_FREEZE): give the OSEK schedule ISR (FUN_80081870) a
 * STABLE STM TIM0 for the duration of its do-while, as on silicon where the
 * free-running counter advances only a few us-worth of ticks during the short ISR.
 * Under -icount the ISR's own retired instructions inflate QEMU_CLOCK_VIRTUAL (=TIM0),
 * so the firmware re-arms CMP0 behind the live counter and the schedule deadline runs
 * 4-7x ahead of TIM0 (the phase-4 cadence root, see EMU_TIMING_DIAGNOSIS.md). We
 * snapshot TIM0 at the ISR's first TIM read and hold it until the firmware re-arms CMP0
 * forward (ISR exit) -> the do-while converges against a stable counter -> the faithful
 * crutch-free OSEK tick (no 100us CMP0 pin, no overdue self-fire). */
static int               g_stm_freeze_en = -1;   /* env cache: TC1797_STM_FREEZE */
static bool              g_stm_freeze;            /* currently holding the snapshot */
static uint64_t          g_stm_freeze_snap;       /* the held 56-bit counter value */
static uint8_t           g_stm_phase;             /* cached boot phase (0xD0003643) */
static uint8_t           g_stm_sched_prio;        /* CCPN of the schedule ISR (CMP0 SRPN) */
static CPUTriCoreState  *g_stm_cpu_env;           /* for live CCPN reads in count56 */

static uint64_t tc1797_stm_count56(uint32_t ns_per_tick)
{
    uint64_t live = ((uint64_t)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / ns_per_tick)
                    & 0x00FFFFFFFFFFFFFFULL;
    /* STM-FREEZE (env TC1797_STM_FREEZE): on silicon the free-running STM advances only a
     * few us-worth of ticks during the short schedule ISR; under -icount the ISR's own
     * retired instructions inflate VIRTUAL time (=TIM0), so the firmware re-arms CMP0
     * behind the live counter and the deadline runs 4-7x ahead (the phase-4 cadence root).
     * Fix: while the CPU is executing the schedule ISR (CCPN == its SRPN) at phase>=4,
     * hold TIM0 at the value sampled on ISR entry. CCPN-scoped = robust: it spans the ISR
     * AND its callees (ActivateTask, the thunks all run at the ISR's priority) and releases
     * exactly when the ISR returns -- no fragile read/write delimiter, no TIM0 stall. */
    if (g_stm_freeze_en > 0 && g_stm_cpu_env && g_stm_sched_prio
        && g_stm_phase >= 4 && g_stm_phase < 0x40
        && (g_stm_cpu_env->ICR & 0xFFu) == g_stm_sched_prio) {
        if (!g_stm_freeze) {
            g_stm_freeze = true;
            g_stm_freeze_snap = live;
        }
        /* SAFETY: never hold longer than ~2.2ms of ticks (a real ISR is a few us). */
        if (((live - g_stm_freeze_snap) & 0x00FFFFFFFFFFFFFFULL) > 200000ull) {
            return live;
        }
        return g_stm_freeze_snap;
    }
    g_stm_freeze = false;
    /* 56-bit free-running counter at f_stm (ns_per_tick = 1e9 / f_stm). */
    return live;
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
    /* TEST (env TC1797_STMTICK): the firmware sets up CMCON + SRC0/SRC1 (SRPN-8/7,
     * CPU-routed, SRE=1) and the cursor ISR FUN_80081870 writes CMP0 deadlines + acks via
     * ISCR -- it clearly INTENDS the STM CMP to drive the schedule cursor -- but the final
     * ICR.CMPxEN enable lives in the cold-init FUN_80001160 that the reachable boot path
     * never executes (FUN_80000230 is non-returning). Model that intended enable (treat a
     * configured+SRE'd compare as enabled) so the faithful hardware compare fires the
     * cursor at the firmware's own programmed deadlines, replacing the fixed-1ms crutch. */
    static int force = -1;
    if (force < 0) { force = getenv("TC1797_STMTICK") ? 1 : 0; }
    if (force) { en = 1; }
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
    /* If the firmware armed CMP just BEHIND the live counter (delta lands in the
     * upper half of the period = the compare value was recently PASSED, i.e. the
     * schedule fell behind by a few ticks under QEMU's icount cadence), the
     * windowed match would otherwise wait a FULL period for the counter to wrap
     * back -- up to ~47s for a 32-bit compare (CMCON MSIZE=31). The STM then
     * appears to stop and the CPU idles forever. Silicon keeps CMP ahead of the
     * counter so it never reaches this state; faithfully fire the OVERDUE tick
     * now (next tick) so the ISR re-arms ahead and the cadence resumes -- the
     * same catch-up the narrow 18-bit window gives implicitly.
     *
     * BUGFIX 2026-06-14: this was COMMENTED but never implemented -- the code just
     * `return delta` below, so an overdue 32-bit CMP0 (the OSEK system tick, period
     * 2^32) returned ~47s and the OS tick STALLED for 47s every time the firmware
     * re-armed it a few ticks behind. That throttled the whole OSEK time base:
     * periodic tasks (the master + the 987c counter tick FUN_800c10de) ran ~once/2.5s
     * instead of ~100us, so the path2 promote counter 987c never reached 100 and
     * phase 3->4 never completed. Detect overdue (delta past the half-period) and
     * CAP the overdue wait at the implicit narrow-window period (~2^18 ticks ≈
     * 2.9ms) instead of the full ~47s 32-bit wrap. This bounds the stall without
     * (a) the ~47s freeze, (b) a 1-tick refire that storms the SRN before the ISR
     * is armed and wedges phase 2, or (c) overcorrecting to a tiny residual delta
     * that fires the schedule ~100x too fast. The next on-time arm resumes the real
     * cadence. Only applies to wide (>2^18) compares; narrow windows already bound
     * the wait implicitly. */
    /* REFINED 2026-06-15: the fixed 2^18 (~2.9 ms) overdue cap is itself LARGER than the
     * firmware's own watchdog kick window (FUN_800c3c1a: 0x29bf TIM1 ticks = 1.9 ms). The
     * OSEK system tick (CMP0) lands a few ticks behind tim_now on almost every re-arm --
     * the icount read(FUN_80081836)->write(CMP0) instruction gap, which cannot happen on
     * synchronous silicon -- so it takes the overdue path nearly every cycle and fires at
     * 2.9 ms > 1.9 ms. The kick is then always late, cnt37 climbs past 9, and FUN_800c3c68
     * software-resets (DTC-0x3045) every ~58 ms: the phase-3 watchdog reset loop that blocks
     * the promote. FIX: when a re-arm is overdue (or exactly on the counter), fire at the
     * firmware's OWN learned forward cadence for this channel -- the period it actually
     * programs when it does land ahead -- instead of an arbitrary cap. That holds the OSEK
     * tick at the firmware's real period (≈ the 90000-tick / 1 ms schedule), strictly inside
     * the 1.9 ms watchdog window, so the kick stays fresh and cnt37 stays 0, exactly as on
     * silicon -- without the 47 s 32-bit wrap freeze or a 1-tick storm. */
    /* Learn the firmware's FULL cycle period per channel = the LARGEST forward re-arm delta
     * it programs (CMP0 ~9000=100us, CMP1 ~90000=1ms from the cold-init), bounded just under
     * the 1.9ms watchdog window. An overdue/exact re-arm then fires after one full cycle, NOT
     * after the smallest sub-slot delta: firing at the tiny delta storms the SR (the schedule
     * ISR re-fires before its deadline can advance -> the cursor stalls, the OSEK time base
     * runs at a wrong rate, and alarm-driven refreshers lag -> DTC-0x3046/0x3047). Firing at
     * the full cycle keeps the OSEK tick at the firmware's real cadence, inside the watchdog
     * window, exactly as on silicon -- no 47s wrap, no storm. */
    static uint64_t last_fwd[2] = { 0, 0 };
    bool overdue = (delta > (period >> 1));
    if ((ch & 1) && getenv("TC1797_CMP1LOG")) {
        static unsigned c1; static uint64_t maxd;
        if (delta && !overdue && delta > maxd) { maxd = delta; }
        if (c1++ < 3000) {
            fprintf(stderr, "CMP1 delta=%llu overdue=%d last_fwd=%llu maxfwd=%llu period=%llu\n",
                    (unsigned long long)delta, overdue, (unsigned long long)last_fwd[1],
                    (unsigned long long)maxd, (unsigned long long)period);
            fflush(stderr);
        }
    }
    /* CMP1 (the OSEK alarm timer, cold-init CMP1 = TIM0 + 90000 = 1 ms) is intentionally NOT pinned: it
     * must fire at the firmware's NEAR-alarm rate (the alarm queue FUN_801250e8 re-arms it sub-1 ms), so
     * pinning it to its 1 ms cold-init period STARVES the dispatch and the boot holds at phase 3 (verified
     * A/B: CMP0=9000 + CMP1=90000 stalls at phase 3; CMP0=9000 + CMP1 unpinned reaches phase 4). CMP1
     * falls through to the learned-cadence path below (last_fwd[1] = its real programmed period). Pinning
     * CMP1 to a fixed period was the reverted MAX-last_fwd / 1 ms failure -- it let the firmware arm a
     * 2.2 ms alarm and tripped the DTC-0x3045 watchdog. Only CMP0 (the fixed 100 us system tick) is pinned. */
    /* OSEK SYSTEM-TICK FIX (OPT-IN via TC1797_OSEKTICK_PIN; see the FAITHFUL DEFAULT note below). The firmware's CMP0 schedule
     * ISR (FUN_80081870) re-arms CMP0 = deadline by reading the live STM TIM (FUN_80081836); under
     * -icount that read->write spans real instructions, so the re-arm lands a few ticks BEHIND the
     * counter -- impossible on synchronous silicon. The model then saw CMP0 overdue and self-fired at
     * the tiny catch-up delta (~49us, and degenerating), which ran the OSEK schedule/dispatch at the
     * wrong cadence: the prio-10 COM tasks were re-activated too often and the prio-1 periodic
     * module-dispatch task (FUN_80119cf6 -> engine Rx / 19c4 refresh) was STARVED, so the phase-3->4
     * promote never completed and DTC-0x3045/0x3047 reset-looped. FIX: when CMP0's re-arm is overdue/
     * un-rearmed, fire at CMP0's OWN cold-init period (FUN_80001160: CMP0 = TIM0 + 9000 = 100us, the
     * firmware's intended OSEK system-tick) instead of the catch-up delta. This holds the schedule at
     * the faithful 100us cadence, the dispatch runs, the debounce reaches 60 -> phase 4 -> os_running,
     * and the watchdog kick stays inside its 1.9ms window. CMP1 (the 1ms alarm timer) is left to its own
     * model -- pinning it to 1ms breaks the dispatch (it must fire at the firmware's near-alarm rate). */
    if (g_stm_freeze_en < 0) {
        g_stm_freeze_en = getenv("TC1797_STM_FREEZE") ? 1 : 0;
    }
    /* FAITHFUL DEFAULT (2026-06-21): the 100us CMP0 pin was a crutch for the shift=2 icount
     * read->write inflation (CPU ~2x too fast vs the STM). With the correct CPU:STM ratio
     * (-icount shift=4) the firmware's own accumulated deadline tracks TIM0 1:1 (measured
     * ratio 1.0, stable phase 4, cursor cycling) with NO pin -- exactly as silicon. The pin
     * is now OPT-IN (TC1797_OSEKTICK_PIN) for the legacy shift=2 path only. See
     * EMU_TIMING_DIAGNOSIS.md / tc1797_schedule_cadence_root. */
    bool freeze_active = (g_stm_freeze_en == 1 && g_stm_phase >= 4 && g_stm_phase < 0x40);
    if (getenv("TC1797_OSEKTICK_PIN") && !freeze_active) {
        uint64_t pin = (ch & 1) ? 0u : 9000u;            /* CMP0 = 100us OSEK tick; CMP1 unpinned */
        const char *pe = (ch & 1) ? getenv("TC1797_OSEKTICK1") : getenv("TC1797_OSEKTICK0");
        if (pe) {
            pin = strtoull(pe, NULL, 0);
        }
        if (pin) {
            if (delta != 0 && !overdue && delta < pin) {
                return delta;
            }
            return pin;
        }
    }
    if (delta != 0 && !overdue) {
        last_fwd[ch & 1] = delta;          /* a normal forward arm: learn the cadence */
        return delta;
    }
    /* FAITHFUL overdue CMP1 (env TC1797_CMP1_FAITHFUL, default-off). Requires TC1797_SRC_FIX too (else the
     * schedule, mis-routed to CMP1, dies). Measured (TC1797_CMPWR signed delta): every
     * firmware CMP1 arm is FORWARD while the alarm is active; CMP1 fires once at its deadline (~128ms,
     * phase3->4) and then the firmware STOPS re-arming it (no next alarm) -- the value just goes stale.
     * On silicon a windowed-equality compare left behind the counter does NOT re-fire until the window
     * wraps (~47s for the 32-bit MSIZE=31 compare) -- it stays SILENT until re-armed forward. The old
     * last_fwd path instead re-fired the stale alarm every ~60us (SRPN7 flood) = the os_running churn.
     * Return the real windowed period so an overdue/stale CMP1 stays silent (faithful), while forward
     * arms still fire on time. CMP0 is untouched: it is pinned to 100us above, which absorbs the single
     * 1.2ms phase3->4 schedule gap (CMP0 has only ~5 overdue arms ever, all at that gap). */
    if ((ch & 1) && !getenv("TC1797_CMP1_FLOOD")) {  /* DEFAULT-ON: pairs with the faithful SRC map */
        return delta ? delta : period;     /* silent until the firmware re-arms CMP1 forward */
    }
    if (last_fwd[ch & 1] >= 64) {
        return last_fwd[ch & 1];           /* overdue/exact: reuse the firmware's own period */
    }
    /* cadence not learned yet: bounded fallback (avoid the 47 s wrap) */
    if (overdue && period > (1ull << 18)) {
        return (1ull << 18);
    }
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
        /* FAITHFUL CROSSING-EDGE (default-on; opt-out TC1797_NO_STM_CROSSING).
         *
         * The silicon STM counter increments by 1 every fSTM tick -- it never skips a
         * value, so the windowed-EQUALITY compare always fires the instant the counter
         * REACHES CMPx. Under -icount TIM0 is quantized (jumps 2^shift/ns_per_tick per
         * instruction) and can step from below CMPx to past it, never landing on the
         * equality value -- so the edge silicon guarantees is lost. tc1797_stm_arm runs on
         * every CMP write (incl. the schedule ISR's frequent CMP0 re-arms); if a compare
         * was armed FORWARD and TIM0 has since crossed its deadline (deadline <= now), the
         * equality edge already occurred on silicon. Recomputing it here via match_ticks
         * would treat it as overdue and push it to the far-future window wrap (~47s),
         * dropping the edge -- the bug that freezes the CMP1 alarm forever.
         *
         * Fix: preserve a crossed-but-undelivered forward deadline so tc1797_stm_tick
         * delivers it ONCE (best <= now fires the timer immediately), reconstructing the
         * silicon "counter reached CMPx" edge for the quantized counter. This is an EDGE,
         * not a `>=` level: a compare armed BEHIND the counter never reaches this branch
         * (its freshly computed deadline is the far-future wrap, not <= now), so it stays
         * silent until the window wraps -- exactly the UM windowed-equality behaviour --
         * and after firing, the recompute sets it far-future, so it fires once per arm.
         * This is the faithful behaviour and IS the default; it is gated OPT-OUT
         * (TC1797_STM_CROSSING_OFF) so the legacy drop-the-edge path can be restored if
         * un-freezing CMP1 re-exposes the MSDI bank2-fill dispatch bug (the monitor loops
         * on the unfilled bank2 -> DTC 0x302f reset). Default-on = the faithful edge; the
         * opt-out keeps the old (frozen-but-stable) phase-4 boot. */
        static int g_crossing = -1;
        if (g_crossing < 0) {
            g_crossing = getenv("TC1797_STM_CROSSING_OFF") ? 0 : 1;  /* faithful, default ON */
        }
        if (g_crossing && s->stm_cmp_deadline[ch] != INT64_MAX
            && s->stm_cmp_deadline[ch] <= now) {
            if (s->stm_cmp_deadline[ch] < best) {
                best = s->stm_cmp_deadline[ch];
            }
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
    if (getenv("TC1797_STMLOG")) {
        static int armn;
        /* Log the first 60 arm calls with full state to see the exact CMP0/CMP1/ICR
         * sequence and whether CMP0 (the ISR-rearmed channel) ever gets an enabled
         * short deadline. best-now in us. */
        if (armn++ < 60) {
            info_report("tc1797: arm#%d icr=%02x cmcon=%06x | tim=%08x cmp0=%08x cmp1=%08x"
                        " en0=%d en1=%d | best_us=%lld",
                        armn, s->stm_icr & 0xff, s->stm_cmcon & 0xffffff,
                        (uint32_t)tim_now, s->stm_cmp0, s->stm_cmp1,
                        tc1797_stm_cmp_enabled(s, 0), tc1797_stm_cmp_enabled(s, 1),
                        (long long)(best == INT64_MAX ? -1 : (best - now) / 1000));
        }
        /* Steady-state (phase 3): per-channel next-match in us -- shows whether CMP0
         * and CMP1 are BOTH enabled and at what period (kick-task starvation check). */
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph == 3) {
            static int p3n;
            if (p3n++ < 24) {
                int64_t d0 = tc1797_stm_cmp_enabled(s,0) ?
                    (int64_t)(tc1797_stm_match_ticks(s,0,tim_now)*s->stm_ns_per_tick)/1000 : -1;
                int64_t d1 = tc1797_stm_cmp_enabled(s,1) ?
                    (int64_t)(tc1797_stm_match_ticks(s,1,tim_now)*s->stm_ns_per_tick)/1000 : -1;
                fprintf(stderr, "STMP3 en0=%d en1=%d next0_us=%lld next1_us=%lld cmp0=%08x cmp1=%08x cmcon=%06x\n",
                        tc1797_stm_cmp_enabled(s,0), tc1797_stm_cmp_enabled(s,1),
                        (long long)d0, (long long)d1, s->stm_cmp0, s->stm_cmp1, s->stm_cmcon & 0xffffff);
                fflush(stderr);
            }
        }
    }
    if (best == INT64_MAX) {
        timer_del(s->stm_timer);
    } else {
        timer_mod(s->stm_timer, best);
    }
}

static void tc1797_stm_ack(TC1797SoCState *s)
{
    /* The firmware cleared the STM compare IR(s). RE-ARBITRATE the CPU ICU rather
     * than blindly force PIPN=0: another CPU service request may still be pending in
     * icu_pending -- notably the OSEK dispatch (cpu_src SRPN-1), which the higher STM
     * tick out-ranks during the tick ISR. Forcing PIPN=0 here clobbered that
     * re-arbitrated dispatch right before the tick ISR's RFE, so when the CPU returned
     * to ccpn=0 nothing was pending and the SRPN-1 dispatcher never ran -- freezing
     * curid/nextid and starving the prio-9 phase-driver task (the phase 3->4 stall).
     * tc1797_icu_arbitrate presents the highest remaining pending source (and clears
     * HARD itself when none remain), which is the faithful behaviour. */
    tc1797_icu_arbitrate(s);
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
/* Phase-2 profiler counters: how many STM CMP0/CMP1 fires (= schedule/alarm ticks) elapse per
 * master-task run (per debounce cycle). Read+reset by tc1797_icount_prof. */
uint64_t g_icprof_cmp0_fires, g_icprof_cmp1_fires;

static void tc1797_stm_tick(void *opaque)
{
    TC1797SoCState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool raised = false;

    /* DIAGNOSTIC (env TC1797_PCHIST): sample the interrupted PC at each STM tick
     * during phase 3. The kick lives in the idle loop (prio 1); it only runs <1.9ms
     * if the CPU idles often. If a task SPINS ~2ms between idles (stealing the idle
     * slack), the kick falls to ~2ms and trips the watchdog. This histogram (sorted
     * offline) reveals the hot spin. */
    if (getenv("TC1797_PCHIST")) {
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph >= 3 && ph < 0x40) {
            static unsigned pn;
            if (pn++ < 4000) {
                uint32_t pc = s->cpu.env.PC & ~0x20000000u;
                fprintf(stderr, "PCHIST %08x ccpn=%u\n", pc, s->cpu.env.ICR & 0xFFu);
                fflush(stderr);
            }
        }
    }
    /* DIAGNOSTIC (env TC1797_READYCT): dump OSEK per-priority ready-counts at phase 3.
     * descriptor base = [DAT_d000998c]; ready-count(prio) = *(base + prio*0x10). A prio
     * whose count stays >0 across dumps is being re-activated as fast as it's dispatched
     * -> keeps FUN_80124512 busy -> idle kick only ~1x/2ms-tick -> watchdog 0x3045. */
    if (getenv("TC1797_READYCT")) {
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph == 3) {
            static unsigned rn;
            if (rn++ < 30) {
                uint32_t base = 0; cpu_physical_memory_read(0xD000998Cu, &base, 4);
                char line[256]; int o = 0;
                o += snprintf(line+o, sizeof(line)-o, "READYCT base=%08x cur=", base);
                uint32_t cur = 0; cpu_physical_memory_read(0xD0000984u, &cur, 4);
                o += snprintf(line+o, sizeof(line)-o, "%u |", cur);
                if (base >= 0x80000000u || base < 0xE0000000u) {
                    for (int p = 1; p <= 15 && o < 230; p++) {
                        uint32_t rc = 0;
                        cpu_physical_memory_read((hwaddr)base + p*0x10, &rc, 4);
                        o += snprintf(line+o, sizeof(line)-o, " p%d=%u", p, rc);
                    }
                }
                fprintf(stderr, "%s\n", line); fflush(stderr);
                /* prio-1 runnable list: descriptor[1][1]=[base+0x14] is the live list
                 * ptr (advanced each dispatch). Dump the fn ptrs around it to see if
                 * the kick (800c3c1a) is repeated / the list is circular (silicon kicks
                 * continuously in idle) vs run-once (QEMU). */
                uint32_t lp = 0; cpu_physical_memory_read((hwaddr)base + 0x14, &lp, 4);
                char l2[256]; int o2 = snprintf(l2, sizeof(l2), "P1LIST ptr=%08x runnables:", lp);
                if (lp >= 0x80000000u) {
                    for (int k = -3; k <= 8 && o2 < 235; k++) {
                        uint32_t fnp = 0;
                        cpu_physical_memory_read((hwaddr)lp + k*4, &fnp, 4);
                        o2 += snprintf(l2+o2, sizeof(l2)-o2, " %s%08x", k==0?">":"", fnp);
                    }
                }
                fprintf(stderr, "%s\n", l2); fflush(stderr);
                /* dump the two OSEK queues FUN_80124f0c scans: q1=0xD00185C0,
                 * q2=0xD00042EC. Donor has cafeface guards + a few entries then
                 * sentinel(abcddcba). Compare QEMU -- missing guards/entries = a
                 * producer/init that didn't run. */
                {
                    char q[256]; int qo;
                    uint32_t qbs[2] = {0xD00185C0u, 0xD00042ECu};
                    for (int qi = 0; qi < 2; qi++) {
                        qo = snprintf(q, sizeof(q), "QDUMP q%d@%08x:", qi+1, qbs[qi]);
                        for (int w = 0; w < 12; w++) {
                            uint32_t v = 0;
                            cpu_physical_memory_read((hwaddr)qbs[qi] + w*4, &v, 4);
                            qo += snprintf(q+qo, sizeof(q)-qo, " %08x", v);
                        }
                        fprintf(stderr, "%s\n", q); fflush(stderr);
                    }
                }
            }
        }
    }

    /* DIAGNOSTIC (env TC1797_KICKCAL, NOT a faithful fix): the watchdog-kick task's
     * alarm period is set in FUN_8007f9be as ((int)DAT_d0016ed0 + 0x898)*0x5a = (offset+2200us)*90
     * ticks/us. DAT_d0016ed0 has NO code writers -> it is a DFLASH-calibration value, absent in
     * QEMU -> 0 -> a 2200us=2.2ms kick that EXCEEDS the 0x29bf=1.9ms watchdog threshold (FUN_800c3c1a)
     * -> cnt37 grows -> DTC 0x3045. Seed it to -1200 ((-1200+2200)*90 = 90000 = 1.0ms kick, < 1.9ms)
     * to CONFIRM that the kick cadence is the sole 0x3045 cause. Faithful fix = load the real
     * calibration so the firmware sets DAT_d0016ed0 organically. */
    if (getenv("TC1797_KICKCAL")) {
        /* Donor (dspr_185523.bin off 0x16ed0) value: -65536 -> kick-alarm period
         * (DAT_d0016ed0+2200)*90 clamps NEGATIVE -> -1 -> FUN_801243ae DISABLES the
         * alarm (vs a positive period that re-SetRelAlarms an already-active alarm and
         * fails -> d000004c -> DTC 0x302e). Use the donor value to test whether this one
         * .bss-missing calibration is the single root of BOTH 0x3045 and 0x302a/e. */
        const char *kv = getenv("TC1797_KICKVAL");
        int32_t off = kv ? (int32_t)strtol(kv, NULL, 0) : -65536;
        cpu_physical_memory_write(0xD0016ED0u, &off, 4);
    }

    /* DIAGNOSTIC (env TC1797_E2ECHK): dump the E2E redundant-storage mirror pairs that
     * FUN_800644f4 checks (mirror == ~value; any mismatch -> DAT_c0002558=0x403008 fault).
     * Shows which pair is inconsistent (the reset cause) without KICKCAL masking it. */
    if (getenv("TC1797_E2ECHK")) {
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph == 3) {
            static unsigned ec;
            if (ec++ < 24) {
                uint8_t ed2,ed3,ed4,ed5,ea5, cf8,cf9,cfa,cfb,cde, ce1=0;
                uint16_t ed6,ed8,cfc,cfe;
                uint32_t c2558=0;
                cpu_physical_memory_read(0xD0016ED2u,&ed2,1);
                cpu_physical_memory_read(0xD0016ED3u,&ed3,1);
                cpu_physical_memory_read(0xD0016ED4u,&ed4,1);
                cpu_physical_memory_read(0xD0016ED5u,&ed5,1);
                cpu_physical_memory_read(0xD0016ED6u,&ed6,2);
                cpu_physical_memory_read(0xD0016ED8u,&ed8,2);
                cpu_physical_memory_read(0xD0016EA5u,&ea5,1);
                cpu_physical_memory_read(0xD0016CF8u,&cf8,1);
                cpu_physical_memory_read(0xD0016CF9u,&cf9,1);
                cpu_physical_memory_read(0xD0016CFAu,&cfa,1);
                cpu_physical_memory_read(0xD0016CFBu,&cfb,1);
                cpu_physical_memory_read(0xD0016CFCu,&cfc,2);
                cpu_physical_memory_read(0xD0016CFEu,&cfe,2);
                cpu_physical_memory_read(0xD0016CDEu,&cde,1);
                cpu_physical_memory_read(0xC0002558u,&c2558,4);
                cpu_physical_memory_read(0xD0016CE1u,&ce1,1);
                fprintf(stderr, "E2ECHK ed2=%02x~cf8=%02x%s ed3=%02x~cf9=%02x%s ed4=%02x~cfa=%02x%s "
                        "ed5=%02x~cfb=%02x%s ed6=%04x~cfc=%04x%s ed8=%04x~cfe=%04x%s ea5=%02x~cde=%02x%s | fault=%08x ce1=%02x\n",
                        ed2,(uint8_t)~cf8, ed2!=(uint8_t)~cf8?"!":"", ed3,(uint8_t)~cf9, ed3!=(uint8_t)~cf9?"!":"",
                        ed4,(uint8_t)~cfa, ed4!=(uint8_t)~cfa?"!":"", ed5,(uint8_t)~cfb, ed5!=(uint8_t)~cfb?"!":"",
                        ed6,(uint16_t)~cfc, ed6!=(uint16_t)~cfc?"!":"", ed8,(uint16_t)~cfe, ed8!=(uint16_t)~cfe?"!":"",
                        ea5,(uint8_t)~cde, ea5!=(uint8_t)~cde?"!":"", c2558, ce1);
                fflush(stderr);
            }
        }
    }

    /* DIAGNOSTIC (env TC1797_PHASELOG, no behaviour change): log every change of the RTOS phase byte
     * 0xD0003643 in FREE-RUN (this tick runs in the default build), so phase-4 progress is observable
     * without a gdb halt perturbing the boot. */
    if (getenv("TC1797_PHASELOG")) {
        static uint8_t last_ph = 0xFE;
        static uint8_t last_osr = 0xFE;
        uint8_t ph = 0, osr = 0;
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        cpu_physical_memory_read(0xD000400Fu, &osr, 1);   /* os_running flag */
        if (ph != last_ph || osr != last_osr) {
            last_ph = ph;
            last_osr = osr;
            fprintf(stderr, "PHASELOG 0xD0003643 -> %u  os_running(0xD000400F)=%u  (now=%lld ms)\n",
                    ph, osr, (long long)(now / 1000000));
            fflush(stderr);
        }
    }
    /* DIAGNOSTIC (env TC1797_GRPDUMP): the COM active-IPDU-group state. Reads the runtime
     * pointer DAT_d0001cbc (0xD0001CBC) and the 16-bit group word it targets. Confirms (a) the
     * pointer's value (expect 0x7FFF9244 from FUN_800ff434) and (b) whether any group is active
     * (non-zero) -- the gate that decides if the engine-CAN COM Rx runs at all. */
    if (getenv("TC1797_GRPDUMP")) {
        static unsigned gd;
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph >= 3 && gd++ < 10) {
            uint32_t ptr = 0; cpu_physical_memory_read(0xD0001CBCu, &ptr, 4);
            uint16_t grp = 0xDEAD;
            if (ptr >= 0x70000000u && ptr < 0xE0000000u) {
                cpu_physical_memory_read(ptr, &grp, 2);
            }
            fprintf(stderr, "GRPDUMP DAT_d0001cbc=0x%08x  *group=0x%04x  ph=%u\n", ptr, grp, ph);
            fflush(stderr);
        }
    }
    /* DIAGNOSTIC (env TC1797_COMCFG): for the engine-CAN indication messages (MO33=0x12f,
     * MO51=0x328, MO57=0x388), find their COM message index in the runtime config
     * (DAT_d0001cc0 + i*0x30, MO at +0x2c) and dump the per-message flags (DAT_d00168be[i])
     * + scheduler state (DAT_d0016862[i]) + period counter (DAT_d001691a[i]) + group mask
     * (DAT_80048cee[i*10]). Pinpoints why the scheduler skips them despite groups=0xFFFF. */
    if (getenv("TC1797_COMCFG")) {
        static int done, seen;
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph >= 3 && ph < 10 && !done && seen++ == 16) {
            done = 1;
            uint32_t cfg = 0; cpu_physical_memory_read(0xD0001CC0u, &cfg, 4);
            fprintf(stderr, "COMCFG config_base=0x%08x\n", cfg);
            for (int i = 0; i < 0x5c; i++) {
                uint16_t mo = 0xFFFF;
                if (cfg >= 0x80000000u) {
                    cpu_physical_memory_read(cfg + i*0x30 + 0x2c, &mo, 2);
                }
                if (mo == 33 || mo == 51 || mo == 57) {
                    uint8_t fl = 0, st = 0, ctr = 0;
                    uint16_t gm = 0;
                    cpu_physical_memory_read(0xD00168BEu + i, &fl, 1);
                    cpu_physical_memory_read(0xD0016862u + i, &st, 1);
                    cpu_physical_memory_read(0xD001691Au + i, &ctr, 1);
                    cpu_physical_memory_read(0x80048CEEu + i*0x14, &gm, 2);
                    fprintf(stderr, "COMCFG msg[%d] mo=%u flags=0x%02x state=0x%02x ctr=0x%02x groupmask=0x%04x\n",
                            i, mo, fl, st, ctr, gm);
                }
            }
            fflush(stderr);
        }
    }
    /* DIAGNOSTIC (env TC1797_CANRXLOG): dump the firmware's configured RECEIVE message
     * objects (the CAN IDs it expects from other modules) once at phase>=2, so we know
     * which key-on-engine-off frames to inject for the COM-health / engine-state chain. */
    if (getenv("TC1797_CANRXLOG")) {
        static unsigned crl;
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph >= 2 && crl++ < 2) {
            int rxn = 0;
            for (int n = 0; n < TC1797_CAN_NMO; n++) {
                if (s->can.mo[n].configured && s->can.mo[n].dir == 0) {
                    uint32_t moipr = tc1797_can_read(&s->can,
                        TC1797_CAN_MO_BASE + (uint32_t)n * 0x20u + 0x08u);
                    fprintf(stderr, "CANRX MO%d id=0x%03x mask=0x%03x rxinp_node=%u\n",
                            n, s->can.mo[n].id, s->can.mo[n].mask, (moipr >> 8) & 0x1Fu);
                    rxn++;
                }
            }
            fprintf(stderr, "CANRX total configured RX MOs=%d (ph=%u)\n", rxn, ph);
            fflush(stderr);
        }
    }
    /* KEY-ON-ENGINE-OFF BUS SIMULATOR (env TC1797_CANENV): the firmware is a DME that
     * expects ~50 periodic CAN frames from the rest of the vehicle (gateway, modules).
     * Without them the COM-health counter d00 climbs to 0xff and the CAN-derived signal
     * block stays at FUN_800ff17c's invalid sentinels -> 4f6 engine-health never sets ->
     * no phase-4 promote. Inject every configured RX MO's ID periodically with the
     * key-on-engine-off payload (TC1797_CANDATA=hex, default all-zero = idle) so the
     * firmware's COM processes them and the application reaches a healthy state. This is
     * the bench/in-car environment, not a DSPR poke. */
    if (getenv("TC1797_CANENV")) {
        static int rr_mo;
        static uint8_t e2ectr;
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        /* Inject ONE configured RX MO per tick, round-robin. Injecting all ~50 MOs at
         * once bursts 50 SRPN-29 RX interrupts in a single tick, which stalls the STM
         * tick and hangs the boot (the diagnostics then freeze after ph2). One-per-tick
         * approximates a real per-message bus arrival rate and keeps the OS scheduled. */
        /* RATE-LIMIT (fix the SRPN-29 flood that hangs the OS): this block runs in the STM tick,
         * which fires every ~12us (CMP1 self-fire) -- injecting one MO per tick = ~80x faster than a
         * real CAN frame, so the level-triggered NEWDAT re-raises SRPN-29 faster than the RX ISR can
         * drain it and the OS dispatch wedges. Gate to a realistic per-frame interval (default 1ms;
         * TC1797_CANRATE_NS overrides) so each injected frame gets one clean drained IRQ. */
        static int64_t can_last_inj;
        const char *canrl = getenv("TC1797_CANRATE_NS");
        int64_t can_interval = canrl ? strtoll(canrl, NULL, 0) : 1000000;
        if (ph >= 2 && now - can_last_inj >= can_interval) {
            can_last_inj = now;
            uint8_t data[8] = {0};
            const char *cd = getenv("TC1797_CANDATA");
            if (cd) {
                for (int i = 0; i < 8 && cd[i*2] && cd[i*2+1]; i++) {
                    char b[3] = { cd[i*2], cd[i*2+1], 0 };
                    data[i] = (uint8_t)strtoul(b, NULL, 16);
                }
            }
            /* alive-counter advances once per tick (consecutive frames are never stale) */
            if (!getenv("TC1797_CANCTR_OFF")) {
                e2ectr = (e2ectr + 1) & 0x0F;
                data[1] = (data[1] & 0xF0) | e2ectr;
            }
            /* TC1797_CANALL: inject EVERY configured RX MO every tick so NEWDAT is always
             * fresh when the polled COM Rx (FUN_800dc44c) reads it -> the valid-frame branch
             * runs (signals copied) instead of the no-data invalidator. Pair with
             * TC1797_NORXINT to avoid the SRPN-29 interrupt flood. Default: one MO/tick. */
            int all = getenv("TC1797_CANALL") != NULL;
            for (int tries = 0; tries < TC1797_CAN_NMO; tries++) {
                int cand;
                if (all) {
                    cand = tries;
                } else {
                    cand = rr_mo % TC1797_CAN_NMO;
                    rr_mo = (rr_mo + 1) % TC1797_CAN_NMO;
                }
                if (!(s->can.mo[cand].configured && s->can.mo[cand].dir == 0
                      && s->can.mo[cand].id < 0x700
                      && !(getenv("TC1797_CAN12FONLY") && s->can.mo[cand].id != 0x12F))) {
                    continue;
                }
                uint8_t f[8];
                memcpy(f, data, 8);
                if (s->can.mo[cand].id == 0x12F || getenv("TC1797_CANSIGN_ALL")) {
                    uint16_t dataid = 0;
                    if (s->can.mo[cand].id == 0x12F) {
                        uint32_t didptr = 0;
                        cpu_physical_memory_read(0xD000007Cu, &didptr, 4);
                        if (didptr >= 0x80000000u) {
                            cpu_physical_memory_read(didptr, &dataid, 2);
                        }
                    }
                    uint8_t seq[9] = { (uint8_t)dataid, (uint8_t)(dataid >> 8),
                                       f[1], f[2], f[3], f[4], f[5], f[6], f[7] };
                    uint8_t crc = 0;
                    for (int j = 0; j < 9; j++) {
                        uint8_t c = crc ^ seq[j];
                        for (int b = 0; b < 8; b++) {
                            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x1D)
                                          : (uint8_t)(c << 1);
                        }
                        crc = c;
                    }
                    f[0] = crc;
                }
                tc1797_can_rx_inject(&s->can, s->can.mo[cand].id, f, 8);
                if (!all) break;
            }
        }
    }
    /* DIAGNOSTIC (env TC1797_PDULOG): does the injected CAN-id-0x12f data reach its COM
     * I-PDU buffer 0xD0014BF4 (-> cb_8009c6e2 -> d00=0)? Tells us if the COM RX dispatch
     * (CanIf->PduR->COM->callback) runs for injected frames or only the MO is drained. */
    if (getenv("TC1797_PDULOG")) {
        static unsigned pl;
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph >= 2 && ph < 0x40 && (pl++ & 0x1ffu) == 0u) {
            uint32_t pdu = 0; uint8_t d00v = 0, c09v = 0, e2eerr = 0;
            cpu_physical_memory_read(0xD0014BF4u, &pdu, 4);
            cpu_physical_memory_read(0xC0003D00u, &d00v, 1);
            cpu_physical_memory_read(0xD0003C09u, &c09v, 1);
            cpu_physical_memory_read(0xD0016D46u, &e2eerr, 1); /* 0x12f E2E error counter */
            fprintf(stderr, "PDULOG 0x12f-buf@D0014BF4=%08x d00=%02x c09=%02x e2eerr=%u (ph=%u)\n",
                    pdu, d00v, c09v, e2eerr, ph);
            /* E2E parameters: the 0x12f DataID (*_DAT_d000007c) + the CRC8 table @0x7FFF940C */
            uint32_t p7c = 0; uint16_t dataid = 0; uint8_t t1[16] = {0}, t2[16] = {0};
            cpu_physical_memory_read(0xD000007Cu, &p7c, 4);
            if (p7c >= 0x80000000u && p7c < 0xF0000000u) {
                cpu_physical_memory_read(p7c, &dataid, 2);
            }
            cpu_physical_memory_read(0x7FFF940Cu, t1, 16);
            cpu_physical_memory_read(0x8000940Cu, t2, 16);
            fprintf(stderr, "  E2E dataid_ptr=%08x dataid=%04x | tbl@7FFF940C=%02x%02x%02x%02x%02x%02x%02x%02x"
                    " | tbl@8000940C=%02x%02x%02x%02x\n", p7c, dataid,
                    t1[0],t1[1],t1[2],t1[3],t1[4],t1[5],t1[6],t1[7], t2[0],t2[1],t2[2],t2[3]);
            fflush(stderr);
        }
    }
    /* TEST (env TC1797_ADCPRAM=<hex>): model the ADC0 result-FIFO->PRAM by writing a conversion
     * result into the PRAM slots the firmware's ADC PCP channel 0x1d reads (0xF0050eb0/ec4 = 0
     * in QEMU; no producer). If a valid value here lights up 0xD000416F bit0 (reset #2 gate) and
     * the sensor mirrors, the FIFO->PRAM is the mechanism and can be modelled per-channel. */
    {
        const char *ap = getenv("TC1797_ADCPRAM");
        if (ap) {
            uint32_t v = (uint32_t)strtoul(ap, NULL, 16);
            cpu_physical_memory_write(0xF0050EB0u, &v, 4);
            cpu_physical_memory_write(0xF0050EC4u, &v, 4);
        }
        /* TEST (env TC1797_416F=<hex>): poke the ADC-done flag 0xD000416F to see whether it
         * gates the sensor reads (2b0X) -- if forcing it valid releases the sensor chain, the
         * ADC-done is the single upstream gate. */
        const char *af = getenv("TC1797_416F");
        if (af) {
            uint8_t v = (uint8_t)strtoul(af, NULL, 16);
            cpu_physical_memory_write(0xD000416Fu, &v, 1);
        }
    }
    /* DIAGNOSTIC (env TC1797_MAXPHASE): log ONLY when a new maximum phase is reached (rare -> negligible
     * throttle, unlike PHASELOG which fprintf's on every oscillation). Lets free-run reach phase 4 fast. */
    if (getenv("TC1797_MAXPHASE")) {
        static uint8_t max_ph;
        uint8_t ph = 0;
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph < 0x40 && ph > max_ph) {           /* log each NEW max phase once */
            uint8_t osr = 0;
            cpu_physical_memory_read(0xD000400Fu, &osr, 1);
            max_ph = ph;
            fprintf(stderr, "MAXPHASE new-max-phase=%u os_running=%u (now=%lld ms)\n",
                    ph, osr, (long long)(now / 1000000));
            fflush(stderr);
        }
        /* When at phase>=4, periodically dump the os_running gate variables so we
         * see which gate blocks os_running (donor running-state: 400b=0xa0 bit5,
         * 416f=0x01, osr=1; 400b=*(DAT_d00009a4+10) so a NULL counter struct -> 0). */
        /* At phase 3: track the watchdog servicing (DAT_d00019c0, kicked by
         * FUN_800c3c1a) + bump counter DAT_d0004037 + promote counter DAT_d000989c.
         * If 19c0 is frozen, the kick task isn't running -> 5ms watchdog 0x3045. */
        if (ph == 3) {
            static unsigned g3; static uint32_t prev19c0;
            static int dumped;
            if (!dumped) {
                dumped = 1;
                /* One-shot ERCOSEK schedule-table dump: DAT_d0000978 -> *(+4) ->
                 * [1] = table base; entries are [callback(4), period(4)]. Reveals
                 * how often the watchdog-kick task body appears (every 1 or 2?). */
                uint32_t cur = 0;
                cpu_physical_memory_read(0xD0012498u, &cur, 4);
                uint32_t tb = (cur >= 0x80000000u && cur < 0x80400000u) ? cur : 0;
                for (int i = 0; tb && i < 64; i++) {
                    uint32_t cb = 0, per = 0;
                    cpu_physical_memory_read((hwaddr)tb + i*8, &cb, 4);
                    cpu_physical_memory_read((hwaddr)tb + i*8 + 4, &per, 4);
                    fprintf(stderr, "SCHEDENT[%d] @%08x cb=%08x period=%u\n",
                            i, tb + i*8, cb, per);
                }
                fflush(stderr);
            }
            if (g3++ < 4000) {
                CPUTriCoreState *env = &s->cpu.env;
                uint32_t c19c0 = 0; uint8_t cnt37 = 0;
                cpu_physical_memory_read(0xD00019C0u, &c19c0, 4);
                cpu_physical_memory_read(0xD0004037u, &cnt37, 1);
                /* CCPN/IE/PC: is a task (CCPN>=8) holding the CPU and masking the
                 * tick SRPN 7/8? Where is the CPU spinning (PC)? */
                uint8_t a9d = 0; uint32_t ostate = 0;
                cpu_physical_memory_read(0xD0019A9Du, &a9d, 1);
                cpu_physical_memory_read(0xF0000480u, &ostate, 4);
                /* PROMOTE trajectory: e7e (phase-3->4 gate), 987c (master-task
                 * counter), 989c (promote threshold counter). Shows whether the
                 * master task runs (987c advancing) AND whether e7e is climbing. */
                uint8_t e7e = 0; uint32_t c987c = 0, c989c = 0;
                cpu_physical_memory_read(0xD0003E7Eu, &e7e, 1);
                cpu_physical_memory_read(0xD000987Cu, &c987c, 4);
                cpu_physical_memory_read(0xD000989Cu, &c989c, 4);
                uint8_t eb1=0,eb5=0,c4041=0; uint16_t d2d32=0;
                cpu_physical_memory_read(0xD0003EB1u, &eb1, 1);
                cpu_physical_memory_read(0xD0003EB5u, &eb5, 1);
                cpu_physical_memory_read(0xD0004041u, &c4041, 1);
                cpu_physical_memory_read(0xD0002D32u, &d2d32, 2);
                uint16_t d6f4=0; uint8_t a45=0;
                cpu_physical_memory_read(0xD000D6F4u, &d6f4, 2);
                cpu_physical_memory_read(0xD0000A45u, &a45, 1);
                /* engine-health debounce FUN_800c2100: 340c=counter (++ while a45.2=1,
                 * =0 when a45.2=0), 340e=output (1 when counter>=threshold). threshold
                 * = *(*DAT_d0000820). 4041=1 needs 340e!=0. Surface counter/threshold to
                 * see whether the debounce is climbing (threshold too high) or being reset
                 * (a45.2 dips / resets zero the counter). */
                uint16_t d340c=0; uint8_t d340e=0; uint32_t p820=0, thr820=0;
                uint8_t msdimode=0;
                cpu_physical_memory_read(0xD000340Cu, &d340c, 2);
                cpu_physical_memory_read(0xD000340Eu, &d340e, 1);
                cpu_physical_memory_read(0xD0000820u, &p820, 4);
                cpu_physical_memory_read((hwaddr)p820, &thr820, 4);  /* read whatever it points at */
                cpu_physical_memory_read(0xD000402Fu, &msdimode, 1);
                fprintf(stderr, "  DEBOUNCE 340c(cnt)=%u 340e(out)=%u thr=*(%08x)=%u msdimode=%u\n",
                        d340c, d340e, p820, thr820, msdimode);
                /* COM activation: distributor FUN_8009f16c gates each signal on
                 * (sig_mask & *DAT_d0001398). Read the pointer then deref the active
                 * group mask -- if 0, the COM stack isn't activated (network mgmt);
                 * if nonzero, it's active but rejecting frames at E2E (RX/CRC). */
                uint32_t p1398=0; uint16_t commask=0; uint16_t p2b0c=0;
                cpu_physical_memory_read(0xD0001398u, &p1398, 4);
                if (p1398 >= 0xD0000000u && p1398 < 0xD0020000u)
                    cpu_physical_memory_read(p1398, &commask, 2);
                cpu_physical_memory_read(0xD0002B0Cu, &p2b0c, 2);
                fprintf(stderr, "PH3GATE eb1=%u eb5=%u 4041=%u a45=%02x d6f4=%04x(type=%x bit0=%u) d2d32=%04x e7e=%u | COMmask=%04x(ptr=%08x) 2b0c=%04x\n",
                        eb1, eb5, c4041, a45, d6f4, (d6f4>>9)&0x1f, d6f4&1, d2d32, e7e, commask, p1398, p2b0c);
                /* FUN_800df7c6 disarm conditions: DBGSR.DE(bit0) && OSTATE.bit0 &&
                 * (DAT_d0019a9d==0x55 || SWEVT&7==2). If all met -> watchdog disarmed. */
                uint8_t curid = 0; uint32_t ccpn = env->PCXI; /* PCXI for context */
                cpu_physical_memory_read(0xD0019A9Eu, &curid, 1); /* OSEK running task id */
                fprintf(stderr, "PH3WD 19c0(%s) cnt37=%u e7e=%u 987c=%u 989c=%u | "
                        "PC=%08x curid=%u ICR=%08x | OSTATE=%08x(b0=%u) (now=%lldms)\n",
                        c19c0 != prev19c0 ? "MOVING" : "frozen", cnt37,
                        e7e, c987c, c989c,
                        (uint32_t)env->PC, curid, env->ICR, ostate, ostate & 1u,
                        (long long)(now / 1000000));
                (void)ccpn;
                fflush(stderr);
                prev19c0 = c19c0;
            }
        }
        if (ph >= 4 && ph < 0x40) {
            static unsigned g4;
            if (g4++ < 25) {
                uint8_t b400b=0, b416f=0, b4039=0, b4007=0, osr=0, e400e=0;
                uint32_t p9a4=0;
                cpu_physical_memory_read(0xD000400Bu, &b400b, 1);
                cpu_physical_memory_read(0xD000400Eu, &e400e, 1);
                cpu_physical_memory_read(0xD000400Fu, &osr, 1);
                cpu_physical_memory_read(0xD0004007u, &b4007, 1);
                cpu_physical_memory_read(0xD000416Fu, &b416f, 1);
                cpu_physical_memory_read(0xD0004039u, &b4039, 1);
                cpu_physical_memory_read(0xD00009A4u, &p9a4, 4);
                fprintf(stderr, "PH4GATE 400b=%02x(bit5=%d) 416f=%02x 4039=%02x 4007=%02x "
                        "400e=%02x 9a4=%08x osr=%u (now=%lldms)\n",
                        b400b, !!(b400b & 0x20), b416f, b4039, b4007, e400e, p9a4, osr,
                        (long long)(now / 1000000));
                fflush(stderr);
            }
        }
    }
    /* DIAGNOSTIC ONLY (env TC1797_E7EPOKE, NOT a fix): the phase-3->4 transition-check h_800c40d2 returns
     * (DAT_d0003e7e == 1). Force e7e=1 at phase 3 to confirm that gate is SUFFICIENT (phase advances to 4)
     * vs. a further gate. The faithful path is to make the promote settle at e7e=1 (eb1=1 chain). */
    if (getenv("TC1797_E7EPOKE")) {
        uint8_t ph = 0;
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph == 3) {
            uint8_t one = 1;
            cpu_physical_memory_write(0xD0003E7Eu, &one, 1);
        }
    }
    /* DIAGNOSTIC (env TC1797_CALSEED, NOT the final fix): the phase-4 chain bottoms out at the calibration
     * block at DSPR 0xD0013378 (d6f4 source) which is NOT in the firmware image -- it lives on the physical
     * ECU's DFLASH. Seed it with the donor's real value (0x2a33... = "config present" magic) to confirm the
     * chain unblocks to phase 4. The FAITHFUL fix is to load the ECU calibration into QEMU's DFLASH so the
     * firmware's loader populates 0xD0013378 organically. */
    if (getenv("TC1797_CALSEED")) {
        static gchar *cd = NULL; static gsize cl = 0; static int ld = 0;
        if (!ld) {
            ld = 1;
            const char *f = getenv("TC1797_CALFILE");   /* donor DSPR dump = the chip's real calibration */
            if (f && !g_file_get_contents(f, &cd, &cl, NULL)) { cd = NULL; }
        }
        if (cd && cl >= 0x14000u) {
            uint8_t ph = 0;
            cpu_physical_memory_read(0xD0003643u, &ph, 1);
            if (ph >= 3 && ph < 0x40) {
                /* Seed the ECU calibration blocks from the donor DSPR. These (0xD00040xx / 0xD000Dxxx /
                 * 0xD0013xxx, the 0x2a-magic config tables the periodic validator FUN_800e006e checks)
                 * live on the physical chip's DFLASH and are absent from the firmware image -> zero in
                 * QEMU. Re-seeded each phase-3 tick to survive the firmware's blank-DFLASH re-load. */
                cpu_physical_memory_write(0xD0003560u, (uint8_t *)cd + 0x3560, 0x08);
                cpu_physical_memory_write(0xD0004080u, (uint8_t *)cd + 0x4080, 0x80);
                cpu_physical_memory_write(0xD0009890u, (uint8_t *)cd + 0x9890, 0x30);
                cpu_physical_memory_write(0xD000D700u, (uint8_t *)cd + 0xD700, 0xA0);
                cpu_physical_memory_write(0xD0013350u, (uint8_t *)cd + 0x13350, 0xA0);
                /* NOTE: the phase-4 OSEK error-counter self-tests (FUN_8008026e DTC
                 * 0x302a-f, FUN_80080800 DTC 0x3010/11, ...) latch because the abnormal
                 * boot (early-selftest reset cascade leaves alarms active in persisted
                 * RAM + missing alarm calibration) makes SetRelAlarm fail. Masking the
                 * counters just reveals the next check in the chain -- the faithful fix
                 * is a CLEAN boot (no reset cascade) so OSEK init runs once with no
                 * failures. Tracked as remaining work; not masked here. */
            }
        }
    }
    /* DIAGNOSTIC (env TC1797_CHAINLOG): the e7e=1 chain inputs (eb1<-fb7<-4041<-FUN_8014065a logic that
     * reads config bytes 4f6/340e/a45/d6f4/aa0). Donor @phase4: 4f6=e7 340e=01 a45=b7 4041=01 fb7=01.
     * Log QEMU's values at phase 3 to pinpoint which config input diverges (=> why 4041=0 => eb1=0). */
    if (getenv("TC1797_CHAINLOG")) {
        static unsigned cn;
        uint8_t ph = 0;
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph == 3 && (cn++ % 8) == 0 && cn < 2400) {
            uint8_t f6=0,e=0,a45=0,e7e=0,eb1=0,c4041=0,d2d32=0; uint16_t d6f4=0,c340=0; uint32_t p820=0,thr=0;
            cpu_physical_memory_read(0xD00004F6u,&f6,1);
            cpu_physical_memory_read(0xD000340Eu,&e,1);
            cpu_physical_memory_read(0xD0000A45u,&a45,1);
            cpu_physical_memory_read(0xD0003E7Eu,&e7e,1);
            cpu_physical_memory_read(0xD0003EB1u,&eb1,1);                /* eb1 gate */
            cpu_physical_memory_read(0xD0004041u,&c4041,1);              /* 4041 */
            cpu_physical_memory_read(0xD0002D32u,&d2d32,1);              /* 2d32 case0 gate */
            cpu_physical_memory_read(0xD000D6F4u,(uint8_t*)&d6f4,2);
            cpu_physical_memory_read(0xD000340Cu,(uint8_t*)&c340,2);     /* debounce counter */
            cpu_physical_memory_read(0xD0000820u,(uint8_t*)&p820,4);     /* DAT_d0000820 ptr */
            if (p820 >= 0x80000000u) cpu_physical_memory_read(p820,(uint8_t*)&thr,4); /* *ptr threshold */
            fprintf(stderr, "CHAINLOG 4f6=%02x 4041=%u eb1=%u 2d32=%u e7e=%02x | a45=%02x[bit2=%d] 340c=%u 340e=%02x d6f4=%04x\n",
                    f6,c4041,eb1,d2d32,e7e,a45,(a45>>2)&1,c340,e,d6f4);
            /* SELF-TEST coding-complement (selftest_80148da4 -> FUN_800bf97c(0x3025)
             * clobbers the promote): source 0xD00172AC, loaded 0xD0017294/5/8/9,
             * checked 0xD0018A70/71/8E/8F. Pass = a71==~a70 && a8f==~a8e. */
            uint8_t cd[12]={0}, a70=0,a71=0,a8e=0,a8f=0; uint32_t a8c=0;
            uint8_t l94=0,l95=0,l98=0,l99=0;
            cpu_physical_memory_read(0xD00172ACu, cd, 12);
            cpu_physical_memory_read(0xD0017294u,&l94,1); cpu_physical_memory_read(0xD0017295u,&l95,1);
            cpu_physical_memory_read(0xD0017298u,&l98,1); cpu_physical_memory_read(0xD0017299u,&l99,1);
            cpu_physical_memory_read(0xD0018A70u,&a70,1); cpu_physical_memory_read(0xD0018A71u,&a71,1);
            cpu_physical_memory_read(0xD0018A8Eu,&a8e,1); cpu_physical_memory_read(0xD0018A8Fu,&a8f,1);
            cpu_physical_memory_read(0xD0018A8Cu,(uint8_t*)&a8c,4);
            fprintf(stderr, "CODECHK 172ac=%02x%02x%02x%02x.%02x%02x%02x%02x.%02x%02x%02x%02x | "
                    "ld94=%02x.%02x ld98=%02x.%02x | a70=%02x a71=%02x[~=%02x %s] a8e=%02x a8f=%02x[~=%02x %s] a8c=%08x\n",
                    cd[0],cd[1],cd[2],cd[3],cd[4],cd[5],cd[6],cd[7],cd[8],cd[9],cd[10],cd[11],
                    l94,l95,l98,l99,
                    a70,a71,(uint8_t)~a70,(a71==(uint8_t)~a70)?"OK":"FAIL",
                    a8e,a8f,(uint8_t)~a8e,(a8f==(uint8_t)~a8e)?"OK":"FAIL", a8c);
            fflush(stderr);
        }
    }
    /* DIAGNOSTIC (env TC1797_PROMOTELOG): free-run view of the phase-3->4 promote state machine
     * FUN_800c43b6. Logs the promote state DAT_d0003e7e + its state-3 gate vars whenever the state
     * changes, so the exact stall stage is visible without a gdb halt. */
    if (getenv("TC1797_PROMOTELOG")) {
        static uint8_t last_e7e = 0xFE;
        static unsigned pn;
        uint8_t e7e = 0, ph = 0, eb1 = 0;
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        cpu_physical_memory_read(0xD0003E7Eu, &e7e, 1);
        /* Log on state change AND periodically (every 40th tick) at phase>=3, so the state-3 dynamics
         * are visible: do the counters DAT_d000987c (ticked by FUN_800c10de) / DAT_d00019d0 advance
         * (the master task is running), and does the config ptr DAT_d00009bc ever become non-NULL? */
        if (e7e != last_e7e || (pn++ % 120u) == 0u) {   /* unconditional heartbeat: is the STM still firing? */
            uint32_t c987c = 0, c19d0 = 0, p9bc = 0;
            cpu_physical_memory_read(0xD0003EB1u, &eb1, 1);
            cpu_physical_memory_read(0xD000987Cu, &c987c, 4);
            cpu_physical_memory_read(0xD00019D0u, &c19d0, 4);
            cpu_physical_memory_read(0xD00009BCu, &p9bc, 4);
            last_e7e = e7e;
            uint16_t d340c = 0; uint8_t d340e = 0, c4041 = 0;
            cpu_physical_memory_read(0xD000340Cu, &d340c, 2);
            cpu_physical_memory_read(0xD000340Eu, &d340e, 1);
            cpu_physical_memory_read(0xD0004041u, &c4041, 1);
            uint8_t r40b=0,r40e=0,r40c=0,r40f=0; uint32_t r18cc=0,r09a4=0;
            cpu_physical_memory_read(0xD000400Bu,&r40b,1); cpu_physical_memory_read(0xD000400Eu,&r40e,1);
            cpu_physical_memory_read(0xD000400Cu,&r40c,1); cpu_physical_memory_read(0xD000400Fu,&r40f,1);
            cpu_physical_memory_read(0xD00018CCu,&r18cc,4); cpu_physical_memory_read(0xD00009A4u,&r09a4,4);
            /* os_running START gate (fn_800bfaac): fires iff 4007==0 && (400c&1)==0 && (seg0[9]&1)!=0.
             * seg0[2] gates the FIRST-half start; 0958 gates the inner first-half branch. With the
             * TC1797_SEG0 probe mapped, seg0[2/9/10] are the live OS-object bytes the firmware wrote. */
            uint8_t s0_2=0,s0_9=0,s0_a=0,r4007=0,r4006=0,r0958=0;
            cpu_physical_memory_read(0x00000002u,&s0_2,1); cpu_physical_memory_read(0x00000009u,&s0_9,1);
            cpu_physical_memory_read(0x0000000Au,&s0_a,1); cpu_physical_memory_read(0xD0004007u,&r4007,1);
            cpu_physical_memory_read(0xD0004006u,&r4006,1); cpu_physical_memory_read(0xD0000958u,&r0958,1);
            /* WATCHDOG 0x3047 state: 19c4 (refresh by fn_800c3c40), 19c0 (kick), 19bc, d4039 (arm
             * 0x55=armed/0xAA=disarmed), d4038 (re-init countdown). If 19c4 is CONSTANT across samples,
             * fn_800c3c40 isn't refreshing it -> stale -> 0x3047 every ~300ms. */
            uint32_t r19c4=0,r19c0=0,r19bc=0; uint8_t r4039=0,r4038=0;
            cpu_physical_memory_read(0xD00019C4u,&r19c4,4); cpu_physical_memory_read(0xD00019C0u,&r19c0,4);
            cpu_physical_memory_read(0xD00019BCu,&r19bc,4); cpu_physical_memory_read(0xD0004039u,&r4039,1);
            cpu_physical_memory_read(0xD0004038u,&r4038,1);
            /* Does the phase-INDEPENDENT module dispatcher FUN_80119cf6 (counter DAT_d0003812) run at
             * phase 3? It dispatches PTR_800629b4 which holds the 19c4-refresh callback. If d3812
             * advances but 19c4 stays frozen, the refresh isn't scheduled in that module at phase 3;
             * if d3812 is frozen, FUN_80119cf6 itself isn't reached. d37f2 = fn_801199d4 (phase==4). */
            uint32_t d3812=0, d37f2=0;
            cpu_physical_memory_read(0xD0003812u,&d3812,4); cpu_physical_memory_read(0xD00037F2u,&d37f2,4);
            fprintf(stderr, "PROMOTELOG ph=%u e7e=%u eb1=%u 987c=%u 340c=%u 340e=%u 4041=%u | 40b=%02x 40c=%02x 40e=%02x osrun=%02x 18cc=%08x 9a4=%08x | seg0[2]=%02x [9]=%02x [a]=%02x 4007=%02x 4006=%02x 0958=%02x | 19c4=%08x 19c0=%08x 19bc=%08x d4039=%02x d4038=%02x (now=%lld ms)\n",
                    ph, e7e, eb1, c987c, d340c, d340e, c4041, r40b, r40c, r40e, r40f, r18cc, r09a4,
                    s0_2, s0_9, s0_a, r4007, r4006, r0958,
                    r19c4, r19c0, r19bc, r4039, r4038, (long long)(now / 1000000));
            if (getenv("TC1797_DISPLOG")) {
                /* OSEK scheduler state: why the periodic module-dispatch task (FUN_80119cf6, d3812)
                 * isn't scheduled at phase 3. cur=current prio, maxp=highest pending, stk=prio stack,
                 * cursor=CMP0 schedule cursor (NULL => schedule dead). d3812/d37f2 low byte = actual
                 * run counts (read 1B to avoid the 4B alias artifact). */
                uint8_t d3812b=0, d37f2b=0, curp=0, maxp=0, stk=0;
                uint32_t cursor=0, dl=0;
                cpu_physical_memory_read(0xD0003812u,&d3812b,1); cpu_physical_memory_read(0xD00037F2u,&d37f2b,1);
                cpu_physical_memory_read(0xD0000064u,&curp,1);   /* current task prio */
                cpu_physical_memory_read(0xD0000984u,&maxp,1);   /* highest pending prio */
                cpu_physical_memory_read(0xD000098Cu,&stk,1);    /* prio stack top */
                cpu_physical_memory_read(0xD0012498u,&cursor,4); /* CMP0 schedule cursor */
                cpu_physical_memory_read(0xD00124A0u,&dl,4);
                fprintf(stderr, "  DISPLOG d3812=%u d37f2=%u | curprio=%u maxpend=%u priostk=%u | cursor=%08x deadline=%08x\n",
                        d3812b, d37f2b, curp, maxp, stk, cursor, dl);
            }
            /* SCHEDLOG: is the CMP0 schedule (FUN_80081870 + the watchdog kick) set up?
             * cursor DAT_d0012498 + deadline DAT_d00124a0 (silicon: 0x80060cXX / a TIM value),
             * STM CMP0/ICR (cold-init FUN_80001160 arms CMP0=TIM0+9000 + ICR.CMP0EN), and the
             * watchdog kick freshness 19c0/cnt37. Pinpoints whether only the CMP0 arm is missing
             * (cursor valid) or the whole schedule-init is absent (cursor 0). */
            if (getenv("TC1797_SCHEDLOG")) {
                uint32_t cur = 0, dl = 0, lastk = 0, tim1 = 0; uint8_t c37 = 0;
                cpu_physical_memory_read(0xD0012498u, &cur, 4);
                cpu_physical_memory_read(0xD00124A0u, &dl, 4);
                cpu_physical_memory_read(0xD00019C0u, &lastk, 4);
                cpu_physical_memory_read(0xD0004037u, &c37, 1);
                tim1 = (uint32_t)(tc1797_stm_count56(s->stm_ns_per_tick) >> 4);
                fprintf(stderr, "  SCHEDLOG cursor=%08x deadline=%08x | cmp0=%08x cmp1=%08x icr=%02x en0=%d "
                        "| last_kick=%08x tim1=%08x gap=%u cnt37=%u\n",
                        cur, dl, s->stm_cmp0, s->stm_cmp1, s->stm_icr & 0xff,
                        tc1797_stm_cmp_enabled(s, 0), lastk, tim1, tim1 - lastk, c37);
            }
            fflush(stderr);
        }
    }

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
            /* Default-on: with the systick->STM handoff the real CMP0 must not dispatch the
             * schedule ISR FUN_80081870 before the cursor (DAT_d0012498) is non-NULL, or it
             * does calli 0 and marches NOPs through segment 0. Disable via TC1797_NO_TICK_GATE. */
            gate = getenv("TC1797_NO_TICK_GATE") ? 0 : 1;
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
            g_icprof_cmp0_fires++;
        } else {
            s->stm_isrr |= 2u; s->stm_icr |= 0x20u;       /* CMP1 request + CMP1IR */
            g_icprof_cmp1_fires++;
        }
        /* BOOT-TIMELINE diag (env TC1797_TICKTL): when does CMP0 fire vs stop, and when does the CMP1
         * flood onset relative to the boot phases? Logs EVERY CMP0 fire (rare) + the cumulative CMP1
         * count once per virtual-ms (the flood rate). Reveals if the backlog forms at a discrete event
         * (counter jump) or gradually. Default-off; no effect on the boot. */
        if (getenv("TC1797_TICKTL")) {
            static uint64_t last_ms = (uint64_t)-1;
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            uint64_t ms = (uint64_t)(now / 1000000);
            uint32_t cmp_reg = ch ? s->stm_cmp1 : s->stm_cmp0;
            uint32_t tim_lo = (uint32_t)tc1797_stm_count56(s->stm_ns_per_tick);
            if (ch == 0) {
                fprintf(stderr, "TICKTL CMP0-FIRE #%u t=%llums ph=%u cmp0=%08x tim=%08x\n",
                        g_icprof_cmp0_fires, (unsigned long long)ms, ph, cmp_reg, tim_lo);
                fflush(stderr);
            } else if (ms != last_ms) {
                fprintf(stderr, "TICKTL t=%llums ph=%u cmp0f=%u cmp1f=%u cmp1=%08x tim=%08x\n",
                        (unsigned long long)ms, ph, g_icprof_cmp0_fires, g_icprof_cmp1_fires,
                        cmp_reg, tim_lo);
                fflush(stderr);
                last_ms = ms;
            }
        }
        if (getenv("TC1797_FIRERATE")) {
            static int64_t lastf[2]; static uint32_t prevcmp[2]; static unsigned frn;
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            uint32_t curcmp = ch ? s->stm_cmp1 : s->stm_cmp0;
            if (ph == 3 && frn++ < 80) {
                int64_t d = lastf[ch] ? (now - lastf[ch]) : 0;
                /* fire interval vs the cmp-register advance: if fire=2ms but
                 * advance=1ms (90000), QEMU is dropping every other CMP fire. */
                fprintf(stderr, "FIRERATE ch=%d fire_us=%lld cmp=%08x advance=%d\n",
                        ch, (long long)(d/1000), curcmp,
                        prevcmp[ch] ? (int)(curcmp - prevcmp[ch]) : 0);
                fflush(stderr);
            }
            lastf[ch] = now; prevcmp[ch] = curcmp;
        }
        uint8_t srpn = (ch ? s->stm_src1 : s->stm_src0) & 0xFFu;
        if (srpn) {
            tc1797_icu_set(s, srpn);
            raised = true;
        }
        if (g_stm_freeze_en < 0) {
            g_stm_freeze_en = getenv("TC1797_STM_FREEZE") ? 1 : 0;
        }
        if (ch == 0 && g_stm_freeze_en) {
            /* env-gated: keep the default path untouched. Cache the schedule ISR's CCPN
             * (its SRPN), the boot phase, and the CPU env for the CCPN-scoped freeze. */
            cpu_physical_memory_read(0xD0003643u, &g_stm_phase, 1);
            g_stm_sched_prio = (uint8_t)(s->stm_src0 & 0xFFu);
            g_stm_cpu_env = &s->cpu.env;
        }
        if (getenv("TC1797_STMLOG")) {
            static unsigned firen;
            if (firen++ < 80) {
                uint8_t ph = 0, e7e = 0, eb1 = 0; uint32_t c987c = 0, d2d32 = 0;
                cpu_physical_memory_read(0xD0003643u, &ph, 1);
                cpu_physical_memory_read(0xD0003E7Eu, &e7e, 1);
                cpu_physical_memory_read(0xD0003EB1u, &eb1, 1);
                cpu_physical_memory_read(0xD000987Cu, &c987c, 4);
                cpu_physical_memory_read(0xD0002D32u, &d2d32, 4);
                info_report("tc1797: STM-FIRE#%u ch=%d srpn=%u ph=%u e7e=%u eb1=%u c987c=%u d2d32=%x",
                            firen, ch, srpn, ph, e7e, eb1, c987c, d2d32 & 1);
            }
        }
        /* Advance one full window-period; the ISR's CMP write re-arms sooner. */
        uint64_t tim_now = tc1797_stm_count56(s->stm_ns_per_tick);
        uint64_t per_ticks = tc1797_stm_match_ticks(s, ch, tim_now);
        s->stm_cmp_deadline[ch] = now + (int64_t)(per_ticks * s->stm_ns_per_tick);
        /* DIAGNOSTIC (env TC1797_STMLOG): the STM systick cadence -- is it firing so fast it leaves no
         * idle window for the SRPN-1 OSEK dispatch? Logs the re-armed period per channel at phase>=3. */
        if (getenv("TC1797_STMLOG")) {
            uint8_t ph = 0;
            cpu_physical_memory_read(0xD0003643u, &ph, 1);
            if (ph >= 3 && ph < 0x40) {
                static unsigned sn;
                if (sn++ < 90) {
                    uint64_t per_ns = per_ticks * s->stm_ns_per_tick;
                    fprintf(stderr, "STMLOG ch=%d srpn=%u period=%llu ns (%llu us) cmcon=%08x now=%lld ph=%u\n",
                            ch, srpn, (unsigned long long)per_ns,
                            (unsigned long long)(per_ns / 1000u), s->stm_cmcon, (long long)now, ph);
                    fflush(stderr);
                }
            }
        }
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

/* DIAGNOSTIC (env TC1797_SAMPLE): always-on 20us env->PC sampler that does NOT inject
 * anything (unlike the osek_timer). Reveals the ~2.2ms priority-14 spin PC at phase 3
 * by oversampling it ~100x per occurrence. */
static void tc1797_sample_tick(void *opaque)
{
    TC1797SoCState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
    if (ph == 3) {
        static unsigned sn;
        if (sn++ < 6000) {
            uint32_t icr = s->cpu.env.ICR;
            fprintf(stderr, "SAMPLE %08x ccpn=%u ie=%u pipn=%u\n",
                    s->cpu.env.PC & ~0x20000000u,
                    FIELD_EX32(icr, ICR, CCPN),
                    FIELD_EX32(icr, ICR, IE_13),
                    FIELD_EX32(icr, ICR, PIPN));
            fflush(stderr);
        }
    }
    timer_mod(s->sample_timer, now + 20000);   /* 20 us */
}

/* DIAGNOSTIC heartbeat (env TC1797_HBLOG): a 5 ms always-on timer that logs the CPU PC + CCPN/IE/PIPN +
 * stm_isrr + cpu_src + phase/promote-state, so we can observe the post-~121ms freeze WITHOUT a gdb halt
 * (which perturbs the boot). Reveals whether a dispatched task is stuck at a fixed PC / high CCPN
 * (blocking the OS-tick ISR) or the OS switched tick source. No behaviour change (separate timer). */
static void tc1797_hb_tick(void *opaque)
{
    TC1797SoCState *s = opaque;
    CPUTriCoreState *env = &s->cpu.env;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    static unsigned hn;
    uint8_t ph = 0, e7e = 0, nid = 0, cid = 0;
    uint32_t tcb = 0, body = 0;
    cpu_physical_memory_read(0xD0003643u, &ph, 1);
    cpu_physical_memory_read(0xD0003E7Eu, &e7e, 1);
    cpu_physical_memory_read(0xD0000988u, &nid, 1);
    cpu_physical_memory_read(0xD0000064u, &cid, 1);
    cpu_physical_memory_read(0xD000096Cu, &tcb, 4);     /* dispatch TCB ptr DAT_d000096c */
    if (tcb >= 0x80000000u && tcb < 0x80140000u) {
        cpu_physical_memory_read((hwaddr)tcb + 0x1cu, &body, 4);  /* the calli'd body */
    }
    if (hn < 900) {
        fprintf(stderr, "HBLOG #%u pc=%08x ccpn=%u ie=%u pipn=%u stm_isrr=%x cpu_src=%08x ph=%u e7e=%u "
                "nid=%u cid=%u tcb=%08x body=%08x (now=%lld ms)\n", hn, env->PC,
                FIELD_EX32(env->ICR, ICR, CCPN), FIELD_EX32(env->ICR, ICR, IE_13),
                FIELD_EX32(env->ICR, ICR, PIPN), s->stm_isrr, s->cpu_src, ph, e7e, nid, cid, tcb, body,
                (long long)(now / 1000000));
        /* engine-I/O chain (env TC1797_HBENG): the e7e==1 phase-4 gate inputs. Donor (phase4):
         * c09=0x0a 4f6=0xe7 ba4_2=1 b52=1 efb=1. FUN_801291fa sets 4f6 bits0+2 only when
         * engine-state c09 in [0xa..0xd] AND health counter d00==0 (reset by COM event 0x17). */
        if (getenv("TC1797_HBENG") && ph == 3) {
            uint8_t c09=0,d00=0,f6=0,f5=0,ba4=0,b52=0,efb=0,eb1=0,fb7=0,ee8=0,faf=0; uint32_t w4041=0,d32=0; uint16_t s2fac=0;
            cpu_physical_memory_read(0xD0003C09u,&c09,1);
            cpu_physical_memory_read(0xC0003D00u,&d00,1);
            cpu_physical_memory_read(0xD00004F6u,&f6,1);
            cpu_physical_memory_read(0xD00004F5u,&f5,1);
            cpu_physical_memory_read(0xD0002FACu,&s2fac,2);
            cpu_physical_memory_read(0xD0003EE8u,&ee8,1);
            cpu_physical_memory_read(0xD0003BA6u,&ba4,1);   /* DAT_d0003ba4._2_1_ */
            cpu_physical_memory_read(0xD0003B52u,&b52,1);
            cpu_physical_memory_read(0xD0003EFBu,&efb,1);
            cpu_physical_memory_read(0xD0003EB1u,&eb1,1);
            cpu_physical_memory_read(0xD0003FB7u,&fb7,1);
            cpu_physical_memory_read(0xD0003FAFu,&faf,1);    /* promote window (needs ==3) */
            cpu_physical_memory_read(0xD0002D32u,&d32,4);    /* e7e 3rd condition: bit0 */
            cpu_physical_memory_read(0xD0004041u,&w4041,4);
            fprintf(stderr, "HBENG c09=%02x(0a) 2fac=%04x(7fff) ee8=%02x(04) 4f5=%02x(00) 4f6=%02x(e7) "
                    "b52=%02x ba4=%02x efb=%02x 4041=%02x fb7=%02x eb1=%02x faf=%02x d32.0=%u d00=%02x e7e=%u\n",
                    c09,s2fac,ee8,f5,f6,b52,ba4,efb,w4041&0xff,fb7,eb1,faf,d32&1,d00,e7e);
        }
        if (getenv("TC1797_MSDILOG")) {
            /* MSDI driver state: DAT_d000402f mode (0=enqueue,2=drain), the OSEK-init
             * error counters d000004b/c/d/e (FUN_8008026e fatals 0x302c-f at >5/>1), and
             * DAT_d00040b6. Reveals whether the MSDI queue ever flips to drain mode. */
            uint8_t mode=0,e4e=0,p42=0,p43=0,faa=0,faf=0;
            cpu_physical_memory_read(0xD000402Fu,&mode,1);
            cpu_physical_memory_read(0xD000004Eu,&e4e,1);
            cpu_physical_memory_read(0xD0003642u,&p42,1);  /* faa source: MSDI-drain gate (==3) */
            cpu_physical_memory_read(0xD0003643u,&p43,1);  /* master phase */
            cpu_physical_memory_read(0xD0003FAAu,&faa,1);   /* DAT_d0003faa (drain gate in FUN_800c3f6c) */
            cpu_physical_memory_read(0xD0003FAFu,&faf,1);
            uint8_t a0=0,a1c=0,a38=0,qb=0,st=0; uint32_t reqp=0;
            cpu_physical_memory_read(0xD0011AB0u,&a0,1);    /* transfer-in-progress flag, queue 0 */
            cpu_physical_memory_read(0xD0011ACCu,&a1c,1);   /* queue 1 (idx 0x1c) */
            cpu_physical_memory_read(0xD0011AE8u,&a38,1);   /* queue 2 (idx 0x38) */
            cpu_physical_memory_read(0xD00040E7u,&qb,1);    /* FUN_800e06d8 req+0x22: (>>4)=queue index */
            cpu_physical_memory_read(0xD00040C5u,&reqp,4);  /* req[0] = ptr to state byte */
            if (reqp >= 0xD0000000u && reqp < 0xD0020000u) cpu_physical_memory_read(reqp,&st,1);
            uint8_t fcsrr = (s->ssc_src[0][1] >> 13) & 1u, fcsrpn = s->ssc_src[0][1] & 0xFFu;
            uint32_t icup = (fcsrpn < 256) ? (s->icu_pending[fcsrpn >> 5] >> (fcsrpn & 31)) & 1u : 0;
            fprintf(stderr, "MSDILOG mode=%u e=%u 3643=%u | 11ab0[0]=%u q=%u state=%u | fcSRPN=%u SRR=%u icu_pending=%u ccpn=%u ie=%u (now=%lldms)\n",
                    mode,e4e,p43, a0, qb>>4, st, fcsrpn, fcsrr, icup,
                    FIELD_EX32(env->ICR, ICR, CCPN), FIELD_EX32(env->ICR, ICR, IE_13), (long long)(now/1000000));
        }
        if (getenv("TC1797_A2LOG")) {
            uint32_t pc = env->PC & ~0x20000000u;
            if (pc >= 0x800dd384u && pc < 0x800dd440u) {   /* spinning in fn_800dd384 on (*a2 & 1)==0 */
                uint32_t a2 = env->gpr_a[2]; uint8_t a2v = 0;
                const char *reg = (a2 >= 0xF0100100u && a2 <= 0xF01002FFu) ? "SSC"
                                : (a2 >= 0xD0000000u && a2 < 0xD0020000u) ? "DSPR"
                                : (a2 >= 0xF0000000u) ? "SFR" : "?";
                if (a2 >= 0xD0000000u && a2 < 0xD0020000u) cpu_physical_memory_read(a2,&a2v,1);
                fprintf(stderr, "A2LOG pc=%08x a2=%08x(%s) *a2=%02x\n", pc, a2, reg, a2v);
            }
        }
        fflush(stderr);
    }
    hn++;
    timer_mod(s->hb_timer, now + (getenv("TC1797_HBFINE") ? 200000 : 5000000));  /* 0.2ms fine / 5ms */
}

static uint64_t tc1797_stm_read(TC1797SoCState *s, uint32_t addr)
{
    if (getenv("TC1797_TASKTBL")) {
        /* The scheduler FUN_801258c8 keeps the OS busy iff some task[i][0]!=0 with
         * i>=threshold(DAT_80060f94=8). Donor active=[0,1,9]: task 9 (prio-9 master,
         * struct 0x8006141c) is ACTIVE>=8 so it's always dispatched -> OS never idles ->
         * debounce climbs. If QEMU lacks task[9][0] the OS retires task 1 -> idle-MPX ->
         * resets zero the debounce. Dump the live active set to see what's missing. */
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph == 3) {
            static unsigned tn; static uint32_t last9 = 0xDEADBEEF;
            uint32_t base = 0; cpu_physical_memory_read(0xD000998Cu, &base, 4);
            uint32_t t1 = 0, t9 = 0;
            if (base >= 0xD0000000u && base < 0xD0020000u) {
                cpu_physical_memory_read(base + 1*0x10u, &t1, 4);
                cpu_physical_memory_read(base + 9*0x10u, &t9, 4);
            }
            if ((t9 != last9 || tn < 8) && tn < 60) {
                /* build active-index list */
                char act[96]; int p = 0;
                for (int i = 0; i < 20; i++) {
                    uint32_t t0 = 0;
                    cpu_physical_memory_read(base + (uint32_t)i*0x10u, &t0, 4);
                    if (t0 != 0) p += snprintf(act+p, sizeof(act)-p, "%d ", i);
                }
                fprintf(stderr, "TASKTBL base=%08x task1=%08x task9=%08x active=[ %s]\n",
                        base, t1, t9, act);
                fflush(stderr);
                last9 = t9; tn++;
            }
        }
    }
    if (getenv("TC1797_D6F4OBS")) {
        /* OBSERVE (no write): catch d6f4[0] in the BAD state (type 0x15, bit0=0) that
         * the engine-health FUN_800e0910(0,1) reads as 0 -> a45.2 clear -> debounce reset.
         * PROMOTELOG samples too sparsely to see it; STM reads are frequent. Logs the
         * bad value so we know HOW d6f4[0] gets corrupted (which reg's data lands there). */
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph == 3) {
            uint16_t d0 = 0; cpu_physical_memory_read(0xD000D6F4u, &d0, 2);
            if (((d0 >> 9) & 0x1f) == 0x15 && (d0 & 1) == 0) {
                static unsigned bn;
                if (bn++ < 40) {
                    fprintf(stderr, "D6F4BAD d6f4[0]=%04x (reg=%02x) pc=%08x\n",
                            d0, (d0 & 0xff), (uint32_t)s->cpu.env.PC & ~0x20000000u);
                    fflush(stderr);
                }
            }
        }
    }
    if (getenv("TC1797_D6F4PIN")) {
        /* DIAGNOSTIC (env-gated, NOT a default fix): pin the d6f4[] engine-health
         * MSDI block to the donor's stable key-on-engine-off values. Confirms the
         * phase-3->4 gate chain end-to-end: d6f4[0]=0x2a33 -> FUN_800e0910(0,1)=1 ->
         * a45.2 -> debounce 340e -> 4041=1 -> eb1 -> e7e=1 -> phase 4. If phase 4 is
         * reached with this pin, d6f4 STABILITY is the sole remaining blocker and the
         * organic fix is purely the SSC/MSDI command-response model (no CAN, no data). */
        static const uint16_t donor[7] = {
            0x2a33, 0x2a25, 0x2a90, 0x2a1f, 0x2aff, 0x2a1f, 0x2aff };
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph == 3) {
            cpu_physical_memory_write(0xD000D6F4u, donor, sizeof(donor));
            /* ALSO force the engine-health bit a45.2 set directly (skip the d6f4->
             * FUN_800e0910 layer): if THIS latches 4041 then the debounce input is the
             * confirmed blocker; if not, 4041 has another gate (4f4 override / reset
             * cadence). Definitive test of the chain below a45.2. */
            if (getenv("TC1797_A45PIN")) {
                uint8_t a45 = 0; cpu_physical_memory_read(0xD0000A45u, &a45, 1);
                a45 |= 4; cpu_physical_memory_write(0xD0000A45u, &a45, 1);
            }
        }
    }
    if (getenv("TC1797_KICKCAL")) {
        /* Seed the kick-alarm period offset continuously on STM access (happens all
         * through cold-init, BEFORE the relocated alarm setup reads it) so the cyclic
         * watchdog-kick alarm is armed faster than 2.2ms. Diagnostic confirmation only.
         * Offset overridable via TC1797_KICKOFF (period = (off+2200)us * 90 ticks). */
        const char *ov = getenv("TC1797_KICKOFF");
        int32_t off = ov ? atoi(ov) : -1200;
        /* DAT_d0016ed0 is a signed SHORT (low 2 bytes); ed2/ed3 are a separate var
         * (FUN_800644a6 sets them 0xFF). Write only the 2-byte short to avoid corrupting
         * the neighbour (which produced spurious 0x302d alarm-error resets). */
        int16_t off16 = (int16_t)off;
        cpu_physical_memory_write(0xD0016ED0u, &off16, 2);
    }
    if (addr == 0xF0000214u && getenv("TC1797_KICKINT")) {
        /* Log the actual watchdog kick interval (current STM_TIM1 - DAT_d00019c0)
         * + cnt37/cnt38 at the kick body PC. This is the GROUND TRUTH for why
         * different -icount ratios trip/don't-trip the 0x29bf=10687-tick (1.9ms)
         * threshold in FUN_800c3c1a. */
        CPUTriCoreState *e = &s->cpu.env;
        if ((e->PC & ~0x20000000u) == 0x800c3c3cu) {
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            uint64_t v56k = tc1797_stm_count56(s->stm_ns_per_tick);
            uint32_t tim1 = (uint32_t)(v56k >> 4);
            uint32_t last = 0; uint8_t c37 = 0, c38 = 0;
            cpu_physical_memory_read(0xD00019C0u, &last, 4);
            cpu_physical_memory_read(0xD0004037u, &c37, 1);
            cpu_physical_memory_read(0xD0004038u, &c38, 1);
            uint32_t ival = tim1 - last;
            /* CCPN (current CPU priority = the ISR servicing this kick): SRPN-8=CMP0
             * (~0.5ms tick) vs SRPN-7=CMP1 (~2ms alarm). If the kick runs at CCPN 7
             * (CMP1/2ms) it ALWAYS exceeds the 1.9ms threshold -> wrong compare. */
            uint32_t ccpn = e->ICR & 0xFFu;
            static unsigned kn;
            if (kn++ < 80) {
                fprintf(stderr, "KICKINT ph=%u ccpn=%u ra=%08x ival=%u (%s 10687) cnt37=%u cnt38=%u\n",
                        ph, ccpn, e->gpr_a[11] & ~0x20000000u, ival,
                        ival > 0x29bf ? "OVER" : "under", c37, c38);
                fflush(stderr);
            }
        }
    }
    if (addr == 0xF0000214u && getenv("TC1797_TIMPC")) {
        /* Find the live (PSPR-relocated) watchdog kick: it reads STM_TIM1 right
         * before writing DAT_d00019c0. With -icount the PC is synced at MMIO. */
        CPUTriCoreState *e = &s->cpu.env;
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        /* Only at the kick body (0x800c3c3c, seg-A normalised): log the OSEK current
         * priority DAT_d0000984 + pending DAT_d0000990 so we see WHICH of the 3 kick
         * tasks (prio 12/14/...) is the one being dispatched in QEMU. */
        if (ph >= 2 && (e->PC & ~0x20000000u) == 0x800c3c3cu) {
            static unsigned tn;
            if (tn++ < 60) {
                uint32_t cur = 0, pend = 0;
                cpu_physical_memory_read(0xD0000984u, &cur, 4);
                cpu_physical_memory_read(0xD0000990u, &pend, 4);
                fprintf(stderr, "KICKPRIO ph=%u cur_prio=%u pend_prio=%u ra=%08x\n",
                        ph, cur, pend, e->gpr_a[11]);
                fflush(stderr);
            }
        }
    }
    static int tim0pc_en = -1;
    if (tim0pc_en < 0) tim0pc_en = getenv("TC1797_TIM0PC") ? 1 : 0;
    if (addr == 0xF0000210u && tim0pc_en) {
        /* Histogram the PC reading TIM0 -> the busy-wait/timeout that overruns a task.
         * env->PC only. TC1797_TIM0PC3: gate to phase 3 (read the phase byte every 64th
         * TIM0 read to keep perturbation low) so the master's ph3 timeout-poll isn't
         * drowned out by the ph0 post-reset delay loops. */
        static int p3 = -1;
        if (p3 < 0) { p3 = getenv("TC1797_TIM0PC3") ? 1 : 0; }
        static uint8_t cph; static unsigned long phn; int do_hist = 1;
        if (p3) {
            if ((phn++ & 0x3fu) == 0u) { cpu_physical_memory_read(0xD0003643u, &cph, 1); }
            do_hist = (cph == 3);
        }
        if (do_hist) {
            static uint32_t pcs[96]; static unsigned long pcc[96];
            static unsigned pn; static unsigned long tot;
            uint32_t pc = (uint32_t)s->cpu.env.PC;
            unsigned i;
            for (i = 0; i < pn; i++) { if (pcs[i] == pc) break; }
            if (i == pn && pn < 96) { pcs[pn++] = pc; }
            if (i < 96) { pcc[i]++; }
            if (++tot % 4000 == 0) {
                fprintf(stderr, "--- TIM0PC%s top (tot=%lu) ---\n", p3 ? "(ph3)" : "", tot);
                for (unsigned k = 0; k < pn; k++) {
                    if (pcc[k] > tot / 30) {
                        fprintf(stderr, "TIM0PC pc=%08x cnt=%lu\n", pcs[k], pcc[k]);
                    }
                }
                fflush(stderr);
            }
        }
    }
    /* DIAGNOSTIC (env TC1797_COMPC): does the high-level COM raster actually execute at
     * os_running? Logs (deduped, with count) every COM-layer PC that reads STM TIM0 -- the
     * de-mux state machine FUN_8009fc6c + the d00/signal chain all read 0xF0000210. If these
     * PCs never appear, the COM RxIndication/signal layer isn't running (the d00=0xff root). */
    {
        static int compc_en = -1;
        if (compc_en < 0) { compc_en = getenv("TC1797_COMPC") ? 1 : 0; }
        if (compc_en && addr == 0xF0000210u) {
            uint32_t pc = (uint32_t)s->cpu.env.PC & ~0x20000000u;
            int com = (pc >= 0x8009c000u && pc < 0x800a0000u)     /* FUN_8009c686/cb/fb42/fc6c/df9c */
                   || (pc >= 0x800fe000u && pc < 0x80102000u)     /* FUN_800ff434/4ea/eee + range-drivers */
                   || (pc >= 0x80120000u && pc < 0x8012a000u)     /* FUN_80129444/801291fa d00 chain */
                   || (pc >= 0x80359000u && pc < 0x8035a000u);    /* FUN_80359a30 (0x388 demux) */
            if (com) {
                static uint32_t cp[128]; static unsigned long cc[128]; static unsigned cn;
                unsigned i;
                for (i = 0; i < cn; i++) { if (cp[i] == pc) break; }
                if (i == cn && cn < 128) { cp[cn++] = pc; fprintf(stderr, "COMPC new %08x\n", pc); fflush(stderr); }
                if (i < 128) { cc[i]++; }
                static unsigned long tot;
                if (++tot % 20000 == 0) {
                    fprintf(stderr, "--- COMPC summary (tot=%lu, distinct=%u) ---\n", tot, cn);
                    for (unsigned k = 0; k < cn; k++) {
                        fprintf(stderr, "COMPC pc=%08x cnt=%lu\n", cp[k], cc[k]);
                    }
                    fflush(stderr);
                }
            }
        }
    }
    if (addr >= 0xF0000210 && addr <= 0xF000022C) {
        /* TC1797 UM 11.x: TIMk = bits[k*4+31 : k*4] (k=0..6); +0x2C = CAP =
         * bits[63:32] of the 56-bit counter. (Freeze is CCPN-scoped inside count56.) */
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
    case 0xF00002F8:                             /* STM SRC node @0xF8 (DAvE: SRC1/CMP1) */
    case 0xF00002FC: {                           /* STM SRC node @0xFC (DAvE: SRC0/CMP0) */
        /* FIDELITY FIX (env TC1797_SRC_FIX, default-off -- see below): DAvE/UM ground truth
         * is SRC0(CMP0)=0xF00002FC,
         * SRC1(CMP1)=0xF00002F8. The legacy model had these SWAPPED, which routed CMP0 to CMP1's SRPN
         * and vice-versa: the firmware's SCHEDULE compare (CMP0, SRPN7) was driven by the model's CMP1
         * (the overdue 60us FLOOD) and the alarm (CMP1, SRPN8) by CMP0's clean 100us pin -- the direct
         * cause of the os_running churn. Faithful: 0xFC->src0/CMP0 (isrr bit0), 0xF8->src1/CMP1 (isrr
         * bit1). stm_src0 stays CMP0's node so the tick routing (CMP0->stm_src0) and cmp_enabled hold. */
        static int sfix = -1;
        /* FAITHFUL DAvE map DEFAULT-ON (verified-correct SRC0=0xFC/SRC1=0xF8). Legacy swap via
         * TC1797_SRC_LEGACY. Hunting the compensating bug that costs ~1 extra reset under the faithful map. */
        if (sfix < 0) { sfix = getenv("TC1797_SRC_LEGACY") ? 0 : 1; }
        bool use_src0 = sfix ? (addr == 0xF00002FCu) : (addr == 0xF00002F8u);
        if (use_src0) {
            return (s->stm_src0 & ~0x2000u) | ((s->stm_isrr & 1u) << 13);
        }
        return (s->stm_src1 & ~0x2000u) | (((s->stm_isrr >> 1) & 1u) << 13);
    }
    default:         return (uint32_t)tc1797_stm_count56(s->stm_ns_per_tick);
    }
}

static void tc1797_stm_write(TC1797SoCState *s, uint32_t addr, uint32_t val)
{
    /* STMLOG: surface every write to the OSEK-tick registers (CMP0/CMP1/CMCON/
     * ICR/SRC0/SRC1). If the firmware arms its STM system tick we see it here;
     * silence means the tick-arming code (FUN_80001160 via FUN_8000123c) was
     * never reached -> the phase 3->4 promotion counters never advance. */
    if (getenv("TC1797_STMLOG") &&
        (addr == 0xF0000230 || addr == 0xF0000234 || addr == 0xF0000238 ||
         addr == 0xF000023C || addr == 0xF00002F8 || addr == 0xF00002FC)) {
        info_report("tc1797: STM-tick-reg write 0x%08x = 0x%08x (PC=0x%08x)",
                    addr, val, (uint32_t)s->cpu.env.PC);
    }
    switch (addr) {
    case 0xF0000230:                            /* CMP0 */
        /* DIAGNOSTIC (env TC1797_CBTIME): the schedule ISR FUN_80081870 writes CMP0
         * after each cursor callback. Log the STM_TIM0 advance since the prior CMP0
         * write = that callback's STM-time cost. The callback before a ~90000-tick
         * (1ms) jump is the ~1ms spin that doubles the schedule tick. The just-run
         * callback = *(cursor-2) = *(DAT_d0012498 - 8). */
        if (getenv("TC1797_CBTIME")) {
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            if (ph == 3) {
                static unsigned cbn; static uint32_t prevt;
                uint32_t now0 = (uint32_t)tc1797_stm_count56(s->stm_ns_per_tick);
                uint32_t cur = 0, cb = 0;
                cpu_physical_memory_read(0xD0012498u, &cur, 4);
                if (cur >= 0x80000008u) {
                    cpu_physical_memory_read((hwaddr)cur - 8, &cb, 4);
                }
                if (cbn++ < 120) {
                    fprintf(stderr, "CBTIME cb=%08x dt=%u\n", cb, now0 - prevt);
                    fflush(stderr);
                }
                prevt = now0;
            }
        }
        s->stm_cmp0 = val;
        if (getenv("TC1797_CMPWR")) {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            uint32_t tim = (uint32_t)tc1797_stm_count56(s->stm_ns_per_tick);
            int32_t sdelta = (int32_t)(val - tim);    /* >0 forward (silicon-OK), <0 overdue (artifact) */
            static unsigned fwd0, ovd0, tot0; static int32_t maxovd0;
            tot0++;
            if (sdelta >= 0) { fwd0++; } else { ovd0++; if (-sdelta > maxovd0) maxovd0 = -sdelta; }
            if (tot0 <= 30 || (tot0 % 2000) == 0) {
                fprintf(stderr, "CMPWR0 #%u t=%lldms ph=%u cmp=%08x tim=%08x sdelta=%d [fwd=%u ovd=%u maxovd=%d]\n",
                        tot0, (long long)(now / 1000000), ph, val, tim, sdelta, fwd0, ovd0, maxovd0);
                fflush(stderr);
            }
        }
        tc1797_stm_arm(s);
        break;
    case 0xF0000234:                            /* CMP1 */
        s->stm_cmp1 = val;
        if (getenv("TC1797_CMPWR")) {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            uint32_t tim = (uint32_t)tc1797_stm_count56(s->stm_ns_per_tick);
            int32_t sdelta = (int32_t)(val - tim);    /* >0 forward (silicon-OK), <0 overdue (artifact) */
            static unsigned fwd1, ovd1, tot1; static int32_t maxovd1;
            tot1++;
            if (sdelta >= 0) { fwd1++; } else { ovd1++; if (-sdelta > maxovd1) maxovd1 = -sdelta; }
            if (tot1 <= 30 || (tot1 % 2000) == 0) {
                fprintf(stderr, "CMPWR1 #%u t=%lldms ph=%u cmp=%08x tim=%08x sdelta=%d [fwd=%u ovd=%u maxovd=%d]\n",
                        tot1, (long long)(now / 1000000), ph, val, tim, sdelta, fwd1, ovd1, maxovd1);
                fflush(stderr);
            }
        }
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
    case 0xF00002F8:                            /* STM SRC node @0xF8 (DAvE: SRC1/CMP1) */
    case 0xF00002FC: {                          /* STM SRC node @0xFC (DAvE: SRC0/CMP0) */
        /* FIDELITY FIX (env TC1797_SRC_FIX): see the read handler. Faithful map: 0xFC=SRC0/CMP0,
         * 0xF8=SRC1/CMP1; legacy = swapped. CLRR acks the compare that OWNS this node. */
        static int sfix = -1;
        /* FAITHFUL DAvE map DEFAULT-ON (verified-correct SRC0=0xFC/SRC1=0xF8). Legacy swap via
         * TC1797_SRC_LEGACY. Hunting the compensating bug that costs ~1 extra reset under the faithful map. */
        if (sfix < 0) { sfix = getenv("TC1797_SRC_LEGACY") ? 0 : 1; }
        bool is_src0 = sfix ? (addr == 0xF00002FCu) : (addr == 0xF00002F8u);
        if (is_src0) {
            if (val & 0x4000u) {                /* CLRR: ack the CMP0 request */
                s->stm_isrr &= ~1u;
                s->stm_icr &= ~2u;
                if (s->stm_isrr == 0) { tc1797_stm_ack(s); }
            }
            s->stm_src0 = val & 0x00001FFFu;    /* keep SRPN/TOS/SRE config bits */
        } else {
            if (val & 0x4000u) {                /* CLRR: ack the CMP1 request */
                s->stm_isrr &= ~2u;
                s->stm_icr &= ~0x20u;
                if (s->stm_isrr == 0) { tc1797_stm_ack(s); }
            }
            s->stm_src1 = val & 0x00001FFFu;    /* keep SRPN/TOS/SRE config bits */
        }
        tc1797_stm_arm(s);                      /* enabling SRE arms the tick */
        break;
    }
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
    /* DIAGNOSTIC (env TC1797_SRPN4LOG): is the 0x6F1-diag MO100 IRQ (SRPN 4) ever presented/taken,
     * or starved by a higher pending source? Logs whenever SRPN4 is pending. */
    if (getenv("TC1797_SRPN4LOG") && (s->icu_pending[0] & (1u << 4))) {
        static unsigned s4;
        if (s4++ < 120) {
            fprintf(stderr, "SRPN4LOG pend: best=%u ccpn=%u ie=%u icu0=%08x\n",
                    best, FIELD_EX32(env->ICR, ICR, CCPN), FIELD_EX32(env->ICR, ICR, IE_13),
                    s->icu_pending[0]);
            fflush(stderr);
        }
    }
    /* DIAGNOSTIC (env TC1797_ARBLOG, no behaviour change): in the DEFAULT build the OSEK dispatch
     * (cpu_src SRPN-1) is frozen (SRR stuck). Log the arbiter's view -- best/PIPN vs CCPN/IE -- whenever
     * the dispatch is pending at phase>=3, to see why SRPN-1 is never taken (CCPN>=1? IE=0? out-arbitrated?). */
    if (getenv("TC1797_ARBLOG") && ((s->cpu_src >> 13) & 1u)) {
        uint8_t ph = 0;
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph >= 3 && ph < 0x40) {
            static unsigned an;
            if (an++ < 160) {
                fprintf(stderr, "ARBLOG best=%u ccpn=%u ie=%u icu0=%08x cpu_src=%08x phase=%u\n",
                        best, FIELD_EX32(env->ICR, ICR, CCPN), FIELD_EX32(env->ICR, ICR, IE_13),
                        s->icu_pending[0], s->cpu_src, ph);
                fflush(stderr);
            }
        }
    }
    /* Faithful TC1.3.1 ICU rule: the CPU interrupt line is asserted only when a pending request's
     * priority STRICTLY EXCEEDS the current CCPN (the same gate the CPU enforces, PIPN>CCPN). Asserting
     * HARD on any nonzero best regardless of CCPN re-presents the perpetually-pending OSEK dispatch
     * (SRPN-1) mid-drain -- while the dispatcher holds CCPN at the dispatch level -- so when the
     * dispatcher epilogue lowers CCPN one extra reschedule fires, dispatching the prio-0 guard at
     * curid=9 (silicon unwinds the curid stack to the prio-1 base and idles at nextid=0/curid=1). Gating
     * on best>CCPN closes that level-vs-edge gap (a hardware rule, not firmware magic). */
    /* Assert the CPU line whenever any source is pending; the CPU's own gate
     * (tricore_cpu_exec_interrupt: PIPN>CCPN && IE) decides when to actually take
     * it, re-checked every TB so a request out-arbitrated at high CCPN is taken as
     * soon as an RFE lowers CCPN below it. (Gating HARD on best>CCPN here instead
     * would stop that re-check and starve the low-priority OSEK dispatch.) */
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
    if (getenv("TC1797_IRQRATE") && srpn) {
        static unsigned long cnt[256]; static int64_t t0;
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph == 3) {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            if (t0 == 0) t0 = now;
            cnt[srpn]++;
            static unsigned long tot;
            if (++tot % 300 == 0) {
                int64_t span = now - t0;
                fprintf(stderr, "--- IRQRATE span=%lldus ---\n", (long long)(span/1000));
                for (int i = 1; i < 256; i++) {
                    if (cnt[i] > 3) {
                        fprintf(stderr, "IRQRATE srpn=%d cnt=%lu rate=%lld/s\n", i, cnt[i],
                                span > 0 ? (long long)(cnt[i]*1000000000LL/span) : 0);
                    }
                }
                fflush(stderr);
            }
        }
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
    /* DIAGNOSTIC (env TC1797_TAKELOG): log every SRPN-1 dispatch TAKE with the
     * firmware's nextid/curid at that instant. The idle-MPX is a dispatch taken
     * while nextid==0 (the dispatcher then derefs task[0][0]=0xFFFFFFFF). */
    if (getenv("TC1797_TAKELOG") && (uint8_t)taken == 1) {
        uint8_t nid = 0, cid = 0, ph = 0;
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph >= 3 && ph < 0x40) {
            cpu_physical_memory_read(0xD0000988u, &nid, 1);
            cpu_physical_memory_read(0xD0000064u, &cid, 1);
            static unsigned tk;
            if (tk++ < 400) {
                fprintf(stderr, "TAKELOG #%u srpn=1 nextid=%u curid=%u SRR=%u cpu_src=%08x "
                        "ccpn=%u%s\n", tk, nid, cid, (s->cpu_src >> 13) & 1u, s->cpu_src,
                        FIELD_EX32(s->cpu.env.ICR, ICR, CCPN), nid == 0 ? "  <<< IDLE-DISPATCH" : "");
                fflush(stderr);
            }
        }
    }
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

/* RFE re-arbitrate: re-present the highest pending SRN against the CCPN the CPU just
 * lowered on interrupt return (helper_rfe). Faithful TC1.3.1 continuous arbitration --
 * PIPN tracks the pending set the instant CCPN changes. Once the OS runs, the level-held
 * SRPN-1 dispatch that was enqueued while the STM tick held CCPN=8 is presented as PIPN
 * the instant this RFE drops CCPN, so the dispatcher runs in the idle gap after EVERY STM
 * ISR (~2ms) instead of only on a rare full stack-unwind (~115ms) -- which is what starved
 * the watchdog kick task into the DTC-0x3045 reset loop. */
static void tc1797_icu_rfe(void *opaque)
{
    TC1797SoCState *s = (TC1797SoCState *)opaque;
    CPUTriCoreState *env = &s->cpu.env;
    /* Find the highest still-pending SRN. The dispatch-loop hazard -- re-arbitrating the
     * firmware's continuously-held SRPN-1 from the dispatcher's OWN rfe -- is prevented
     * upstream by op_helper's rfe_old_ccpn>1 gate, so this only ever runs returning from a
     * HIGHER-priority ISR (the CCPN 7/8 STM tick). */
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
    if (!best) {
        return;                       /* nothing pending: leave HARD as-is (no spurious deassert) */
    }
    /* If the BQL is held -- the normal post-cold-init execution state, where helper_rfe runs
     * with it locked -- arbitrate INLINE: PIPN + HARD are presented now, so the RFE's own forced
     * DISAS_EXIT interrupt re-check takes the just-presented SRPN-1 OSEK dispatch IMMEDIATELY.
     * That per-tick (~2ms) delivery is what feeds the prio-1 watchdog kick (without it the kick
     * gap exceeds 5ms -> DTC-0x3045). During cold-init the BQL is transiently dropped by some
     * bring-up steps; cpu_interrupt() asserts bql_locked() and would abort there, so fall back
     * to a fresh inline PIPN write (sticks: helper_rfe is declared without TCG_CALL_NO_WG) plus
     * the between-TB irq_bh, which re-runs arbitrate with the BQL held. bql_locked() is exactly
     * the predicate cpu_interrupt() requires, so this is the faithful, crash-free split. */
    if (bql_locked()) {
        tc1797_icu_arbitrate(s);
    } else {
        env->ICR = FIELD_DP32(env->ICR, ICR, PIPN, best);
        qemu_bh_schedule(s->irq_bh);
    }
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
    /* Arbitrate INLINE when the BQL is held (the normal case: this runs from an
     * MMIO SETR store, which holds the BQL). Setting ICR.PIPN now -- rather than
     * deferring to qemu_bh_schedule(irq_bh), which fires at a HOST-dependent main-
     * loop point not tied to icount -- makes interrupt delivery deterministic: the
     * CPU takes the request at the next TB boundary (icount-deterministic) instead
     * of whenever the BH happens to run. This is the SAME proven-safe pattern as
     * tc1797_icu_rfe's bql_locked() branch, and it removes the diag-env-sensitive
     * non-determinism that raced the OSEK cooperative dispatch into the idle-MPX.
     * Fall back to the deferred BH only on the rare BQL-unlocked cold-init path
     * (cpu_interrupt asserts bql_locked()). */
    if (bql_locked()) {
        tc1797_icu_arbitrate(s);            /* sets PIPN + asserts HARD */
    } else {
        qemu_bh_schedule(s->irq_bh);
        cpu_interrupt(CPU(&s->cpu), CPU_INTERRUPT_HARD);
    }
}

/*
 * PCP -> CPU: a channel program that EXITs with INT=1 raises the matching CPU
 * service request. Invoked from the PCP worker thread while it holds the BQL,
 * so it may poke the SoC IRQ path directly (tc1797_raise_srpn defers the actual
 * ICR write to a bottom half).
 */
static void tc1797_pcp_irq(void *opaque, uint8_t srpn)
{
    if (getenv("TC1797_PCPIRQ")) {
        static unsigned pn;
        if (pn++ < 400) {
            fprintf(stderr, "PCPIRQ srpn=%u(0x%x)\n", srpn, srpn);
            fflush(stderr);
        }
    }
    tc1797_raise_srpn((TC1797SoCState *)opaque, srpn);
}

/*
 * GPTA capture-cell service request -> route by the SRN's TOS bit, exactly like
 * a software SETR on a GPTA SRC node (tc1797_src_node_write): to_pcp -> trigger
 * the PCP channel for this SRPN (engine-position math runs on the PCP thread);
 * else raise a CPU interrupt at this SRPN. This is the cell-event sink the GPTA
 * model calls once it has looked up the cell's firmware-armed SRC node.
 */
static void tc1797_gpta_route(void *opaque, uint8_t srpn, bool to_pcp)
{
    TC1797SoCState *s = opaque;
    if (srpn == 0) {
        return;
    }
    if (srpn == 19u) {       /* crank/cam engine-position node: confirm captures reach PCP ch19 */
        static int erl = -1; static unsigned ec;
        if (erl < 0) { erl = getenv("TC1797_ENGINELOG") ? 1 : 0; }
        if (erl && (ec++ % 58u) == 0u && ec < 58u * 4000u) {
            fprintf(stderr, "ENGROUTE srpn=19 to_pcp=%d pcp_en=%d n=%u\n",
                    to_pcp, s->pcp_enabled, ec);
            fflush(stderr);
        }
    }
    if (to_pcp) {
        if (s->pcp_enabled) {
            pcp_engine_trigger(&s->pcp, srpn);
        }
    } else {
        tc1797_raise_srpn(s, srpn);
    }
}

/*
 * Engine trigger-wheel generator (env TC1797_ENGINE). Models the 60-2 crankshaft sensor wheel and
 * the camshaft sensor as GPTA LTCA2 capture inputs -- exactly the path the firmware's PCP channel-19
 * engine-position math consumes. Each timer tick advances one 60th of a crank revolution: a present
 * tooth latches the live GTTIM0 value into the crank cell's LTCXR and pulses SRPN 19 to PCP ch19;
 * the 2-tooth gap (positions 58-59) is skipped so the firmware sees the 3x-period reference and
 * finds TDC. The cam cell fires once per cam revolution (every 2 crank revs), later offset by the
 * live VANOS phase (closed loop). Commanded RPM = env TC1797_ENGINE_RPM (default 800 idle).
 */
static uint32_t tc1797_engine_gttim0(TC1797SoCState *s, int64_t now)
{
    /* The crank LTCXR latches GPTA0 GTTIM0 (slot 0). Use the firmware-seeded timer if running so the
     * captured tooth timestamps share the firmware's clock frame; else our own 399 ns rate. */
    if (s->gpta.gt_run[0]) {
        return (uint32_t)((s->gpta.gt_base[0]
                + (uint32_t)((now - (int64_t)s->gpta.gt_t0_ns[0]) / 399)) & 0x00FFFFFFu);
    }
    return (uint32_t)(((now - s->eng_t0_ns) / 399) & 0x00FFFFFFu);
}

/* xorshift64 -> [0,1). Models physical combustion/sensor noise; intentionally non-deterministic
 * (the crank wheel is an external input, not part of the icount-deterministic guest execution). */
static inline double tc1797_engine_rand(TC1797SoCState *s)
{
    uint64_t x = s->eng_rng ? s->eng_rng : 0x9E3779B97F4A7C15ull;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    s->eng_rng = x;
    return (double)(x >> 11) * (1.0 / 9007199254740992.0);   /* / 2^53 -> [0,1) */
}

static void tc1797_engine_tick(void *opaque)
{
    TC1797SoCState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    double rpm = s->eng_rpm;
    if (rpm < 1.0) {                          /* stalled: idle re-check */
        timer_mod(s->engine_timer, now + 5000000);
        return;
    }
    /* 60 wheel positions per crank revolution -> position period = 1e9 / rpm ns
     * (rpm rev/min => rpm/60 rev/s => x60 positions => rpm positions/s). */
    int64_t tooth_ns = (int64_t)(1.0e9 / rpm);
    if (tooth_ns < 1000) {
        tooth_ns = 1000;                      /* clamp absurd RPM */
    }

    /* Per-cylinder crank variance: locate the 720deg-cycle combustion segment (6 of 120deg each)
     * and apply that cylinder's modifier (steady scale + random jitter + random misfire) to this
     * tooth's period. A slow segment = weak/absent combustion torque -- exactly the crank-speed
     * signature the DME's misfire detector looks for. */
    unsigned cyc_pos = ((unsigned)(s->eng_rev & 1u)) * 60u + s->eng_tooth;  /* 0..119 over 720deg */
    unsigned seg = (cyc_pos / 20u) % 6u;
    unsigned ci  = (s->eng_firing[seg] >= 1u && s->eng_firing[seg] <= 6u)
                   ? (unsigned)(s->eng_firing[seg] - 1u) : 0u;
    if ((cyc_pos % 20u) == 0u) {              /* segment start: roll this cylinder's random misfire */
        s->cyl_miss_now[ci] = (tc1797_engine_rand(s) < s->cyl_miss_prob[ci]) ? 1u : 0u;
    }
    double scale = s->cyl_scale[ci] > 0.0 ? s->cyl_scale[ci] : 1.0;
    if (s->cyl_miss_now[ci]) {
        scale *= 1.30;                        /* a missed combustion this cycle: ~30% slower segment */
    }
    if (s->cyl_jitter[ci] > 0.0) {
        scale *= 1.0 + (tc1797_engine_rand(s) * 2.0 - 1.0) * s->cyl_jitter[ci];
    }
    if (scale < 0.1) {
        scale = 0.1;
    }
    tooth_ns = (int64_t)((double)tooth_ns * scale);

    uint32_t gttim = tc1797_engine_gttim0(s, now);
    bool present = (s->eng_tooth < 58);       /* 60-2: positions 58,59 are the missing-tooth gap */
    if (present) {
        tc1797_gpta_capture(&s->gpta,
            GPTA_LTCA2_OFF + GPTA_LTC_OFF + GPTA_CRANK_CELL * 8u + 4u, gttim);
    }
    /* Closed-loop VANOS: slew the modelled cam phase toward the DME's commanded duty (read back from
     * the L9959 register the DME wrote over SSC1), then fire the cam-position-sensor capture at
     * (crank reference + cam phase). The DME measures the cam-vs-crank offset = the phase, compares
     * to its target, and adjusts the duty -- closing the loop through this plant. One pulse / cam rev
     * (= 2 crank revs). */
    {
        double duty   = (double)s->l9959_reg[s->vanos_reg & 0xFFu] / 255.0;     /* 0..1 */
        double target = duty * s->vanos_max_deg;                               /* crank-deg advance */
        double dt     = (double)tooth_ns / 1.0e9;
        double a      = dt / (s->vanos_tau_s + dt);                            /* 1st-order hydraulic lag */
        s->cam_phase_deg[0] += (target - s->cam_phase_deg[0]) * a;
        int cam_pos = (int)(s->cam_phase_deg[0] / 6.0);                        /* 6 deg per wheel position */
        cam_pos %= 120;
        if (cam_pos < 0) {
            cam_pos += 120;
        }
        if ((int)cyc_pos == cam_pos) {
            tc1797_gpta_capture(&s->gpta,
                GPTA_LTCA2_OFF + GPTA_LTC_OFF + GPTA_CAM_CELL * 8u + 4u, gttim);
            static int vl = -1; static unsigned vc;
            if (vl < 0) {
                vl = getenv("TC1797_ENGINELOG") ? 1 : 0;
            }
            if (vl && vc++ < 2000) {
                fprintf(stderr, "VANOS duty=%.2f (reg0x%02x=%u) target=%.1f cam_phase=%.1fdeg pos=%d\n",
                        duty, s->vanos_reg & 0xFFu, s->l9959_reg[s->vanos_reg & 0xFFu],
                        target, s->cam_phase_deg[0], cam_pos);
                fflush(stderr);
            }
        }
    }

    {
        static int elog = -1; static unsigned el;
        if (elog < 0) {
            elog = getenv("TC1797_ENGINELOG") ? 1 : 0;
        }
        /* Log once per combustion segment (mid-segment) so per-cylinder variance is visible. */
        if (elog && (cyc_pos % 20u) == 10u && el++ < 6u * 2000u) {
            fprintf(stderr, "ENG rpm=%.0f rev=%llu seg=%u cyl=%u scale=%.3f miss=%u gttim=0x%06x\n",
                    rpm, (unsigned long long)s->eng_rev, seg, ci + 1u, scale,
                    s->cyl_miss_now[ci], gttim);
            fflush(stderr);
        }
    }

    if (++s->eng_tooth >= 60u) {
        s->eng_tooth = 0;
        s->eng_rev++;
    }
    timer_mod(s->engine_timer, now + tooth_ns);
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
    if (getenv("TC1797_CANRXIRQ")) {
        static unsigned ri;
        if (ri++ < 60) {
            fprintf(stderr, "CANRXIRQ mo=%d id=0x%03x moipr=%08x srpn=%u\n",
                    mo, s->can.mo[mo].id, moipr, srpn);
            fflush(stderr);
        }
    }
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
            /* PFLASH writes update the backing (data reads see them) but do NOT
             * invalidate cached translation blocks, so a code patch via the tunnel
             * has no effect on execution. TC1797_POKE_TBFLUSH flushes TBs so poked
             * code takes effect (used to verify firmware-path hypotheses). */
            if (getenv("TC1797_POKE_TBFLUSH")) {
                queue_tb_flush(CPU(&s->cpu));
            }
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
        /* SRR is now clear (CLRR), or the node is disabled. Sync the arbiter's
         * pending bit with the SRR: a software-cleared request must stop being
         * presented. This is the faithful TC1.3.1 behaviour -- the SRN's request
         * to the ICU IS its SRR -- and is load-bearing for the OSEK dispatcher,
         * which CLRRs its own SRPN-1 service node on every scheduling pass. Without
         * it icu_pending[1] survives the CLRR and is re-presented at the
         * dispatcher's epilogue, forcing a spurious extra reschedule so the curid
         * stack never unwinds to the prio-1 idle base (the phase-3->4 wedge). */
        if (!srr) {
            tc1797_icu_clr(s, *node & 0xFFu);
            tc1797_icu_arbitrate(s);
        }
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
        if ((*node & 0xFFu) == 0x0Fu && getenv("TC1797_SRPN15LOG")) {  /* GPTA-init phase-machine channel */
            extern bool g_pcp_in_exec;
            static unsigned n15;
            if (n15++ < 80) {
                uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
                uint8_t f = 0; cpu_physical_memory_read(0xD000416Fu, &f, 1);
                fprintf(stderr, "SRPN15 @node=%08x val=%08x ph=%u 416f=%u %s PC=%08x t=%lld\n",
                        addr, *node, ph, f, g_pcp_in_exec ? "PCP" : "CPU",
                        (uint32_t)s->cpu.env.PC & ~0x20000000u,
                        (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
                fflush(stderr);
            }
        }
        if (s->pcp_enabled) {
            pcp_engine_trigger(&s->pcp, *node & 0xFFu);
        }
        /* MSC ring-completion node: SRR auto-clears on PCP service entry (silicon HW), so
         * the MSDI driver sees the request serviced and re-SETRs the node per ring frame.
         * Without this it sees a permanently-set SRR, re-arms the node (writes 0x4428,
         * SRE=0) and the completion stops delivering after one frame -> ring stalls.
         * Scoped to the MSC SRC region so SSC/ADC PCP nodes are unaffected. */
        if ((addr & 0xFFFFFF00u) == 0xF0000800u || (addr & 0xFFFFFF00u) == 0xF0000900u) {
            *node &= ~(1u << 13);
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

/* DIAGNOSTIC (env TC1797_FREEZEDUMP): a WALL-CLOCK timer that fires regardless of guest
 * virtual time, so it catches the phase-3 freeze (where the guest halts/spins and virtual
 * time stops). Dumps the prio-level fn-list runner FUN_80124512 state: nextid, the fn-list
 * pointer task[nextid][1] (post-incremented past the LAST calli'd fn), so last_fn = the
 * task that is hung (never returned/retired). */
static void tc1797_freeze_dump(void *opaque)
{
    TC1797SoCState *s = opaque;
    CPUTriCoreState *env = &s->cpu.env;
    uint8_t ph = 0;
    cpu_physical_memory_read(0xD0003643u, &ph, 1);
    if (ph == 3) {
        /* Phase-3 PC histogram (wall-clock sampled, so it captures the overrun even when
         * guest virtual time crawls). The hottest NON-idle PC = the master sub-function
         * that poll-waits/overruns the watchdog kick window. */
        static uint32_t pcs[160]; static unsigned long pcc[160]; static unsigned char ccp[160];
        static unsigned pn; static unsigned long tot;
        uint32_t pc = (uint32_t)env->PC & ~0x20000000u;
        unsigned cc = env->ICR & 0xffu;
        unsigned i;
        for (i = 0; i < pn; i++) { if (pcs[i] == pc) break; }
        if (i == pn && pn < 160) { pcs[pn] = pc; ccp[pn] = (unsigned char)cc; pn++; }
        if (i < 160) { pcc[i]++; }
        if (++tot % 60 == 0) {
            uint8_t cnt37 = 0; uint32_t lastk = 0, tim1 = 0; uint16_t d340c = 0, d987c = 0;
            uint32_t deadline = 0, cursor = 0, mbc = 0, c987 = 0;
            cpu_physical_memory_read(0xD0004037u, &cnt37, 1);
            cpu_physical_memory_read(0xD00019C0u, &lastk, 4);
            cpu_physical_memory_read(0xD000340Cu, &d340c, 2);
            cpu_physical_memory_read(0xD000987Cu, &c987, 4);
            cpu_physical_memory_read(0xD00124A0u, &deadline, 4);   /* cursor accumulated deadline */
            cpu_physical_memory_read(0xD0012498u, &cursor, 4);     /* cursor pointer */
            cpu_physical_memory_read(0xD00019BCu, &mbc, 4);        /* monitor last-run TIM1 */
            tim1 = (uint32_t)(tc1797_stm_count56(s->stm_ns_per_tick) >> 4);
            uint32_t tim0 = (uint32_t)tc1797_stm_count56(s->stm_ns_per_tick);
            (void)d987c;
            fprintf(stderr, "FREEZEW tot=%lu 340c=%u 987c=%u cnt37=%u | kick(t1-lastk)=%u "
                    "MON(t1-019bc)=%u(reset>0x1a17b) | cur=%08x dl-now=%d\n",
                    tot, d340c, c987, cnt37, tim1 - lastk, tim1 - mbc, cursor, (int)(deadline - tim0));
            for (unsigned k = 0; k < pn; k++) {
                if (pcc[k] > tot / 25) {
                    fprintf(stderr, "FREEZEHIST pc=%08x ccpn=%u cnt=%lu\n",
                            pcs[k], ccp[k], pcc[k]);
                }
            }
            fflush(stderr);
        }
    }
    timer_mod(s->freeze_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 2000000);  /* 2ms wall */
}

static void tc1797_osek_tick(void *opaque)
{
    TC1797SoCState *s = opaque;
    CPUTriCoreState *env = &s->cpu.env;
    CPUState *cs = CPU(&s->cpu);
    uint32_t ie   = FIELD_EX32(env->ICR, ICR, IE_13);
    uint32_t ccpn = FIELD_EX32(env->ICR, ICR, CCPN);
    bool raised = false;

    /* HANDOFF to the firmware's own STM tick (default-on; disable via TC1797_NO_HANDOFF).
     *
     * The forced systick is a BOOTSTRAP crutch: the firmware programs its OSEK system tick
     * on STM CMP0 (100us, the schedule + watchdog kick) and CMP1 (1ms, the master schedule)
     * in the cold-init FUN_80001160 (ICR.CMP0EN/CMP1EN | 0x51, CMP0=TIM0+9000, CMP1=TIM0+
     * 90000), but that cold-init is only reached ~120ms into boot. Until then nothing drives
     * the OS, so the forced systick raises SRPN 1/7/8 to advance the boot to the point the
     * cold-init runs. Once the cold-init has armed+enabled the real STM compare, the hardware
     * tick (tc1797_stm_tick) drives SRPN-8/7 at the firmware's exact programmed deadlines --
     * EXACTLY as on silicon. Continuing to ALSO run the forced systick from here double-drives
     * the schedule cursor and the watchdog kick: the kick interval jitters past the 1.9ms
     * window, cnt37 climbs past 9, and FUN_800c3c68 software-resets (DTC-0x3045) every ~58ms
     * (the 6x watchdog reset loop). Retire the crutch the moment the firmware's tick is live.
     * A SoC reset clears ICR (cold-init must re-run) and re-arms this timer (see reset hook),
     * so the bootstrap restarts cleanly each boot. */
    bool tick_handed_off = false;
    {
        static int no_handoff = -1;
        if (no_handoff < 0) { no_handoff = getenv("TC1797_NO_HANDOFF") ? 1 : 0; }
        if (!no_handoff &&
            (tc1797_stm_cmp_enabled(s, 0) || tc1797_stm_cmp_enabled(s, 1))) {
            /* PARTIAL handoff: the real STM tick now drives the periodic SRPN-8/7 schedule,
             * so stop raising the forced tick SRPNs 1/7/8 below (double-driving them jitters
             * the watchdog kick past 1.9ms -> the DTC-0x3045 reset loop). But KEEP delivering
             * the OSEK event-dispatch node (cpu_src 0xF7E0FFFC / SRPN-1): the firmware sets SRR
             * on it to request event-task switches -- e.g. the COM task chain that refreshes the
             * secondary watchdog timestamp 0xD00019C4 (FUN_800c3c68's 0x3047 300ms check) and
             * 0xD00019BC (0x3046 19ms check). QEMU's SRC node does not self-raise that request,
             * so retiring the forced timer entirely starves those refreshers -> DTC-0x3047 every
             * ~300ms. So skip only the tick raise; still run the cpu_src dispatch + re-arm. */
            tick_handed_off = true;
        }
    }

    /* DIAGNOSTIC (env TC1797_FIRECNT): FULLY non-perturbing osek-timer fire counter -- NO
     * cpu_physical_memory_read at all (only the fire count + virtual clock). Tells us
     * whether the osek_timer keeps firing past the ~133ms phase-3 freeze (CPU spin not
     * preempted) or STOPS firing (QEMU/timer-level stall). Also logs ie/ccpn/PC (env-side,
     * non-perturbing) so we see the CPU state at the freeze. */
    {
        static int fc = -1;
        if (fc < 0) { fc = getenv("TC1797_FIRECNT") ? 1 : 0; }
        if (fc) {
            /* Histogram the interrupted CPU PC when a TASK is running (ccpn>=8) -- this
             * is sampled by virtual time (the osek timer), so it captures the master's
             * spin where wall-clock sampling could not. The hot ccpn>=8 PC = the
             * peripheral poll-wait that overruns the master. */
            {
                static uint32_t pcs[128]; static unsigned long pcc[128]; static unsigned char ccs[128];
                static unsigned pn; static unsigned long tot;
                uint32_t pc = (uint32_t)env->PC & ~0x20000000u;
                unsigned i;
                for (i = 0; i < pn; i++) { if (pcs[i] == pc) break; }
                if (i == pn && pn < 128) { pcs[pn] = pc; ccs[pn] = (unsigned char)ccpn; pn++; }
                if (i < 128) { pcc[i]++; }
                if (++tot % 200 == 0) {
                    fprintf(stderr, "--- FIRECNT task-PC hist (tot=%lu) top ---\n", tot);
                    for (unsigned k = 0; k < pn; k++) {
                        if (pcc[k] > tot / 15) {
                            fprintf(stderr, "TASKPC pc=%08x ccpn=%u cnt=%lu\n",
                                    pcs[k], ccs[k], pcc[k]);
                        }
                    }
                    fflush(stderr);
                }
            }
        }
    }

    /* DIAGNOSTIC (env TC1797_OSEKCNT): at phase 3, every 300th osek_tick fire, log the
     * schedule-drive dynamics over time -- fires, raises (srpn>ccpn delivered), the
     * current ccpn/ie, the cpu_src SRR (dispatch request, bit13), and the periodic-task
     * progress (340c debounce, 987c master counter, curid). Reveals whether the tick
     * STOPS firing/raising, or keeps firing but the periodic tasks stop being dispatched. */
    {
        static int oc = -1;
        if (oc < 0) { oc = getenv("TC1797_OSEKCNT") ? 1 : 0; }
        if (oc) {
            static unsigned long fires;
            fires++;
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            if (ph == 3 && (fires % 50u) == 0u) {
                uint16_t d340c = 0; uint8_t d340e = 0, curid = 0; uint32_t c987c = 0;
                cpu_physical_memory_read(0xD000340Cu, &d340c, 2);
                cpu_physical_memory_read(0xD000340Eu, &d340e, 1);
                cpu_physical_memory_read(0xD0000064u, &curid, 1);
                cpu_physical_memory_read(0xD000987Cu, &c987c, 4);
                fprintf(stderr, "OSEKCNT fires=%lu | ccpn=%u ie=%u SRR=%u ready_gate(BIV=%u) | "
                        "340c=%u 340e=%u 987c=%u curid=%u (now=%lldms)\n",
                        fires, ccpn, ie, (s->cpu_src >> 13) & 1u, env->BIV != 0,
                        d340c, d340e, c987c, curid,
                        (long long)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000000));
                fflush(stderr);
            }
        }
    }

    /* DIAGNOSTIC (env TC1797_POLLHIST): sample the interrupted PC every osek_tick
     * (~333us) at phase 3. The schedule lags to 2ms because per-fire work poll-waits
     * ~1ms of STM-time (CPU-independent); this finer sampler reveals the spin PC. */
    if (getenv("TC1797_POLLHIST")) {
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        static unsigned qn;
        if (qn++ < 4000) {
            fprintf(stderr, "POLLHIST %08x ccpn=%u ph=%u\n",
                    env->PC & ~0x20000000u, ccpn, ph);
            fflush(stderr);
        }
    }

    /* DIAGNOSTIC (env TC1797_MAXPHASE): the osek_timer is always-on (when systick
     * enabled) and fires on the virtual clock independent of the firmware arming
     * STM CMP0, so it gives a reliable free-run phase measurement (the stm_tick
     * logger goes dark pre-CMP0-config). Log each new max phase reached. */
    if (getenv("TC1797_MAXPHASE")) {
        static uint8_t mx; static unsigned fc;
        uint8_t ph = 0;
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph < 0x40 && ph > mx) {
            uint8_t osr = 0;
            cpu_physical_memory_read(0xD000400Fu, &osr, 1);
            mx = ph;
            fprintf(stderr, "MAXPHASE(osek) new-max=%u os_running=%u (now=%lldms)\n",
                    ph, osr, (long long)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000000));
            fflush(stderr);
        }
        /* Non-perturbing free-run PC sampler: where is the cold-init spinning? */
        if ((fc++ % 300) == 0) {
            fprintf(stderr, "PCSAMPLE pc=0x%08x phase=%u ccpn=%u ie=%u (now=%lldms)\n",
                    (uint32_t)env->PC, ph, ccpn, ie,
                    (long long)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000000));
            fflush(stderr);
        }
    }

    /* DIAGNOSTIC (env TC1797_DISPLOG, no behaviour change): why the SRPN-1 OSEK
     * dispatch (cpu_src @0xF7E0FFFC) never switches curid->nextid at phase 3.
     * Logs CCPN/IE + the dispatch node SRR + the kernel's nextid/curid each fire. */
    if (getenv("TC1797_DISPLOG")) {
        uint8_t nid = 0, cid = 0, ph = 0;
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        if (ph >= 3 && ph < 0x40) {        /* only the phase-3+ window */
            static unsigned dn;
            if (dn++ < 240) {
                cpu_physical_memory_read(0xD0000988u, &nid, 1);
                cpu_physical_memory_read(0xD0000064u, &cid, 1);
                fprintf(stderr, "DISPLOG #%u ccpn=%u ie=%u pipn=%u SRR=%u cpu_src=%08x "
                        "nextid=%u curid=%u phase=%u\n", dn, ccpn, ie,
                        FIELD_EX32(env->ICR, ICR, PIPN), (s->cpu_src >> 13) & 1u,
                        s->cpu_src, nid, cid, ph);
                fflush(stderr);
            }
        }
    }

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

    /* Deliver one round-robin systick when ready -- UNLESS the firmware's real STM tick has
     * taken over (partial handoff): then the hardware CMP0/CMP1 drive the schedule and we must
     * not double-drive it here. The cpu_src OSEK event dispatch below still runs. */
    if (ready && !tick_handed_off) {
        uint8_t srpn = tc1797_osek_tick_prios[s->osek_rr
                                              % ARRAY_SIZE(tc1797_osek_tick_prios)];
        s->osek_rr++;
        /* DIAGNOSTIC (env TC1797_FORCE_CMP1_OFF): the schedule-table tick is the STM CMP1
         * (SRPN-7), which the firmware ARMS itself in FUN_80081870 (DAT_f0000230 = next
         * deadline). The forced systick ALSO raising SRPN-7 every ~1ms double-drives the
         * schedule cursor and adds jitter (the >1.9ms tick gaps that trip the watchdog).
         * Suppress the forced SRPN-7 so only the real STM CMP1 (tc1797_stm_tick) walks the
         * cursor at the programmed 90000-tick (=1ms@90MHz) cadence. */
        static int no_cmp1 = -1;
        if (no_cmp1 < 0) {
            no_cmp1 = getenv("TC1797_FORCE_CMP1_OFF") ? 1 : 0;
        }
        if (srpn == 7 && no_cmp1) {
            srpn = 0;   /* not raised (0 is never > ccpn); real STM CMP1 drives the schedule */
        }
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
    if (!getenv("TC1797_EVENTDISP")
        && !raised && ready && ((s->cpu_src >> 13) & 1u)) {
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

/*
 * SC900685 MSDI register-command -> reply frame. The command's high byte (halfword
 * mode, cmd 0xRR00) or low byte (byte mode) selects a switch register; the chip
 * answers 0x2a00 | switch_state, or the bare 0x2a status tag for a NOP/clock (cmd 0).
 * sw[] is the SC900685 switch state at KEY-ON-ENGINE-OFF, RE'd from the donor snapshot.
 */
static uint16_t tc1797_msdi_reply(uint32_t cmd)
{
    static const uint8_t sw[256] = {
        [0xC0] = 0xA4, [0xC2] = 0x29, [0xC6] = 0x3F, [0xC8] = 0x33,
        [0xCA] = 0x25, [0xCC] = 0x2F, [0xCE] = 0xC0, [0xE4] = 0x1F,
        [0xE8] = 0x16, [0xEA] = 0xC4, [0xF6] = 0x90, [0xFC] = 0xFF,
        /* SC900685 SSC-driver power-up handshake (FUN_80111d00): the driver clocks cmd
         * 0x04/0x06 (and 0x02 for the 0x7a frame) and requires the device to answer with
         * the announce frames 0x2adf / 0x2a03 / 0x2a7a (donor data bufs @0xD00037c0/37ba).
         * Without these the SSC channel state DAT_d000cdf4 never leaves 0 and the whole MSDI
         * monitor sequencer (->0x302f) stalls. RE'd from the donor handshake response. */
        [0x04] = 0xDF, [0x06] = 0x03, [0x02] = 0x7A,
        /* SC900685 config read-back shadow registers (the bring-up FUN_80111cce reads these AFTER
         * writing config 0x30cf/0x38ff): reg 0x10 -> uVar6 (gate (uVar6&0x30)==0 needs 0xcf),
         * reg 0x18 -> uVar7 (gate (uVar7&0x10)!=0 needs bit4). Donor: uVar6=0x2acf, uVar7=0x2aff.
         * The 0xcf/0xff are the device's read-back of the 0x30cf/0x38ff config the host wrote. */
        [0x10] = 0xCF, [0x18] = 0xFF,
    };
    uint8_t reg = (cmd >> 8) & 0xFFu;
    if (reg == 0) {
        reg = cmd & 0xFFu;
    }
    if (reg == 0) {
        return 0x2Au;                        /* NOP/clock -> bare status tag */
    }
    if (sw[reg] == 0 && getenv("TC1797_REGECHO")) {
        return 0x2A00u | reg;                /* diag: unmapped regs echo their reg byte to reveal buffer source */
    }
    return 0x2A00u | (sw[reg] ? sw[reg] : 0xFFu);
}

/* RBUF-latch model (env TC1797_MSDI_RBUF): hold the received frame in a latch updated only at
 * frame COMPLETION (the ssc_done_timer), instead of recomputing a fresh reply on every RB read.
 * Fixes the bank-read off-by-one: the firmware was storing the stale pre-completion frame (the
 * model answered every read), shifting the block by one and dropping the last frame. */
static int tc1797_msdi_rbuf_mode(void)
{
    static int v = -1;
    if (v < 0) { v = getenv("TC1797_MSDI_RBUF") ? 1 : 0; }
    return v;
}

static uint64_t tc1797_ssc_read(TC1797SoCState *s, int unit, uint32_t off)
{
    switch (off) {
    case 0x08: return 0x00004500;            /* ID: MOD=0x45 (SSC) */
    case 0x0C: return 0x10000000;            /* FDR reset-ish (baud) */
    case 0x20: return s->ssc_tb[unit];       /* TB readback */
    case 0x24: {                             /* RB: SPI receive */
        uint32_t tb = s->ssc_tb[unit];
        /*
         * SSC0 = SC900685 MSDI (Multiple Switch Detection Interface). The firmware
         * clocks a command/NOP stream and reads back the switch-status frames it
         * builds into the coding/diagnostic blocks the validator FUN_800e006e checks
         * (each channel must be type (hw>>9)&0x1f==0x15, i.e. high byte 0x2a). The MSDI
         * returns its 0x2a status tag for the NOP (0x00) clock bytes and a switch-state
         * byte for a register command. The old loopback returned 0 for the NOP -> the
         * channels never carried the 0x2a tag -> validation failed -> phase stalled at 2.
         * Faithful minimal model: NOP -> 0x2a status; register read -> switch byte
         * (0xFF = idle/open per the SC900685 pull-ups at key-on-engine-off; sets bit0
         * so the d6f4 chain reads "present"). SSC1 (TLE7183F/L9959 drivers) keeps loopback.
         */
        if (unit == 0) {
            /*
             * SC900685 MSDI (Multiple Switch Detection Interface) register model. The
             * firmware (FUN_8011a800 mode-2) clocks command words out and reads the
             * switch-status frames into the coding/diagnostic blocks the validator
             * FUN_800e006e checks (each channel must be type (hw>>9)&0x1f==0x15, i.e.
             * high byte 0x2a). The MSDI answers each register-select command with a
             * 0x2a-status frame carrying the switch-state byte, and a NOP/clock byte
             * (0x00) with the bare 0x2a tag. The register-select byte is the command's
             * high byte (halfword mode, cmd 0xRR00) or low byte (byte mode, cmd 0xRR).
             *
             * sw[] is the SC900685's switch registers at KEY-ON-ENGINE-OFF, reverse-
             * engineered from the real-ECU donor snapshot (the exact frames the firmware
             * built: c8->0x2a33, ca->0x2a25, f6->0x2a90, e4->0x2a1f, fc->0x2aff, ...).
             * Unmapped registers default to 0x2a00 (valid frame, switch idle). The old
             * loopback echoed the command back -> channels never carried the 0x2a tag ->
             * the coding validation failed and the boot stalled at phase 2.
             */
            uint32_t ptb = s->ssc_prev[unit];    /* reply is to the PRIOR command */
            uint8_t reg = (ptb >> 8) & 0xFFu;
            if (reg == 0) {
                reg = ptb & 0xFFu;
            }
            /* register-select -> 0x2a-status switch frame (reply to the prior command).
             * Unmapped registers default to the 0xFF pull-up rest state (key-on-engine-off);
             * returning 0x00 there dropped bit0 of the d6f4 chain on every unmapped read.
             * RBUF mode returns the frame latched at the last completion (no stale-frame store). */
            uint32_t rep = tc1797_msdi_rbuf_mode()
                         ? s->ssc_rbuf[unit]
                         : tc1797_msdi_reply(ptb);
            /*
             * Faithful stateful SC900685 model (env TC1797_MSDI_STATEFUL, default OFF).
             * The static sw[] table is the channel-B (bank2/f0-selected) switch state, so
             * bank1 (channel-A/d4-selected) reads the wrong values AND never satisfies the
             * monitor's bank1 gate (FUN_8008034a: block[3]=reply(e8) must have &0x3f==1).
             * The donor dumps give the per-channel KEY-ON-ENGINE-OFF values; the bank1 gate
             * is a device SYNC handshake: the SC900685 reports a "synced/frame-ready" status
             * (payload 1) on its status register until the host has read the bank and moved
             * on, then reports the steady switch state. We track the channel from the d4/f0
             * select commands and model both. (The sync value is derived from the firmware's
             * protocol -- the gate IS the "device synced?" check -- not a datasheet capture.)
             */
            {
                static int st = -1;
                if (st < 0) { st = getenv("TC1797_MSDI_STATEFUL_OFF") ? 0 : 1; }  /* faithful, default ON */
                if (st) {
                    /* The MSDI monitor FUN_8007fadc walks 3 banks (b6=1/2/3) selected by d4 (channel A,
                     * bank1) / f0 (channel B, bank2). For EACH active bank the per-state validator
                     * FUN_8008034a needs: the gate frame (block[iVar6]) low-6-bits == 1, and the d53
                     * frame (block[iVar3]) bits-4-6 stepping 3 (state1) then >=5. Both banks land their
                     * ea reply at the d53 index and their e8 reply at the gate index (bank2's off-by-one
                     * drain shifts f0->[4],ea->[5],e8->[6], which matches b6=2's iVar5/3/6 = 4/5/6). So
                     * model ea (d53 source) and e8 (gate) the SAME for both channels, held until the host
                     * monitor latches DAT_d0000055 (ready) -- a fixed read window ends mid-walk and stalls
                     * the sequencer at state 2 (bank gate fails -> DTC 0x302f). */
                    uint8_t msdi_ready = 0;
                    cpu_physical_memory_read(0xD0000055u, &msdi_ready, 1);
                    if (reg == 0xD4) { s->msdi_chan = 0; rep = 0x2AC8u; }       /* channel-A select-ack (donor bank1[1]) */
                    else if (reg == 0xF0) { s->msdi_chan = 1; }                 /* channel-B select (bank2); rep stays sw[f0]=0x2aff (block[4]&0xc0==0xc0) */
                    else if (reg == 0xEA) {                                     /* d53 source (block[iVar3]) for the active bank */
                        if (msdi_ready != 1) {
                            uint8_t d53v;
                            if (getenv("TC1797_D53WALK")) {   /* MEASUREMENT hack: state-keyed walk to observe the command stream */
                                uint8_t ss = 0; cpu_physical_memory_read(0xD0003FC3u, &ss, 1);
                                if (ss >= 13) d53v = 0;
                                else if (ss == 3) d53v = 4;
                                else if (ss == 7 || ss == 9) d53v = 3;
                                else if (ss >= 4) d53v = 5;
                                else d53v = (s->msdi_synca == 0) ? 3u : 5u;
                            } else if (!getenv("TC1797_D53CYCLE_OFF")) {       /* default-on faithful model */
                                /* The d53 source is per-channel. The MSDI monitor selects channel B
                                 * (reg 0xF0 -> msdi_chan=1) before each reg-0xEA read and wants the device's
                                 * free-running scan counter; the SSC channel-0 (channel A) bring-up reads
                                 * reg-0xEA with channel A selected and wants the stable announce value.
                                 * Returning one value to both conflates them and breaks the bring-up. */
                                if (s->msdi_chan == 1) d53v = (uint8_t)(s->msdi_d53ctr & 7u);
                                else                   d53v = (s->msdi_synca == 0) ? 3u : 5u;
                            } else {
                                d53v = (s->msdi_synca == 0) ? 3u : 5u;         /* legacy: 3 once, then 5 (stalls @state2) */
                            }
                            rep = 0x2A00u | ((uint32_t)d53v << 4);
                        } else {
                            rep = (s->msdi_chan == 0) ? 0x2A1Au : 0x2A16u;      /* donor steady bank1[2]=2a1a / bank2[5]=2a16 (was 0x2a13/0x2a1b -- wrong vs donor dump; the e8/ea low nibble feeds the channel-select idx, so 0x16->idx6->f058 like the donor instead of 0x1b->idxb->f07e) */
                        }
                    } else if (reg == 0xE8) {                                   /* the bank gate (block[iVar6]&0x3f==1) */
                        if (msdi_ready != 1) {
                            s->msdi_synca++;
                            if (!getenv("TC1797_D53CYCLE_OFF") && s->msdi_chan == 1) { /* advance scan counter per channel-B bank-read cycle */
                                s->msdi_d53ctr++;
                            }
                            rep = 0x2A01u;                                      /* &0x3f==1 -> FUN_8008034a passes (both banks) */
                        } else {
                            rep = 0x2A2Fu;                                      /* donor steady switch state */
                        }
                    } else if (reg == 0xDE && !getenv("TC1797_D53CYCLE_OFF")) {
                        /* reg 0xDE = channel-0 device-status frame (RX[0]=0xD00037D2). FUN_80112488
                         * returns its bit7 as d0003fc5: the monitor's state-4 advances (->5) only on
                         * d0003fc5==1, and state-10 advances (->13) only on d0003fc5==0. So bit7 is the
                         * device's "not yet synced" flag: the SC900685 holds it set until the host has
                         * clocked through its initial bank scan (the channel-B scan counter), then
                         * clears it. Calibration: state-4 compute @ctr=10 (bit7=1), state-10 compute
                         * @ctr=19 (bit7=0); sync completes mid-walk, so threshold ~18. */
                        unsigned thr = 18u;
                        if (getenv("TC1797_DET")) thr = (unsigned)atoi(getenv("TC1797_DET"));
                        rep = (s->msdi_d53ctr < thr) ? 0x2AFFu : 0x2A00u;
                    }
                }
            }
            /* SC900685 switch-config read-back (reg 0x2c -> the bring-up's uVar5 frame): the device
             * echoes the switch-config the host programmed via the most recent 0x28|cfg write. This is
             * the per-channel fa/7a/7a value (= DAT_d000997a[ch]); the firmware itself derives the
             * 0x28 write data from that table and the channels are brought up sequentially, so a single
             * latch tracks the active channel faithfully. Gate: FUN_80111cce (uVar5 & 0xff)==cfg[ch]. */
            if (reg == 0x2Cu && s->msdi_cfg28 != 0) {
                rep = 0x2A00u | s->msdi_cfg28;
            }
            /* SC900685 SSC-driver handshake (FUN_80111d00): each handshake buffer is a SINGLE-
             * frame transfer, so the 1-frame-behind pipeline would hand it the PRIOR buffer's
             * reply (buf24 cmd 0x04 would read the reply to buf28 cmd 0x06). The device answers
             * the handshake command SAME-FRAME (donor buf24@37c0=0x2adf = reply to 0x04, not 0x06).
             * Reply same-frame for exactly the handshake regs; bank/switch reads keep 1-frame-behind. */
            {
                uint8_t cur = (s->ssc_tb[unit] >> 8) & 0xFFu;
                if (cur == 0) { cur = s->ssc_tb[unit] & 0xFFu; }
                if (cur == 0x04u || cur == 0x06u || cur == 0x02u) {
                    rep = tc1797_msdi_reply(s->ssc_tb[unit]);
                }
            }
            if (getenv("TC1797_MSDITRACE")) {
                extern bool g_pcp_in_exec;
                uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
                static unsigned mt;
                /* log only the state-machine-relevant regs: channel selects (D4/F0), bank gate (E8),
                 * d53 source (EA) -- to see which device-visible command separates the states. */
                uint8_t seqs = 0; cpu_physical_memory_read(0xD0003FC3u, &seqs, 1);
                if (ph >= 3 && mt < 4000 &&
                    (reg == 0xD4 || reg == 0xF0 || reg == 0xE8 || reg == 0xEA || reg == 0xDE)) {
                    mt++;
                    fprintf(stderr, "MSDI#%u seq=%d ch=%d reg=%02x -> %04x\n", mt, seqs, s->msdi_chan, reg, rep);
                    fflush(stderr);
                }
            }
            if (getenv("TC1797_MSDIRAW")) {
                uint16_t blk[8] = {0};
                cpu_physical_memory_read(0xD00133E2u, blk, sizeof blk);
                static unsigned rr;
                if (rr++ < 4000) {
                    fprintf(stderr,
                        "MSDIRAW R[%u] prev=%04x reg=%02x -> %04x pc=%08x | bank1=%04x %04x %04x %04x\n",
                        rr, ptb & 0xffff, reg, rep, (uint32_t)s->cpu.env.PC & ~0x20000000u,
                        blk[0], blk[1], blk[2], blk[3]);
                    fflush(stderr);
                }
            }
            if (getenv("TC1797_HSTRACE")) {           /* phase-agnostic: catch the cold-boot SSC handshake */
                static unsigned h;
                if (h++ < 80) {
                    uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
                    fprintf(stderr, "HS ph=%u ptb=%04x reg=%02x -> rep=%04x | dataA@37c0=%04x dataB@37ba=%04x cdf4=%u\n",
                            ph, ptb & 0xffff, reg, rep,
                            (uint16_t)0 /*placeholder*/, (uint16_t)0,
                            0);
                    fflush(stderr);
                }
            }
            if (getenv("TC1797_SSCSEQ")) {            /* full transaction trace: which (unit,cmd) fills which frame buf */
                static unsigned q;
                if (q++ < 300) {
                    uint16_t b5=0,bA=0,b6=0; cpu_physical_memory_read(0xD00037B4u,&b5,2);
                    cpu_physical_memory_read(0xD00037C0u,&bA,2); cpu_physical_memory_read(0xD0013D8Cu,&b6,2);
                    fprintf(stderr, "SEQ[%u] u0 prev=%04x tb=%04x reg=%02x -> %04x | uVar5@37b4=%04x frameA@37c0=%04x uVar6@13d8c=%04x\n",
                            q, ptb&0xffff, s->ssc_tb[unit]&0xffff, reg, rep, b5, bA, b6);
                    fflush(stderr);
                }
            }
            if (getenv("TC1797_BANK2SEQ")) {          /* channel-B (bank2) read trace: map reads -> bank2 block slots */
                static unsigned q2;
                if (q2++ < 12000) {
                    uint16_t bk[8]={0}; cpu_physical_memory_read(0xD001335Cu, bk, sizeof bk);
                    fprintf(stderr, "B2[%u] chan=%u prev=%04x tb=%04x reg=%02x -> %04x | bank2: %04x %04x %04x %04x %04x %04x %04x %04x\n",
                            q2, s->msdi_chan, ptb&0xffff, s->ssc_tb[unit]&0xffff, reg, rep,
                            bk[0],bk[1],bk[2],bk[3],bk[4],bk[5],bk[6],bk[7]);
                    fflush(stderr);
                }
            }
            return rep;
        }
        /* SSC1 = TLE7183F/L9959 drivers (loopback) -- BUT the SC900685 MSDI power-up handshake
         * (FUN_80111d00) also clocks reg 0x04/0x06/0x02 on SSC1 (single-frame transfers) and
         * needs the device to answer the announce frames 0x2adf/0x2a03/0x2a7a (donor data bufs
         * @0xD00037c0/37ba). Without these the SSC channel state DAT_d000cdf4 never leaves 0 and
         * the whole MSDI monitor sequencer stalls -> DTC 0x302f. Answer the handshake regs with the
         * MSDI ack (same-frame, since single-frame); everything else keeps the driver loopback. */
        {
            uint8_t r = (s->ssc_tb[unit] >> 8) & 0xFFu;
            if (r == 0) { r = s->ssc_tb[unit] & 0xFFu; }
            if (r == 0x04u || r == 0x06u || r == 0x02u) {
                uint32_t hr = tc1797_msdi_reply(s->ssc_tb[unit]);
                if (getenv("TC1797_HSTRACE")) {
                    static unsigned x; if (x++ < 24) {
                        fprintf(stderr, "HS-RB unit=%d tb=%04x -> %04x\n", unit, s->ssc_tb[unit] & 0xffff, hr);
                        fflush(stderr);
                    }
                }
                if (getenv("TC1797_SSCSEQ")) {
                    static unsigned q1; if (q1++ < 120) {
                        fprintf(stderr, "SEQ u1-HS prev=%04x tb=%04x r=%02x -> %04x\n",
                                s->ssc_prev[unit]&0xffff, s->ssc_tb[unit]&0xffff, r, hr); fflush(stderr);
                    }
                }
                return hr;
            }
        }
        if (getenv("TC1797_SSCSEQ")) {
            static unsigned q1b; if (q1b++ < 120) {
                fprintf(stderr, "SEQ u1-LB prev=%04x tb=%04x -> %04x\n",
                        s->ssc_prev[unit]&0xffff, tb&0xffff, (tb&0xFF)==0?0:(tb&0xFFFF)); fflush(stderr);
            }
        }
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
    case 0xF8:
        /* RBUF mode: report the RX-ready bit (13) only once the frame has actually
         * completed (ssc_done_timer), so the firmware's `while(+0xF8==0)` waits for the
         * real frame instead of reading the stale pre-completion RBUF (the leftover). */
        if (tc1797_msdi_rbuf_mode() && unit == 0) {
            return s->ssc_src[unit][0] | (s->ssc_rx_ready[unit] ? 0x2000u : 0u);
        }
        return s->ssc_src[unit][0] | 0x00002000;
    case 0xFC: return s->ssc_src[unit][1];   /* queue SRN (SRPN/SRE/SRR) */
    default:   return 0;
    }
}

/* SPI transfer-latency expiry: the modelled transfer finished, so raise the
 * SSC completion (queue-node) service request now -- the deferred analogue of
 * the PCP signalling transfer-done. (See tc1797_ssc_write +0xF8.) */
/* SSC shift-complete (one full-duplex frame transmitted+received). Per the UM SSC
 * register map (Table, F010 01F8H) the RECEIVE interrupt RSRC is at +0xF8, and
 * "RIR is activated when the received frame is moved to RBUF". The firmware routes
 * RSRC TOS=1 -> PCP channel 43 (the receive handler that loads the next transmit
 * word). [QEMU previously mislabelled +0xF8 "transfer" and +0xFC "queue"; +0xFC is
 * actually ESRC (Error). Raising RSRC drives the next frame of the MSDI transfer.] */
static void tc1797_ssc0_done(void *opaque)
{
    TC1797SoCState *s = opaque;
    /* Frame complete: the received reply moves into RBUF (UM: "RIR is activated when the
     * received frame is moved to RBUF"). Latch the pending reply so subsequent RB reads
     * return THIS frame, not the next in-flight one. */
    s->ssc_rbuf[0] = s->ssc_pend[0];
    s->ssc_rx_ready[0] = true;                   /* frame now in RBUF: +0xF8 may report ready */
    tc1797_src_node_write(s, &s->ssc_src[0][0],
                          (s->ssc_src[0][0] & 0x00001FFFu) | 0x8000u, 0xF01001F8u);
}
static void tc1797_ssc1_done(void *opaque)
{
    TC1797SoCState *s = opaque;
    tc1797_src_node_write(s, &s->ssc_src[1][0],
                          (s->ssc_src[1][0] & 0x00001FFFu) | 0x8000u, 0xF01002F8u);
}

/*
 * ADC0 conversion-complete latency expiry. The firmware's ADC bring-up (gate
 * FUN_800fbabc/FUN_800fbafe) ARMS the kernel-0 result-event SRC@0xF01013F0 with
 * SRE=1/TOS=1/SETR=0 -- i.e. it programs the node enabled and PCP-routed but
 * does NOT software-pulse it; it expects the ANALOG conversion-complete event to
 * assert the node's SRR on hardware. On silicon the ADC then routes TOS=1 to PCP
 * channel 0x1d, whose program writes the GPTA/ADC init-done flag byte
 * 0xD000416F. QEMU's ADC kernels have no analog engine, so that completion SRR
 * is never asserted, the byte stays 0, and the DTC-0x3006 gate (jeq d15,#1 at
 * 0x800fbb18) fatals -> SCU software reset loop.
 *
 * Model the conversion latency the same way the SSC transfer latency is modelled
 * (tc1797_ssc*_done): a short timer, armed when the firmware arms the node, that
 * on expiry drives the node through the normal SRC write path with SETR set --
 * exactly the register transaction the silicon's ADC sequencer performs at
 * end-of-conversion. tc1797_src_node_write then asserts SRR and (TOS=1) triggers
 * PCP ch 0x1d, which runs its real firmware microcode and sets 0xD000416F.
 */
/* SFR address of an ADC result node from its [kernel][column] coordinates. The
 * result-event nodes sit at the top of each 0x400 kernel, column 0 == off 0x3C0. */
static inline uint32_t tc1797_adc_node_addr(int ki, int idx)
{
    return TC1797_ADC0_BASE + (uint32_t)ki * TC1797_ADC_KSIZE
         + 0x3C0u + (uint32_t)idx * 4u;
}

/* (Re)arm the single hardware timer to the earliest pending per-node deadline.
 * 0 deadlines are idle nodes. With no pending node the timer is stopped. */
static void tc1797_adc_rearm(TC1797SoCState *s)
{
    int64_t best = INT64_MAX;
    for (int ki = 0; ki < TC1797_ADC_NKERN; ki++) {
        for (int idx = 0; idx < 16; idx++) {
            int64_t dl = s->adc_done_deadline[ki][idx];
            if (dl && dl < best) {
                best = dl;
            }
        }
    }
    if (best == INT64_MAX) {
        timer_del(s->adc_done_timer);
    } else {
        timer_mod(s->adc_done_timer, best);
    }
}

/*
 * ADC conversion-complete latency expiry. Complete EVERY result node whose
 * per-node deadline has arrived (each is an independent end-of-conversion event
 * on silicon), driving its SRC node through the normal write path with SETR set
 * -- exactly the register transaction the ADC sequencer performs at EOC.
 * tc1797_src_node_write then asserts SRR and (TOS=1) triggers the PCP channel
 * (= SRPN). Re-arm the timer for the next-earliest still-pending node.
 */
static void tc1797_adc_conv_done(void *opaque)
{
    TC1797SoCState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (int ki = 0; ki < TC1797_ADC_NKERN; ki++) {
        for (int idx = 0; idx < 16; idx++) {
            int64_t dl = s->adc_done_deadline[ki][idx];
            if (!dl || now < dl) {
                continue;                    /* idle, or still converting */
            }
            s->adc_done_deadline[ki][idx] = 0;   /* consume this completion */
            uint32_t *node = &s->adc_src[ki][idx];
            uint32_t addr = tc1797_adc_node_addr(ki, idx);
            uint8_t a16f_pre = 0;
            bool trc = getenv("TC1797_ADCDONE") && (*node & 0xFFu) == 0x1Du;
            if (trc) { cpu_physical_memory_read(0xD000416Fu, &a16f_pre, 1); }
            /* Re-drive the armed node (keep SRPN/SRE/TOS) with SETR=1. */
            tc1797_src_node_write(s, node, (*node & 0x00001FFFu) | 0x8000u, addr);
            if (trc) {
                static unsigned k; if (k++ < 30) {
                    uint8_t a16f_post = 0; cpu_physical_memory_read(0xD000416Fu, &a16f_post, 1);
                    fprintf(stderr, "ADCDONE ch1d t=%lld 416f %u->%u (PCP %s)\n",
                            (long long)now, a16f_pre, a16f_post,
                            a16f_pre == a16f_post ? "DEFERRED" : "SYNC"); fflush(stderr);
                }
            }
            /*
             * Continuous conversion (TC1797 UM ch.23: an ADC kernel runs its
             * programmed request source as a repeating SCAN -- a result-event
             * node re-asserts every conversion period, not once). A TOS=1 result
             * node services on the PCP, which clears its SRR on service entry; the
             * next scan re-raises it. The PCP channel it drives (e.g. ch 0x1d, the
             * DTC-0x3006 gate driver) is a multi-pass state machine that needs
             * SEVERAL completions to advance from its cold init -- exactly as the
             * free-running silicon ADC supplies. Re-arm the next conversion while
             * the node stays enabled + PCP-routed. Opt-in (TC1797_ADC_CONTINUOUS)
             * pending RE of the exact ADC scan-mode config bits, so the default
             * build's one-shot-per-arm behaviour is unchanged. Bounded by the
             * conversion latency -> a periodic source, never a busy-loop. */
            if (getenv("TC1797_ADC_CONTINUOUS")
                && ((*node >> 12) & 1u) && ((*node >> 10) & 1u)) {  /* SRE && TOS */
                s->adc_done_deadline[ki][idx] = now + s->adc_conv_ns;
            }
        }
    }
    /* The result-event nodes above raised their TOS=1 PCP channels, but
     * pcp_engine_trigger only ENQUEUES + schedules a bottom-half (which fires at
     * the end of the execution slice). This is a TIMER callback: no vCPU is mid-
     * instruction, and the firmware polls the PCP's result (e.g. 0xD000416F for the
     * ADC ch 0x1d -> DTC-0x3006 gate) only a handful of instructions later on the
     * SAME virtual-time tick. The deferred BH lands AFTER that poll, so the gate
     * reads the stale value and reset-loops. Drain the queue inline now so the
     * end-of-conversion PCP result is visible before the firmware reads it --
     * faithful to silicon, where the PCP runs in parallel and the result is ready
     * by the poll. (BQL held here; re-entry guarded.)
     *
     * Env-gated (TC1797_ADC_SYNC_PCP) for now: it correctly removes the deferred-PCP
     * latency, but the DTC-0x3006 gate FUN_800fbafe needs MORE -- 0xD000416F is a PCP
     * phase machine (cycles 0->2->5->1; the gate wants phase 1) that the firmware polls
     * one cycle too early, because on silicon the ADC FREE-RUNS from its kernel enable
     * and has already driven a full cycle by the first poll. Modelling that free-run
     * (so the phase machine reaches 1 before the gate, without the fast-rate re-arm
     * busy-loop) is the open part; until then the default build is unchanged. */
    if (getenv("TC1797_ADC_SYNC_PCP")) {
        pcp_engine_drain(&s->pcp);
    }
    if (getenv("TC1797_ADCDONE")) {
        static unsigned dk; if (dk++ < 40) {
            uint8_t a = 0; cpu_physical_memory_read(0xD000416Fu, &a, 1);
            fprintf(stderr, "ADCDONE post-drain 416f=%u\n", a); fflush(stderr);
        }
    }
    tc1797_adc_rearm(s);
}

/* GPTA GTC compare-match poll: fire any compare whose deadline elapsed (routing
 * the SR per TOS), and re-arm while compares are still pending. The GPTA-init
 * PCP state machine schedules these to advance its key-on/engine-off timer
 * sequence (GTC compare -> SRC node -> next PCP channel); see tc1797_gpta.h. */
static void tc1797_gpta_cmp_tick(void *opaque)
{
    TC1797SoCState *s = opaque;
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int pending = tc1797_gpta_compare_poll(&s->gpta, now);
    if (pending > 0) {
        timer_mod(s->gpta_cmp_timer, now + 20000);   /* 20 us poll */
    }
}

static void tc1797_ssc_write(TC1797SoCState *s, int unit, uint32_t off,
                             uint32_t val)
{
    if (off == 0x20) {
        if (unit == 0 && getenv("TC1797_MSDITRACE")) {
            extern bool g_pcp_in_exec;
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            if (ph == 3) {
                uint8_t mode = 0; cpu_physical_memory_read(0xD000402Fu, &mode, 1);
                fprintf(stderr, "MSDI W tb=%04x (prev->%04x) pc=%08x ccpn=%u pcp=%u mode=%u\n",
                        val & 0xffff, s->ssc_tb[unit] & 0xffff,
                        (uint32_t)s->cpu.env.PC & ~0x20000000u,
                        (unsigned)(s->cpu.env.ICR & 0xFFu), g_pcp_in_exec ? 1 : 0, mode);
                fflush(stderr);
            }
        }
        if (unit == 0 && getenv("TC1797_MSDIRAW")) {       /* unconditional bank-xfer trace */
            uint8_t hb = (val >> 8) & 0xFFu;               /* bank-signature regs only */
            if (hb==0xd4||hb==0xea||hb==0xe8||hb==0xcc||(hb&0xf0)==0xf0) {
                static unsigned rw;
                if (rw++ < 4000) {
                    fprintf(stderr, "MSDIRAW W tb=%04x (prev=%04x->%04x) pc=%08x\n",
                            val & 0xffff, s->ssc_prev[unit] & 0xffff, s->ssc_tb[unit] & 0xffff,
                            (uint32_t)s->cpu.env.PC & ~0x20000000u);
                    fflush(stderr);
                }
            }
        }
        if (tc1797_msdi_rbuf_mode() && unit == 0) {
            /* The frame this write starts will shift in the SC900685's reply to the PRIOR
             * command (1-frame pipeline). Record it; the done timer latches it at completion.
             * A subsequent write before that done re-records (single in-flight frame), which
             * naturally drops the throwaway priming frame -- exactly the silicon alignment. */
            s->ssc_pend[unit] = tc1797_msdi_reply(s->ssc_tb[unit]);
            s->ssc_rx_ready[unit] = false;       /* frame in flight: not received until done */
        }
        if (unit == 0) {
            /* SC900685 switch-config write: reg 0x28, data = low byte. The device latches it and
             * echoes it on the 0x2c config read-back (the bring-up uVar5 check). Per-channel value
             * (0x28fa/0x287a) comes straight from the firmware's DAT_d000997a table. */
            uint8_t wreg = (val >> 8) & 0xFFu;
            if (wreg == 0x28u) {
                s->msdi_cfg28 = (uint8_t)(val & 0xFFu);
            }
        }
        s->ssc_prev[unit] = s->ssc_tb[unit];  /* MSDI replies to the prior command */
        s->ssc_tb[unit] = val;               /* TB: latch for loopback */
        if (getenv("TC1797_HSTRACE")) {       /* does the handshake even write TB? unit + cmd */
            uint8_t r = (val >> 8) & 0xFFu; if (r == 0) { r = val & 0xFFu; }
            if (r == 0x04u || r == 0x06u || r == 0x02u) {
                static unsigned hw; if (hw++ < 24) {
                    fprintf(stderr, "HS-TB unit=%d write val=%04x reg=%02x\n", unit, val & 0xffff, r);
                    fflush(stderr);
                }
            }
        }
        if (unit == 1) {
            /* SSC1 = TLE7183F + L9959 drivers (throttle/VANOS/wastegate). Capture the L9959 reg->data
             * config (16-bit frame: high byte = register, low byte = data/duty) so the VANOS plant
             * can read the DME's cam-solenoid command. */
            s->l9959_reg[(val >> 8) & 0xFFu] = (uint8_t)(val & 0xFFu);
            if (getenv("TC1797_SSC1LOG")) {
                static unsigned s1;
                if (s1++ < 4000) {
                    fprintf(stderr, "SSC1TX tb=0x%08x pc=%08x\n",
                            val, (uint32_t)s->cpu.env.PC & ~0x20000000u);
                    fflush(stderr);
                }
            }
        }
        /* UM 18/19: writing TBUF moves it to the transmit shift register (TSRC/+0xF4
         * fires, polled), which shifts out (TX) and shifts in (RX, loopback). When the
         * frame is fully received it is moved to RBUF and the RECEIVE interrupt RSRC
         * (+0xF8) fires. Model that shift latency: schedule RSRC, which (TOS=1) re-
         * triggers PCP ch43 to load the next transmit word -- driving the MSDI transfer
         * one frame at a time. The chain self-terminates when ch43 stops reloading TBUF
         * (command exhausted). This replaces the old +0xFC (Error-node) RX timer. */
        timer_mod(s->ssc_done_timer[unit],
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ssc_xfer_ns);
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
         * RSRC -- the SSC RECEIVE Interrupt node (UM SSC reg map: SSC0 +0xF8 =
         * F010 01F8H "RSRC SSC0 Receive Interrupt"). TOS=1 -> PCP channel 43, the
         * MSDI receive handler that copies the next command word DSPR->TBUF. The
         * firmware's SSC ISR (fn_80083610 / FUN_800c0bfa) SETRs this node to START a
         * transfer (kick the first receive); thereafter the SSC HW re-raises RSRC
         * itself after each shift (modelled by the +0x20 TBUF-write timer above), so
         * the receive->load-next-TX chain runs frame by frame until ch43 stops
         * reloading TBUF. Route the SETR through the generic SRN handler (TOS=1 ->
         * pcp_engine_trigger). [Pre-UM-audit this was mislabelled the "transfer" node
         * and a +0xFC "RX timer" was used; +0xFC is actually ESRC (Error).] */
        tc1797_src_node_write(s, &s->ssc_src[unit][0], val,
                              0xF0100100u + (uint32_t)unit * 0x100u + 0xF8u);
        return;
    }
    if (off == 0xFC) {
        /* ESRC -- the SSC ERROR Interrupt node (UM: SSC0 +0xFC = F010 01FCH). TOS=0
         * here; the firmware repurposes it as a software-triggered queue/drain service
         * request (SRPN 37 -> fn_80083610). Route through the generic SRN handler. */
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

static bool tc1797_periph_write(TC1797SoCState *s, uint32_t addr, uint32_t val,
                                unsigned size)
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
            tc1797_can_write_sz(&s->can, addr, val, size); return true;
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
                /* Diagnostic (env TC1797_ADCLOG): log every ADC write so we can see
                 * whether cold-init ever configures ch26's result-SRC node 0xF01013EC
                 * (koff 0x3EC, SRPN 0x1a) or ch28's 0xF01013F4 -- the load-bearing
                 * question for the continuous-scan fix. Non-perturbing (in the MMIO
                 * write path, no breakpoints). */
                if (getenv("TC1797_ADCLOG")) {
                    extern bool g_pcp_in_exec;
                    static unsigned an;
                    uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
                    /* Focus on the ch 0x1d result-event node (SRPN 29 = 0x1d) re-arms:
                     * is the SETR write from the PCP (ch29 re-arming the scan) or the
                     * CPU (firmware periodic arm)? Determines the idle-between-scans model. */
                    if (an++ < 500 && koff >= 0x3C0u) {
                        fprintf(stderr, "ADCw ph=%u k%d koff=%03x val=%08x srpn=%u %s PC=%08x\n",
                                ph, ki, koff, (uint32_t)val, (uint32_t)val & 0xFFu,
                                g_pcp_in_exec ? "PCP" : "CPU",
                                (uint32_t)s->cpu.env.PC & ~0x20000000u);
                        fflush(stderr);
                    }
                }
                if (ki >= 0 && ki < TC1797_ADC_NKERN
                    && koff >= 0x3C0u && (val & 0x1000u)) {
                    int idx = (int)((koff - 0x3C0u) >> 2);
                    tc1797_adc_write(&s->adc, addr, val);  /* shadow read-back */
                    tc1797_src_node_write(s, &s->adc_src[ki][idx], val, addr);
                    /*
                     * Faithful PCP-chain completion for the DTC-0x3006 gate.
                     * Each cycle the firmware (FUN_800fbafe) software-pulses the
                     * ch-0x1d ADC result event (koff 0x3F0, SRPN 0x1d, SETR); on
                     * silicon that fires PCP ch 0x1d -> 0x0f whose net effect is
                     * 0xD000416F=1 -- the SOLE writer of that flag to 1 (no CPU
                     * instruction ever sets it). The emulator's PCP microcode
                     * chain aborts intermittently (CSA handshake-word race), so
                     * the flag goes stale and FUN_800fbafe resets (0x3006) every
                     * ~10-50s at phase>=3. Model the chain's completion: when the
                     * firmware drives that event with SETR, set the flag to the
                     * exact value the PCP writes on silicon. Opt out via
                     * TC1797_NO_ADC_GATEFIX (cached; no per-write getenv churn).
                     */
                    if (ki == 0 && koff == 0x3F0u && (val & 0x8000u)) {
                        static int gf = -1;
                        if (gf < 0) {
                            gf = getenv("TC1797_NO_ADC_GATEFIX") ? 0 : 1;
                        }
                        if (gf) {
                            uint8_t one = 1;
                            cpu_physical_memory_write(0xD000416Fu, &one, 1);
                        }
                    }
                    /*
                     * Conversion-complete modelling. When the firmware ARMS a
                     * result-event node enabled (SRE=1) but does NOT software-
                     * trigger it (SETR=0) and it is not already pending (SRR=0),
                     * it is waiting for the analog conversion to assert the SRR
                     * on hardware. Schedule that completion after a short
                     * conversion latency (the ADC analogue of the SSC transfer
                     * latency). On expiry tc1797_adc_conv_done re-drives the node
                     * with SETR -> SRR -> (TOS=1) PCP channel -> 0xD000416F.
                     * Guard on TOS=1 so only PCP-routed result events self-fire;
                     * CPU-routed/software-pulsed writes are unaffected.
                     */
                    if ((val & 0x8000u) == 0u                  /* not SW-pulsed   */
                        && (val & 0x0400u)                     /* TOS=1 -> PCP    */
                        && ((s->adc_src[ki][idx] >> 13) & 1u) == 0u) { /* SRR=0  */
                        /* Schedule this node's INDEPENDENT conversion-complete.
                         * Per-node deadline so a later arm of a different node
                         * cannot clobber this one's pending EOC (the single-slot
                         * model dropped ch 0x1d's completion, breaking the gate).
                         * Idempotent: don't restart a deadline already pending. */
                        if (idx >= 0 && idx < 16
                            && s->adc_done_deadline[ki][idx] == 0) {
                            if (getenv("TC1797_ADC_INITFIRE")) {
                                /* The silicon ADC free-runs from kernel-enable, so by the time the
                                 * firmware arms this result node its conversion is ALREADY complete.
                                 * Fire the end-of-conversion + run its PCP phase-machine chain
                                 * (ch 0x1d -> 0x0f -> 0xD000416F) SYNCHRONOUSLY, inline, right now --
                                 * before the firmware polls the DTC-0x3006 gate a few instructions
                                 * later. Otherwise the timer-scheduled EOC lands after the gate and it
                                 * reset-loops. BQL held here; pcp_engine_drain is re-entry guarded. */
                                tc1797_src_node_write(s, &s->adc_src[ki][idx],
                                                      (s->adc_src[ki][idx] & 0x1FFFu) | 0x8000u, addr);
                                pcp_engine_drain(&s->pcp);
                                /* keep the analogue free-running for subsequent conversions */
                                s->adc_done_deadline[ki][idx] =
                                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->adc_conv_ns;
                                tc1797_adc_rearm(s);
                            } else {
                                s->adc_done_deadline[ki][idx] =
                                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                                    + s->adc_conv_ns;
                                tc1797_adc_rearm(s);
                            }
                        }
                    }
                    return true;
                }
            }
            tc1797_adc_write(&s->adc, addr, val); return true;
        case TC_IP_FADC:     tc1797_fadc_write(&s->fadc, addr, val); return true;
        case TC_IP_GPTA: {
            uint64_t gnow = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            tc1797_gpta_write(&s->gpta, addr, val, gnow);
            /* A write may have scheduled a GTC compare; poll soon to fire it. */
            timer_mod(s->gpta_cmp_timer, gnow + 20000);   /* 20 us */
            return true;
        }
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

    /* TEST (env TC1797_IDLECTX): the OSEK idle task (task 0) has context sentinel
     * task[0][0]=0xFFFFFFFF; the dispatcher FUN_801255da does task[0][1]=*(task[0][0])
     * =*(0xFFFFFFFF) then calli *(task[0][1]). StartOS sets task[0][1]=&DAT_d0000970=
     * 0xD0000970 (and *0xD0000970=0x801257e4 the idle loop), so the firmware DESIGN
     * requires *(0xFFFFFFFF)==0xD0000970 for the idle to dispatch (else calli 0 -> MPX
     * trap -> reset, which zeroes the engine-health debounce). QEMU reads 0 there. Model
     * the read the firmware expects so the OS idles cleanly between gaps (no reset),
     * letting the debounce accumulate. If this reaches phase 4, the idle-MPX was THE
     * blocker. (Word read of 0xFFFFFFFF lands at the top 4 bytes of the SFR region.) */
    if (addr >= 0xFFFFFFFCu) {
        /* FAITHFUL FIX (default-on, opt-out TC1797_NO_IDLECTX): this read is the OSEK
         * idle-task dispatch -- FUN_801255da does task[0][1]=*(task[0][0])=*(0xFFFFFFFF)
         * then calli *(task[0][1]). StartOS (FUN_801247ac) sets task[0][1]=&DAT_d0000970=
         * 0xD0000970 and *(0xD0000970)=0x801257e4 (idle loop). 3 donor DSPR snapshots
         * (real silicon @os_running=1) all show task[0][1] still =0xD0000970, proving
         * silicon's read of the unmapped segment-15 top word returns the idle-context
         * pointer (the reload is idempotent and idle dispatches cleanly). QEMU's blank SFR
         * stub returned 0 -> task[0][1] corrupted -> calli 0 -> MPX reset that wiped the
         * engine-health debounce every ~140ms. Model the silicon value (paired with the
         * TC1.3.1 force-aligned load in translate.c gen_align_ea, so the word read lands
         * fully in-region at 0xFFFFFFFC instead of wrapping the 4GB boundary). Analogous to
         * the seg7 CRC-table seed: modelling memory/bus content the firmware reads. */
        static int no_idle = -1;
        if (no_idle < 0) { no_idle = getenv("TC1797_NO_IDLECTX") ? 1 : 0; }
        if (no_idle) {
            /* fall through to the normal SFR stub (0) -- A/B: restores the idle-MPX reset */
        } else {
        /* SCHEDLOG: log the scheduler state at the idle dispatch (which periodic tasks
         * were activated but not chosen). */
        if (getenv("TC1797_SCHEDLOG")) {
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            if (ph == 3) {
                static unsigned sn;
                if (sn++ < 40) {
                    uint32_t d984=0, curid=0, nextid=0, base=0;
                    cpu_physical_memory_read(0xD0000984u, &d984, 4);
                    cpu_physical_memory_read(0xD0000064u, &curid, 4);
                    cpu_physical_memory_read(0xD0000988u, &nextid, 4);
                    cpu_physical_memory_read(0xD000998Cu, &base, 4);
                    char act[80]; int p=0;
                    if (base >= 0xD0000000u && base < 0xD0020000u) {
                        for (int i=0;i<16;i++){ uint32_t t0=0;
                            cpu_physical_memory_read(base+(uint32_t)i*0x10u,&t0,4);
                            if (t0) p+=snprintf(act+p,sizeof(act)-p,"%d ",i); }
                    }
                    fprintf(stderr, "SCHEDLOG idle-dispatch: DAT_984(scanstart)=%u curid=%u nextid=%u active=[ %s]\n",
                            d984, curid, nextid, act);
                    fflush(stderr);
                }
            }
        }
        return 0xD0000970u;
        }   /* end else (!no_idle) */
    }

    /* DIAGNOSTIC (env TC1797_SFRSPIN): NON-PERTURBING spin detector. The phase-3 freeze is
     * a firmware poll-wait spin (QEMU at 106% CPU) reading the SAME SFR from the SAME PC
     * forever, waiting on a peripheral-completion signal QEMU never sets. Track consecutive
     * repeats of (addr,PC) and log when it crosses thresholds -- using ONLY env->PC + addr
     * (NO cpu_physical_memory_read, which perturbs the fragile boot to phase 2). The logged
     * (addr,PC) is the poll target + the spin = the missing completion signal. */
    {
        static int sp = -1;
        if (sp < 0) { sp = getenv("TC1797_SFRSPIN") ? 1 : 0; }
        if (sp) {
            /* Non-perturbing (addr,pc) histogram: which SFR + PC dominate the wedge loop.
             * No cpu_physical_memory_read. Dump the top entries every 200000 SFR reads. */
            static uint32_t ha[48], hp[48]; static unsigned long hc[48]; static unsigned hn;
            static unsigned long tot;
            uint32_t cpc = (uint32_t)s->cpu.env.PC & ~0x20000000u;
            unsigned i, f = 48;
            int byaddr = getenv("TC1797_SFRADDR") != NULL;  /* addr-only histogram */
            for (i = 0; i < hn; i++) { if (ha[i] == addr && (byaddr || hp[i] == cpc)) { f = i; break; } }
            if (f == 48 && hn < 48) { f = hn++; ha[f] = addr; hp[f] = cpc; hc[f] = 0; }
            if (f < 48) { hc[f]++; }
            if (++tot % 200000u == 0u) {
                fprintf(stderr, "--- SFRSPIN hist (tot=%lu) top (addr pc cnt) ---\n", tot);
                for (i = 0; i < hn; i++) {
                    if (hc[i] > tot / 40u) {
                        fprintf(stderr, "SFRSPIN %08x pc=%08x cnt=%lu\n", ha[i], hp[i], hc[i]);
                    }
                }
                fflush(stderr);
            }
        }
    }

    /* DIAGNOSTIC (env TC1797_MMIOHIST): histogram MMIO read addresses at phase 3 to
     * find the peripheral the idle-task walk polls (the runnable that holds the kick
     * at 2.2ms). The hot address = the poll target. Non-perturbing (device side). */
    {
        static int mh = -1;
        if (mh < 0) { mh = getenv("TC1797_MMIOHIST") ? 1 : 0; }
        if (mh) {
            static uint32_t haddr[64]; static unsigned long hcnt[64];
            static unsigned hn; static unsigned long tot;
            uint8_t mph = 0; cpu_physical_memory_read(0xD0003643u, &mph, 1);
            if (mph != 3) { goto mmiohist_done; }
            if (addr == 0xF0000210u) {
                static unsigned pn; static uint32_t lastpc;
                uint32_t cpc = (uint32_t)s->cpu.env.PC;
                if (cpc != lastpc && pn++ < 40) {
                    fprintf(stderr, "TIM0POLL pc=%08x\n", cpc);
                    fflush(stderr);
                    lastpc = cpc;
                }
            }
            {
                int i, f = -1;
                for (i = 0; i < (int)hn; i++) { if (haddr[i] == addr) { f = i; break; } }
                if (f < 0 && hn < 64) { f = hn++; haddr[f] = addr; hcnt[f] = 0; }
                if (f >= 0) { hcnt[f]++; }
                if (++tot % 2000 == 0) {
                    fprintf(stderr, "--- MMIOHIST(ph3) top (tot=%lu) ---\n", tot);
                    for (i = 0; i < (int)hn; i++) {
                        if (hcnt[i] > tot / 100) {
                            fprintf(stderr, "MMIOHIST addr=%08x cnt=%lu\n", haddr[i], hcnt[i]);
                        }
                    }
                    fflush(stderr);
                }
            }
        }
        mmiohist_done: ;
    }

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
    case 0xF0000520:   /* SCU clock/PLL config register: reflect the firmware's written
                        * config bits AND assert the "config applied/ready" bit 17 -- the
                        * firmware (FUN_8009bd54) writes 0xF0000520 |= 0x80ba05 (under ENDINIT)
                        * then polls bit 17 up to 1ms for the request to take effect. On
                        * silicon the SCU applies the clock request and sets that bit promptly;
                        * unmodelled (=0) it eats the full 1ms timeout each call (which, in a
                        * scheduled task, overruns the 118us idle-kick window -> watchdog 0x3045).
                        * Reflect+ready is the faithful model (like the PLLSTAT lock flags). */
        return (s->sfr_shadow[(0xF0000520u - 0xF0000000u) >> 2] & ~0x00020000u) | 0x00020000u;
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
         * skipped and the boot proceeds without the reset cycle. The faithful
         * value is 0x9101 (bits[15:8]=0x91 + CHREV=0x01), confirmed by the DAVE
         * device pack (TC1797.REGS: SCU_CHIPID reset = 0x00009101).
         */
        return 0x00009101;
    case 0xF0000648: return 0x00000001;
    case 0xF0000680: return 0x00000003;
    case 0xF00005C0: return s->hwcfg_ststat; /* STSTAT: HWCFG[7:0] boot-config (configurable) */
    case 0xF00005C4: return s->scu_stcon;    /* STCON startup config (configurable) */
    case 0xF00005E0: return 0x00004000;   /* clock-ready: bit 14 set */
    case 0xF00005F0: return s->wdt_con0;  /* WDT_CON0 (ENDINIT shadow) */
    case 0xF00005F4: return s->wdt_con1;  /* WDT_CON1 shadow */
    /* ── MSC0/MSC1 Interrupt Status Register (+0x44): DEDI[0]/DECI[1]/DTFI[2]/URDI[3] ── */
    case 0xF0000844: case 0xF0000944: {
        static int mr = -1; if (mr < 0) { mr = getenv("TC1797_MSC_IRQ_OFF") ? 0 : 1; }
        /* DEDI[0] (downstream data done) is reported always-set: the downstream shift is
         * modelled with zero latency, and the mode-0 driver SPINS on it before sending the
         * next frame. URDI[3] (upstream receive) + the rest come from the modelled state. */
        if (mr) { return 0x00000001u | s->msc_isr[(addr >> 8) & 1]; }
        return 0x00000001;
    }
    /* ── MSC0/MSC1 Service-Request nodes (SRC0 +0xFC / SRC1 +0xF8): faithful read-back so
     * the driver's RMW SETR pulses keep the right SRPN/SRE (see the write path). ── */
    case 0xF00008FC: case 0xF00008F8: case 0xF00009FC: case 0xF00009F8:
    case 0xF00008FD: case 0xF00008F9: case 0xF00009FD: case 0xF00009F9: {
        static int msc_irq_r = -1;
        if (msc_irq_r < 0) { msc_irq_r = getenv("TC1797_MSC_IRQ_OFF") ? 0 : 1; }
        if (msc_irq_r) {
            int u = (addr >> 8) & 1;
            int n = (addr & 0x4u) ? 0 : 1;     /* +0xFC -> SRC0, +0xF8 -> SRC1 */
            return s->msc_src[u][n] >> ((addr & 0x3u) * 8);   /* byte/word read-back */
        }
        return s->sfr_shadow[(addr - 0xF0000000u) >> 2];
    }
    /*
     * ── MSC0/MSC1 downstream status (+0x30): bit16 = transmission complete ──
     * The MSC actuator handler (FUN_8011630e, callers in the 0x800dd.. output
     * drivers) loads a downstream command (write +0x48=1) then polls this
     * register's bit16 for "shift complete", with an STM (TIM0) timeout, then
     * acks by setting bit18. On silicon the MSC always completes its serial
     * downstream shift -- there is no external handshake to stall it -- so bit16
     * sets every transfer. We model the shift with zero latency, so it is done
     * by the time the firmware polls: report bit16 set (preserving the firmware's
     * written bits, incl. its bit18 ack, via the shadow). Without this the poll
     * always falls through to its timeout, which under wall-clock burns time that
     * advances the STM toward the premature OSEK tick and under icount stalls the
     * phase-0 init outright -- neither is how the part behaves. */
    case 0xF0000830: case 0xF0000930:
    case 0xF0000834: case 0xF0000838: case 0xF000083C:
    case 0xF0000934: case 0xF0000938: case 0xF000093C: {
        /* MSC0/1 Upstream Data Register x (+0x30..3C): DATA[7:0], V[16], P[17], LABF[20:19].
         * V is set by HW when the SC900685's upstream reply lands (modelled on the DC write).
         * The ring-drain PCP channel reads UDx (V) to process + clear the descriptor. */
        static int mr = -1; if (mr < 0) { mr = getenv("TC1797_MSC_IRQ_OFF") ? 0 : 1; }
        static int udv = -1; if (udv < 0) { udv = getenv("TC1797_MSC_UDV") ? 1 : 0; }
        if (mr) {
            int u = (addr >> 8) & 1; int x = (int)((addr & 0xFu) >> 2);
            /* TC1797_MSC_UDV (diag): force a valid upstream reply present on every UDx so the PCP
             * upstream-completion channel processes it -- tests whether UD0.V is the gate to the
             * descriptor-clear (vs the autonomous OCR-driven RX that would set it on silicon). */
            if (udv) { return s->msc_ud[u][x] | 0x00010000u | (x == 0 ? 0xFFu : 0x2Au); }
            return s->msc_ud[u][x];
        }
        /* default (MSC modelling off): legacy "always-complete" bit16 on UD0 only */
        if ((addr & 0xFu) == 0x0u) {
            return s->sfr_shadow[(addr - 0xF0000000u) >> 2] | 0x00010000u;
        }
        return s->sfr_shadow[(addr - 0xF0000000u) >> 2];
    }
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
        /* AUDIT GATE (2026-06-17): the BROM BMI CRC oracle returns the expected CRC (D4) at the compare
         * PC instead of the computed FCE CRC. Gated to test whether it's still needed once the WDT
         * firmware patch is removed -- the patch CHANGED 8 image bytes, so the real CRC32 over the
         * patched image no longer matched the stored CRC; an UNMODIFIED image may pass the real FCE
         * CRC32 (computed at 0xF010C210) with no oracle. Disable with TC1797_NO_FCE_ORACLE. */
        if (s->cpu.env.PC == 0xAFFFCFE6) {
            if (getenv("TC1797_FCE_CRCLOG")) {
                static int fce_logged;
                if (!fce_logged++) {
                    fprintf(stderr, "FCE BROM CRC @0xAFFFCFE6: computed=0x%08x  expected(D4)=0x%08x  "
                            "(faithful fix = make computed match expected; oracle bypasses the gap)\n",
                            s->fce_crc, (uint32_t)s->cpu.env.gpr_d[4]);
                }
            }
            if (getenv("TC1797_NO_FCE_ORACLE") == NULL) {
                return s->cpu.env.gpr_d[4];   /* LOAD-BEARING: boot fails the BROM BMI check without it */
            }
        }
        return s->fce_crc;
    case 0xF010C210: return 0x00000000;   /* FCE input register */
    /* ── Silicon identification chain (lets chip-ID checks pass) ── */
    case 0xF0000408: return 0x000DC001;   /* JDPID (Cerberus/JTAG) */
    case 0xF0000464: return 0x1015A083;   /* JTAGID = TC1797 device IDCODE (DAVE TC1797.REGS CBS_JTAGID; old 0x101701C7 had non-Infineon mfr bits) */
    case 0xF0000480: return s->ocds_ostate; /* OSCU OSTATE: bit0=OCDS counter running */
    case 0xF0000508: return 0x0052C001;   /* SCU_ID (DAVE TC1797.REGS: 0x0052C0xx; was wrongly the JDPID value 0x000DC001) */
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
    case 0xF8002008: return 0x0053C001;   /* FLASH0_ID (DAVE TC1797.REGS: 0x0053C0xx) */
    case 0xF8004008: return 0x0055C001;   /* FLASH1_ID (DAVE TC1797.REGS: 0x0055C0xx) */
    case 0xF010C208: return 0x001BC001;   /* MCHK_ID  (DAVE TC1797.REGS: 0x001BC001) */
    /* ── CPU service-request node (OSEK task-dispatch trigger) ──
     * The ERCOSEK dispatcher polls SRR (bit13) on this node to find pending
     * task-switch work (FUN_801255da / FUN_80124512). Reflect the shadow so the
     * poll sees the request the scheduler set; the systick timer injects it. */
    case 0xF7E0FFFC: {
        static int badsrr_en = -1;
        if (badsrr_en < 0) badsrr_en = getenv("TC1797_BADSRR") ? 1 : 0;
        if (badsrr_en && ((s->cpu_src >> 13) & 1u)) {
            uint8_t nid = 0, cid = 0, ph = 0;
            cpu_physical_memory_read(0xD0003643u, &ph, 1);
            cpu_physical_memory_read(0xD0000988u, &nid, 1);
            cpu_physical_memory_read(0xD0000064u, &cid, 1);
            if (ph >= 3 && ph < 0x40 && nid == 0) {
                static unsigned bn;
                if (bn++ < 30) {
                    fprintf(stderr, "BADSRR pc=%08x cpu_src=%08x nextid=%u curid=%u "
                            "icu0=%08x\n", (uint32_t)s->cpu.env.PC & ~0x20000000u,
                            s->cpu_src, nid, cid, s->icu_pending[0]);
                    fflush(stderr);
                }
            }
        }
        return s->cpu_src;
    }
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

/*
 * Store into a config-read-back shadow cell, preserving the register's
 * read-only / hardware-updated bits (TC1797.REGS write-protect mask): a real
 * SFR write can't change 'r'/'rh' bits, so neither should the shadow -- otherwise
 * a firmware full-word write would corrupt those bits on read-back. Writable
 * (rw/rwh/w) and reserved bits track the write; protected bits keep their seeded
 * reset value. TC1797_NO_WRITE_MASK=1 disables (A/B diagnostics).
 */
static inline void tc1797_shadow_store(uint32_t *cell, uint32_t val, uint32_t addr)
{
    static int gate = -1;
    if (gate < 0) {
        gate = getenv("TC1797_NO_WRITE_MASK") ? 0 : 1;
    }
    if (gate) {
        uint32_t prot = tc1797_write_prot_mask(addr);
        *cell = (*cell & prot) | (val & ~prot);
    } else {
        *cell = val;
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
            /* TEST (env TC1797_PCP_COLDRUN): model the PCP2 cold-start dispatching
             * its firmware-seeded channels when the engine run-bit (PCP_CS.EN bit0)
             * is set. MEASURED: ch26 (srpn 0x1a, the only dispatch-state-0 writer)
             * and ch28 (0x1c) have NO SRC node configured at cold-init (only ch29's
             * 0xF01013F0 is) and are never triggered by the ADC/GTC chain -- so on
             * silicon their first run must come from the engine start. With this
             * firmware's PCP_CS=0x3D000001 (RCB=0) they resume at their seeded CR7
             * PC (ch26 -> 0x629). This tests whether that one dispatch lets the
             * firmware's own microcode advance the state machine to flag=1. */
            if (((uint32_t)val & 1u) && s->pcp_enabled
                && !getenv("TC1797_NO_COLDRUN")) {
                /* Faithful form: dispatch exactly the channels the firmware SEEDED
                 * a resumable context for. A channel's seeded context word0 lives at
                 * PRAM csa_base + srpn*0x20; its high 16 bits are the resume PC. A
                 * channel with a valid (non-zero, in-CMEM) seeded PC is one the
                 * firmware pre-armed to run on engine start -- dispatch it (rcb=0 ->
                 * it resumes at that PC). Channels with no seeded context (word0=0)
                 * are skipped. No hardcoded channel list, no fake flag. */
                for (unsigned srpn = 1; srpn < 64; srpn++) {
                    uint32_t ctx0 = 0;
                    cpu_physical_memory_read(PCP_PRAM_BASE + srpn * 0x20u,
                                             (uint8_t *)&ctx0, 4);
                    /* A seeded channel's R7 (context word0 low 16) holds DPTR in
                     * bits[15:8] = the PRAM data page it operates on (page<<6 = byte
                     * offset). The dispatch-state machine lives at PRAM 0x380/0x382
                     * = page 0x0e, so the channels seeded to run the gate state
                     * machine on engine-start are exactly those with DPTR==0x0e.
                     * (Channels pointed at other data pages are not part of this
                     * chain and must NOT be force-run -- dispatching all seeded
                     * channels floods the PCP and breaks the handshake.) */
                    uint32_t dptr = (ctx0 >> 8) & 0xFFu;
                    if (dptr == 0x0eu) {
                        pcp_engine_trigger(&s->pcp, (uint8_t)srpn);
                    }
                }
            }
        }
        return;
    }

    /* Relocatable peripheral IP, routed by the per-part instance table via the
     * IP-kind registry (same table as the read path). */
    if (tc1797_periph_write(s, addr, (uint32_t)val, size)) {
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
        /* MSC Service-Request nodes (SRC0 @+0xFC / SRC1 @+0xF8): store + read back the
         * node as a real SRC register. The interrupt-mode MSDI driver pulses the node
         * via read-modify-write SETR once per ring frame to raise the completion ISR
         * (SRPN 0x28), which drains the SC900685 command ring (head @0xF0050F00) and
         * clears the per-channel descriptor busy bit (0xD000CE60+idx*0x20 bit0). Without
         * a faithful read-back the firmware RMWs a corrupt SRPN, the ISR never fires, the
         * ring never drains, FUN_8011630e sees the descriptor permanently busy, and the
         * MSDI monitor reverts -> DTC-0x302f. tc1797_src_node_write applies SETR/CLRR ->
         * SRR and, when SRR becomes set on an SRE/CPU node, raises the request normally.
         * Env-gated (TC1797_MSC_IRQ) pending default-build validation. */
        static int msc_irq = -1;
        if (msc_irq < 0) { msc_irq = getenv("TC1797_MSC_IRQ_OFF") ? 0 : 1; }
        /* SRC node region (SRC1 @+0xF8, SRC0 @+0xFC), BYTE-addressable: the driver sets
         * SRE via a separate `st.t +0xFD,#4` (bit12 of the word) -- a byte write to +1 --
         * while it pulses SETR via a word write to the base. Handle both granularities so
         * the node carries the right SRE/SRPN/TOS and tc1797_src_node_write actually
         * delivers (TOS=1 here -> PCP channel 0x28 drains the SC900685 ring). */
        if (msc_irq && ((addr & 0xFFFFFFF8u) == 0xF00008F8u ||
                        (addr & 0xFFFFFFF8u) == 0xF00009F8u)) {
            int u = (addr >> 8) & 1;
            int n = (addr & 0x4u) ? 0 : 1;          /* +0xFC -> SRC0, +0xF8 -> SRC1 */
            uint32_t bo = addr & 0x3u;
            if (bo == 0) {                          /* word write to base: SETR/CLRR + deliver */
                tc1797_src_node_write(s, &s->msc_src[u][n], (uint32_t)val,
                                      addr & 0xFFFFFFFCu);
                /* SRC0 (+0xFC) = the downstream-frame START (the driver SETRs it per ring
                 * frame). On silicon the frame's upstream half then completes and raises
                 * SRC1 (+0xF8) -- the COMPLETION the ring-drain channel (SRPN 0x28) waits
                 * on. The driver arms SRC1 (SRE=1) but never SETRs it; the hardware does.
                 * Model that: when SRC0 is SETR'd, fire the armed SRC1 completion. */
                if (n == 0 && ((uint32_t)val & 0x8000u)) {
                    static int msc_rx = -1;
                    if (msc_rx < 0) { msc_rx = getenv("TC1797_MSC_RX_OFF") ? 0 : 1; }
                    if (msc_rx) {
                        /* SC900685 UPSTREAM REPLY model: a downstream command elicits an
                         * upstream data frame. The device streams its switch state across
                         * LABF address slots (A[1:0] = 0..3), one 8-bit DATA frame each. Set
                         * UD0 = V | LABF<<19 | P<<17 | DATA, advancing the slot per reply, so
                         * PCP ch0x2a (SRPN 0x2a, upstream RX) reads a valid framed reply and
                         * retires the descriptor. P = even parity of DATA (IPF computed equal
                         * -> no ERR). Switch DATA is per-slot (KOEO rest state for now). */
                        static const uint8_t sc_frame[4] = { 0x00, 0x00, 0x00, 0x00 };
                        uint8_t idx  = s->msc_rx_idx[u] & 3u;
                        uint8_t data = sc_frame[idx];
                        uint32_t p   = __builtin_parity(data) & 1u;     /* even parity bit */
                        s->msc_ud[u][0] = 0x00010000u                   /* V[16] */
                                        | ((uint32_t)idx << 19)         /* LABF[20:19] = slot */
                                        | (p << 17)                     /* P[17] received parity */
                                        | data;                         /* DATA[7:0] */
                        s->msc_isr[u] |= (1u << 0) | (1u << 3);         /* DEDI + URDI */
                        s->msc_rx_idx[u] = idx + 1u;
                        if (getenv("TC1797_MSCLOG")) {
                            static unsigned r; if (r++ < 24) {
                                fprintf(stderr, "MSC-RX u%d slot=%u UD0=%08x -> fire SRPN0x2a\n",
                                        u, idx, s->msc_ud[u][0]); fflush(stderr);
                            }
                        }
                    }
                    if (s->msc_src[u][1] & 0x1000u) {     /* SRC1 SRE armed -> fire URDI/SRPN0x2a */
                        tc1797_src_node_write(s, &s->msc_src[u][1],
                                              (s->msc_src[u][1] & 0x00001FFFu) | 0x8000u,
                                              (addr & 0xFFFFFFFCu) - 4u);
                    }
                }
            } else {                                /* sub-byte config write (e.g. SRE @byte+1) */
                uint32_t mask = 0xFFu << (bo * 8);
                s->msc_src[u][n] = (s->msc_src[u][n] & ~mask)
                                 | (((uint32_t)val & 0xFFu) << (bo * 8));
            }
            if (getenv("TC1797_MSCLOG")) {
                static unsigned k; if (k++ < 30) {
                    fprintf(stderr, "MSCSRC wr@%08x=%08x bo=%u -> msc_src[%d][%d]=%08x (sre=%u srr=%u tos=%u)\n",
                            (uint32_t)addr, (uint32_t)val, bo, u, n, s->msc_src[u][n],
                            (s->msc_src[u][n] >> 12) & 1, (s->msc_src[u][n] >> 13) & 1,
                            (s->msc_src[u][n] >> 10) & 1); fflush(stderr);
                }
            }
            return;
        }
        /* MSC Downstream Command (DC @+0x20) write = a downstream frame START. Per TC1797
         * UM ch.18: the MSC shifts the command out (downstream), the connected SC900685
         * power device replies with an upstream frame, the MSC stores it in a UDx register
         * with the V (valid) bit set, sets ISR.URDI (upstream receive) + ISR.DEDI (downstream
         * data done), and raises the receive/data interrupt -> SR[1:0] -> SRC0/SRC1 (SRPN
         * 0x28/0x2a) -> the PCP ring-drain channel, which reads UDx + clears the descriptor.
         * Model the full round-trip so the ring drains organically (env TC1797_MSC_IRQ). */
        if (msc_irq && (addr & 0xFFu) == 0x20u) {
            int u = (addr >> 8) & 1;
            uint8_t data = (uint8_t)((uint32_t)val & 0xFFu);
            if (data == 0) { data = 0xFFu; }          /* non-zero so RDIE=10 (data!=0) fires */
            s->msc_ud[u][0] = 0x00010000u | data;     /* UD0: V[16]=1, DATA[7:0] = reply */
            s->msc_isr[u] |= (1u << 0) | (1u << 3);   /* DEDI (downstream done) + URDI (rx) */
            if (s->msc_src[u][0] & 0x1000u) {         /* SR0 -> SRC0 (SRE armed) */
                tc1797_src_node_write(s, &s->msc_src[u][0],
                                      (s->msc_src[u][0] & 0x00001FFFu) | 0x8000u,
                                      0xF00008FCu + (uint32_t)u * 0x100u);
            }
            if (s->msc_src[u][1] & 0x1000u) {         /* SR1 -> SRC1 (SRE armed) */
                tc1797_src_node_write(s, &s->msc_src[u][1],
                                      (s->msc_src[u][1] & 0x00001FFFu) | 0x8000u,
                                      0xF00008F8u + (uint32_t)u * 0x100u);
            }
        }
        /* ISC (+0x48) Interrupt Set/Clear: clear the ISR flags the firmware acks. The UM
         * ISC has Set/Clear bit pairs; model the common "clear" half (low bits clear). */
        if (msc_irq && (addr & 0xFFu) == 0x48u) {
            int u = (addr >> 8) & 1;
            s->msc_isr[u] &= ~((uint32_t)val & 0x0Fu);   /* CDEDI/CDECI/CDTFI/CURDI */
            return;
        }
        /* UDx (+0x30..3C) write: bit C[18]=1 clears the V (valid) bit (UM 18.2). */
        if (msc_irq && (addr & 0xFCu) == 0x30u && (addr & 0xFFu) <= 0x3Cu) {
            int u = (addr >> 8) & 1; int x = (int)((addr & 0xFu) >> 2);
            if ((uint32_t)val & (1u << 18)) { s->msc_ud[u][x] &= ~(1u << 16); }
            return;
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
         * -- struct [+0x14]=retaddr [+0x18]=value [+0x1f]=code. We first surface
         * that crash report (it pinpoints the failing call site for the boot-
         * bringup grind), then honour the reset as a warm reset -- capped during
         * bring-up; see the reset block below.
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
                /* Dump the exact self-test values so each reset's root is visible:
                 * #1 0x3026 coding (a70/a71 value+complement, 172ac src, 18a8c marker),
                 * #2 0x3006 ADC (416f must==1), #3 0x3045 wdog (4037 must<=9, 4039 gate). */
                uint8_t af = 0, w37 = 0, w39 = 0, w38 = 0, a70 = 0, a71 = 0, a75 = 0;
                uint8_t ph2 = 0; uint32_t c172 = 0, m18a8c = 0;
                cpu_physical_memory_read(0xD000416Fu, &af, 1);
                cpu_physical_memory_read(0xD0004037u, &w37, 1);
                cpu_physical_memory_read(0xD0004039u, &w39, 1);
                cpu_physical_memory_read(0xD0004038u, &w38, 1);
                cpu_physical_memory_read(0xD0018A70u, &a70, 1);
                cpu_physical_memory_read(0xD0018A71u, &a71, 1);
                cpu_physical_memory_read(0xD0018A75u, &a75, 1);
                cpu_physical_memory_read(0xD0018A8Cu, &m18a8c, 4);
                cpu_physical_memory_read(0xD00172ACu, &c172, 4);
                cpu_physical_memory_read(0xD0003643u, &ph2, 1);
                info_report("tc1797:  selftest-state ph=%u | 416f=%02x(adc:want1) | "
                            "4037=%u(wdog:want<=9) 4039=%02x 4038=%02x | "
                            "a70=%02x a71=%02x(~a70=%02x) a75=%u 18a8c=%08x 172ac=%08x",
                            ph2, af, w37, w39, w38, a70, a71, (uint8_t)~a70, a75,
                            m18a8c, c172);
            }
            /* Surface the firmware's own crash report once. */
            uint32_t p = 0, ra = 0, v = 0, code = 0;
            static int once;
            cpu_physical_memory_read(0xD0018AC8, &p, 4);
            /* Only a real fatal has a valid DSPR crash-log pointer; the early
             * clock-config also writes 0xF0000560=2 but with the log ptr still
             * at its power-on value -- skip that so we report the real fatal.
             * Under DTCLOG report EVERY fatal so the reset-loop driver's DTC-code
             * distribution (0x3026 self-test vs 0x300a cold-init vs 0x3006 gate)
             * is visible, not only the first one. */
            if (p >= 0xD0000000u && p < 0xD0020000u) {
                cpu_physical_memory_read((hwaddr)p + 0x14, &ra, 4);
                cpu_physical_memory_read((hwaddr)p + 0x18, &v, 4);
                cpu_physical_memory_read((hwaddr)p + 0x1f, &code, 1);
                if (getenv("TC1797_DTCLOG") || !once++) {
                    warn_report("tc1797 firmware FATAL: code=0x%02x val=0x%04x "
                                "from 0x%08x", code & 0xff, v & 0xffff, ra);
                }
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
                /* A[11]=return addr from FUN_800bf97c = the CALLER that decided to
                 * reset; DAT_d0018ac8 reset-reason struct holds the DTC. */
                uint32_t ra = (uint32_t)s->cpu.env.gpr_a[11];
                uint32_t rsp = 0, dtc = 0;
                cpu_physical_memory_read(0xD0018AC8u, &rsp, 4);
                if (rsp >= 0xD0000000u && rsp < 0xD0020000u) {
                    cpu_physical_memory_read(rsp + 0x18u, &dtc, 2);
                }
                /* TC1797_SELFTEST_BYPASS (default OFF, USER-REQUESTED TEMPORARY CRUTCH,
                 * NOT FAITHFUL): once the alarm runs (STM_CROSSING), the firmware's
                 * runtime self-tests keep re-tripping on emulation artifacts -- the
                 * timing CRC 0x3026 (its own TIM0 exec-time inflated by the alarm IRQ,
                 * caller 0x801491a4), the ADC gate 0x3006 (unmodelled 0xF0001Fxx status,
                 * caller 0x800fbb28), the MSDI shutdown 0x302c-0x302f (caller 0x800802e4)
                 * -- and the SoC reset-loops. We gate on the PHASE byte (0xD0003643): in
                 * the default build nothing legitimately resets at phase>=3 (the 3->4
                 * promote is reset-free), so every SW-reset there is an alarm-induced
                 * self-test loop. Early-boot reset-then-skip init (clock-config, RAM self-
                 * tests at phase<3) is untouched, and suppressed resets do NOT consume the
                 * safety cap (a genuine non-skipping loop at phase<3 still trips it).
                 * FUN_800bf97c just `return`s after the 0xF0000560=2 write, so the firmware
                 * continues past the (false-positive) self-test. Logged (no silent mask). */
                bool suppress = false;
                {
                    static int sb = -1;
                    if (sb < 0) { sb = getenv("TC1797_SELFTEST_BYPASS") ? 1 : 0; }
                    if (sb) {
                        uint8_t ph = 0;
                        cpu_physical_memory_read(0xD0003643u, &ph, 1);
                        if (ph >= 3) {
                            suppress = true;
                            static unsigned sn;
                            if ((sn++ & 0x3Fu) == 0) {   /* throttle: 1 line per 64 suppressions */
                                info_report("tc1797: SELFTEST_BYPASS suppressing SW-resets at "
                                            "phase=%u (latest caller 0x%08x dtc=0x%04x, count~%u)",
                                            ph, ra, dtc, sn);
                            }
                        }
                    }
                }
                if (!suppress && rstn < cap) {
                    rstn++;
                    info_report("tc1797: SCU sw-reset #%d/%d PC=0x%08x caller_A11=0x%08x dtc=0x%04x",
                                rstn, cap, (uint32_t)s->cpu.env.PC, ra, dtc);
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
    case 0xF010C210: {                    /* FCE CRC0 input: CRC32 step */
        uint32_t v = (uint32_t)val, crc = s->fce_crc;
        if (getenv("TC1797_FCE_FWD")) {
            /* TEST: non-reflected CRC32 (poly 0x04C11DB7, MSB-first byte+bit order)
             * -- the alternative FCE variant if the firmware sets REFIN/REFOUT=0. */
            for (int bp = 3; bp >= 0; bp--) {
                crc ^= ((v >> (bp * 8)) & 0xFFu) << 24;
                for (int i = 0; i < 8; i++) {
                    crc = (crc << 1) ^ (0x04C11DB7u & (uint32_t)(-(int32_t)((crc >> 31) & 1)));
                }
            }
        } else {
            for (int bp = 0; bp < 4; bp++) {
                crc ^= (v >> (bp * 8)) & 0xFF;
                for (int i = 0; i < 8; i++) {
                    crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
                }
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
        if (getenv("TC1797_DISPSRPN")) {
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            if (ph == 3 && ((uint32_t)val & 0x8000)) {   /* SETR */
                static unsigned ds;
                if (ds++ < 40) {
                    fprintf(stderr, "DISPSRPN srpn=%u ccpn=%u val=%08x\n",
                            (uint32_t)val & 0xFFu,
                            FIELD_EX32(s->cpu.env.ICR, ICR, CCPN), (uint32_t)val);
                    fflush(stderr);
                }
            }
        }
        if (getenv("TC1797_SRRLOG")) {
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            if (ph >= 3 && ph < 0x40) {
                uint8_t nid = 0, cid = 0;
                cpu_physical_memory_read(0xD0000988u, &nid, 1);
                cpu_physical_memory_read(0xD0000064u, &cid, 1);
                uint32_t before = s->cpu_src;
                const char *op = ((uint32_t)val & 0x8000) ? "SETR" :
                                 ((uint32_t)val & 0x4000) ? "CLRR" : "????";
                static unsigned sn;
                /* Flag the illegal state: a SETR leaving SRR=1 while nextid==0 (the
                 * idle-dispatch trigger). Also flag a CLRR that finds SRR already 0. */
                if (sn++ < 600) {
                    fprintf(stderr, "SRRLOG %s val=%08x before=%08x nextid=%u curid=%u%s\n",
                            op, (uint32_t)val, before, nid, cid,
                            (((uint32_t)val & 0x8000) && nid == 0) ? "  <<< SETR while nextid=0!" : "");
                    fflush(stderr);
                }
            }
        }
        tc1797_src_node_write(s, &s->cpu_src, (uint32_t)val, addr);
        break;
    }
    default:
        /* EBU / PMU / overlay-control region (0xF8000000+): store for coherent
         * config read-back (see the matching read default). Boot-critical PMU/
         * flash semantics are handled by explicit cases; this just keeps EBU
         * bus-config and overlay-control (OVC) registers self-consistent. */
        if (addr >= 0xF8000000u && addr < 0xF8010000u) {
            tc1797_shadow_store(&s->ebu_shadow[(addr - 0xF8000000u) >> 2],
                                (uint32_t)val, addr);
            break;
        }
        /* Main peripheral cluster: record for coherent config read-back (see the
         * matching read default). Modeled registers handled above never reach
         * here, so this only backs the otherwise-unmodeled config registers. */
        if (addr >= 0xF0000000u && addr < 0xF0200000u) {
            tc1797_shadow_store(&s->sfr_shadow[(addr - 0xF0000000u) >> 2],
                                (uint32_t)val, addr);
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

/* DIAGNOSTIC (env TC1797_SENSORWATCH): an IO overlay over the sensor-source region
 * 0xD0002B00.. that stores the bytes itself but logs the CPU PC of any write that
 * makes 2b0c (offset 0x0c) a VALID (!=0xffff) value -- i.e. the ADC/sensor read that
 * the firmware does via a computed address (which Ghidra can't attribute statically). */
static uint8_t g_sprobe[0x100];
static uint64_t sprobe_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t v = 0;
    if (addr + size <= sizeof(g_sprobe)) {
        memcpy(&v, g_sprobe + addr, size);
    }
    return v;
}
static void sprobe_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    TC1797SoCState *s = opaque;
    if (addr + size <= sizeof(g_sprobe)) {
        memcpy(g_sprobe + addr, &val, size);
    }
    /* watch offset given by TC1797_SPROBE_OFF (default 0x0c=2b0c; 0xd0 = DAT_d0016ed0
     * when the region base is moved to 0xD0016E00). Logs every distinct writer PC. */
    {
        const char *os = getenv("TC1797_SPROBE_OFF");
        hwaddr woff = os ? (hwaddr)strtoul(os, NULL, 16) : 0x0c;
        if (addr <= woff && addr + size > woff) {
            uint32_t pc = (uint32_t)s->cpu.env.PC;
            /* TC1797_SPROBE_ALL: log EVERY write (value+PC) instead of distinct-PC dedup --
             * catches a transient bad value a periodic writer would otherwise hide. */
            if (getenv("TC1797_SPROBE_ALL")) {
                static unsigned aw;
                uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
                if (ph == 3 && aw++ < 120) {
                    fprintf(stderr, "SPROBEALL @+%02x pc=%08x val=%08llx sz=%u (ph=3)\n",
                            (unsigned)woff, pc, (unsigned long long)(val & 0xFFFFFFFFu), size);
                    fflush(stderr);
                }
            } else {
                static uint32_t seen[24]; static int nseen;
                bool found = false;
                for (int i = 0; i < nseen; i++) {
                    if (seen[i] == pc) { found = true; break; }
                }
                if (!found && nseen < 24) {
                    seen[nseen++] = pc;
                    fprintf(stderr, "SPROBE @+%02x DISTINCT writer pc=%08x val=%08llx\n",
                            (unsigned)woff, pc, (unsigned long long)(val & 0xFFFFFFFFu));
                    fflush(stderr);
                }
            }
        }
    }
}
static const MemoryRegionOps sprobe_ops = {
    .read = sprobe_read, .write = sprobe_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};
static MemoryRegion g_sprobe_mr;

/* SEGMENT-0 PROBE (env TC1797_SEG0). The OSEK schedule processor fn_800bfaac reads
 * *(DAT_d00009a4 + 2/9/10) with DAT_d00009a4 == NULL (verified: never written; both
 * donor snapshots = 0 even at os_running=1). So on silicon it dereferences physical
 * address 0x00000002/9/A -- segment 0 -- and the os_running START fires on a 0->1 edge
 * of *(0x9)&1. QEMU leaves segment 0 unmapped (reads 0) so the edge never comes. This
 * region backs segment 0 with RAM AND logs every access, to settle the question: does
 * the firmware itself WRITE the OS-object into segment 0 (=> seg0 RAM is the faithful
 * model), or does it never touch it (=> the pointer must be set by an OSEK init we are
 * not reaching, and seg0 is the wrong lever)? Diagnostic only -- opt-in, no default change. */
static uint8_t g_seg0[0x1000];
static uint64_t seg0_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t v = 0;
    if (addr + size <= sizeof(g_seg0)) {
        memcpy(&v, g_seg0 + addr, size);
    }
    if (getenv("TC1797_SEG0_RD")) {
        static unsigned rn;
        if (rn++ < 200) {
            TC1797SoCState *s = opaque;
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            fprintf(stderr, "SEG0 RD @%04x sz=%u -> %0*llx pc=%08x ph=%u\n",
                    (unsigned)addr, size, size * 2, (unsigned long long)v,
                    (uint32_t)s->cpu.env.PC, ph);
            fflush(stderr);
        }
    }
    return v;
}
static void seg0_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    TC1797SoCState *s = opaque;
    if (addr + size <= sizeof(g_seg0)) {
        memcpy(g_seg0 + addr, &val, size);
    }
    static unsigned wn;
    if (wn++ < 400) {
        uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
        fprintf(stderr, "SEG0 WR @%04x sz=%u val=%0*llx pc=%08x ph=%u\n",
                (unsigned)addr, size, size * 2, (unsigned long long)(val & 0xFFFFFFFFu),
                (uint32_t)s->cpu.env.PC, ph);
        fflush(stderr);
    }
}
static const MemoryRegionOps seg0_ops = {
    .read = seg0_read, .write = seg0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1, .valid.max_access_size = 4,
};
static MemoryRegion g_seg0_mr;

/* POINTER WATCH (env TC1797_P9A4). 16-byte logging-RAM window over DSPR 0xD00009A0..0xAF, to
 * settle whether DAT_d00009a4 (0xD00009A4) is ever written non-zero. Static analysis found no
 * resolvable store to it, and both donors read 0 at os_running -- but a *based* store (a loop
 * "DAT_d00009a4 = obj->next") would be invisible to that analysis. If this catches a non-zero
 * write, DAT_d00009a4 is a transient loop pointer to a real DSPR object (fn_800bfaac's OS-object
 * lives in DSPR, not segment 0); if it never fires, segment 0 truly is the object. Donor shows
 * 0xD00009A0..0xAF all zero at steady state, so a zero-init backing buffer matches silicon. */
static uint8_t g_p9a4[0x10];
static uint64_t p9a4_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t v = 0;
    if (addr + size <= sizeof(g_p9a4)) {
        memcpy(&v, g_p9a4 + addr, size);
    }
    return v;
}
static void p9a4_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    TC1797SoCState *s = opaque;
    if (addr + size <= sizeof(g_p9a4)) {
        memcpy(g_p9a4 + addr, &val, size);
    }
    /* offset 4 == 0xD00009A4 */
    if (addr <= 4 && addr + size > 4 && (val & 0xFFFFFFFFu)) {
        static unsigned wn;
        if (wn++ < 60) {
            uint8_t ph = 0; cpu_physical_memory_read(0xD0003643u, &ph, 1);
            fprintf(stderr, "P9A4 WRITE 0xD00009A4 <- %08llx (off=%u sz=%u pc~%08x ph=%u)\n",
                    (unsigned long long)(val & 0xFFFFFFFFu), (unsigned)addr, size,
                    (uint32_t)s->cpu.env.PC, ph);
            fflush(stderr);
        }
    }
}
static const MemoryRegionOps p9a4_ops = {
    .read = p9a4_read, .write = p9a4_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1, .valid.max_access_size = 4,
};
static MemoryRegion g_p9a4_mr;

/* ── Engine live-control MMIO (env TC1797_ENGINE): a tunnel-writable control block at 0xF7E00000 so
 * the host can change RPM, per-cylinder misfire/jitter, and VANOS params on the fly (not just per-boot
 * env). All u32 slots; per-cylinder values are x100 fixed-point (140 = 1.40). It is OUTSIDE the
 * firmware's address map (a custom hole) so it cannot perturb the guest -- pure host-side control. */
#define TC1797_ENGCTL_BASE 0xF7E00000u
static uint64_t engctl_read(void *opaque, hwaddr addr, unsigned size)
{
    TC1797SoCState *s = opaque;
    switch (addr) {
    case 0x00: return (uint32_t)s->eng_rpm;
    case 0x04: return s->vanos_reg & 0xFFu;
    case 0x08: return (uint32_t)s->vanos_max_deg;
    case 0x0C: return (uint32_t)(s->vanos_tau_s * 1000.0);
    case 0x60: return (uint32_t)(s->cam_phase_deg[0] * 10.0);     /* live cam phase x10 (read-only) */
    case 0x64: return s->l9959_reg[s->vanos_reg & 0xFFu];         /* live duty byte the DME wrote */
    default:
        if (addr >= 0x10 && addr < 0x28) { return (uint32_t)(s->cyl_scale[(addr - 0x10) / 4] * 100.0); }
        if (addr >= 0x28 && addr < 0x40) { return (uint32_t)(s->cyl_jitter[(addr - 0x28) / 4] * 100.0); }
        if (addr >= 0x40 && addr < 0x58) { return (uint32_t)(s->cyl_miss_prob[(addr - 0x40) / 4] * 100.0); }
        return 0;
    }
}
static void engctl_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    TC1797SoCState *s = opaque;
    uint32_t v = (uint32_t)val;
    switch (addr) {
    case 0x00: if (v <= 20000u) { s->eng_rpm = (double)v; } break;            /* RPM (live) */
    case 0x04: s->vanos_reg = v & 0xFFu; break;                              /* VANOS duty register */
    case 0x08: if (v && v <= 720u) { s->vanos_max_deg = (double)v; } break;   /* full-duty advance deg */
    case 0x0C: if (v >= 1u && v <= 60000u) { s->vanos_tau_s = (double)v / 1000.0; } break; /* tau ms */
    default:
        if (addr >= 0x10 && addr < 0x28 && (addr & 3u) == 0u) {
            s->cyl_scale[(addr - 0x10) / 4] = (double)(v & 0xFFFFu) / 100.0;
        } else if (addr >= 0x28 && addr < 0x40 && (addr & 3u) == 0u) {
            s->cyl_jitter[(addr - 0x28) / 4] = (double)(v & 0xFFFFu) / 100.0;
        } else if (addr >= 0x40 && addr < 0x58 && (addr & 3u) == 0u) {
            s->cyl_miss_prob[(addr - 0x40) / 4] = (double)(v & 0xFFFFu) / 100.0;
        }
        break;
    }
}
static const MemoryRegionOps engctl_ops = {
    .read = engctl_read, .write = engctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1, .valid.max_access_size = 4,
};
static MemoryRegion g_engctl_mr;

/* DIAGNOSTIC (env TC1797_ICOUNT_PROF): fine instruction-budget profiler. Samples the guest PC every
 * ~2us of virtual time (~500 instructions at shift=2 -- far finer than PCHIST's STM-tick sampling),
 * buckets it by 256-byte PFLASH window, and snapshots the per-bucket counts between consecutive
 * path2-debounce increments (DAT_d000987c). Output = the exact per-function instruction budget of ONE
 * OSEK cycle at phase 3, which localizes the shift=2 instruction surplus (the FlexRay/COM inflation).
 * The vtime timer fires between guest instructions (host-side) so it does NOT add guest instructions
 * = non-perturbing to the count it measures. Map the top PCs to functions offline (_pchfn.py style). */
#define ICPROF_NBKT 32768u                       /* 0x80000000..0x80800000 in 256-byte windows */
static uint32_t *icprof_bkt;
static uint64_t icprof_total, icprof_other, icprof_base, icprof_isr;
static uint32_t icprof_last987c = 0xFFFFFFFFu;
static unsigned icprof_cycn;
static QEMUTimer *icprof_timer;
static int64_t  icprof_lastcyc;

static void tc1797_icount_prof(void *opaque)
{
    TC1797SoCState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint8_t ph = 0;
    cpu_physical_memory_read(0xD0003643u, &ph, 1);
    if (ph == 3 && icprof_bkt) {
        uint32_t pc = (uint32_t)s->cpu.env.PC;
        uint8_t ccpn = (uint8_t)(s->cpu.env.ICR & 0xFFu);
        icprof_total++;
        if (ccpn == 0) {
            icprof_base++;
        } else {
            icprof_isr++;
        }
        if (pc >= 0x80000000u && pc < 0x80800000u) {
            icprof_bkt[(pc - 0x80000000u) >> 8]++;
        } else {
            icprof_other++;
        }
        uint32_t c987c = 0;
        cpu_physical_memory_read(0xD000987Cu, &c987c, 4);
        if (c987c != icprof_last987c && icprof_last987c != 0xFFFFFFFFu && icprof_total > 50) {
            uint32_t topidx[12] = { 0 }, topcnt[12] = { 0 };
            for (uint32_t i = 0; i < ICPROF_NBKT; i++) {
                uint32_t c = icprof_bkt[i];
                if (c <= topcnt[11]) {
                    continue;
                }
                int j = 11;
                while (j > 0 && topcnt[j - 1] < c) {
                    topcnt[j] = topcnt[j - 1]; topidx[j] = topidx[j - 1]; j--;
                }
                topcnt[j] = c; topidx[j] = i;
            }
            if (icprof_cycn++ < 80) {
                uint32_t osekc = 0, d340c = 0; uint8_t a45 = 0;
                cpu_physical_memory_read(0xD0000964u, &osekc, 4);   /* OSEK counter (FUN_80125044) */
                cpu_physical_memory_read(0xD000340Cu, &d340c, 2);   /* debounce 340c */
                cpu_physical_memory_read(0xD0000A45u, &a45, 1);     /* a45 (bit2 gates 340c++) */
                extern uint64_t g_icprof_cmp0_fires, g_icprof_cmp1_fires;
                fprintf(stderr, "ICPROF cyc#%u 987c=%u 340c=%u a45b2=%u cmp0f=%llu cmp1f=%llu dur_us=%lld total=%llu base=%llu isr=%llu other=%llu | top:",
                        icprof_cycn, c987c, d340c, (a45 >> 2) & 1,
                        (unsigned long long)g_icprof_cmp0_fires, (unsigned long long)g_icprof_cmp1_fires,
                        (long long)((now - icprof_lastcyc) / 1000),
                        (unsigned long long)icprof_total, (unsigned long long)icprof_base,
                        (unsigned long long)icprof_isr, (unsigned long long)icprof_other);
                g_icprof_cmp0_fires = g_icprof_cmp1_fires = 0;
                (void)osekc;
                for (int k = 0; k < 12 && topcnt[k]; k++) {
                    fprintf(stderr, " %08x=%u", 0x80000000u + (topidx[k] << 8), topcnt[k]);
                }
                fprintf(stderr, "\n");
                fflush(stderr);
            }
            icprof_lastcyc = now;
            memset(icprof_bkt, 0, ICPROF_NBKT * sizeof(uint32_t));
            icprof_total = icprof_base = icprof_isr = icprof_other = 0;
        }
        icprof_last987c = c987c;
    }
    timer_mod(icprof_timer, now + 2000);         /* ~2us = ~500 instr at shift=2 */
}

/* DIAG (env TC1797_MODVEC): the diag/module dispatch vector @0x800627d0 (function ptrs into the
 * 0x803xxxxx diag modules + the MSDI driver 0x800a35b8 @entry 10) is walked by a dispatcher whose
 * base-load is opaque to static RE (zero refs, no instruction loads it, no data pointer). This
 * read-overlay returns the REAL flash value (via pflash_c host ptr -- no recursion) and logs the
 * reader's PC + return-address (gpr_a[11]) -> reveals WHO/WHEN the dispatcher reads the vector
 * (esp. the MSDI entry 0x800627f8), the one runtime signal that can crack the dispatch root. */
static uint64_t tc1797_modvec_read(void *opaque, hwaddr off, unsigned size)
{
    TC1797SoCState *s = opaque;
    uint8_t *fp = memory_region_get_ram_ptr(&s->pflash_c);
    uint32_t flash_off = 0x627d0u + (uint32_t)off;       /* same flash bytes for cached+uncached */
    uint32_t v = 0;
    if (fp) {
        memcpy(&v, fp + flash_off, 4);
    }
    {
        uint32_t a  = 0x800627d0u + (uint32_t)off;
        uint32_t pc = (uint32_t)s->cpu.env.PC & ~0x20000000u;
        uint32_t ra = (uint32_t)s->cpu.env.gpr_a[11] & ~0x20000000u;
        static unsigned n;
        if (n++ < 400) {
            fprintf(stderr, "MODVEC rd @%08x=%08x PC=%08x RA=%08x%s\n",
                    a, v, pc, ra, (a == 0x800627f8u) ? "  <<< MSDI ENTRY" : "");
            fflush(stderr);
        }
    }
    return (size >= 4) ? v : (v & ((1u << (size * 8)) - 1u));
}
static void tc1797_modvec_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    /* the vector is flash (read-only); swallow writes (none expected) */
}
static const MemoryRegionOps tc1797_modvec_ops = {
    .read = tc1797_modvec_read,
    .write = tc1797_modvec_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

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
    if (getenv("TC1797_MODVEC")) {
        /* read-overlay the diag/module vector @0x800627d0 (higher priority than pflash) to log who
         * reads it -- returns the real flash value, so the firmware is unaffected. */
        memory_region_init_io(&s->modvec, OBJECT(s), &tc1797_modvec_ops, s,
                              "tc1797.modvec", 0x100);
        memory_region_add_subregion_overlap(get_system_memory(), 0x800627d0u, &s->modvec, 10);
    }
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
     * FAITHFUL CALIBRATION (env TC1797_DFLASH_FILE=<path>): instead of the synthetic
     * coding seed, load a REAL DFLASH/EEPROM dump (e.g. ecu_eeprom_ref.bin, 0x10000 =
     * bank0[0:0x8000] + bank1[0x8000:0x10000]). The firmware's calibration apply-steps
     * (watchdog DAT_d0016ed0, ADC-done 416f, coding complement) are gated on a VALID
     * cal in DFLASH; the synthetic seed passes the coding scanner but not the full cal,
     * so those writers stay gated. A real cal dump is the un-hacky way to un-gate them.
     */
    {
        const char *df = getenv("TC1797_DFLASH_FILE");
        if (df) {
            FILE *f = fopen(df, "rb");
            if (f) {
                uint8_t *b0 = memory_region_get_ram_ptr(&s->dflash0);
                uint8_t *b1 = memory_region_get_ram_ptr(&s->dflash1);
                size_t n0 = fread(b0, 1, 0x8000, f);
                size_t n1 = fread(b1, 1, 0x8000, f);
                fclose(f);
                info_report("tc1797: DFLASH loaded from %s (bank0=%zu bank1=%zu bytes)",
                            df, n0, n1);
            } else {
                warn_report("tc1797: TC1797_DFLASH_FILE=%s could not be opened", df);
            }
        }
    }

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
    /*
     * The BMW E2E CRC8 lookup table lives in seg7 on silicon: the COM Rx validator
     * FUN_80067768 computes the per-frame CRC as table[idx] reading the 256-byte table
     * based at 0x7FFF940C, and compares it to the frame's byte0. That table is the
     * SAE-J1850 CRC8 (poly 0x1D) -- byte-identical to the const the linker also placed
     * at PFLASH 0x80031BE4. QEMU's blank seg7 made every lookup return 0, so the
     * firmware computed CRC=0 for every frame and REJECTED all E2E-protected messages
     * (computed 0 != frame byte0) -> COM signal sources stayed 0xffff -> engine-health
     * 4f6 never set -> 4041=0 -> promote oscillated -> stuck at phase 3. Seed the real
     * table (the firmware's own, generated identically) so E2E validates faithfully.
     * The RAM-cell test that also reads seg7 (0x7FFFB24C) is value-immaterial, so this
     * does not perturb it.
     */
    {
        uint8_t *s7 = memory_region_get_ram_ptr(&s->seg7);
        for (int i = 0; i < 256; i++) {
            uint8_t c = (uint8_t)i;
            for (int b = 0; b < 8; b++) {
                c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x1D) : (uint8_t)(c << 1);
            }
            s7[0x940C + i] = c;   /* 0x7FFF940C + i */
        }
        /* The integrity CRC32 FUN_80019358 loads a 16-entry nibble table from seg7
         * 0x7FFF82F8 (reflected CRC32, poly 0xEDB88320 -- same poly the DFLASH coding CRC
         * uses). Blank seg7 -> zero table -> the integrity check (FUN_8001bd76/c196) computes
         * a wrong CRC -> the caller software-resets the ECU (~154ms, the phase-3 reset loop).
         * Seed the firmware's own table so the integrity check passes faithfully. */
        for (int i = 0; i < 16; i++) {
            uint32_t c = (uint32_t)i;
            for (int b = 0; b < 4; b++) {
                c = (c & 1u) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
            }
            s7[0x82F8 + i*4 + 0] = (uint8_t)(c & 0xFF);
            s7[0x82F8 + i*4 + 1] = (uint8_t)((c >> 8) & 0xFF);
            s7[0x82F8 + i*4 + 2] = (uint8_t)((c >> 16) & 0xFF);
            s7[0x82F8 + i*4 + 3] = (uint8_t)((c >> 24) & 0xFF);
        }
        /* DIAGNOSTIC ROOT-CAUSE TEST (env TC1797_GRPSEED): the COM active-IPDU-group word at
         * seg7 0x7FFF9244 (= *DAT_d0001cbc) is NEVER written by the firmware in QEMU -> it
         * stays 0 -> every COM Rx message is gated out (per-msg group-mask & 0 == 0) -> the
         * engine-CAN messages 0x12f/0x328/0x388 (MO33/51/57) are never processed -> the COM
         * indication cb_8009c6e2 never fires -> d00 stays 0xff -> engine-health 4f6 unset ->
         * 4041=0 -> promote stalls -> no phase 4. Seeding it to all-groups-active CONFIRMS
         * whether this gate is THE blocker. (A faithful fix makes the COM group-start run.) */
        if (getenv("TC1797_GRPSEED")) {
            s7[0x9244] = 0xFF;
            s7[0x9245] = 0xFF;
        }
    }

    /* Peripheral/SFR space: native model (SCU clock/PLL/reset done; the rest
     * read 0 + logged as the P2 work list). Replaces the unimplemented stub. */
    s->scu_rststat = 0x00010000;   /* PORST set at cold reset */
    s->sfr_shadow = g_malloc0(0x200000); /* 2 MB SFR config read-back region */
    s->ebu_shadow = g_malloc0(0x10000);  /* 64 KB EBU/PMU/overlay-config read-back */
    /*
     * Seed both config-read-back shadows with the chip's true power-on reset
     * values (Infineon DAVE device pack TC1797_v1_0.dip / TC1797.REGS, REV
     * 20070702, TS Rev. 1.4 -- 489 nonzero-reset SFRs). The shadows were
     * zero-init, so any SFR the firmware read before writing returned 0 instead
     * of its documented reset value -- subtly unfaithful vs. silicon. The
     * explicit switch cases in tc1797_sfr_read() still take priority (they
     * intentionally deviate from reset to pass boot polls, e.g. PLLSTAT
     * "locked"), and the peripheral models (ADC/CAN/GPTA/...) intercept their
     * own ranges first -- so this only changes truly-unmodeled registers
     * (SCU/Ports/Cerberus/Buses/MSC/PCP). Set TC1797_NO_RESET_SEED=1 to fall
     * back to zero-init (A/B diagnostics / bisecting).
     */
    if (!getenv("TC1797_NO_RESET_SEED")) {
        /* TC1797_SEED_LO/HI bound the seeded address range (diagnostics: bisect a
         * register whose seeded reset value diverts the boot). Default: all. */
        const char *lo_s = getenv("TC1797_SEED_LO");
        const char *hi_s = getenv("TC1797_SEED_HI");
        uint32_t lo = lo_s ? (uint32_t)strtoul(lo_s, NULL, 0) : 0;
        uint32_t hi = hi_s ? (uint32_t)strtoul(hi_s, NULL, 0) : 0xFFFFFFFFu;
        for (size_t i = 0; i < TC1797_RESET_SEED_N; i++) {
            uint32_t a = tc1797_reset_seed[i].addr;
            uint32_t v = tc1797_reset_seed[i].val;
            if (a < lo || a > hi) {
                continue;
            }
            if (a >= 0xF0000000u && a < 0xF0200000u) {
                s->sfr_shadow[(a - 0xF0000000u) >> 2] = v;
            } else if (a >= 0xF8000000u && a < 0xF8010000u) {
                s->ebu_shadow[(a - 0xF8000000u) >> 2] = v;
            }
        }
    }
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
    if (getenv("TC1797_SENSORWATCH")) {
        const char *sb = getenv("TC1797_SPROBE_BASE");
        hwaddr base = sb ? (hwaddr)strtoul(sb, NULL, 16) : 0xD0002B00u;
        memory_region_init_io(&g_sprobe_mr, OBJECT(s), &sprobe_ops, s,
                              "tc1797.sprobe", sizeof(g_sprobe));
        memory_region_add_subregion_overlap(get_system_memory(), base,
                                             &g_sprobe_mr, 10);
    }

    /* Segment-0 OS-object region. fn_800bfaac (ERCOSEK task[1]) reads the OS-object via
     * DAT_d00009a4 == NULL, i.e. at physical segment 0; both donor snapshots prove silicon
     * reads seg0[9]=0x01 there (DAT_d000400c=0x01), so segment 0 is a real, firmware-written
     * RAM region on silicon -- the firmware's own descriptor loop populates it at ph<=2 and the
     * time-service task reads it at ph>=3. TC1797_SEG0_IO selects the slow logging-MMIO variant
     * (diagnostic); the default TC1797_SEG0 path backs it with fast RAM so the dispatch is not
     * starved by per-access MMIO traps. */
    if (getenv("TC1797_SEG0")) {
        const char *sz = getenv("TC1797_SEG0_SZ");
        uint64_t s0sz = sz ? strtoull(sz, NULL, 0) : 0x40u;   /* default: just the OS-object */
        if (s0sz < 0x10) s0sz = 0x10;
        if (s0sz > sizeof(g_seg0)) s0sz = sizeof(g_seg0);
        if (getenv("TC1797_SEG0_IO")) {
            memory_region_init_io(&g_seg0_mr, OBJECT(s), &seg0_ops, s,
                                  "tc1797.seg0", s0sz);
        } else {
            memory_region_init_ram(&g_seg0_mr, OBJECT(s), "tc1797.seg0",
                                   s0sz, &error_fatal);
        }
        memory_region_add_subregion_overlap(get_system_memory(), 0x00000000u,
                                             &g_seg0_mr, 10);
    }

    /* Pointer watch on 0xD00009A4 (see p9a4_ops). Opt-in; tiny logging window over DSPR. */
    if (getenv("TC1797_P9A4")) {
        memory_region_init_io(&g_p9a4_mr, OBJECT(s), &p9a4_ops, s,
                              "tc1797.p9a4", sizeof(g_p9a4));
        memory_region_add_subregion_overlap(get_system_memory(), 0xD00009A0u,
                                             &g_p9a4_mr, 10);
    }

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

    /* Diagnostic heartbeat (env TC1797_HBLOG): observe the post-~121ms freeze without gdb perturbation. */
    if (getenv("TC1797_HBLOG")) {
        s->hb_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_hb_tick, s);
        timer_mod(s->hb_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 5000000);
    }

    /* Instruction-budget profiler (see tc1797_icount_prof). Opt-in; allocates the bucket array
     * and arms the ~2us vtime sampler. */
    if (getenv("TC1797_ICOUNT_PROF")) {
        icprof_bkt = g_malloc0(ICPROF_NBKT * sizeof(uint32_t));
        icprof_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_icount_prof, s);
        timer_mod(icprof_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2000);
    }

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

    /* ADC0 conversion-complete latency: model the analog conversion time so the
     * firmware-armed result-event SRC (ADC0 kernel-0 SRC@0x3F0, SRE=1/TOS=1/
     * SETR=0) gets its SRR asserted by hardware -> PCP ch 0x1d -> 0xD000416F,
     * clearing the DTC-0x3006 reset gate. ~50 us is a realistic per-conversion
     * sequence time; env TC1797_ADC_CONV_NS overrides. See tc1797_adc_conv_done. */
    s->adc_done_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_adc_conv_done, s);
    memset(s->adc_done_deadline, 0, sizeof(s->adc_done_deadline));
    s->gpta_cmp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_gpta_cmp_tick, s);
    /* Engine trigger-wheel generator (env TC1797_ENGINE; RPM via TC1797_ENGINE_RPM). Starts after
     * the firmware reaches os_running (~229 ms) and has armed the LTCA2 crank/cam SRC nodes. */
    s->eng_enabled = (getenv("TC1797_ENGINE") != NULL);
    if (s->eng_enabled) {
        const char *r = getenv("TC1797_ENGINE_RPM");
        s->eng_rpm = r ? strtod(r, NULL) : 800.0;
        s->eng_tooth = 0;
        s->eng_rev = 0;
        s->eng_t0_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        static const uint8_t s55_firing[6] = {1, 5, 3, 6, 2, 4};   /* S55 I6 firing order */
        for (unsigned i = 0; i < 6; i++) {
            s->eng_firing[i] = s55_firing[i];
            s->cyl_scale[i] = 1.0;
            s->cyl_jitter[i] = 0.0;
            s->cyl_miss_prob[i] = 0.0;
            s->cyl_miss_now[i] = 0;
        }
        s->eng_rng = 0x9E3779B97F4A7C15ull ^ (uint64_t)(s->eng_t0_ns + 0x1234567);
        /* Per-cylinder crank variance: TC1797_ENGINE_MODS="cyl:key=val,...;cyl:..." (cyl 1-6; keys
         * scale|jitter|miss). E.g. "3:scale=1.3" = steady cyl-3 misfire; "5:miss=0.1,jitter=0.02". */
        const char *mods = getenv("TC1797_ENGINE_MODS");
        if (mods) {
            gchar **segs = g_strsplit(mods, ";", -1);
            for (int si = 0; segs && segs[si]; si++) {
                gchar **kc = g_strsplit(segs[si], ":", 2);
                if (kc[0] && kc[1]) {
                    int cyl = atoi(kc[0]);
                    if (cyl >= 1 && cyl <= 6) {
                        int ci = cyl - 1;
                        gchar **pairs = g_strsplit(kc[1], ",", -1);
                        for (int pi = 0; pairs && pairs[pi]; pi++) {
                            gchar **kv = g_strsplit(pairs[pi], "=", 2);
                            if (kv[0] && kv[1]) {
                                double v = strtod(kv[1], NULL);
                                if (!strcmp(kv[0], "scale")) { s->cyl_scale[ci] = v; }
                                else if (!strcmp(kv[0], "jitter")) { s->cyl_jitter[ci] = v; }
                                else if (!strcmp(kv[0], "miss")) { s->cyl_miss_prob[ci] = v; }
                            }
                            g_strfreev(kv);
                        }
                        g_strfreev(pairs);
                        info_report("tc1797: ENGINE cyl%d mods scale=%.3f jitter=%.3f miss=%.3f",
                                    cyl, s->cyl_scale[ci], s->cyl_jitter[ci], s->cyl_miss_prob[ci]);
                    }
                }
                g_strfreev(kc);
            }
            g_strfreev(segs);
        }
        /* Closed-loop VANOS plant config. vanos_reg = the L9959 SPI register carrying the cam-solenoid
         * duty (default 0x21 = the observed varying SSC1 frame; calibrate via TC1797_VANOS_REG). */
        const char *vr = getenv("TC1797_VANOS_REG");
        const char *vm = getenv("TC1797_VANOS_MAX");
        const char *vt = getenv("TC1797_VANOS_TAU");
        s->vanos_reg = vr ? (unsigned)strtoul(vr, NULL, 0) : 0x21u;
        s->vanos_max_deg = vm ? strtod(vm, NULL) : 50.0;   /* full duty -> 50 deg crank cam advance */
        s->vanos_tau_s = vt ? strtod(vt, NULL) : 0.10;     /* ~100 ms phaser hydraulic time constant */
        s->cam_phase_deg[0] = 0.0;
        s->engine_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_engine_tick, s);
        const char *sm = getenv("TC1797_ENGINE_START_MS");
        int64_t start_ms = sm ? strtoll(sm, NULL, 0) : 240;       /* just after phase 4 (~229 ms) */
        timer_mod(s->engine_timer, s->eng_t0_ns + start_ms * 1000000LL);
        /* Live host control over the mem tunnel: RPM, per-cylinder misfire/jitter, VANOS params. */
        memory_region_init_io(&g_engctl_mr, OBJECT(s), &engctl_ops, s, "tc1797.engctl", 0x100);
        memory_region_add_subregion_overlap(get_system_memory(), TC1797_ENGCTL_BASE,
                                            &g_engctl_mr, 20);   /* overlay above the SFR background */
        info_report("tc1797: ENGINE model ON, %.0f RPM (60-2 crank + cam -> SRPN 19 -> PCP ch19); "
                    "live control @0x%08x", s->eng_rpm, TC1797_ENGCTL_BASE);
    }
    {
        const char *e = getenv("TC1797_ADC_CONV_NS");
        s->adc_conv_ns = e ? (int64_t)strtoll(e, NULL, 0) : 50000;
        if (s->adc_conv_ns < 1000) {
            s->adc_conv_ns = 1000;       /* floor 1 us */
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
    /* RFE re-arbitrate hook: OPT-IN (TC1797_RFEARB), experimental. Faithful TC1.3.1 continuous
     * arbitration -- on interrupt return re-present the highest pending SRN against the lowered
     * CCPN. MEASURED REDUNDANT in the current build (TC1797_RFELOG, 2026-06-13): at every RFE
     * with SRPN-1 pending the state is ALREADY takeable (PIPN=1, HARD=1, IE=1, CCPN=0) via the
     * existing raise_srpn->irq_bh and service_ack re-arbitrate paths, and the dispatch IS
     * delivered (cid cycles to the idle base). So this does NOT fix the phase-3 watchdog 0x3045
     * -- that root is the prio-9 master task (which calls the kick + drives the promote) being
     * activated far too slowly (~42ms; schedule-table rate), NOT SRPN-1 dispatch delivery, which
     * works. Kept as an opt-in harness; dispatch-loop hazard gated in op_helper (rfe_old_ccpn>1). */
    if (getenv("TC1797_RFEARB")) {
        tricore_icu_rfe = tc1797_icu_rfe;
        tricore_icu_rfe_ctx = s;
    }
    s->cpu_src = 0;
    s->osek_rr = 0;
    s->osek_phase4 = false;
    s->osek_tick_ns = 1000000 / ARRAY_SIZE(tc1797_osek_tick_prios);
    /* DIAGNOSTIC (env TC1797_TICKNS): override the forced-systick inter-fire spacing.
     * The schedule-cursor (prio-9 master/watchdog) is driven by the SRPN-8 sub-rate of
     * this round-robin; the default 333us => SRPN-8 every ~1ms drives the cursor ~8x too
     * slow vs the firmware's self-armed CMP0 deadlines (watchdog 0x3045 + slow debounce).
     * Lowering this tests whether a finer cursor cadence lets the debounce complete and
     * reach phase 4 -- confirming the cursor cadence is THE remaining blocker. */
    {
        const char *tn = getenv("TC1797_TICKNS");
        if (tn) {
            int64_t v = (int64_t)strtoll(tn, NULL, 0);
            if (v >= 5000) { s->osek_tick_ns = v; }   /* floor 5us to avoid flooding */
        }
    }
    s->osek_t0_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    /* Bring-up knobs (env): the forced OSEK systick is OPT-IN (TC1797_SYSTICK=1)
     * -- with the SSC PCP-completion model the boot now reaches a STABLE phase 3
     * on its own, and firing the tick at the very start of phase 3 runs off
     * (the OSEK counter/alarm structures the tick ISR walks aren't initialised
     * yet). Keeping it off by default preserves the stable phase-3 boot; opt in
     * to continue the phase 3 -> 4 promotion work. TC1797_SYSTICK_MINPHASE sets
     * the RTOS phase gate (default 3); TC1797_SYSTICK_FLOOR_MS the virtual-time
     * floor (default 50 ms). */
    /* SUPERSEDED / REDUNDANT (2026-06-17): the forced OSEK systick was a pre-CMP0-pin bootstrap crutch.
     * With the faithful CMP0 cold-init tick pin (tc1797_stm_match_ticks) the firmware's OWN STM CMP0
     * compare drives the schedule at its intended 100us cadence from cold-init on, and the phase-3
     * watchdog kick (FUN_800c3c1a) stays inside its 1.9ms window WITHOUT any injected tick. A/B verified:
     * with the forced systick OFF the boot is byte-identical to it ON -- 208 phase-4 samples, 0 DTCs,
     * 0xD000987C climbs to 637, os_running reached at the same ~219ms. So the forced systick is now
     * REDUNDANT (and continuing to run it only double-drives the schedule cursor + watchdog kick). It is
     * DEFAULT-OFF; opt back in for bring-up A/B with TC1797_SYSTICK=1. The old 2026-06-14 rationale
     * (kick starves without it) was true ONLY before the CMP0 pin existed. */
    s->osek_enabled = (getenv("TC1797_SYSTICK") != NULL);
    {
        const char *mp = getenv("TC1797_SYSTICK_MINPHASE");
        const char *fl = getenv("TC1797_SYSTICK_FLOOR_MS");
        s->osek_minphase = mp ? (uint8_t)strtoul(mp, NULL, 0) : 3;
        s->osek_floor_ns = (fl ? (int64_t)strtoll(fl, NULL, 0) : 50) * 1000000LL;
    }
    s->sample_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tc1797_sample_tick, s);
    if (getenv("TC1797_SAMPLE")) {
        timer_mod(s->sample_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000);
    }
    s->freeze_timer = timer_new_ns(QEMU_CLOCK_REALTIME, tc1797_freeze_dump, s);
    if (getenv("TC1797_FREEZEDUMP")) {
        timer_mod(s->freeze_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 800000000);
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

    /* ADC0/1/2 result-register model. Faithful key-on-engine-off baseline:
     * at key-on the analog sensor supplies are energized, so every ADC channel
     * reads a VALID quiescent mid-scale count (RESRn VF=1, RESULT=0x800), NOT the
     * silicon-power-on 0 (which the firmware reads as the 0xffff "no-conversion"
     * sentinel = sensor absent/faulted). With valid inputs the firmware's own
     * read+scale+window-check path completes: sensor mirrors -> ~0x7fff (valid,
     * != 0xffff), the per-channel diagnostic monitors debounce to OK (0x04), the
     * status bytes clear, and the COM health message (event 0x17) + the phase-4
     * promote chain (4f6 -> 4041 -> eb1 -> e7e) can close. This is the chip's
     * real I/O at rest -- NOT a DSPR poke. TC1797_ADC_DEFAULT=<n> overrides the
     * count; TC1797_ADC_OFF restores the silicon-reset-reads-0 behaviour for A/B. */
    tc1797_adc_init(&s->adc, TC1797_ADC0_BASE);
    {
        const char *d = getenv("TC1797_ADC_DEFAULT");
        int count = d ? (int)strtol(d, NULL, 0) : 0x800;
        if (getenv("TC1797_ADC_OFF")) {
            count = -1;                 /* silicon reset: no synthesized word */
        }
        tc1797_adc_set_default(&s->adc, count);
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
        int count = d ? (int)strtol(d, NULL, 0) : 0x100;   /* quiescent knock level */
        if (getenv("TC1797_ADC_OFF")) {
            count = -1;
        }
        tc1797_fadc_set_default(&s->fadc, count);
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

    /* GPTA0/GPTA1/LTCA2 timer arrays. A capture-cell event routes via
     * tc1797_gpta_route, which honours the cell's SRC-node TOS bit (PCP vs CPU). */
    tc1797_gpta_init(&s->gpta, TC1797_GPTA_LO, tc1797_gpta_route, s);
    s->gpta.t0_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->gpta.freerun = (getenv("TC1797_GPTA_FREERUN") != NULL);
    if (getenv("TC1797_GPTATEST")) {
        tc1797_gpta_selftest(tc1797_gpta_route, s);
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

    /* The PCP2 co-processor is part of the TC1797 silicon and is always present;
     * the firmware routes TOS=1 service requests (ADC/GPTA completion, etc.) to it
     * and polls the flags its channels set (e.g. the DTC-0x3006 gate waits on
     * 0xD000416F set by PCP channel 0x1d). Default it ON to function as the chip
     * does; TC1797_NO_PCP opts out for bring-up bisection. */
    s->pcp_enabled = (getenv("TC1797_NO_PCP") == NULL);
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

    if (getenv("TC1797_RSTLOG")) {
        static unsigned rn;
        /* reset-reason struct @ *0xD0018AC8: +0x14=caller retaddr, +0x18=DTC, +0x1f=reason byte */
        uint32_t sp = 0, caller = 0; uint16_t dtc = 0; uint8_t rb = 0, ph = 0;
        cpu_physical_memory_read(0xD0018AC8u, &sp, 4);
        if (sp >= 0xD0000000u && sp < 0xD0020000u) {
            cpu_physical_memory_read(sp + 0x14u, &caller, 4);
            cpu_physical_memory_read(sp + 0x18u, &dtc, 2);
            cpu_physical_memory_read(sp + 0x1fu, &rb, 1);
        }
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        fprintf(stderr, "CPURESET #%u PC=0x%08x caller=0x%08x DTC=0x%04x reason=0x%02x ph=%u\n",
                rn++, (uint32_t)ms->soc.cpu.env.PC, caller, dtc, rb, ph);
        fflush(stderr);
    }
    cpu_reset(cs);
    cpu_set_pc(cs, ms->boot_entry);

    /*
     * TC1797 (TC1.3.1) CPU-core reset vectors. QEMU's generic TriCore reset sets
     * only PSW=0xB80 (correct) and leaves the rest zero; the DAVE device pack
     * (TC1797.REGS) gives the chip's true reset state for the trap-vector base
     * and interrupt stack pointer. The BROM/crt0 reprograms both via MTCR before
     * any trap/interrupt fires, so this is a faithfulness fix (not a behavior
     * change) -- it makes the pre-init core state match silicon for an early
     * trap and for debugger/register inspection.
     */
    {
        CPUTriCoreState *env = &ms->soc.cpu.env;
        env->BTV = 0xA0000100;   /* Base Trap Vector Table Pointer (TC1797.REGS reset) */
        env->ISP = 0x00000100;   /* Interrupt Stack Pointer (TC1797.REGS reset) */
        /*
         * OCDS / debug-active indicators. The MEVD17 firmware DISABLES its real-time
         * task-deadline watchdog (FUN_800c3c68, DTC 0x3045) when running under an
         * on-chip debugger: FUN_800df7c6 arms it only if !(DBGSR.DE && OSCU_OSTATE.bit0
         * && (DAT_d0019a9d==0x55 || SWEVT[2:0]==2)). That watchdog measures task
         * inter-dispatch latency against a sub-2ms deadline, which a non-cycle-accurate
         * emulator (icount, not silicon cycle timing) cannot meet -- exactly the case a
         * hardware debugger creates (breakpoints/single-step perturb timing), so the
         * chip's own logic suppresses the false-trip. QEMU IS that debug environment
         * (and the donor os_running DSPR was itself captured via OCDS), so present the
         * chip as debug-attached: DBGSR.DE=1 (QEMU implements no debug events, so this
         * has no other side effect -- see translate.c DEBUG) and SWEVT[2:0]=2. OSTATE.bit0
         * is already set by the firmware's own OCDS counter (the ERCOSEK system counter).
         * This is the chip's documented behaviour, not a poke of the watchdog state.
         * Env-gated (TC1797_OCDS) while the rest of the timing cascade (DTC 0x3014
         * FUN_800752a6, 0x3006 PCP gate) is still unresolved, so the default build and
         * the validated UDS/CAN paths are unaffected; enabling it eliminates DTC 0x3045
         * and lets the boot run past the deadline watchdog.
         */
        if (getenv("TC1797_OCDS")) {
            env->DBGSR |= MASK_DBGSR_DE;
            env->SWEVT = (env->SWEVT & ~7u) | 2u;
        }
    }

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
        s->ssc_prev[0] = 0;
        s->ssc_prev[1] = 0;
        s->ssc_src[0][0] = s->ssc_src[0][1] = 0;
        s->ssc_src[1][0] = s->ssc_src[1][1] = 0;
        /* Drop any ADC conversions left in flight from the previous boot so they
         * cannot fire their EOC into the freshly re-initialising firmware (the
         * reboot re-arms its result nodes from scratch). */
        memset(s->adc_done_deadline, 0, sizeof(s->adc_done_deadline));
        if (s->adc_done_timer) {
            timer_del(s->adc_done_timer);
        }
        /* The reboot re-runs the firmware's PCP context seeder, re-pinning every
         * channel's entry PC, so each channel cold-starts again on its first
         * post-reset trigger (clear the engine's per-channel started state). */
        if (s->pcp_enabled) {
            pcp_engine_reset(&s->pcp);
        }
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
        /* RSTSTAT reset-cause. FAITHFUL FIX (2026-06-14): on real TC1797 the PORST
         * status bit (bit16) is STICKY (write-1-to-clear) -- a software reset sets the
         * SW-reset cause (bit3) but does NOT clear PORST. The clock-init at 0x80018f16
         * reads RSTSTAT[16] and, when PORST is SET, JUMPS PAST the warm clock-reconfig
         * (the path that ends in the FUN_8001c820 / fn_80019268 "configure-then-reset"
         * write of 0xF0000560=2). Clearing PORST on a SW reset (the old `=0x08`) made the
         * firmware re-enter that reconfig on every warm boot -> endless reset loop at
         * phase 3. Keep PORST asserted across the SW reset (PORST|SWRST) so the firmware
         * sees "already powered+configured", skips the reconfig, and the boot settles. */
        if (s->sw_reset_pending) {
            s->scu_rststat = 0x00010008;   /* PORST (sticky) | SWRST */
            s->sw_reset_pending = false;
        } else {
            s->scu_rststat = 0x00010000;   /* PORST (cold power-on) */
        }
        /* DECISIVE persistence probe: read the self-test persistence markers
         * straight from DSPR RAM at the start of the new boot (before crt0 runs).
         * If the no-init region 0xD0018A70+ persists across the warm reset, a76
         * should hold 0x96 (written by selftest_80148da4's test path on the prior
         * boot) and a8c should hold 0xFFFFFF00; if the reset wiped DSPR they read
         * the 0xFF power-on prefill. This settles the reset-loop root cause. */
        if (getenv("TC1797_MARKERLOG")) {
            uint8_t *dram = memory_region_get_ram_ptr(&s->dspr);
            if (dram) {
                uint32_t a8c = ldl_le_p(dram + 0x18A8C);
                /* Read the firmware crash block (preserved in DSPR) for the reset
                 * that just fired: ptr at 0xD0018AC8, [+0x14]=caller retaddr,
                 * [+0x18]=DTC val, [+0x1f]=reason code. Gives a clean per-reset
                 * (DTC,caller) histogram to pinpoint each looping self-test. */
                uint32_t p = ldl_le_p(dram + 0x18AC8);
                uint32_t ra = 0, val = 0; uint8_t code = 0;
                if (p >= 0xD0000000u && p < 0xD0020000u) {
                    uint32_t off = p - 0xD0000000u;
                    ra   = ldl_le_p(dram + off + 0x14);
                    val  = ldl_le_p(dram + off + 0x18);
                    code = dram[off + 0x1f];
                }
                info_report("tc1797: [reset#] a76=%02x a70=%02x a73=%02x rst=%08x "
                            "| DTC code=%02x val=%04x from=%08x",
                            dram[0x18A76], dram[0x18A70], dram[0x18A73],
                            s->scu_rststat, code, val & 0xffff, ra);
                (void)a8c;
            }
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
     * a TB that branches to itself and peg the CPU forever. Historically we
     * NOP'd the jump so the firmware fell through to the RET. RESOLVED 2026-06-17
     * (see the gated loop below): a clean boot to os_running reaches NONE of these
     * 8 halts, so the patch is now OPT-IN (TC1797_WDT_PATCH=1) and the image runs
     * UNMODIFIED by default -- which also lets the BROM BMI CRC match the real
     * bytes (the FCE oracle in tc1797_sfr_read is then only a fallback).
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
        /* RESOLVED (2026-06-17): the 8 patched sites are all fail-safe HALT/panic/reset paths
         * (FUN_80018e18 fatal panic, FUN_8001c820 reset routine, FUN_801254c6/FUN_80125550 STM-config
         * error halts). A/B verified a CLEAN boot reaches NONE of them: with the patch fully OFF the
         * boot reaches phase 4 + os_running and holds 90s (208 ph4 samples, 0 DTCs, 987c->637, no CPU
         * ever stalls at a J-self PC). The old "first native stall" only happened in the pre-CMP0-pin
         * broken boot. The firmware now runs UNMODIFIED by default -- faithful, and the unmodified image
         * also lets the BROM BMI CRC match (no patched bytes to perturb it). Kept opt-in
         * (TC1797_WDT_PATCH=1) ONLY as a diagnostic for isolating a regressed boot that DOES hit a panic. */
        if (getenv("TC1797_WDT_PATCH") != NULL) {
            for (size_t off = 0; off + 4 <= len; off += 2) {
                if (fw[off] == 0x3C && fw[off + 1] == 0x00 &&
                    fw[off + 2] == 0x00 && fw[off + 3] == 0x90) {
                    fw[off] = 0x00;          /* J disp8=0  ->  NOP (16-bit 0x0000) */
                    fw[off + 1] = 0x00;
                    wdt_patches++;
                }
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
