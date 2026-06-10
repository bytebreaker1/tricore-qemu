/*
 * TC1797 GPTA0/GPTA1/LTCA2 timer arrays — native model (see tc1797_gpta.h).
 */
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "tc1797_gpta.h"

void tc1797_gpta_init(Tc1797Gpta *g, uint32_t base, GptaRouteFn route_fn, void *opaque)
{
    memset(g, 0, sizeof(*g));
    g->base = base;
    g->route_fn = route_fn;
    g->route_opaque = opaque;
}

static inline uint32_t gpta_idx(uint32_t addr)
{
    return (addr - TC1797_GPTA_LO) >> 2;
}

uint32_t tc1797_gpta_read(Tc1797Gpta *g, uint32_t addr, uint64_t now_ns)
{
    addr = TC1797_GPTA_LO + (addr - g->base);   /* relocate to IP-relative space */
    uint32_t idx = gpta_idx(addr);
    if (idx >= TC1797_GPTA_NREG) {
        return 0;
    }
    /*
     * Config / capture-latch registers the firmware wrote read back exactly.
     * An un-written register reads 0 by default (silicon reset; the firmware's
     * boot polls tolerate this). Free-running cell timers (a virtual-clock
     * count, 100 MHz) are opt-in (g->freerun): exposing an advancing value on
     * EVERY unwritten register perturbs a firmware status branch and stalls the
     * boot, so it is gated rather than default.
     */
    uint32_t rv;
    if (g->written[idx]) {
        rv = g->shadow[idx];
    } else if (g->freerun) {
        rv = (uint32_t)((now_ns - g->t0_ns) / 10u);   /* 10 ns == 100 MHz */
    } else {
        rv = 0;
    }
    if (getenv("TC1797_GPTALOG")) {
        static unsigned gc;
        if (gc++ < 6000) {
            fprintf(stderr, "GPTAR [0x%08x] = 0x%08x\n", (uint32_t)addr, rv);
            fflush(stderr);
        }
    }
    return rv;
}

void tc1797_gpta_write(Tc1797Gpta *g, uint32_t addr, uint32_t val)
{
    addr = TC1797_GPTA_LO + (addr - g->base);
    uint32_t idx = gpta_idx(addr);
    if (idx < TC1797_GPTA_NREG) {
        g->shadow[idx] = val;
        g->written[idx] = 1;
    }
    /*
     * Software service request on a service-request node (per-module SRC region
     * 0x768..0x7FC, node N at 0x7FC - N*4). The firmware programs the node with
     * SRPN/TOS/SRE then pulses SETR[15] to raise the request in software -- the
     * GPTA analogue of the ADC/SSC SRC SETR the SoC already routes. The GPTA/ADC
     * init driver (FUN_800fbabc) does exactly this: it writes GPTA0_SRC20
     * (0xF0001FAC) = 0x940f (SRPN 15, SRE=1, TOS=1, SETR=1) to fire PCP channel
     * 15 -- the operational dispatch channel (cold PC 0x938) that walks the
     * multi-channel state machine and writes the DTC-0x3006 gate flag
     * 0xD000416F. Route it exactly like a capture-cell event: TOS=1 -> PCP
     * channel SRPN, TOS=0 -> CPU interrupt SRPN. Only a SETR pulse on an
     * SRE-enabled node fires (config writes without SETR are shadow-only), so
     * the firmware's many plain SRC config writes are unaffected. Without this
     * the SETR is dropped and the dispatch channel never runs, so the boot
     * reset-loops on the 0x3006 gate.
     */
    uint32_t in_mod = (addr - TC1797_GPTA_LO) & (GPTA_MOD_STRIDE - 1u);
    if (in_mod >= GPTA_SRC_TOP - 37u * 4u && in_mod <= GPTA_SRC_TOP
        && (val & 0x8000u)                       /* SETR: software trigger   */
        && (val & 0x1000u)                       /* SRE: node enabled        */
        && g->route_fn) {
        g->route_fn(g->route_opaque, val & 0xFFu, (val >> 10) & 1u);  /* TOS */
    }
}

void tc1797_gpta_capture(Tc1797Gpta *g, uint32_t cell_off, uint32_t value)
{
    uint32_t idx = cell_off >> 2;
    if (idx < TC1797_GPTA_NREG) {
        g->shadow[idx] = value;     /* latch the captured timer value (LTCXR) */
        g->written[idx] = 1;
    }
    /*
     * Route the cell's service request to the SRC node it is wired to. Within a
     * module, GTC/LTC cell N's SR line maps to SRC node (N & 7) (the LTC-array's
     * 8-line grouping — see TC1797_GPTA_FINDINGS.md). The SRC node register the
     * firmware configured for that node lives in the shadow (the firmware's
     * SRPN/TOS/SRE write flowed through tc1797_gpta_write); read it back and
     * route per TOS, only if the node is enabled (SRE=1) — exactly as silicon.
     */
    uint32_t mbase  = cell_off & ~(GPTA_MOD_STRIDE - 1u);
    uint32_t in_mod = cell_off &  (GPTA_MOD_STRIDE - 1u);
    uint32_t cell;
    if (in_mod >= GPTA_LTC_OFF && in_mod < GPTA_LTC_OFF + 64u * 8u) {
        cell = (in_mod - GPTA_LTC_OFF) / 8u;
    } else if (in_mod >= GPTA_GTC_OFF && in_mod < GPTA_GTC_OFF + 32u * 8u) {
        cell = (in_mod - GPTA_GTC_OFF) / 8u;
    } else {
        return;                     /* not a capture-cell offset */
    }
    uint32_t src_idx = (mbase + GPTA_SRC_TOP - (cell & 7u) * 4u) >> 2;
    uint32_t src = (src_idx < TC1797_GPTA_NREG) ? g->shadow[src_idx] : 0;
    if (((src >> 12) & 1u) && g->route_fn) {            /* SRE armed */
        g->route_fn(g->route_opaque, src & 0xFFu, (src >> 10) & 1u);  /* TOS */
    }
}

/* ───────────────────────── opt-in self-test ─────────────────────────────── */
static unsigned gpta_test_srpn;
static bool     gpta_test_pcp;
static void gpta_test_route(void *o, uint8_t srpn, bool to_pcp)
{
    (void)o; gpta_test_srpn = srpn; gpta_test_pcp = to_pcp;
}

void tc1797_gpta_selftest(GptaRouteFn route_fn, void *opaque)
{
    Tc1797Gpta g;
    unsigned pass = 0, fail = 0;
#define CHK(c) do { if (c) pass++; else { fail++; \
        error_report("GPTA selftest FAIL @%d", __LINE__); } } while (0)

    tc1797_gpta_init(&g, TC1797_GPTA_LO, gpta_test_route, NULL);
    g.freerun = true;

    /* free-running timer advances with virtual time */
    uint32_t t_a = tc1797_gpta_read(&g, 0xF0002020, 1000);
    uint32_t t_b = tc1797_gpta_read(&g, 0xF0002020, 100000);
    CHK(t_b > t_a);

    /* config write reads back exactly (no longer free-running) */
    tc1797_gpta_write(&g, 0xF0002020, 0xCAFE1234);
    CHK(tc1797_gpta_read(&g, 0xF0002020, 999999) == 0xCAFE1234);

    /* Arm LTCA2_SRC02 as the firmware does (PCP, SRPN 19, SRE=1), then a crank
     * capture must latch its LTCXR and route to PCP channel 19. */
    tc1797_gpta_write(&g, 0xF0002FF4, 0x00001413);   /* LTCA2_SRC02: SRE|TOS|19 */
    gpta_test_srpn = 0; gpta_test_pcp = false;
    tc1797_gpta_capture(&g, GPTA_LTCA2_OFF + GPTA_LTC_OFF + GPTA_CRANK_CELL * 8u + 4u,
                        0x00ABCDEF);
    CHK(gpta_test_srpn == 19 && gpta_test_pcp
        && tc1797_gpta_read(&g, 0xF0002A14, 0) == 0x00ABCDEF);

    /* An un-armed node (SRE=0) must NOT route — faithful to silicon. */
    tc1797_gpta_write(&g, 0xF0002FF0, 0x00000413);   /* LTCA2_SRC03: TOS|19, SRE=0 */
    gpta_test_srpn = 0xFF;
    tc1797_gpta_capture(&g, GPTA_LTCA2_OFF + GPTA_LTC_OFF + GPTA_CAM_CELL * 8u + 4u,
                        0x00001234);
    CHK(gpta_test_srpn == 0xFF);                      /* route_fn not called */

    info_report("tc1797 GPTA selftest: %u passed, %u failed", pass, fail);
    (void)route_fn; (void)opaque;
#undef CHK
}
