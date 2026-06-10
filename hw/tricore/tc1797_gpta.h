/*
 * TC1797 GPTA0/GPTA1 + LTCA2 timer arrays — native timer/capture model.
 *
 * Register blocks (TC1797 UM segment-15 map): GPTA0 0xF0001800, GPTA1
 * 0xF0002000, LTCA2 0xF0002800 (2 KB each). The GPTA drives engine-timing:
 * crank/cam capture (input) and ignition/injection compare (output).
 *
 * This models the register space as a config read-back shadow plus free-running
 * cell timers driven from the virtual clock (so the firmware's timer polls see
 * advancing counts like silicon), and exposes a capture-injection entry point
 * so a crank/cam edge can latch a capture register and raise its service
 * request — the path the bridge drives post-phase-4 for engine position.
 */
#ifndef HW_TRICORE_TC1797_GPTA_H
#define HW_TRICORE_TC1797_GPTA_H

#define TC1797_GPTA_LO   0xF0001800u
#define TC1797_GPTA_HI   0xF0003000u    /* GPTA0+GPTA1+LTCA2 (exclusive)     */
#define TC1797_GPTA_NREG ((TC1797_GPTA_HI - TC1797_GPTA_LO) >> 2)

/*
 * Uniform per-module layout (each module is 0x800 wide; see TC1797_GPTA_FINDINGS.md):
 *   base + 0x100 + N*8 : GTCCTRn (+0) / GTCXRn (+4)   N = 0..31
 *   base + 0x200 + N*8 : LTCCTRn (+0) / LTCXRn (+4)   N = 0..63 (LTCA2: 0..31)
 *   base + 0x7FC - N*4 : SRCn (service request node)  N = 0..37 (GPTA), 0..7 (LTCA2)
 */
#define GPTA_MOD_STRIDE  0x800u
#define GPTA_GTC_OFF     0x100u   /* first GTC cell control, within a module */
#define GPTA_LTC_OFF     0x200u   /* first LTC cell control, within a module */
#define GPTA_SRC_TOP     0x7FCu   /* SRC node 0 (others below at -4 each)    */

/* LTCA2 is the input-multiplexed crank/cam capture array; both engine-position
 * master nodes feed PCP channel SRPN 19. node = LTC cell & 7 (8-node array). */
#define GPTA_LTCA2_OFF   (2u * GPTA_MOD_STRIDE)         /* 0x1000: LTCA2 base */
#define GPTA_CRANK_CELL  2u   /* LTCA2 LTC cell 2 -> SRC02 (crank)            */
#define GPTA_CAM_CELL    3u   /* LTCA2 LTC cell 3 -> SRC03 (cam)              */

/*
 * A capture/compare cell raised a service request. Route it the way the SRN's
 * TOS bit dictates: to_pcp=1 -> PCP channel (pcp_engine_trigger); to_pcp=0 ->
 * CPU interrupt at this SRPN. Implemented by the SoC, which owns both paths.
 */
typedef void (*GptaRouteFn)(void *opaque, uint8_t srpn, bool to_pcp);

typedef struct Tc1797Gpta {
    uint32_t shadow[TC1797_GPTA_NREG];   /* config + capture-latch read-back */
    uint8_t  written[TC1797_GPTA_NREG];  /* 1 once firmware wrote the offset */
    uint64_t t0_ns;                      /* virtual-clock epoch for free-run */
    bool     freerun;                    /* expose free-running cell timers   */
    GptaRouteFn route_fn;
    void *route_opaque;
    uint32_t base;                       /* block base (relocatable per part) */
} Tc1797Gpta;

void tc1797_gpta_init(Tc1797Gpta *g, uint32_t base, GptaRouteFn route_fn, void *opaque);
uint32_t tc1797_gpta_read(Tc1797Gpta *g, uint32_t addr, uint64_t now_ns);
void tc1797_gpta_write(Tc1797Gpta *g, uint32_t addr, uint32_t val);
/*
 * Model a capture-cell input edge: latch `value` into the cell's capture
 * register at `cell_off` (offset from TC1797_GPTA_LO), then route the service
 * request to the SRC node that cell is wired to. The cell's firmware-configured
 * SRC node (SRPN/TOS/SRE) is read back from the shadow; if SRE=1 it routes per
 * TOS (PCP or CPU). No-op for an un-armed (SRE=0) node — faithful to silicon.
 */
void tc1797_gpta_capture(Tc1797Gpta *g, uint32_t cell_off, uint32_t value);

void tc1797_gpta_selftest(GptaRouteFn route_fn, void *opaque);

#endif /* HW_TRICORE_TC1797_GPTA_H */
