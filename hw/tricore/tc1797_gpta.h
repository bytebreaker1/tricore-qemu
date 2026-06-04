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

/* Capture/compare cell raised a service request at this SRPN. */
typedef void (*GptaIrqFn)(void *opaque, uint8_t srpn);

typedef struct Tc1797Gpta {
    uint32_t shadow[TC1797_GPTA_NREG];   /* config + capture-latch read-back */
    uint8_t  written[TC1797_GPTA_NREG];  /* 1 once firmware wrote the offset */
    uint64_t t0_ns;                      /* virtual-clock epoch for free-run */
    bool     freerun;                    /* expose free-running cell timers   */
    GptaIrqFn irq_fn;
    void *irq_opaque;
} Tc1797Gpta;

void tc1797_gpta_init(Tc1797Gpta *g, GptaIrqFn irq_fn, void *opaque);
uint32_t tc1797_gpta_read(Tc1797Gpta *g, uint32_t addr, uint64_t now_ns);
void tc1797_gpta_write(Tc1797Gpta *g, uint32_t addr, uint32_t val);
/* Latch `value` into the capture register at `cell_off` (offset within the
 * GPTA block) and raise `srpn` — model of a crank/cam input edge. */
void tc1797_gpta_capture(Tc1797Gpta *g, uint32_t cell_off,
                         uint32_t value, uint8_t srpn);

void tc1797_gpta_selftest(GptaIrqFn irq_fn, void *opaque);

#endif /* HW_TRICORE_TC1797_GPTA_H */
