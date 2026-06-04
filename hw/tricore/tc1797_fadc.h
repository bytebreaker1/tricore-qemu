/*
 * TC1797 Fast ADC (FADC) — native result-register model.
 *
 * The TC1797 carries one Fast ADC (10/12-bit, high sample rate) in addition to
 * the three SAR kernels (ADC0/1/2). This firmware uses it for knock sensing:
 * it writes a per-channel conversion count, then the firmware's own read+scale
 * path consumes it (the faithful alternative to DSPR-level injection used by
 * the bridge reference, bridge_server.py knock-burst path at 0xF0100500+ch*4).
 *
 * Base 0xF0100400 (UM Table 8-3 module map). Result registers at +0x100+ch*4
 * (RESRx): VF[31] valid, RESULT in the low bits. CLC(+0x00) DISR/DISS enable,
 * module ID(+0x08). Other registers are coherent config read-back.
 */
#ifndef HW_TRICORE_TC1797_FADC_H
#define HW_TRICORE_TC1797_FADC_H

#define TC1797_FADC_BASE 0xF0100400u
#define TC1797_FADC_SIZE 0x200u          /* regs (0x000..) + result block (0x100..) */
#define TC1797_FADC_NREG (TC1797_FADC_SIZE >> 2)
#define TC1797_FADC_NCH  8               /* result slots at +0x100 + ch*4 */

typedef struct Tc1797Fadc {
    int32_t  input[TC1797_FADC_NCH];     /* persistent channel count, -1 = none */
    int32_t  default_count;              /* idle baseline, -1 = silicon reset (read 0) */
    uint32_t regs[TC1797_FADC_NREG];     /* CLC + config read-back */
} Tc1797Fadc;

void tc1797_fadc_init(Tc1797Fadc *f);
uint32_t tc1797_fadc_read(Tc1797Fadc *f, uint32_t addr);
void tc1797_fadc_write(Tc1797Fadc *f, uint32_t addr, uint32_t val);
/* Persistent per-channel conversion count (e.g. live knock amplitude). */
void tc1797_fadc_set_input(Tc1797Fadc *f, int ch, int count);
/* Idle baseline for un-set channels (-1 = silicon reset / read 0). */
void tc1797_fadc_set_default(Tc1797Fadc *f, int count);

void tc1797_fadc_selftest(void);

#endif /* HW_TRICORE_TC1797_FADC_H */
