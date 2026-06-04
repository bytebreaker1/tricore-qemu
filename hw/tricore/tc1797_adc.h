/*
 * TC1797 ADC kernels (ADC0/1/2) — faithful result-register model.
 *
 * Ported from bridge/tc1797_adc.py. The firmware reads 12-bit conversion
 * results from RESRn and scales them to sensor values (e.g. FUN_8014416A).
 * A conversion count injected here flows through the firmware's own ADC
 * read+scale path — the faithful alternative to DSPR-level sensor injection.
 *
 * Per-kernel offsets (TC1797 UM ch.24): 0x038 status; 0x180+n*4 RESRn (read
 * clears VF); 0x1C0+n*4 RESRDn (debug read, VF preserved). RESRn fields:
 * VF[31] valid, CHNR[27:24] channel, RESULT[13:0].
 */
#ifndef HW_TRICORE_TC1797_ADC_H
#define HW_TRICORE_TC1797_ADC_H

#define TC1797_ADC0_BASE  0xF0101000u
#define TC1797_ADC_KSIZE  0x400u
#define TC1797_ADC_NKERN  3
#define TC1797_ADC_NRES   16
#define TC1797_ADC_LO     0xF0101000u
#define TC1797_ADC_HI     0xF0101C00u    /* 0xF0101000 + 3*0x400 (exclusive) */

typedef struct AdcKernel {
    uint16_t result[TC1797_ADC_NRES];    /* one-shot RESULT[13:0]            */
    uint8_t  vf[TC1797_ADC_NRES];        /* valid flag (one-shot)            */
    uint8_t  chnr[TC1797_ADC_NRES];      /* channel number                   */
    int32_t  input[TC1797_ADC_NRES];     /* persistent continuous input, -1=none */
    int32_t  default_count;              /* idle baseline, -1 = read 0       */
    uint32_t shadow[TC1797_ADC_KSIZE >> 2];
} AdcKernel;

typedef struct Tc1797Adc {
    AdcKernel k[TC1797_ADC_NKERN];
} Tc1797Adc;

void tc1797_adc_init(Tc1797Adc *a);
bool tc1797_adc_is_addr(uint32_t addr);
uint32_t tc1797_adc_read(Tc1797Adc *a, uint32_t addr);
void tc1797_adc_write(Tc1797Adc *a, uint32_t addr, uint32_t val);
/* Persistent analog input (continuous conversion) on kernel.RESRn. */
void tc1797_adc_set_input(Tc1797Adc *a, int kernel, int n, int count, int chnr);
/* Idle baseline count for every un-set channel (-1 = silicon reset / read 0). */
void tc1797_adc_set_default(Tc1797Adc *a, int count);

void tc1797_adc_selftest(void);

#endif /* HW_TRICORE_TC1797_ADC_H */
