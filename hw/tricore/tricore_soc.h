/*
 * TriCore SoC descriptor — the per-variant "spec" that drives a generic SoC.
 *
 * A TriCore part (TC1796/TC1797/.../TC27x/TC37x) is described by a table of
 * peripheral INSTANCES, each naming an IP *implementation* (its `kind`) plus the
 * base address it sits at. An IP-kind registry maps `kind` -> device model, so
 * different parts select different models for the same peripheral class — e.g.
 * CAN is MULTICAN on TC1.3 / TC2xx but Bosch M_CAN (CAN-FD) on TC3xx, ADC is the
 * SAR ADC on TC1.3 but VADC on AURIX, the timer array is GPTA on TC1.3 but GTM on
 * AURIX. Base-relocation of a shared model only covers "same IP, different base"
 * (the within-generation case); cross-generation parts list a different `kind`.
 *
 * Per-part register quirks that are firmware bring-up logic rather than generic
 * IP (SCU clock/PLL/reset, DTS, flash/PMU status) stay in the part's SoC file,
 * not in this descriptor.
 */
#ifndef HW_TRICORE_TRICORE_SOC_H
#define HW_TRICORE_TRICORE_SOC_H

/* IP-implementation kinds selectable per peripheral instance. Add a new kind
 * when a part needs a different model for a peripheral class (e.g. TC_IP_MCAN
 * for TC3xx CAN-FD, TC_IP_GTM for the AURIX timer module, TC_IP_VADC ...). */
typedef enum TriCoreIpKind {
    TC_IP_NONE = 0,
    TC_IP_MULTICAN,   /* Infineon MultiCAN / MultiCAN+ (TC1.3 / TC2xx) */
    TC_IP_ADC,        /* TC1.3 SAR ADC kernels */
    TC_IP_FADC,       /* Fast ADC */
    TC_IP_GPTA,       /* General Purpose Timer Array (TC1.3) */
    TC_IP_DMA,        /* DMA controller */
    TC_IP_ERAY,       /* E-Ray (FlexRay) communication controller */
    TC_IP_ASC,        /* ASC (asynchronous/synchronous serial) UART */
    TC_IP_MLI,        /* Micro Link Interface */
} TriCoreIpKind;

/* One peripheral instance in a part descriptor: an IP implementation placed at
 * a base address. `unit` distinguishes multiple instances of the same kind
 * (ASC0/1, MLI0/1); `id` carries a module-ID for IPs that expose one. */
typedef struct TriCorePeriphCfg {
    TriCoreIpKind kind;
    uint32_t      base;
    uint32_t      size;     /* register window; range is [base, base + size) */
    int           unit;
    uint32_t      id;
} TriCorePeriphCfg;

#endif /* HW_TRICORE_TRICORE_SOC_H */
