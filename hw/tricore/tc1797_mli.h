/*
 * TC1797 MLI0/MLI1 — Micro Link Interface (Micro Link Serial Bus) model.
 *
 * The MLI is a high-speed serial bus that lets one TriCore transparently access
 * another chip's address space (inter-controller link). Bases MLI0 0xF010C000,
 * MLI1 0xF010C100 (UM Table 8-3 module map). Its transmit/receive datapath is an
 * inter-chip link not exercised by a single-controller, internal-flash
 * application; this models the firmware-visible programming surface: CLC enable
 * (DISR/DISS), the module identification register, and coherent config
 * read-back. (The TX/RX serial datapath would need a second MLI peer to model
 * meaningfully and has no effect in a single-device emulation.)
 */
#ifndef HW_TRICORE_TC1797_MLI_H
#define HW_TRICORE_TC1797_MLI_H

#define TC1797_MLI0_BASE 0xF010C000u
#define TC1797_MLI1_BASE 0xF010C100u
#define TC1797_MLI_SIZE  0x100u
#define TC1797_MLI_NREG  (TC1797_MLI_SIZE >> 2)

typedef struct Tc1797Mli {
    uint32_t base;
    uint32_t id;                          /* module identification (read-only) */
    uint32_t regs[TC1797_MLI_NREG];
} Tc1797Mli;

void tc1797_mli_init(Tc1797Mli *m, uint32_t base, uint32_t id);
uint32_t tc1797_mli_read(Tc1797Mli *m, uint32_t addr);
void tc1797_mli_write(Tc1797Mli *m, uint32_t addr, uint32_t val);
void tc1797_mli_selftest(void);

#endif /* HW_TRICORE_TC1797_MLI_H */
