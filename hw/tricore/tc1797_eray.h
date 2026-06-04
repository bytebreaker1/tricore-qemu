/*
 * TC1797 E-Ray (FlexRay Communication Controller) — POC state model.
 *
 * Ported from bridge/tc1797_eray.py. Kernel register block at 0xF0010000 (4 KB).
 * The firmware drives the Protocol Operation Control (POC) by writing SUCC1.CMD
 * and polling CCSV.POCS (hot loop at firmware 0x8009AC9C, phase 4). This models
 * the command->status state machine so that bring-up sequence advances like
 * silicon. (The S55 immobilizer/CP handshake transport rides on E-Ray.)
 */
#ifndef HW_TRICORE_TC1797_ERAY_H
#define HW_TRICORE_TC1797_ERAY_H

#define TC1797_ERAY_BASE 0xF0010000u
#define TC1797_ERAY_SIZE 0x1000u
#define TC1797_ERAY_NREG (TC1797_ERAY_SIZE >> 2)

typedef struct ErayCC {
    uint32_t regs[TC1797_ERAY_NREG];   /* offset/4 shadow */
    uint8_t  pocs;                     /* POC status (CCSV.POCS[5:0]) */
    uint8_t  last_cmd;
    uint32_t cmd_count;
    uint32_t base;                     /* module base address (relocatable per part) */
} ErayCC;

void tc1797_eray_init(ErayCC *e, uint32_t base);
uint32_t tc1797_eray_read(ErayCC *e, uint32_t addr);
void tc1797_eray_write(ErayCC *e, uint32_t addr, uint32_t val);
void tc1797_eray_selftest(void);

#endif /* HW_TRICORE_TC1797_ERAY_H */
