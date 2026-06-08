/*
 * TC1797 JTAG TAP + Cerberus/OCDS debug interface — native model.
 *
 * The TC1797 debug port is an IEEE 1149.1 JTAG TAP that fronts the "Cerberus"
 * on-chip debug module (the OCDS-L1 system-bus access + run control). A host
 * debugger shifts the device IDCODE, then uses Cerberus to read/write the whole
 * SoC address space and to halt/run the cores — the same capability QEMU's
 * gdbstub already provides at a higher level, but modeled here at the pin/shift
 * level so an *external* tool (OpenOCD via the remote_bitbang transport) can
 * attach to the emulator.
 *
 * This models:
 *   - the full 16-state TAP FSM (clocked by TMS/TDI -> TDO);
 *   - the standard IR (reset selects IDCODE) with IDCODE / BYPASS / SAMPLE +
 *     two Cerberus access instructions;
 *   - IDCODE = 0x1015A083 (matches the SoC JTAGID register), BYPASS = 1 bit;
 *   - the Cerberus data path: a 64-bit {address, data} scan that performs a real
 *     system-bus access via address_space_rw (read/write any SoC address, like a
 *     DAP);
 *   - an OpenOCD remote_bitbang server over a QEMU chardev (opt-in).
 *
 * Run-control (halt/run/single-step) is provided by QEMU's gdbstub, which is the
 * functional equivalent of the Cerberus run-control path; the OCDS core-debug
 * trigger registers (DBGSR/EXEVT/CREVT/DBGTCR) live in the CPU model
 * (target/tricore). This file adds the pin-level TAP + the Cerberus bus master.
 *
 * Ground truth: IEEE 1149.1 (TAP FSM, BYPASS/IDCODE semantics) + TC1797 OCDS
 * overview. The exact OCDS-L1 IR opcodes / Cerberus shift framing are in the
 * gated User Manual; IDCODE and BYPASS follow the IEEE standard (so external
 * IDCODE probing is tool-agnostic), and the Cerberus access opcodes/framing
 * here are a documented, self-consistent model.
 */
#ifndef HW_TRICORE_TC1797_JTAG_H
#define HW_TRICORE_TC1797_JTAG_H

#include "chardev/char-fe.h"

#define TC1797_JTAG_IDCODE 0x1015A083u   /* TC1797 device IDCODE (== JTAGID SFR; DAVE TC1797.REGS) */
#define TC1797_JTAG_IR_WIDTH 8

/* TAP FSM states (IEEE 1149.1 order). */
enum {
    TAP_TLR, TAP_RTI,
    TAP_SEL_DR, TAP_CAP_DR, TAP_SH_DR, TAP_EX1_DR, TAP_PAU_DR, TAP_EX2_DR, TAP_UPD_DR,
    TAP_SEL_IR, TAP_CAP_IR, TAP_SH_IR, TAP_EX1_IR, TAP_PAU_IR, TAP_EX2_IR, TAP_UPD_IR,
};

/* Instruction register opcodes. IDCODE/BYPASS are IEEE-standard semantics
 * (reset selects IDCODE; BYPASS = all-ones). CERB_* are model-defined. */
enum {
    IR_EXTEST   = 0x00,
    IR_SAMPLE   = 0x01,
    IR_IDCODE   = 0x02,
    IR_CERB_WR  = 0x0C,   /* Cerberus: scan {addr[63:32],data[31:0]} -> bus write */
    IR_CERB_RD  = 0x0D,   /* Cerberus: scan addr -> bus read; rescan to shift out */
    IR_BYPASS   = 0xFF,   /* all-ones (IEEE mandatory) */
};

typedef struct Tc1797Jtag {
    /* TAP */
    int      state;
    uint32_t idcode;
    uint8_t  ir;                  /* current (latched) instruction */
    uint8_t  ir_shift;            /* IR shift register */
    uint64_t dr_shift;            /* DR shift register (IDCODE 32 / BYPASS 1 / Cerberus 64) */
    int      dr_len;              /* selected DR length */
    int      tdo;                 /* last TDO presented */
    uint32_t bsr;                 /* minimal boundary-scan register (pin sample) */

    /* Cerberus debug module (system-bus access path) */
    uint32_t cer_addr;            /* last access address */
    uint32_t cer_result;          /* last read result (shifted out next scan) */
    uint64_t cer_reads, cer_writes;   /* telemetry */

    /* remote_bitbang transport (OpenOCD) over a chardev (optional). */
    CharFrontend chr;
    bool     have_chr;
    int      last_tck;
} Tc1797Jtag;

void tc1797_jtag_init(Tc1797Jtag *j, uint32_t idcode);
/* Bind an OpenOCD remote_bitbang server to a chardev (e.g. -chardev socket,id=jtag,...). */
void tc1797_jtag_attach_chardev(Tc1797Jtag *j, Chardev *chr);
/* One TCK cycle: advance the TAP with TMS/TDI; returns the TDO bit shifted out. */
int  tc1797_jtag_clock(Tc1797Jtag *j, int tms, int tdi);
/* Current TDO presented in a shift state (for the remote_bitbang 'R' read). */
int  tc1797_jtag_tdo(Tc1797Jtag *j);
/* Force the TAP to Test-Logic-Reset (TRST / 5x TMS=1). */
void tc1797_jtag_reset(Tc1797Jtag *j);

void tc1797_jtag_selftest(void);

#endif /* HW_TRICORE_TC1797_JTAG_H */
