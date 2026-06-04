/*
 * TC1797 ASC0/ASC1 (Asynchronous/Synchronous Serial Interface = UART) —
 * native, chardev-backed.
 *
 * ASC0 @0xF0000A00, ASC1 @0xF0000B00. Functional UART: TBUF write transmits to
 * the attached QEMU chardev (-serial); inbound chardev bytes fill an RX FIFO,
 * set the RX status, and raise the RX service request. Register set follows the
 * TC1796/TC1797 ASC layout (CON/BG/FDV/TBUF/RBUF/FSTAT + TSRC/RSRC/ESRC SRNs).
 * General-purpose (also serves the BROM's ASC bootstrap-loader RX path).
 */
#ifndef HW_TRICORE_TC1797_ASC_H
#define HW_TRICORE_TC1797_ASC_H

#include "chardev/char-fe.h"
#include "qemu/fifo8.h"

#define TC1797_ASC0_BASE 0xF0000A00u
#define TC1797_ASC1_BASE 0xF0000B00u
#define TC1797_ASC_SIZE  0x100u
#define TC1797_ASC_NREG  (TC1797_ASC_SIZE >> 2)
#define TC1797_ASC_RXFIFO 16

typedef void (*AscIrqFn)(void *opaque, uint8_t srpn);

typedef struct Tc1797Asc {
    uint32_t      base;
    CharFrontend  chr;
    Fifo8         rx;
    uint32_t      regs[TC1797_ASC_NREG];
    AscIrqFn      irq_fn;
    void         *irq_opaque;
} Tc1797Asc;

/* chr may be NULL (UART present but unconnected). */
void tc1797_asc_init(Tc1797Asc *a, uint32_t base, Chardev *chr,
                     AscIrqFn irq_fn, void *opaque);
uint32_t tc1797_asc_read(Tc1797Asc *a, uint32_t addr);
void tc1797_asc_write(Tc1797Asc *a, uint32_t addr, uint32_t val);

#endif /* HW_TRICORE_TC1797_ASC_H */
