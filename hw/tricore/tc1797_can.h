/*
 * Infineon TC1797 MultiCAN controller — native model.
 *
 * Register file @0xF0004000 (kernel/node/list) + 128 message objects @0xF0005000
 * (MOFCR/MOFGPR/MOIPR/MOAMR/MODR0/MODR1/MOAR/MOCTR, 0x20 apart; TC1797 UM
 * Table 19-6). Ported from the validated bridge MultiCAN model: TX on MOCTR
 * SET-TXRQ, RX delivery into a matching receive MO (NEWDAT), and the PANCTR
 * list/allocation panel. The on-the-wire side is exposed through a TX callback
 * + an RX-inject entry point so the SoC can bridge to a CAN bus / test driver.
 */
#ifndef HW_TRICORE_TC1797_CAN_H
#define HW_TRICORE_TC1797_CAN_H

#define TC1797_CAN_BASE     0xF0004000u
#define TC1797_CAN_END      0xF0006000u    /* exclusive (kernel regs + 128 MOs) */
#define TC1797_CAN_MO_BASE  0xF0005000u
#define TC1797_CAN_PANCTR   0xF00041C4u
#define TC1797_CAN_NREGS    0x800u          /* (END-BASE)/4 word shadow */
#define TC1797_CAN_NMO      128

/* Frame emitted by the firmware (MOCTR SET-TXRQ). */
typedef void (*CanTxFn)(void *opaque, uint32_t can_id,
                        const uint8_t *data, unsigned len);

typedef struct Tc1797CanMO {
    uint32_t id;            /* arbitration ID (11-bit standard) */
    uint32_t mask;          /* acceptance mask                  */
    uint8_t  dir;           /* 0 = receive, 1 = transmit        */
    bool     configured;    /* MOAR has been written            */
    bool     rx_pending;    /* a frame is waiting to be read    */
    uint32_t rx_id;
    uint8_t  rx_data[8];
} Tc1797CanMO;

typedef struct Tc1797Can {
    uint32_t base;                     /* kernel base (message objects at base + 0x1000) */
    uint32_t regs[TC1797_CAN_NREGS];   /* register shadow (config read-back) */
    Tc1797CanMO mo[TC1797_CAN_NMO];
    uint8_t mo_list[TC1797_CAN_NMO];   /* PANCTR list membership (0=unalloc) */
    CanTxFn tx_cb;
    void *tx_opaque;
    uint64_t tx_count, rx_count;
} Tc1797Can;

void tc1797_can_init(Tc1797Can *c, uint32_t base, CanTxFn tx_cb, void *opaque);
uint32_t tc1797_can_read(Tc1797Can *c, uint32_t addr);
void tc1797_can_write(Tc1797Can *c, uint32_t addr, uint32_t val);
/* Deliver an inbound frame into a matching receive MO. Returns true if a
 * configured RX object accepted it. Safe to call from any thread holding BQL. */
bool tc1797_can_rx_inject(Tc1797Can *c, uint32_t can_id,
                          const uint8_t *data, unsigned len);

/* Opt-in self-test (TC1797_CANTEST=1): exercises TX + RX through the model. */
void tc1797_can_selftest(CanTxFn tx_cb, void *opaque);

#endif /* HW_TRICORE_TC1797_CAN_H */
