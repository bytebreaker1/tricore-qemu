/*
 * TC1797 MultiCAN controller — native model (see tc1797_can.h).
 * Faithful port of the validated bridge MultiCAN model.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"
#include "tc1797_can.h"

/* PANCTR list-command opcodes (TC1797 UM Table 19-7). */
enum {
    PANCMD_NOP = 0x00, PANCMD_INIT_LIST = 0x01, PANCMD_STATIC_ALLOC = 0x02,
    PANCMD_DYNAMIC_ALLOC = 0x03, PANCMD_STATIC_INS_BEFORE = 0x04,
    PANCMD_DYNAMIC_INS_BEFORE = 0x05, PANCMD_STATIC_INS_BEHIND = 0x06,
    PANCMD_DYNAMIC_INS_BEHIND = 0x07,
};

static inline uint32_t can_idx(uint32_t addr)
{
    return (addr - TC1797_CAN_BASE) >> 2;
}
/* word index of MO `n` register at byte offset `moff` */
static inline uint32_t mo_word(unsigned n, unsigned moff)
{
    return (0x1000u + n * 0x20u + moff) >> 2;
}

void tc1797_can_init(Tc1797Can *c, uint32_t base, CanTxFn tx_cb, void *opaque)
{
    memset(c, 0, sizeof(*c));
    c->base = base;
    c->tx_cb = tx_cb;
    c->tx_opaque = opaque;
}

/* Execute a MultiCAN list/allocation command and complete PANCTR (BUSY->0). */
static void can_panel_command(Tc1797Can *c, uint32_t panctr)
{
    unsigned cmd = panctr & 0xFF;
    unsigned panar1 = (panctr >> 16) & 0xFF;
    unsigned panar2 = (panctr >> 24) & 0xFF;
    unsigned err = 0, result_mo = panar1;

    switch (cmd) {
    case PANCMD_INIT_LIST:
        for (int n = 0; n < TC1797_CAN_NMO; n++) {
            c->mo_list[n] = 0;
        }
        break;
    case PANCMD_STATIC_ALLOC:
        if (panar1 < TC1797_CAN_NMO) {
            c->mo_list[panar1] = panar2;
        } else {
            err = 1;
        }
        break;
    case PANCMD_DYNAMIC_ALLOC:
    case PANCMD_DYNAMIC_INS_BEFORE:
    case PANCMD_DYNAMIC_INS_BEHIND: {
        int mo = -1;
        for (int n = 0; n < TC1797_CAN_NMO; n++) {
            if (c->mo_list[n] == 0) { mo = n; break; }
        }
        if (mo >= 0) {
            c->mo_list[mo] = panar2 ? panar2 : 1;
            result_mo = mo;
        } else {
            err = 1;
        }
        break;
    }
    case PANCMD_STATIC_INS_BEFORE:
    case PANCMD_STATIC_INS_BEHIND:
        if (panar1 < TC1797_CAN_NMO && panar2 < TC1797_CAN_NMO) {
            c->mo_list[panar1] = c->mo_list[panar2] ? c->mo_list[panar2] : 1;
        } else {
            err = 1;
        }
        break;
    default:
        break;                 /* NOP / unknown: no change */
    }

    /* Completed: PANCMD=NOP, BUSY/RBUSY=0, PANAR1=result, PANAR2.bit7=ERR. */
    c->regs[can_idx(TC1797_CAN_PANCTR)] =
        PANCMD_NOP | ((result_mo & 0xFF) << 16) | (((err & 1) << 7) << 24);
}

uint32_t tc1797_can_read(Tc1797Can *c, uint32_t addr)
{
    addr = TC1797_CAN_BASE + (addr - c->base);   /* relocate to IP-relative space */
    uint32_t idx = can_idx(addr);
    if (addr >= TC1797_CAN_MO_BASE
        && addr < TC1797_CAN_MO_BASE + TC1797_CAN_NMO * 0x20) {
        unsigned n = (addr - TC1797_CAN_MO_BASE) >> 5;
        unsigned moff = (addr - TC1797_CAN_MO_BASE) & 0x1F;
        Tc1797CanMO *m = &c->mo[n];
        switch (moff) {
        case 0x00:                                  /* MOFCR: NEWDAT(bit5) */
            return c->regs[idx] | (m->rx_pending ? (1u << 5) : 0);
        /* MODR0/MODR1 (data words at +0x10/+0x14) are written into the MO data
         * registers by tc1797_can_rx_inject and served by the default return
         * below. A *read* must NOT clear NEWDAT -- on TC1797 only an explicit
         * MOCTR NEWDAT-reset (the firmware's ack) clears it. The previous code
         * cleared rx_pending on a MODR1 read AND gated the data on rx_pending;
         * that (a) let an unrelated MO scan consume the diagnostic request before
         * the diag task polled MO84, and (b) returned stale data to the diag
         * drain FUN_800dc44c, which resets NEWDAT *before* reading MODR0/1. */
        case 0x18:                                  /* MOAR: ID in [28:18] */
            if (m->rx_pending) {
                return (m->rx_id & 0x7FF) << 18;
            }
            break;
        case 0x1C:                                  /* MOSTAT: NEWDAT(bit3)+RXPND(bit0) */
            /* The firmware's RX path checks NEWDAT in MOSTAT (read of the MOCTR
             * address), not MOFCR. Report a waiting frame here too, else a
             * MOSTAT-polling consumer (the diag/ISO-TP handler) never sees it. */
            {   /* DIAGNOSTIC (env TC1797_MOLOG): what does a MOSTAT read of MO84
                 * (the diag request MO) return + the rx_pending state? */
                static int mol = -1;
                if (mol < 0) {
                    mol = getenv("TC1797_MOLOG") ? 1 : 0;
                }
                if (mol && n == 84) {
                    static unsigned nlog, n1;
                    if (m->rx_pending && n1++ < 30) {
                        fprintf(stderr, "CANRD MO84 MOSTAT rx_pending=1 (firmware SEES the request)\n");
                        fflush(stderr);
                    } else if (!m->rx_pending && nlog++ < 5) {
                        fprintf(stderr, "CANRD MO84 MOSTAT rx_pending=0\n");
                        fflush(stderr);
                    }
                }
            }
            return c->regs[idx] | (m->rx_pending ? ((1u << 3) | (1u << 0)) : 0);
        default:
            break;
        }
        return c->regs[idx];
    }
    return (idx < TC1797_CAN_NREGS) ? c->regs[idx] : 0;
}

void tc1797_can_write(Tc1797Can *c, uint32_t addr, uint32_t val)
{
    addr = TC1797_CAN_BASE + (addr - c->base);
    uint32_t idx = can_idx(addr);

    if (addr == TC1797_CAN_PANCTR) {
        c->regs[idx] = val;
        can_panel_command(c, val);
        return;
    }
    if (idx < TC1797_CAN_NREGS) {
        c->regs[idx] = val;
    }
    if (addr < TC1797_CAN_MO_BASE
        || addr >= TC1797_CAN_MO_BASE + TC1797_CAN_NMO * 0x20) {
        return;
    }

    unsigned n = (addr - TC1797_CAN_MO_BASE) >> 5;
    unsigned moff = (addr - TC1797_CAN_MO_BASE) & 0x1F;
    Tc1797CanMO *m = &c->mo[n];

    switch (moff) {
    case 0x18:                                      /* MOAR: ID + valid */
        m->id = (val >> 18) & 0x7FF;
        m->configured = true;
        break;
    case 0x0C:                                      /* MOAMR: acceptance mask */
        m->mask = val ? ((val >> 18) & 0x7FF) : 0x7FF;
        break;
    case 0x1C:                                      /* MOCTR: SET/RESET command */
        if (val & (1u << 27)) {                     /* SET DIR -> transmit  */
            m->dir = 1;
        } else if (val & (1u << 11)) {              /* RESET DIR -> receive */
            m->dir = 0;
        }
        if (val & (1u << 3)) {                      /* RESET NEWDAT: frame consumed */
            m->rx_pending = false;
        }
        if (val & (1u << 24)) {                     /* SET TXRQ -> transmit */
            uint32_t d0 = c->regs[mo_word(n, 0x10)];
            uint32_t d1 = c->regs[mo_word(n, 0x14)];
            uint32_t ar = c->regs[mo_word(n, 0x18)];
            uint32_t id = (ar >> 18) & 0x7FF;
            uint8_t data[8];
            stl_le_p(data, d0);
            stl_le_p(data + 4, d1);
            c->tx_count++;
            if (c->tx_cb) {
                c->tx_cb(c->tx_opaque, id, data, 8);
            }
        }
        break;
    default:
        break;
    }
}

bool tc1797_can_rx_inject(Tc1797Can *c, uint32_t can_id,
                          const uint8_t *data, unsigned len)
{
    if (len > 8) {
        len = 8;
    }
    static int mol = -1;
    if (mol < 0) {
        mol = getenv("TC1797_MOLOG") ? 1 : 0;
    }
    for (int n = 0; n < TC1797_CAN_NMO; n++) {
        Tc1797CanMO *m = &c->mo[n];
        if (!m->configured || m->dir != 0) {
            if (mol && can_id == 0x7e2 && (n == 84 || m->id == 0x7e2)) {
                static unsigned s84; if (s84++ < 10) {
                    fprintf(stderr, "RXINJ 0x7e2 SKIP MO%d configured=%d dir=%d id=%03x\n",
                            n, m->configured, m->dir, m->id); fflush(stderr); }
            }
            continue;                               /* only receive objects */
        }
        uint32_t mask = m->mask ? m->mask : 0x7FF;
        if ((can_id & mask) == (m->id & mask)) {
            if (mol && can_id == 0x7e2) {
                static unsigned mt; if (mt++ < 10) {
                    fprintf(stderr, "RXINJ 0x7e2 MATCH MO%d id=%03x mask=%03x dir=%d\n",
                            n, m->id, mask, m->dir); fflush(stderr); }
            }
            m->rx_pending = true;
            m->rx_id = can_id;
            memset(m->rx_data, 0, 8);
            memcpy(m->rx_data, data, len);
            /* Persist the frame bytes in the MO data registers (MODR0/MODR1 at
             * +0x10/+0x14) so a read returns them regardless of NEWDAT -- the
             * diag drain FUN_800dc44c resets NEWDAT *before* reading the data,
             * and on silicon the MO data registers retain the last frame. */
            c->regs[mo_word(n, 0x10)] = ldl_le_p(m->rx_data);
            c->regs[mo_word(n, 0x14)] = ldl_le_p(m->rx_data + 4);
            c->rx_count++;
            if (c->rx_irq_cb) {
                c->rx_irq_cb(c->rx_irq_opaque, n);   /* pulse MO RX interrupt */
            }
            return true;
        }
    }
    return false;
}

/* ───────────────────────── opt-in self-test ─────────────────────────────── */
void tc1797_can_selftest(CanTxFn tx_cb, void *opaque)
{
    Tc1797Can c;
    tc1797_can_init(&c, TC1797_CAN_BASE, tx_cb, opaque);
    unsigned pass = 0, fail = 0;
#define CHK(cond) do { if (cond) pass++; else { fail++; \
        error_report("CAN selftest FAIL @%d", __LINE__); } } while (0)

    /* (1) PANCTR static-alloc completes (BUSY clears, no error). */
    tc1797_can_write(&c, TC1797_CAN_PANCTR,
                     PANCMD_STATIC_ALLOC | (5u << 16) | (1u << 24));
    uint32_t pc = tc1797_can_read(&c, TC1797_CAN_PANCTR);
    CHK((pc & 0xFF) == PANCMD_NOP && !((pc >> 24) & 0x80) && c.mo_list[5] == 1);

    /* (2) TX: configure MO 5 as a transmit object id=0x123, data, SET-TXRQ. */
    uint32_t mo5 = TC1797_CAN_MO_BASE + 5 * 0x20;
    tc1797_can_write(&c, mo5 + 0x18, 0x123u << 18);          /* MOAR id */
    tc1797_can_write(&c, mo5 + 0x10, 0x44332211u);           /* MODR0 */
    tc1797_can_write(&c, mo5 + 0x14, 0x88776655u);           /* MODR1 */
    tc1797_can_write(&c, mo5 + 0x1C, (1u << 27));            /* SET DIR (tx) */
    uint64_t tx_before = c.tx_count;
    tc1797_can_write(&c, mo5 + 0x1C, (1u << 24));            /* SET TXRQ -> emit */
    CHK(c.tx_count == tx_before + 1 && c.mo[5].dir == 1);

    /* (3) RX: configure MO 6 as receive id=0x7E8, inject a frame, read it. */
    uint32_t mo6 = TC1797_CAN_MO_BASE + 6 * 0x20;
    tc1797_can_write(&c, mo6 + 0x18, 0x7E8u << 18);          /* MOAR id */
    tc1797_can_write(&c, mo6 + 0x0C, 0x7FFu << 18);          /* MOAMR exact */
    tc1797_can_write(&c, mo6 + 0x1C, (1u << 11));            /* RESET DIR (rx) */
    const uint8_t rx[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04 };
    bool matched = tc1797_can_rx_inject(&c, 0x7E8, rx, 8);
    uint32_t fcr = tc1797_can_read(&c, mo6 + 0x00);          /* NEWDAT? */
    uint32_t d0  = tc1797_can_read(&c, mo6 + 0x10);
    uint32_t d1  = tc1797_can_read(&c, mo6 + 0x14);          /* clears pending */
    uint32_t fcr2 = tc1797_can_read(&c, mo6 + 0x00);
    CHK(matched && (fcr & (1u << 5)) && d0 == 0xEFBEADDE && d1 == 0x04030201
        && !(fcr2 & (1u << 5)));

    /* (4) acceptance mask rejects a non-matching ID. */
    CHK(!tc1797_can_rx_inject(&c, 0x123, rx, 8));            /* 0x123 is a tx MO */

    info_report("tc1797 CAN selftest: %u passed, %u failed", pass, fail);
#undef CHK
}
