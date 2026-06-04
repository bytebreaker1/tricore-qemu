/*
 * TC1797 E-Ray (FlexRay CC) POC state model — native (see tc1797_eray.h).
 * Faithful port of bridge/tc1797_eray.py.
 */
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "tc1797_eray.h"

/* register offsets */
#define ERAY_EIR   0x020
#define ERAY_SIR   0x024
#define ERAY_SUCC1 0x080
#define ERAY_CCSV  0x100

/* SUCC1.CMD[3:0] command codes */
enum { CMD_NONE = 0, CMD_CONFIG, CMD_READY, CMD_WAKEUP, CMD_RUN, CMD_ALL_SLOTS,
       CMD_HALT, CMD_FREEZE, CMD_SEND_MTS, CMD_ALLOW_COLDSTART, CMD_RESET_STATUS,
       CMD_MONITOR, CMD_CLEAR_RAMS };

/* CCSV.POCS[5:0] states */
enum { POCS_DEFAULT_CONFIG = 0x00, POCS_READY = 0x01, POCS_NORMAL_ACTIVE = 0x02,
       POCS_NORMAL_PASSIVE = 0x03, POCS_HALT = 0x04, POCS_MONITOR = 0x05,
       POCS_CONFIG = 0x0F };

#define SUCC1_PBSY  (1u << 7)
#define SUCC1_RESET 0x0C401080u

void tc1797_eray_init(ErayCC *e)
{
    memset(e, 0, sizeof(*e));
    e->regs[ERAY_SUCC1 >> 2] = SUCC1_RESET;
    e->pocs = POCS_DEFAULT_CONFIG;
}

static bool eray_apply_cmd(ErayCC *e, unsigned cmd)
{
    bool accepted = true;
    switch (cmd) {
    case CMD_CONFIG:
        if (e->pocs == POCS_DEFAULT_CONFIG || e->pocs == POCS_READY) {
            e->pocs = POCS_CONFIG;
        } else {
            accepted = false;
        }
        break;
    case CMD_READY:
        if (e->pocs == POCS_CONFIG || e->pocs == POCS_HALT
            || e->pocs == POCS_MONITOR || e->pocs == POCS_NORMAL_ACTIVE
            || e->pocs == POCS_NORMAL_PASSIVE) {
            e->pocs = POCS_READY;
        } else {
            accepted = false;
        }
        break;
    case CMD_WAKEUP:
        accepted = (e->pocs == POCS_READY);
        break;
    case CMD_RUN:
    case CMD_ALLOW_COLDSTART:
        if (e->pocs == POCS_READY) {
            e->pocs = POCS_NORMAL_ACTIVE;
        } else {
            accepted = false;
        }
        break;
    case CMD_ALL_SLOTS:
        accepted = (e->pocs == POCS_NORMAL_ACTIVE || e->pocs == POCS_NORMAL_PASSIVE);
        break;
    case CMD_HALT:
    case CMD_FREEZE:
        e->pocs = POCS_HALT;
        break;
    case CMD_MONITOR:
        if (e->pocs == POCS_READY) {
            e->pocs = POCS_MONITOR;
        } else {
            accepted = false;
        }
        break;
    case CMD_RESET_STATUS:
    case CMD_CLEAR_RAMS:
    case CMD_SEND_MTS:
        break;                            /* no POC state change */
    default:
        accepted = false;
    }
    e->last_cmd = cmd;
    e->cmd_count++;
    return accepted;
}

void tc1797_eray_write(ErayCC *e, uint32_t addr, uint32_t val)
{
    uint32_t off = (addr - TC1797_ERAY_BASE) & ~0x3u;
    if (off >= TC1797_ERAY_SIZE) {
        return;
    }
    if (off == ERAY_SUCC1) {
        unsigned cmd = val & 0xF;
        bool accepted = (cmd != CMD_NONE) ? eray_apply_cmd(e, cmd) : true;
        e->regs[off >> 2] = val & ~SUCC1_PBSY;
        if (!accepted) {
            e->regs[ERAY_EIR >> 2] |= 0x1;     /* EIR.CNA (command not accepted) */
        }
        return;
    }
    if (off == ERAY_EIR || off == ERAY_SIR) {
        e->regs[off >> 2] &= ~val;             /* write-1-to-clear */
        return;
    }
    e->regs[off >> 2] = val;
}

uint32_t tc1797_eray_read(ErayCC *e, uint32_t addr)
{
    uint32_t off = (addr - TC1797_ERAY_BASE) & ~0x3u;
    if (off >= TC1797_ERAY_SIZE) {
        return 0;
    }
    if (off == ERAY_CCSV) {
        return (e->regs[off >> 2] & ~0x3Fu) | (e->pocs & 0x3F);
    }
    if (off == ERAY_SUCC1) {
        return e->regs[off >> 2] & ~SUCC1_PBSY;
    }
    return e->regs[off >> 2];
}

void tc1797_eray_selftest(void)
{
    ErayCC e;
    unsigned pass = 0, fail = 0;
#define CHK(c) do { if (c) pass++; else { fail++; \
        error_report("ERAY selftest FAIL @%d", __LINE__); } } while (0)
    tc1797_eray_init(&e);

    /* reset: POCS = DEFAULT_CONFIG, SUCC1 = reset value */
    CHK((tc1797_eray_read(&e, TC1797_ERAY_BASE + ERAY_CCSV) & 0x3F) == POCS_DEFAULT_CONFIG);
    CHK(tc1797_eray_read(&e, TC1797_ERAY_BASE + ERAY_SUCC1)
        == (SUCC1_RESET & ~SUCC1_PBSY));   /* PBSY (bit7) always reads 0 */

    /* CONFIG -> READY -> RUN bring-up sequence drives POCS through the states */
    tc1797_eray_write(&e, TC1797_ERAY_BASE + ERAY_SUCC1, CMD_CONFIG);
    CHK((tc1797_eray_read(&e, TC1797_ERAY_BASE + ERAY_CCSV) & 0x3F) == POCS_CONFIG);
    tc1797_eray_write(&e, TC1797_ERAY_BASE + ERAY_SUCC1, CMD_READY);
    CHK((tc1797_eray_read(&e, TC1797_ERAY_BASE + ERAY_CCSV) & 0x3F) == POCS_READY);
    tc1797_eray_write(&e, TC1797_ERAY_BASE + ERAY_SUCC1, CMD_RUN);
    CHK((tc1797_eray_read(&e, TC1797_ERAY_BASE + ERAY_CCSV) & 0x3F) == POCS_NORMAL_ACTIVE);

    /* an illegal command (RUN from ACTIVE) sets EIR.CNA */
    e.regs[ERAY_EIR >> 2] = 0;
    tc1797_eray_write(&e, TC1797_ERAY_BASE + ERAY_SUCC1, CMD_RUN);  /* not from READY */
    CHK(tc1797_eray_read(&e, TC1797_ERAY_BASE + ERAY_EIR) & 0x1);

    info_report("tc1797 ERAY selftest: %u passed, %u failed", pass, fail);
#undef CHK
}
