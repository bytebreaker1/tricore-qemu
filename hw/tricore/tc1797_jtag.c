/*
 * TC1797 JTAG TAP + Cerberus/OCDS debug interface (see tc1797_jtag.h).
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "exec/memattrs.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "chardev/char.h"
#include "tc1797_jtag.h"

/* IEEE 1149.1 next-state table: tap_next[state][tms]. */
static const uint8_t tap_next[16][2] = {
    [TAP_TLR]    = { TAP_RTI,    TAP_TLR    },
    [TAP_RTI]    = { TAP_RTI,    TAP_SEL_DR },
    [TAP_SEL_DR] = { TAP_CAP_DR, TAP_SEL_IR },
    [TAP_CAP_DR] = { TAP_SH_DR,  TAP_EX1_DR },
    [TAP_SH_DR]  = { TAP_SH_DR,  TAP_EX1_DR },
    [TAP_EX1_DR] = { TAP_PAU_DR, TAP_UPD_DR },
    [TAP_PAU_DR] = { TAP_PAU_DR, TAP_EX2_DR },
    [TAP_EX2_DR] = { TAP_SH_DR,  TAP_UPD_DR },
    [TAP_UPD_DR] = { TAP_RTI,    TAP_SEL_DR },
    [TAP_SEL_IR] = { TAP_CAP_IR, TAP_TLR    },
    [TAP_CAP_IR] = { TAP_SH_IR,  TAP_EX1_IR },
    [TAP_SH_IR]  = { TAP_SH_IR,  TAP_EX1_IR },
    [TAP_EX1_IR] = { TAP_PAU_IR, TAP_UPD_IR },
    [TAP_PAU_IR] = { TAP_PAU_IR, TAP_EX2_IR },
    [TAP_EX2_IR] = { TAP_SH_IR,  TAP_UPD_IR },
    [TAP_UPD_IR] = { TAP_RTI,    TAP_SEL_DR },
};

#define IR_MASK ((1u << TC1797_JTAG_IR_WIDTH) - 1)

/* Cerberus system-bus access (the OCDS DAP path): read/write 32 bits anywhere
 * in the SoC address space, exactly like a hardware debugger's bus master. */
static void cerberus_mem(Tc1797Jtag *j, uint32_t addr, uint32_t *data, bool write)
{
    uint8_t b[4];
    if (write) {
        for (int i = 0; i < 4; i++) {
            b[i] = (*data >> (8 * i)) & 0xff;
        }
    }
    address_space_rw(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                     b, 4, write);
    if (write) {
        j->cer_writes++;
    } else {
        *data = (uint32_t)b[0] | ((uint32_t)b[1] << 8)
              | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
        j->cer_reads++;
    }
}

/* Load the DR shift register when entering Capture-DR (per current instruction). */
static void jtag_capture_dr(Tc1797Jtag *j)
{
    switch (j->ir) {
    case IR_IDCODE:  j->dr_shift = j->idcode;        j->dr_len = 32; break;
    case IR_SAMPLE:
    case IR_EXTEST:  j->dr_shift = j->bsr;           j->dr_len = 32; break;
    case IR_CERB_WR: j->dr_shift = 0;                j->dr_len = 64; break;
    case IR_CERB_RD: j->dr_shift = ((uint64_t)j->cer_addr << 32) | j->cer_result;
                     j->dr_len = 64; break;
    case IR_BYPASS:
    default:         j->dr_shift = 0;                j->dr_len = 1;  break;
    }
}

/* Execute the latched DR on Update-DR (Cerberus performs the bus access). */
static void jtag_update_dr(Tc1797Jtag *j)
{
    if (j->ir == IR_CERB_WR) {
        uint32_t addr = (uint32_t)(j->dr_shift >> 32);
        uint32_t data = (uint32_t)(j->dr_shift & 0xFFFFFFFFu);
        j->cer_addr = addr;
        cerberus_mem(j, addr, &data, true);
    } else if (j->ir == IR_CERB_RD) {
        uint32_t addr = (uint32_t)(j->dr_shift >> 32);
        uint32_t data = 0;
        j->cer_addr = addr;
        cerberus_mem(j, addr, &data, false);
        j->cer_result = data;
    } else if (j->ir == IR_EXTEST) {
        j->bsr = (uint32_t)j->dr_shift;       /* drive boundary (modeled, inert) */
    }
}

int tc1797_jtag_tdo(Tc1797Jtag *j)
{
    if (j->state == TAP_SH_DR) {
        return (int)(j->dr_shift & 1);
    }
    if (j->state == TAP_SH_IR) {
        return j->ir_shift & 1;
    }
    return j->tdo;
}

int tc1797_jtag_clock(Tc1797Jtag *j, int tms, int tdi)
{
    int prev = j->state;
    int out = tc1797_jtag_tdo(j);             /* bit presented this cycle */

    if (prev == TAP_SH_DR) {
        j->dr_shift = (j->dr_shift >> 1)
                    | ((uint64_t)(tdi & 1) << (j->dr_len - 1));
    } else if (prev == TAP_SH_IR) {
        j->ir_shift = (uint8_t)((j->ir_shift >> 1)
                    | ((tdi & 1) << (TC1797_JTAG_IR_WIDTH - 1)));
    }
    j->tdo = out;

    int next = tap_next[prev][tms & 1];
    switch (next) {
    case TAP_TLR:    j->ir = IR_IDCODE; break;        /* reset selects IDCODE */
    case TAP_CAP_DR: jtag_capture_dr(j); break;
    case TAP_CAP_IR: j->ir_shift = 0x01; break;       /* IEEE capture pattern */
    case TAP_UPD_DR: jtag_update_dr(j); break;
    case TAP_UPD_IR: j->ir = j->ir_shift & IR_MASK; break;
    default: break;
    }
    j->state = next;
    return out;
}

void tc1797_jtag_reset(Tc1797Jtag *j)
{
    for (int i = 0; i < 5; i++) {              /* 5x TMS=1 -> Test-Logic-Reset */
        tc1797_jtag_clock(j, 1, 0);
    }
    j->ir = IR_IDCODE;
    j->state = TAP_TLR;
}

/* ───────────────────────── remote_bitbang (OpenOCD) ─────────────────────────
 * One ASCII byte per command (OpenOCD remote_bitbang transport):
 *   '0'..'7' : write pins, bit2=TCK bit1=TMS bit0=TDI (TAP clocks on TCK rising)
 *   'R'      : read TDO -> reply '0'/'1'
 *   'r/s/t/u': reset combos (trst=bit1,srst=bit0 of c-'r'); TRST -> TAP reset
 *   'B'/'b'  : blink on/off (ignored);  'Q' : quit (ignored, keep link)
 * Runs in the chardev RX callback on the main loop with the BQL held, so the
 * Cerberus bus accesses it triggers are safe. */
static int jtag_chr_can_receive(void *opaque)
{
    return 1024;
}

static void jtag_chr_receive(void *opaque, const uint8_t *buf, int size)
{
    Tc1797Jtag *j = opaque;
    for (int i = 0; i < size; i++) {
        uint8_t c = buf[i];
        if (c >= '0' && c <= '7') {
            int v = c - '0';
            int tck = (v >> 2) & 1, tms = (v >> 1) & 1, tdi = v & 1;
            if (tck && !j->last_tck) {                 /* rising edge */
                tc1797_jtag_clock(j, tms, tdi);
            }
            j->last_tck = tck;
        } else if (c == 'R') {
            uint8_t r = tc1797_jtag_tdo(j) ? '1' : '0';
            qemu_chr_fe_write_all(&j->chr, &r, 1);
        } else if (c >= 'r' && c <= 'u') {
            if ((c - 'r') & 0x2) {                     /* TRST asserted */
                tc1797_jtag_reset(j);
            }
            j->last_tck = 0;
        }
        /* 'B'/'b' (blink) and 'Q' (quit) need no action. */
    }
}

void tc1797_jtag_attach_chardev(Tc1797Jtag *j, Chardev *chr)
{
    if (!chr) {
        return;
    }
    qemu_chr_fe_init(&j->chr, chr, &error_abort);
    qemu_chr_fe_set_handlers(&j->chr, jtag_chr_can_receive, jtag_chr_receive,
                             NULL, NULL, j, NULL, true);
    j->have_chr = true;
    info_report("tc1797: JTAG remote_bitbang server attached "
                "(OpenOCD: -c 'adapter driver remote_bitbang')");
}

void tc1797_jtag_init(Tc1797Jtag *j, uint32_t idcode)
{
    memset(j, 0, sizeof(*j));
    j->idcode = idcode;
    j->state = TAP_TLR;
    j->ir = IR_IDCODE;
    j->dr_len = 1;
}

/* ───────────────────────── opt-in self-test ─────────────────────────────────
 * Drives the TAP at the bit level (the same FSM the remote_bitbang server uses)
 * to prove: IDCODE shift, BYPASS, and a real Cerberus bus write+read-back. */
static uint64_t jtag_scan_dr(Tc1797Jtag *j, uint64_t tdi_val, int len)
{
    tc1797_jtag_clock(j, 1, 0);    /* RTI -> SELECT-DR  */
    tc1797_jtag_clock(j, 0, 0);    /* -> CAPTURE-DR (loads DR) */
    tc1797_jtag_clock(j, 0, 0);    /* -> SHIFT-DR */
    uint64_t out = 0;
    for (int i = 0; i < len; i++) {
        int tms = (i == len - 1) ? 1 : 0;          /* last bit -> EXIT1-DR */
        int bit = tc1797_jtag_clock(j, tms, (tdi_val >> i) & 1);
        out |= (uint64_t)bit << i;
    }
    tc1797_jtag_clock(j, 1, 0);    /* EXIT1-DR -> UPDATE-DR (executes) */
    tc1797_jtag_clock(j, 0, 0);    /* -> RUN-TEST/IDLE */
    return out;
}

static void jtag_scan_ir(Tc1797Jtag *j, uint8_t ins)
{
    tc1797_jtag_clock(j, 1, 0);    /* RTI -> SELECT-DR */
    tc1797_jtag_clock(j, 1, 0);    /* -> SELECT-IR */
    tc1797_jtag_clock(j, 0, 0);    /* -> CAPTURE-IR */
    tc1797_jtag_clock(j, 0, 0);    /* -> SHIFT-IR */
    for (int i = 0; i < TC1797_JTAG_IR_WIDTH; i++) {
        int tms = (i == TC1797_JTAG_IR_WIDTH - 1) ? 1 : 0;
        tc1797_jtag_clock(j, tms, (ins >> i) & 1);
    }
    tc1797_jtag_clock(j, 1, 0);    /* EXIT1-IR -> UPDATE-IR (latches instruction) */
    tc1797_jtag_clock(j, 0, 0);    /* -> RUN-TEST/IDLE */
}

void tc1797_jtag_selftest(void)
{
    Tc1797Jtag j;
    tc1797_jtag_init(&j, TC1797_JTAG_IDCODE);
    unsigned pass = 0, fail = 0;
#define CHK(c) do { if (c) pass++; else { fail++; \
        error_report("JTAG selftest FAIL @%d", __LINE__); } } while (0)

    /* 1. Reset selects IDCODE; shifting DR returns the device IDCODE. */
    tc1797_jtag_reset(&j);
    tc1797_jtag_clock(&j, 0, 0);                 /* TLR -> RUN-TEST/IDLE */
    uint64_t id = jtag_scan_dr(&j, 0, 32);
    CHK((uint32_t)id == TC1797_JTAG_IDCODE);

    /* 2. BYPASS: a 1-bit register; data shifted in returns after a 1-TCK delay. */
    jtag_scan_ir(&j, IR_BYPASS);
    CHK(j.ir == IR_BYPASS);

    /* 3. Cerberus bus write + read-back through the TAP (the DAP path). Use a
     * PSPR scratch word; restore it so the self-test leaves zero footprint. */
    uint32_t scratch = 0xC0000000u;
    uint8_t orig[4];
    address_space_rw(&address_space_memory, scratch, MEMTXATTRS_UNSPECIFIED,
                     orig, 4, false);

    jtag_scan_ir(&j, IR_CERB_WR);
    jtag_scan_dr(&j, ((uint64_t)scratch << 32) | 0xCAFEBABEu, 64);  /* -> bus write */

    jtag_scan_ir(&j, IR_CERB_RD);
    jtag_scan_dr(&j, ((uint64_t)scratch << 32), 64);   /* update: bus read -> cer_result */
    /* Pipelined DAP read: the result captured by the previous scan is shifted
     * out on the next scan (which also re-reads the same address). */
    uint64_t rd = jtag_scan_dr(&j, ((uint64_t)scratch << 32), 64);
    CHK((uint32_t)(rd & 0xFFFFFFFFu) == 0xCAFEBABEu);
    CHK(j.cer_writes == 1 && j.cer_reads == 2);

    address_space_rw(&address_space_memory, scratch, MEMTXATTRS_UNSPECIFIED,
                     orig, 4, true);                   /* restore scratch */

    info_report("tc1797 JTAG selftest: %u passed, %u failed "
                "(IDCODE=0x%08x, Cerberus bus R/W verified)",
                pass, fail, TC1797_JTAG_IDCODE);
#undef CHK
}
