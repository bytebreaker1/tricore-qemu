/*
 * Infineon PCP2 (Peripheral Control Processor v2) — native interpreter +
 * own-thread channel engine for the TC1797 SoC.
 *
 * Faithful C port of the validated bridge reference (bridge/pcp.py,
 * bridge/pcp_engine.py). The PCP is a second processor with its own ISA; it
 * runs short per-priority "channel programs" to service peripherals. Here it
 * executes on its OWN host thread, sharing the guest address space (PRAM/CMEM/
 * SFRs) with the TriCore vCPU and serialising memory access via the BQL — the
 * physically-correct synchronisation point for a shared-bus co-processor.
 *
 * Ground truth: TC1797 UM v1.1 ch.10 + Ghidra tricore.pcp.sinc (bit-exact
 * encodings). Registers: R0 acc, R2 ret, R4 SRC, R5 DST, R6 {CPPN,SRPN,TOS,
 * CNT1}, R7 {DPTR,CEN,IEN,flags}; PC is separate (packed into saved R7[31:16]).
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"     /* info_report */
#include "qemu/main-loop.h"        /* bql_lock/bql_unlock */
#include "exec/memattrs.h"         /* MEMTXATTRS_UNSPECIFIED */
#include "system/address-spaces.h" /* address_space_memory */
#include "system/memory.h"         /* address_space_rw */
#include "pcp.h"

#define MASK32 0xFFFFFFFFu

/* ── shared bus: every PCP access goes to the guest system memory, so the PCP
 * and the TriCore CPU see exactly the same bytes (no mirror). Little-endian. ── */
static uint32_t pcp_read(hwaddr addr, unsigned size)
{
    uint8_t b[4] = {0};
    address_space_rw(&address_space_memory, addr & MASK32,
                     MEMTXATTRS_UNSPECIFIED, b, size, false);
    uint32_t v = 0;
    for (unsigned i = 0; i < size; i++) {
        v |= (uint32_t)b[i] << (8 * i);
    }
    return v;
}

static void pcp_write(hwaddr addr, unsigned size, uint32_t val)
{
    uint8_t b[4];
    for (unsigned i = 0; i < size; i++) {
        b[i] = (val >> (8 * i)) & 0xff;
    }
    if (getenv("TC1797_PCPWLOG")) {
        static unsigned wc;
        if (wc++ < 400) {
            fprintf(stderr, "PCPW: [0x%08x].%u = 0x%08x\n",
                    (uint32_t)(addr & MASK32), size, val);
            fflush(stderr);
        }
    }
    if ((addr & MASK32) == 0xD000416Fu && getenv("TC1797_PCPLOG")) {
        fprintf(stderr, "PCPFLAG: 0xD000416F <- phase 0x%02x\n", val & 0xFF);
        fflush(stderr);
    }
    address_space_rw(&address_space_memory, addr & MASK32,
                     MEMTXATTRS_UNSPECIFIED, b, size, true);
}

static unsigned sz_bytes(unsigned s) { return (unsigned[]){1, 2, 4, 4}[s & 3]; }

/* ── R7 flag bits ── */
enum { F_Z = 0, F_N = 1, F_C = 2, F_V = 3, F_CNZ = 4, F_IEN = 5, F_CEN = 6 };

static inline int pcp_flag(PcpCore *c, int b) { return (c->R[7] >> b) & 1; }
static inline void pcp_setf(PcpCore *c, int b, int v)
{
    c->R[7] = (c->R[7] & ~(1u << b)) | ((v ? 1u : 0u) << b);
}
static inline uint8_t pcp_dptr(PcpCore *c) { return (c->R[7] >> 8) & 0xFF; }
static inline uint32_t pcp_pram_ea(PcpCore *c, uint32_t off6)
{
    return PCP_PRAM_BASE + (((uint32_t)pcp_dptr(c) << 6) + off6);
}

static void set_logic_flags(PcpCore *c, uint32_t res)
{
    pcp_setf(c, F_Z, res == 0);
    pcp_setf(c, F_N, (res >> 31) & 1);
}
static void set_add_flags(PcpCore *c, uint32_t a, uint32_t b, uint32_t res)
{
    pcp_setf(c, F_Z, res == 0);
    pcp_setf(c, F_N, (res >> 31) & 1);
    pcp_setf(c, F_C, ((uint64_t)a + (uint64_t)b) > MASK32);
    pcp_setf(c, F_V, ((~(a ^ b) & (a ^ res)) >> 31) & 1);
}
static void set_sub_flags(PcpCore *c, uint32_t a, uint32_t b, uint32_t res)
{
    pcp_setf(c, F_Z, res == 0);
    pcp_setf(c, F_N, (res >> 31) & 1);
    pcp_setf(c, F_C, a < b);                       /* borrow (UM/Ghidra: C=a<b) */
    pcp_setf(c, F_V, (((a ^ b) & (a ^ res)) >> 31) & 1);
}

/* Condition-code evaluation (R7 flags). code 0..15 (UM Table 10-13). */
static bool eval_cond(int code, uint32_t flags)
{
    int Z = flags & 1, N = (flags >> 1) & 1, C = (flags >> 2) & 1;
    int V = (flags >> 3) & 1, CNZ = (flags >> 4) & 1;
    switch (code & 0xF) {
    case 0x0: return true;            /* UC  */
    case 0x1: return Z == 1;          /* Z   */
    case 0x2: return Z == 0;          /* NZ  */
    case 0x3: return V == 1;          /* V   */
    case 0x4: return C == 1;          /* ULT */
    case 0x5: return (C | Z) == 0;    /* UGT */
    case 0x6: return (N ^ V) == 1;    /* SLT */
    case 0x7: return ((N ^ V) | Z) == 0; /* SGT */
    case 0x8: return N == 1;          /* N   */
    case 0x9: return N == 0;          /* NN  */
    case 0xA: return V == 0;          /* NV  */
    case 0xB: return C == 0;          /* UGE */
    case 0xC: return (N ^ V) == 0;    /* SGE */
    case 0xD: return ((N ^ V) | Z) == 1; /* SLE */
    case 0xE: return CNZ == 1;        /* CNZ */
    case 0xF: return CNZ == 0;        /* CNN */
    }
    return false;
}

/* ── channel context save areas (UM 10.4.2.2): csa_base + SRPN*ctx_size ──
 * The CSA stores the context registers HIGH-to-LOW: CR7 (PC[31:16] | R7low) is
 * at the lowest word, then CR6 (CPPN | R6), then CR5..CR0. Confirmed against
 * this firmware's pre-initialised contexts (e.g. srpn29 @0xF00503A0 word0 =
 * 0x06370e40 -> CR7 -> PC=0x0637; word7=0). Small/Minimum keep the high regs
 * (CR7,CR6,... ) that Full has at its lowest words. */
static const struct { int nwords; uint8_t regs[8]; } CTX[3] = {
    { 8, { 7, 6, 5, 4, 3, 2, 1, 0 } },   /* Full    */
    { 4, { 7, 6, 5, 4 } },               /* Small   */
    { 2, { 7, 6 } },                     /* Minimum */
};

static void pcp_restore_context(PcpCore *c, uint8_t srpn, uint32_t csa_base,
                                bool rcb, int model)
{
    int nwords = CTX[model].nwords;
    uint32_t base = csa_base + (uint32_t)srpn * nwords * 4;
    for (int slot = 0; slot < nwords; slot++) {
        c->R[CTX[model].regs[slot]] = pcp_read(base + slot * 4, 4);
    }
    c->pc = rcb ? ((2 * srpn) & 0xFFFF) : ((c->R[7] >> 16) & 0xFFFF);
    {   /* diagnostic: force a channel's cold entry PC each trigger (env
         * TC1797_PCPCOLD=<srpn>:<pc>) so the cold path can be traced
         * deterministically regardless of the resumable-context state. */
        const char *cf = getenv("TC1797_PCPCOLD");
        if (cf) {
            unsigned csr = strtoul(cf, NULL, 0);
            const char *colon = strchr(cf, ':');
            if (colon && srpn == (uint8_t)csr) {
                c->pc = (uint16_t)strtoul(colon + 1, NULL, 0);
            }
        }
    }
    if (getenv("TC1797_PCPLOG")) {
        fprintf(stderr, "PCPCTX: srpn=%u rcb=%d model=%d base=0x%08x "
                "words=[%08x %08x %08x %08x %08x %08x %08x %08x] pc=0x%x\n",
                srpn, rcb, model, base, c->R[0], c->R[1], c->R[2], c->R[3],
                c->R[4], c->R[5], c->R[6], c->R[7], c->pc);
        fflush(stderr);
    }
    c->R[7] &= 0xFFFF;                  /* live R7 upper 16 bits read 0 */
    c->cur_srpn = srpn;
}

static void pcp_save_context(PcpCore *c, uint8_t srpn, uint32_t csa_base,
                             int ep, int model)
{
    int nwords = CTX[model].nwords;
    uint32_t base = csa_base + (uint32_t)srpn * nwords * 4;
    uint16_t save_pc = (ep ? c->pc : 2 * srpn) & 0xFFFF;
    uint32_t r7_img = ((uint32_t)save_pc << 16) | (c->R[7] & 0xFFFF);
    for (int slot = 0; slot < nwords; slot++) {
        int r = CTX[model].regs[slot];
        pcp_write(base + slot * 4, 4, (r == 7) ? r7_img : c->R[r]);
    }
}

/* ── DMA primitives for COPY/BCOPY (UM 10.12.1 / 10.18.2-3) ── */
static void pcp_copy_xfer(PcpCore *c, unsigned src_mode, unsigned dst_mode,
                          unsigned count, unsigned sz)
{
    int sd = (src_mode == 1 ? 1 : src_mode == 2 ? -1 : 0) * (int)sz;
    int dd = (dst_mode == 1 ? 1 : dst_mode == 2 ? -1 : 0) * (int)sz;
    for (unsigned i = 0; i < count; i++) {
        pcp_write(c->R[5] & MASK32, sz, pcp_read(c->R[4] & MASK32, sz));
        c->R[4] = (uint32_t)(c->R[4] + sd);
        c->R[5] = (uint32_t)(c->R[5] + dd);
    }
}

static void pcp_copy_run(PcpCore *c, unsigned src_mode, unsigned dst_mode,
                         unsigned cnc, unsigned per_xfer, unsigned sz)
{
    if (cnc == 2) {                                /* repeat until CNT1 == 0 */
        unsigned done = 0;
        do {
            pcp_copy_xfer(c, src_mode, dst_mode, per_xfer, sz);
            c->R[6] = (c->R[6] & ~0xFFFu) | ((c->R[6] - 1) & 0xFFFu);
            done += per_xfer;
        } while ((c->R[6] & 0xFFF) != 0 && done < (1u << 20));
        pcp_setf(c, F_CNZ, (c->R[6] & 0xFFF) == 0);
    } else {
        pcp_copy_xfer(c, src_mode, dst_mode, per_xfer, sz);
        if (cnc == 1) {                            /* one block, dec CNT1 */
            c->R[6] = (c->R[6] & ~0xFFFu) | ((c->R[6] - 1) & 0xFFFu);
            pcp_setf(c, F_CNZ, (c->R[6] & 0xFFF) == 0);
        }
    }
}

/* ── one instruction: fetch + decode + execute (bridge/pcp.py decode/_exec) ── */
static void pcp_step(PcpCore *c)
{
    hwaddr pa = PCP_CMEM_BASE + (uint32_t)c->pc * 2;
    uint16_t hw  = pcp_read(pa, 2);
    uint16_t hw2 = pcp_read(pa + 2, 2);
    unsigned am = (hw >> 13) & 7;
    unsigned Rb = (hw >> 6) & 7;
    unsigned Ra = (hw >> 3) & 7;
    unsigned op0912 = (hw >> 9) & 0xF;
    unsigned op1012 = (hw >> 10) & 7;
    uint32_t *R = c->R;

    /* instruction length: 16-bit except am4 ldl.il/iu and am7 jc.a (+imm16) */
    unsigned len_hw = 1;
    if (am == 4 && (op0912 == 8 || op0912 == 9)) {
        len_hw = 2;
    } else if (am == 7 && op1012 == 2) {
        len_hw = 2;
    }
    c->pc += len_hw;
    c->insn_count++;

    if (c->trace) {
        qemu_log("  PCP 0x%08x: am=%u hw=%04x\n", (uint32_t)pa, am, hw);
    }
    {
        const char *tcsr = getenv("TC1797_PCPTRACE");
        if (tcsr && c->cur_srpn == (uint8_t)strtoul(tcsr, NULL, 0)) {
            static unsigned tc;
            if (tc++ < 400) {
                fprintf(stderr, "PCPT[%02x] pc=0x%04x hw=%04x hw2=%04x am=%u Rb=%u "
                        "Ra=%u | R0=%08x R1=%08x R2=%08x R3=%08x R4=%08x R5=%08x "
                        "R6=%08x R7=%08x\n", c->cur_srpn, (uint32_t)(c->pc - len_hw),
                        hw, hw2, am, Rb, Ra, R[0], R[1], R[2], R[3], R[4], R[5],
                        R[6], R[7]);
                fflush(stderr);
            }
        }
    }

    switch (am) {
    case 0: {                                       /* control class */
        unsigned sub = (((hw >> 12) & 1) << 1) | ((hw >> 11) & 1);
        if (sub == 0) {                             /* nop */
            return;
        }
        if (sub == 2) {                             /* exit */
            unsigned cc = hw & 0xF;
            if (!eval_cond(cc, R[7])) {
                return;
            }
            c->ex_st  = (hw >> 10) & 1;
            c->ex_int = (hw >> 9) & 1;
            c->ex_ep  = (hw >> 8) & 1;
            c->ex_ec  = (hw >> 7) & 1;
            if (c->ex_ec) {
                uint32_t cnt1 = (R[6] - 1) & 0xFFF;
                R[6] = (R[6] & ~0xFFFu) | cnt1;
                pcp_setf(c, F_CNZ, cnt1 == 0);
            }
            if (c->ex_st) {
                pcp_setf(c, F_CEN, 0);
            }
            c->exited = true;
            return;
        }
        /* copy (sub==1) / bcopy (sub==3) */
        unsigned dst_mode = (hw >> 9) & 3;
        unsigned src_mode = (hw >> 7) & 3;
        unsigned cnc = (hw >> 5) & 3;
        if (sub == 1) {                             /* copy: honours SIZE */
            unsigned size = hw & 3;
            unsigned sz = (size <= 2) ? sz_bytes(size) : 4;
            unsigned cnt0 = ((hw >> 2) & 7);
            pcp_copy_run(c, src_mode, dst_mode, cnc, cnt0 ? cnt0 : 8, sz);
        } else {                                    /* bcopy: 32-bit words */
            unsigned blk = (hw >> 2) & 3;
            unsigned words = (blk == 2) ? 2 : (blk == 3) ? 4 : 8;
            pcp_copy_run(c, src_mode, dst_mode, cnc, words, 4);
        }
        return;
    }

    case 1: {                                       /* FPI (Ra=addr, SIZE) */
        unsigned sz = sz_bytes(hw & 3);
        uint32_t ea = R[Ra];
        switch (op0912) {
        case 0x9: { uint32_t v = pcp_read(ea, sz); R[Rb] = v; set_logic_flags(c, v); break; } /* ld.f */
        case 0xA: pcp_write(ea, sz, R[Rb] & (sz == 4 ? MASK32 : ((1u << (sz * 8)) - 1))); break; /* st.f */
        case 0x0: { uint32_t b = pcp_read(ea, sz), a = R[Rb], res = a + b; R[Rb] = res; set_add_flags(c, a, b, res); break; } /* add.f */
        case 0x1: { uint32_t b = pcp_read(ea, sz), a = R[Rb], res = a - b; set_sub_flags(c, a, b, res); R[Rb] = res; break; } /* sub.f */
        case 0x2: { uint32_t b = pcp_read(ea, sz), a = R[Rb]; set_sub_flags(c, a, b, a - b); break; } /* comp.f */
        case 0x5: { uint32_t res = R[Rb] & pcp_read(ea, sz); R[Rb] = res; set_logic_flags(c, res); break; } /* and.f */
        case 0x7: { uint32_t res = R[Rb] | pcp_read(ea, sz); R[Rb] = res; set_logic_flags(c, res); break; } /* or.f */
        case 0x8: { uint32_t res = R[Rb] ^ pcp_read(ea, sz); R[Rb] = res; set_logic_flags(c, res); break; } /* xor.f */
        case 0xB: { uint32_t old = pcp_read(ea, sz); pcp_write(ea, sz, R[Rb] & (sz == 4 ? MASK32 : ((1u << (sz * 8)) - 1))); R[Rb] = old; set_logic_flags(c, old); break; } /* xch.f */
        default: goto illegal;
        }
        return;
    }

    case 2: {                                       /* PRAM (DPTR + offset6) */
        uint32_t ea = pcp_pram_ea(c, hw & 0x3F);
        switch (op0912) {
        case 0x9: { uint32_t v = pcp_read(ea, 4); R[Rb] = v; set_logic_flags(c, v); break; } /* ld.pi */
        case 0xA: pcp_write(ea, 4, R[Rb]); break;                                            /* st.pi */
        case 0x0: { uint32_t a = R[Rb], b = pcp_read(ea, 4), res = a + b; R[Rb] = res; set_add_flags(c, a, b, res); break; } /* add.pi */
        case 0x1: { uint32_t a = R[Rb], b = pcp_read(ea, 4), res = a - b; set_sub_flags(c, a, b, res); R[Rb] = res; break; } /* sub.pi */
        case 0x2: { uint32_t a = R[Rb], b = pcp_read(ea, 4); set_sub_flags(c, a, b, a - b); break; } /* comp.pi */
        case 0x5: { uint32_t res = R[Rb] & pcp_read(ea, 4); R[Rb] = res; set_logic_flags(c, res); break; } /* and.pi */
        case 0x7: { uint32_t res = R[Rb] | pcp_read(ea, 4); R[Rb] = res; set_logic_flags(c, res); break; } /* or.pi */
        case 0x8: { uint32_t res = R[Rb] ^ pcp_read(ea, 4); R[Rb] = res; set_logic_flags(c, res); break; } /* xor.pi */
        case 0x6: { uint32_t res = R[Rb] | pcp_read(ea, 4); R[Rb] = res; pcp_write(ea, 4, res); set_logic_flags(c, res); break; } /* mset.pi */
        case 0x4: { uint32_t res = R[Rb] & pcp_read(ea, 4); R[Rb] = res; pcp_write(ea, 4, res); set_logic_flags(c, res); break; } /* mclr.pi */
        case 0xB: { uint32_t old = pcp_read(ea, 4); pcp_write(ea, 4, R[Rb]); R[Rb] = old; set_logic_flags(c, old); break; } /* xch.pi */
        default: goto illegal;
        }
        return;
    }

    case 3: {                                       /* arithmetic (cond ccA) */
        if (!eval_cond(hw & 7, R[7])) {
            return;
        }
        switch (op0912) {
        case 0x0: { uint32_t a = R[Rb], b = R[Ra], res = a + b; R[Rb] = res; set_add_flags(c, a, b, res); break; } /* add */
        case 0x1: { uint32_t a = R[Rb], b = R[Ra], res = a - b; R[Rb] = res; set_sub_flags(c, a, b, res); break; } /* sub */
        case 0x2: { uint32_t a = R[Rb], b = R[Ra]; set_sub_flags(c, a, b, a - b); break; } /* comp */
        case 0x3: { uint32_t res = (uint32_t)(-(int32_t)R[Ra]); R[Rb] = res; set_logic_flags(c, res); break; } /* neg */
        case 0x4: { uint32_t res = ~R[Ra]; R[Rb] = res; set_logic_flags(c, res); break; } /* not */
        case 0x5: { uint32_t res = R[Rb] & R[Ra]; R[Rb] = res; set_logic_flags(c, res); break; } /* and */
        case 0x7: { uint32_t res = R[Rb] | R[Ra]; R[Rb] = res; set_logic_flags(c, res); break; } /* or */
        case 0x8: { uint32_t res = R[Rb] ^ R[Ra]; R[Rb] = res; set_logic_flags(c, res); break; } /* xor */
        case 0xC: { uint32_t res = R[Ra]; R[Rb] = res; set_logic_flags(c, res); break; } /* mov */
        case 0x9: { uint32_t ea = PCP_PRAM_BASE + (((uint32_t)pcp_dptr(c) << 6) + (R[Ra] & 0x3F)); uint32_t v = pcp_read(ea, 4); R[Rb] = v; set_logic_flags(c, v); break; } /* ld.p */
        case 0xA: { uint32_t ea = PCP_PRAM_BASE + (((uint32_t)pcp_dptr(c) << 6) + (R[Ra] & 0x3F)); pcp_write(ea, 4, R[Rb]); break; } /* st.p */
        case 0xD: { unsigned bit = R[Ra] & 0x1F; R[Rb] = (R[Rb] & ~(1u << bit)) | ((uint32_t)pcp_flag(c, F_C) << bit); break; } /* inb */
        case 0xE: { uint32_t a = R[Ra], res = (a == 0) ? 0x20 : (31 - __builtin_clz(a)); R[Rb] = res; set_logic_flags(c, res); break; } /* pri */
        default: goto illegal;
        }
        return;
    }

    case 4: {                                       /* immediate */
        switch (op0912) {
        case 0x0: { uint32_t a = R[Rb], b = hw & 0x3F, res = a + b; R[Rb] = res; set_add_flags(c, a, b, res); break; } /* add.i */
        case 0x1: { uint32_t a = R[Rb], b = hw & 0x3F, res = a - b; set_sub_flags(c, a, b, res); R[Rb] = res; break; } /* sub.i */
        case 0x2: { uint32_t a = R[Rb], b = hw & 0x3F; set_sub_flags(c, a, b, a - b); break; } /* comp.i */
        case 0xC: { uint32_t v = hw & 0x3F; R[Rb] = v; set_logic_flags(c, v); break; } /* ld.i */
        case 0x9: R[Rb] = (R[Rb] & 0xFFFF0000u) | (hw2 & 0xFFFF); break;               /* ldl.il */
        case 0x8: R[Rb] = (R[Rb] & 0x0000FFFFu) | ((uint32_t)(hw2 & 0xFFFF) << 16); break; /* ldl.iu */
        case 0x5: { unsigned n = hw & 0x1F; uint32_t a = R[Rb]; if (n) pcp_setf(c, F_C, (a >> (32 - n)) & 1); uint32_t res = (n >= 32) ? 0 : (a << n); R[Rb] = res; set_logic_flags(c, res); break; } /* shl */
        case 0x4: { unsigned n = hw & 0x1F; uint32_t res = (n >= 32) ? 0 : (R[Rb] >> n); R[Rb] = res; set_logic_flags(c, res); break; } /* shr */
        case 0x7: { unsigned n = (hw & 0x1F) & 31; uint32_t a = R[Rb]; uint32_t res = n ? ((a << n) | (a >> (32 - n))) : a; R[Rb] = res; set_logic_flags(c, res); break; } /* rl */
        case 0x6: { unsigned n = (hw & 0x1F) & 31; uint32_t a = R[Rb]; uint32_t res = n ? ((a >> n) | (a << (32 - n))) : a; R[Rb] = res; set_logic_flags(c, res); break; } /* rr */
        case 0xA: R[Rb] = R[Rb] | (1u << (hw & 0x1F)); break;   /* set */
        case 0xB: R[Rb] = R[Rb] & ~(1u << (hw & 0x1F)); break;  /* clr */
        case 0xE: { unsigned bit = (R[Rb] >> (hw & 0x1F)) & 1; pcp_setf(c, F_C, bit == ((hw >> 5) & 1)); break; } /* chkb */
        case 0xD: { unsigned bit = hw & 0x1F; R[Rb] = (R[Rb] & ~(1u << bit)) | ((uint32_t)pcp_flag(c, F_C) << bit); break; } /* inb.i */
        default: goto illegal;
        }
        return;
    }

    case 5: {                                       /* FPI-immediate */
        unsigned b5 = (hw >> 5) & 1, b9 = (hw >> 9) & 1;
        unsigned sz = b9 ? 4 : (b5 ? 2 : 1);
        unsigned imm5 = hw & 0x1F;
        switch (op1012) {
        case 0x5: { uint32_t res = pcp_read(R[Rb], sz) + imm5; R[Rb] = res; set_logic_flags(c, res); break; } /* ld.if */
        case 0x6: pcp_write(R[Rb] + imm5, sz, R[0] & (sz == 4 ? MASK32 : ((1u << (sz * 8)) - 1))); break;     /* st.if */
        case 0x3: { uint32_t ea = R[Rb]; uint32_t v = (pcp_read(ea, sz) | (1u << imm5)) & (sz == 4 ? MASK32 : ((1u << (sz * 8)) - 1)); pcp_write(ea, sz, v); break; } /* set.f */
        case 0x4: { uint32_t ea = R[Rb]; uint32_t v = (pcp_read(ea, sz) & ~(1u << imm5)) & (sz == 4 ? MASK32 : ((1u << (sz * 8)) - 1)); pcp_write(ea, sz, v); break; } /* clr.f */
        default: goto illegal;
        }
        return;
    }

    case 6: {                                       /* complex math */
        switch (op0912) {
        case 0x0:                                   /* dinit */
            R[0] = 0;
            pcp_setf(c, F_V, R[Ra] == 0);
            pcp_setf(c, F_Z, R[Rb] == 0 && R[Ra] != 0);
            break;
        case 0x1: {                                 /* dstep */
            uint32_t a = R[Ra] ? R[Ra] : 1;
            R[0] = (uint32_t)(((uint64_t)R[0] << 8) + (R[Rb] >> 24));
            R[Rb] = (uint32_t)(((uint64_t)R[Rb] << 8) + (R[0] / a));
            R[0] = R[0] % a;
            pcp_setf(c, F_Z, R[0] == 0);
            break;
        }
        case 0x2:                                   /* minit */
            R[0] = 0;
            pcp_setf(c, F_Z, R[Rb] == 0 || R[Ra] == 0);
            break;
        case 0x3:                                   /* mstep.l (MSTEP32) */
            R[Rb] = (R[Rb] << 8) | (R[Rb] >> 24);
            R[0] = (uint32_t)(((uint64_t)R[0] << 8) + (uint64_t)(R[Rb] & 0xFF) * R[Ra]);
            pcp_setf(c, F_Z, R[0] == 0);
            break;
        case 0x4: {                                 /* mstep.u (MSTEP64, 40-bit) */
            uint64_t temp = ((uint64_t)R[0] + (uint64_t)R[Ra] * (R[Rb] & 0xFF))
                            & 0xFFFFFFFFFFull;
            R[0] = (uint32_t)((temp >> 8) & MASK32);
            R[Rb] = (uint32_t)((R[Rb] >> 8) + ((temp & 0xFF) << 24));
            pcp_setf(c, F_Z, R[0] == 0);
            break;
        }
        default: goto illegal;
        }
        return;
    }

    default: {                                      /* am == 7: jump */
        unsigned ccB = (hw >> 6) & 0xF;
        /*
         * Relative-jump target = NEXT instruction + signed offset. c->pc was
         * already advanced past this (1-hw) jump, so it already points at the
         * next instruction -- add the offset directly. (The bridge/earlier code
         * used `c->pc - 1 + off` = THIS instruction + off, an off-by-one that
         * silently broke the GPTA-init dispatch channel's loops: e.g. the
         * `R3 %= 60` modulo at CMEM 0x82b-0x82e -- `comp;jc.SLT exit;sub;jl top`
         * -- only terminates when jc/jl are next-instruction relative; one short,
         * the jc-exit lands on the loop-back jl and spins forever. Validated
         * end-to-end: with next-instruction targets PCP channel 15 walks its
         * state machine from state 0 and writes 0xD000416F=1; one short it hangs.
         * The Ghidra tricore.pcp.sinc uses inst_start but is a rough community
         * model (it also drops sign-extension) -- the firmware's own loops are
         * the ground truth.
         */
        switch (op1012) {
        case 0x0: {                                 /* jl off10 (signed) */
            int off = hw & 0x3FF;
            if (off & 0x200) {
                off -= 0x400;
            }
            c->pc = (uint16_t)((c->pc + off) & 0xFFFF);
            break;
        }
        case 0x1: {                                 /* jc off6 (signed) */
            if (!eval_cond(ccB, R[7])) {
                return;
            }
            int off = hw & 0x3F;
            if (off & 0x20) {
                off -= 0x40;
            }
            c->pc = (uint16_t)((c->pc + off) & 0xFFFF);
            break;
        }
        case 0x2:                                   /* jc.a abs16 */
            if (eval_cond(ccB, R[7])) {
                c->pc = hw2 & 0xFFFF;
            }
            break;
        case 0x4:                                   /* jc.i (rel via Ra) */
            if (eval_cond(ccB, R[7])) {
                c->pc = (uint16_t)((c->pc + (R[Ra] & 0xFFFF)) & 0xFFFF);  /* next-instr relative */
            }
            break;
        case 0x5:                                   /* jc.ia (abs via Ra) */
            if (eval_cond(ccB, R[7])) {
                c->pc = R[Ra] & 0xFFFF;
            }
            break;
        default: goto illegal;
        }
        return;
    }
    }

illegal:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "PCP: illegal/undecoded half-word 0x%04x at CMEM+0x%x\n",
                  hw, (uint32_t)(pa - PCP_CMEM_BASE));
    c->exited = true;                  /* abort the channel rather than loop */
    c->ex_int = 0;
}

unsigned pcp_core_run_channel(PcpCore *c, uint8_t srpn, uint32_t csa_base,
                              bool rcb, int model, uint32_t max_insns,
                              bool *out_exit_int)
{
    pcp_restore_context(c, srpn, csa_base, rcb, model);
    c->exited = false;
    unsigned ran = 0;
    while (ran < max_insns) {
        pcp_step(c);
        ran++;
        if (c->exited) {
            break;
        }
    }
    pcp_save_context(c, srpn, csa_base, c->exited ? c->ex_ep : 1, model);
    if (out_exit_int) {
        *out_exit_int = c->exited && c->ex_int;
    }
    return ran;
}

/* ──────────────────────────── engine (own thread) ──────────────────────── */

static void *pcp_thread_fn(void *arg)
{
    PcpEngine *e = arg;
    qemu_mutex_lock(&e->mutex);
    while (e->running) {
        while (e->running && e->qcount == 0) {
            qemu_cond_wait(&e->cond, &e->mutex);
        }
        if (!e->running) {
            break;
        }
        uint8_t srpn = e->queue[e->qhead];
        e->qhead = (e->qhead + 1) & 0xFF;
        e->qcount--;
        qemu_mutex_unlock(&e->mutex);

        /* Per-channel restart vs resume (UM 10.20): the entry table (PC=2*SRPN)
         * is used only for a channel's FIRST activation after reset; once it has
         * run (saving its EP-resumable context) it resumes at its saved PC even
         * while PCP_CS.RCB=1. This is the per-channel Channel-Resume disposition,
         * not a chip-global flag -- so a channel driven repeatedly by a busy
         * source (continuous ADC scan -> ch 0x1d) advances its state machine
         * across triggers instead of re-cold-starting every conversion. */
        bool chan_started = (e->started_mask[srpn >> 5] >> (srpn & 31)) & 1u;
        bool eff_rcb = e->rcb && !chan_started;
        e->started_mask[srpn >> 5] |= 1u << (srpn & 31);

        /* Run the channel holding the BQL: this serialises the shared bus
         * against the vCPU exactly like real silicon arbitrates the LMB. */
        bql_lock();
        bool exit_int = false;
        unsigned ran = pcp_core_run_channel(&e->core, srpn, e->csa_base, eff_rcb,
                                            e->context_model, e->max_insns,
                                            &exit_int);
        if (exit_int && e->irq_fn) {
            e->irq_fn(e->irq_opaque, srpn);     /* PCP -> CPU service request */
        }
        if (getenv("TC1797_PCPLOG")) {
            uint32_t flag = pcp_read(0xD000416F, 1) & 0xFF;
            fprintf(stderr, "PCPLOG: ran ch %u: insns=%u exited=%d exit_int=%d "
                    "ex_ep=0x%x  [0xD000416F now=0x%02x]\n", srpn, ran,
                    e->core.exited, exit_int, e->core.ex_ep, flag);
            fflush(stderr);
        }
        bql_unlock();

        qemu_mutex_lock(&e->mutex);
        e->st_channels++;
        e->st_insns += ran;
        if (e->core.exited) {
            e->st_exits++;
        }
        if (exit_int) {
            e->st_irqs++;
        }
    }
    qemu_mutex_unlock(&e->mutex);
    return NULL;
}

void pcp_engine_init(PcpEngine *e, uint32_t csa_base, int context_model,
                     bool rcb, PcpIrqFn irq_fn, void *opaque)
{
    memset(e, 0, sizeof(*e));
    e->csa_base = csa_base;
    e->context_model = context_model;
    e->rcb = rcb;
    e->max_insns = 100000;
    e->irq_fn = irq_fn;
    e->irq_opaque = opaque;
    qemu_mutex_init(&e->mutex);
    qemu_cond_init(&e->cond);
}

void pcp_engine_reset(PcpEngine *e)
{
    /* The firmware's reboot re-runs its PCP context seeder (every channel's CR7
     * is re-pinned to its entry PC), so each channel must cold-start again on
     * its first post-reset service request. Drop any queued triggers from the
     * previous boot too. */
    qemu_mutex_lock(&e->mutex);
    memset(e->started_mask, 0, sizeof(e->started_mask));
    e->qhead = e->qtail = e->qcount = 0;
    qemu_mutex_unlock(&e->mutex);
}

void pcp_engine_start(PcpEngine *e)
{
    if (e->started) {
        return;
    }
    e->running = true;
    e->started = true;
    qemu_thread_create(&e->thread, "tc1797-pcp", pcp_thread_fn, e,
                       QEMU_THREAD_JOINABLE);
}

void pcp_engine_stop(PcpEngine *e)
{
    if (!e->started) {
        return;
    }
    qemu_mutex_lock(&e->mutex);
    e->running = false;
    qemu_cond_signal(&e->cond);
    qemu_mutex_unlock(&e->mutex);
    qemu_thread_join(&e->thread);
    e->started = false;
}

void pcp_engine_trigger(PcpEngine *e, uint8_t srpn)
{
    if (srpn == 0) {
        return;
    }
    qemu_mutex_lock(&e->mutex);
    if (e->qcount < sizeof(e->queue)) {
        e->queue[e->qtail] = srpn;
        e->qtail = (e->qtail + 1) & 0xFF;
        e->qcount++;
        qemu_cond_signal(&e->cond);
    }
    qemu_mutex_unlock(&e->mutex);
}

/* ───────────────────────── opt-in interpreter self-test ─────────────────── */
void pcp_selftest(void)
{
    /*
     * Hand-assembled channel for SRPN 2 (RCB=1 -> PC = 2*SRPN = 4 half-words ->
     * CMEM_BASE + 8 bytes). Exercises immediate, PRAM store (via DPTR) and EXIT:
     *   ld.i  R3, #0x15          ; 0x98D5  (am4 op0xC, Rb3, imm6 0x15)
     *   st.pi R3, [DPTR+0]       ; 0x54C0  (am2 op0xA, Rb3, off6 0)
     *   exit  INT=1              ; 0x1200  (am0 sub=exit, INT bit9)
     * Context (Full): 8 words at PRAM_BASE + SRPN*32; R7 = DPTR(0x10)<<8, so the
     * store lands at PRAM_BASE + (0x10<<6) = 0xF0050400.
     */
    static const uint16_t prog[3] = { 0x98D5, 0x54C0, 0x1200 };
    uint32_t ctx = PCP_PRAM_BASE + 2u * 8u * 4u;   /* SRPN 2, Full = 8 words */
    uint32_t tgt = PCP_PRAM_BASE + (0x10u << 6);   /* = 0xF0050400 */

    for (int i = 0; i < 3; i++) {
        pcp_write(PCP_CMEM_BASE + 8 + i * 2, 2, prog[i]);
    }
    for (int i = 0; i < 8; i++) {
        pcp_write(ctx + i * 4, 4, 0);
    }
    pcp_write(ctx + 7 * 4, 4, 0x1000);             /* R7: DPTR = 0x10 */
    pcp_write(tgt, 4, 0xDEADBEEF);                 /* sentinel to overwrite */

    PcpCore c;
    memset(&c, 0, sizeof(c));
    bool exit_int = false;
    unsigned ran = pcp_core_run_channel(&c, 2, PCP_PRAM_BASE, true, 0, 1000,
                                        &exit_int);
    uint32_t got = pcp_read(tgt, 4);
    uint32_t saved_r3 = pcp_read(ctx + 3 * 4, 4);
    bool ok = (ran == 3) && exit_int && (got == 0x15) && (saved_r3 == 0x15);
    info_report("tc1797 PCP selftest: insns=%u exit_int=%d PRAM[0x400]=0x%08x "
                "saved_R3=0x%08x -> %s",
                ran, exit_int, got, saved_r3, ok ? "PASS" : "FAIL");
}

/* ───────────────────── opt-in OWN-THREAD self-test ──────────────────────── */
/* Records the SRPN that an EXIT.INT raised (the engine invokes this from the
 * worker thread while holding the BQL). */
static void pcp_test_irq(void *opaque, uint8_t srpn)
{
    *(uint8_t *)opaque = srpn;
}

/*
 * Proves the FULL own-thread path that pcp_selftest() (which runs inline) does
 * NOT cover: CPU-trigger -> queue -> worker thread wakes -> grabs BQL -> runs
 * the channel -> writes guest memory -> EXIT.INT -> IRQ callback. Uses an
 * ISOLATED PcpEngine (not the SoC's s->pcp), so it can never disturb live
 * firmware PCP state, and runs at realize (before the vCPU executes), so the
 * CMEM/PRAM scratch it uses is guaranteed free. The realizing (caller) thread
 * holds the BQL; the worker needs it to run the channel, so we drop the BQL
 * while waiting for completion and re-take it afterwards.
 */
void pcp_thread_selftest(void)
{
    static const uint16_t prog[3] = { 0x98D5, 0x54C0, 0x1200 };
    uint32_t ctx = PCP_PRAM_BASE + 2u * 8u * 4u;   /* SRPN 2, Full = 8 words */
    uint32_t tgt = PCP_PRAM_BASE + (0x10u << 6);   /* = 0xF0050400 */

    for (int i = 0; i < 3; i++) {
        pcp_write(PCP_CMEM_BASE + 8 + i * 2, 2, prog[i]);
    }
    for (int i = 0; i < 8; i++) {
        pcp_write(ctx + i * 4, 4, 0);
    }
    pcp_write(ctx + 7 * 4, 4, 0x1000);             /* R7: DPTR = 0x10 */
    pcp_write(tgt, 4, 0xDEADBEEF);                 /* sentinel to overwrite */

    uint8_t irq_srpn = 0;
    PcpEngine e;
    pcp_engine_init(&e, PCP_PRAM_BASE, /*Full*/0, /*rcb=*/true,
                    pcp_test_irq, &irq_srpn);
    pcp_engine_start(&e);
    pcp_engine_trigger(&e, 2);

    /* The worker blocks on bql_lock() until we release it; poll for the channel
     * to complete (bounded ~2 s) with the BQL dropped, then re-acquire. */
    bool done = false;
    bql_unlock();
    for (int i = 0; i < 2000 && !done; i++) {
        qemu_mutex_lock(&e.mutex);
        done = (e.st_channels > 0);
        qemu_mutex_unlock(&e.mutex);
        if (!done) {
            g_usleep(1000);
        }
    }
    bql_lock();

    pcp_engine_stop(&e);
    qemu_mutex_destroy(&e.mutex);
    qemu_cond_destroy(&e.cond);

    uint32_t got = pcp_read(tgt, 4);
    uint32_t saved_r3 = pcp_read(ctx + 3 * 4, 4);
    bool ok = done && (got == 0x15) && (saved_r3 == 0x15) && (irq_srpn == 2)
              && (e.st_channels == 1) && (e.st_irqs == 1);
    info_report("tc1797 PCP THREAD selftest: ran_on_worker=%d PRAM[0x400]=0x%08x "
                "saved_R3=0x%08x irq_srpn=%u channels=%llu irqs=%llu -> %s",
                done, got, saved_r3, irq_srpn,
                (unsigned long long)e.st_channels,
                (unsigned long long)e.st_irqs, ok ? "PASS" : "FAIL");
}
