/*
 *  Copyright (c) 2012-2014 Bastian Koppelmann C-Lab/University Paderborn
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "cpu.h"
#include "qemu/host-utils.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "qemu/plugin.h"
#include <zlib.h> /* for crc32 */


/* TC1797 OSEK value-trace: logs regs + key OS-CB memory at watched PCs.
 * Emitted only for watched PCs and only when TC1797_DBG is set (see translate.c). */
void helper_tc_dbg(CPUTriCoreState *env, uint32_t pc)
{
    static int n;
    /* TC1797_MSDI_BYPASS (USER-REQUESTED TEMPORARY CRUTCH, default OFF -- NOT FAITHFUL).
     * At the return of the MSDI bank-check FUN_8008034a (ret @0x800803de): force the return
     * value d2=0 ("bank OK") so the monitor advances its own bank sequence (reads bank2/3 via
     * step3, b6->2 like the donor) instead of looping on bank1, and clamp the four MSDI error
     * counters to 0 so the shutdown monitor FUN_8008026e never trips 0x302c/0x302d/0x302e/0x302f.
     * This bypasses the unmodelled SC900685 init transient so the boot stays stable WITH the
     * STM crossing fix (alarm running). The faithful SC900685 init model remains the open item. */
    if ((pc & ~0x20000000u) == 0x800803deu) {
        static int mb = -1;
        if (mb < 0) { mb = getenv("TC1797_MSDI_BYPASS") ? 1 : 0; }
        if (mb) {
            env->gpr_d[2] = 0;                       /* FUN_8008034a returns "bank OK" */
            cpu_stb_data(env, 0xD000004Du, 0);       /* d4d  (d4d>1  -> 0x302f) */
            cpu_stb_data(env, 0xD000004Cu, 0);       /* d4c  (d4c!=0 -> 0x302e) */
            cpu_stb_data(env, 0xD000004Eu, 0);       /* d4e  (d4e>5  -> 0x302d) */
            cpu_stb_data(env, 0xD0000048u, 0);       /* d48  (d48>5  -> 0x302c) */
            cpu_stb_data(env, 0xD0000049u, 0);       /* d48.hi byte, kept clear too */
            static unsigned bn;
            if (bn++ < 3) { fprintf(stderr, "MSDI_BYPASS: FUN_8008034a forced OK, counters cleared\n"); fflush(stderr); }
            return;
        }
    }
    {
        static int ip = -1;
        if (ip < 0) { ip = getenv("TC1797_IDLEPROF") ? 1 : 0; }
        if (ip) {
            static unsigned long cnt[6]; static unsigned long total;
            uint32_t nn = pc & ~0x20000000u;
            int idx = (nn==0x8010d8bau)?0:(nn==0x800c3c3cu)?1:(nn==0x8033bc24u)?2:
                      (nn==0x8036bc44u)?3:(nn==0x80119a7cu)?4:(nn==0x800c0300u)?5:-1;
            if (idx >= 0) {
                cnt[idx]++; total++;
                if (total % 100000 == 0) {
                    fprintf(stderr, "IDLEHIST 8010d8ba=%lu 800c3c3c(kick)=%lu 8033bc24=%lu "
                            "8036bc44=%lu 80119a7c=%lu 800c0300=%lu\n",
                            cnt[0],cnt[1],cnt[2],cnt[3],cnt[4],cnt[5]);
                    fflush(stderr);
                }
            }
        }
    }
    /* TC1797_COLDLOG: non-perturbing cold-init trace -- log each cold-init function as the firmware
     * executes it (inline, no gdb halt). Shows how far StartOS/FUN_8000123c gets before stalling, and
     * whether FUN_80001160 (STM CMP0 config) is ever reached. */
    if (getenv("TC1797_COLDLOG")) {
        const char *nm = NULL;
        switch (pc) {
        case 0x800002e8u: nm = "StartOS->coldinit caller 0x800002e8"; break;
        case 0x8000123cu: nm = "FUN_8000123c (coldinit ENTRY)"; break;
        case 0x800010eau: nm = "FUN_800010ea"; break;
        case 0x800012d2u: nm = "FUN_800012d2"; break;
        case 0x800013ceu: nm = "FUN_800013ce"; break;
        case 0x8000136au: nm = "FUN_8000136a"; break;
        case 0x80001160u: nm = "FUN_80001160 *** STM CMP0 CONFIG REACHED ***"; break;
        default: break;
        }
        if (nm) {
            static unsigned cl;
            if (cl++ < 400) { fprintf(stderr, "COLDLOG %s\n", nm); fflush(stderr); }
        }
    }
    /* TC1797_PH4FN: at the phase-driver's per-node condition-call site (fn_800fdec0 calli a15 @0x800fdfa4),
     * log the condition-function pointer (a15) together with the current phase byte. The phase-4 node's
     * condition fn is the direct gate for the 4->3 demote (the residual oscillation); a15 names it. */
    if (pc == 0x800fdfa4u && getenv("TC1797_PH4FN")) {
        uint8_t ph = 0;
        cpu_physical_memory_read(0xD0003643u, &ph, 1);
        static uint64_t seen[64]; static int ns;
        uint32_t fn = env->gpr_a[15];
        uint64_t key = ((uint64_t)ph << 32) | fn; int found = 0;
        for (int i = 0; i < ns; i++) { if (seen[i] == key) { found = 1; break; } }
        if (!found && ns < 64) {
            seen[ns++] = key;
            fprintf(stderr, "PH4FN phase=%u cond_fn=0x%08x  (d1=%u)\n", ph, fn, env->gpr_d[1]);
            fflush(stderr);
        }
    }
    /* TC1797_WDLOG: measure the software-watchdog servicing cadence. FUN_800c3c1a (kick) bumps
     * DAT_d0004037 whenever it runs > 0x29bf STM_TIM1 ticks (1.88ms) apart; 10 bumps -> DTC 0x3045.
     * Log STM_TIM1 + the bump counter at each kick/check so we can see the actual inter-call interval. */
    /* TC1797_WDLOG: at the STM-tick schedule ISR FUN_80081870, dump the cursor
     * DAT_d0012498, the dispatched callback *(cursor), and the entry period
     * cursor[1] -- reveals the schedule sequence (which task bodies + how often). */
    if (pc == 0x80081870u && getenv("TC1797_WDLOG")) {
        static unsigned sn;
        if (sn++ < 30) {
            uint32_t cur = cpu_ldl_le_data(env, 0xD0012498u);
            uint32_t cb = 0, per = 0;
            if (cur >= 0xD0000000u && cur < 0xD0020000u) {
                cb = cpu_ldl_le_data(env, cur);
                per = cpu_ldl_le_data(env, cur + 4);
            }
            uint32_t t0 = cpu_ldl_le_data(env, 0xF0000210u);
            fprintf(stderr, "SCHED n=%u cursor=%08x cb=%08x period=%u tim0=%08x\n",
                    sn, cur, cb, per, t0);
            fflush(stderr);
        }
    }
    /* TC1797_KICKCAL2: seed DAT_d0016ed0=-1200 RIGHT BEFORE FUN_8007f9be reads it (0x8007fa92),
     * so the kick-alarm period becomes ((-1200)+2200)*90 = 90000 = 1.0ms (<1.9ms threshold).
     * Confirms the kick cadence is the sole DTC-0x3045 cause (the value is otherwise a missing
     * DFLASH calibration -> 0 -> 2.2ms). */
    if ((pc & ~0x20000000u) == 0x8007fa92u && getenv("TC1797_KICKCAL2")) {
        int32_t off = -1200;
        cpu_stl_le_data(env, 0xD0016ED0u, (uint32_t)off);
        { static int kn; if (kn++ < 5) { fprintf(stderr, "KICKCAL2 hit pc=%08x set 16ed0=-1200\n", pc); fflush(stderr); } }
    }
    {
    uint32_t npc = pc & ~0x20000000u;   /* normalise uncached seg-A alias 0xA->0x8 */
    if ((npc == 0x800c3c1au || npc == 0x800c3c68u) && getenv("TC1797_WDLOG")) {
        static unsigned n; static uint32_t prev_t1;
        if (n < 80) {
            uint32_t t1 = cpu_ldl_le_data(env, 0xF0000214u);   /* STM_TIM1 */
            uint8_t cnt = cpu_ldub_data(env, 0xD0004037u);
            uint8_t ph = cpu_ldub_data(env, 0xD0003643u);
            uint32_t ra = env->gpr_a[11];                      /* caller (which task body) */
            uint32_t dt1 = (npc == 0x800c3c1au) ? (t1 - prev_t1) : 0;
            fprintf(stderr, "WDLOG %s n=%u pc=%08x ra=%08x TIM1=%08x dTIM1=%u(%s) cnt37=%u ph=%u\n",
                    npc == 0x800c3c1au ? "KICK " : "CHECK", n, pc, ra, t1, dt1,
                    dt1 > 0x29bf ? "BUMP" : "ok", cnt, ph);
            fflush(stderr);
            if (npc == 0x800c3c1au) { prev_t1 = t1; }
            n++;
        }
    }
    }
    /* TC1797_MONTRACE: log the first execution of each PC in the MSDI monitor block
     * (0x8007f900..0x80080200), so the runtime control flow through the (Ghidra-undecoded)
     * bank-sequencer is visible -- which call sites it reaches, where it branches away from
     * the bank2 read (FUN_800803f6 @~0x8007fc7a) / select (FUN_8008040c @~0x8007fc76). */
    if (getenv("TC1797_MONTRACE")) {
        uint32_t npc = pc & ~0x20000000u;
        if (npc >= 0x8007f900u && npc < 0x80080200u) {
            static uint8_t seen[0x900];          /* one bit-ish per 2 bytes of the region */
            static unsigned distinct;
            uint32_t idx = (npc - 0x8007f900u);
            if (idx < sizeof seen && !seen[idx]) {
                seen[idx] = 1;
                if (distinct++ < 600) {
                    uint8_t ph = cpu_ldub_data(env, 0xD0003643u);
                    fprintf(stderr, "MONTRACE pc=%08x ra=%08x ph=%u\n",
                            npc, env->gpr_a[11] & ~0x20000000u, ph);
                    fflush(stderr);
                }
            }
        }
    }
    /* TC1797_MSDILOG2: trace the MSDI cycler to find why bank2 (b6->2) never runs. */
    if (getenv("TC1797_MSDILOG2")) {
        uint32_t npc = pc & ~0x20000000u;
        static unsigned mc;
        if (npc == 0x800e0764u) {                 /* cycler entry: dump full state */
            static unsigned cn;
            if (cn++ < 60) {
                uint8_t c9 = cpu_ldub_data(env, 0xD00040C9u);
                uint8_t c8 = cpu_ldub_data(env, 0xD00040C8u);
                uint8_t b6 = cpu_ldub_data(env, 0xD00040B6u);
                uint8_t s1 = cpu_ldub_data(env, 0xD00040C5u);   /* bank1 status */
                uint16_t bk1 = cpu_lduw_le_data(env, 0xD00133E4u); /* block[1] */
                uint8_t head = cpu_ldub_data(env, 0xD0011AACu);
                uint8_t tail = cpu_ldub_data(env, 0xD0011AADu);
                uint8_t drain = cpu_ldub_data(env, 0xD0011AB0u);
                uint8_t cap = cpu_ldub_data(env, 0x80056060u);
                fprintf(stderr, "MSDIC cyc c9=%u c8=%u b6=%u stat1=%u blk1=%04x q[h=%u t=%u cap=%u drain=%u]\n",
                        c9, c8, b6, s1, bk1, head, tail, cap, drain);
                fflush(stderr);
            }
        } else if (npc == 0x800e06d8u) {          /* step1 entry */
            if (mc++ < 4000) {
                uint8_t s1 = cpu_ldub_data(env, 0xD00040C5u);
                static unsigned k; if (k++ < 30) { fprintf(stderr, "MSDIC step1 stat1=%u\n", s1); fflush(stderr); }
            }
        } else if (npc == 0x800e06a4u) {          /* step2 chan-select */
            fprintf(stderr, "MSDIC STEP2-chansel param(d4)=%08x\n", env->gpr_d[4]); fflush(stderr);
        } else if (npc == 0x800e070eu) {          /* step3 bank2 runner -- the smoking gun */
            fprintf(stderr, "MSDIC *** STEP3-BANK2-RUN ***\n"); fflush(stderr);
        } else if (npc == 0x800df5c2u) {
            static unsigned k; if (k++ < 8) { fprintf(stderr, "MSDIC MSDI-INIT FUN_800df5c2\n"); fflush(stderr); }
        } else if (npc == 0x800c09ccu) {
            static unsigned k; if (k++ < 8) { fprintf(stderr, "MSDIC MODE2-SET FUN_800c09cc\n"); fflush(stderr); }
        } else if (npc == 0x800e09ceu) {
            static unsigned k; if (k++ < 8) { fprintf(stderr, "MSDIC CMD-BUILD FUN_800e09ce\n"); fflush(stderr); }
        } else if (npc == 0x800a57f2u) {
            static unsigned k; if (k++ < 8) { fprintf(stderr, "MSDIC TASK-A FUN_800a57f2\n"); fflush(stderr); }
        } else if (npc == 0x800a5954u) {
            static unsigned k; if (k++ < 8) { fprintf(stderr, "MSDIC TASK-B FUN_800a5954\n"); fflush(stderr); }
        } else if (npc == 0x800c466cu) {            /* StartOS MSDI-worker dispatcher */
            static unsigned k; if (k++ < 8) {
                uint32_t wp = cpu_ldl_le_data(env, 0xD0019A28u);  /* worker fn ptr */
                fprintf(stderr, "CHAIN c466c DISPATCH worker_ptr=%08x\n", wp); fflush(stderr);
            }
        } else if (npc == 0x800a5e1au) {            /* the worker (gated DAT_d0019748) */
            static unsigned k; if (k++ < 8) {
                uint8_t f = cpu_ldub_data(env, 0xD0019748u);
                uint8_t f3f = cpu_ldub_data(env, 0xD001973Fu);
                fprintf(stderr, "CHAIN a5e1a WORKER flags=%02x (g1:&2==0?%d) 73f=%02x\n",
                        f, (f & 2) == 0, f3f); fflush(stderr);
            }
        } else if (npc == 0x800a58deu) {            /* bank-read caller (gated cnt<300) */
            static unsigned k; if (k++ < 12) {
                uint16_t cnt = cpu_lduw_le_data(env, 0xD00196D0u);
                fprintf(stderr, "CHAIN a58de BANKRD-CALLER cnt=%u (gate cnt<300?%d) arg0=%d\n",
                        cnt, cnt < 300, env->gpr_d[4]); fflush(stderr);
            }
        } else if (npc == 0x80081840u) {            /* OSEK schedule cursor dispatcher */
            static unsigned k; if ((k++ & 0x3FFFu) == 0) { fprintf(stderr, "SCHED dispatcher FUN_80081840 hit#%u\n", k); fflush(stderr); }
        } else if (npc == 0x800d3bb0u) {            /* the cycler's schedule entry */
            static unsigned k; fprintf(stderr, "SCHED *** CYCLER-ENTRY FUN_800d3bb0 RAN (#%u) ***\n", ++k); fflush(stderr);
        } else if (npc == 0x80115964u) {            /* worker-dispatch wrapper */
            static unsigned k; fprintf(stderr, "SCHED worker-wrapper FUN_80115964 ran (#%u)\n", ++k); fflush(stderr);
        } else if (npc == 0x8007fadcu) {            /* MSDI sequencer: log state + ready + counters */
            static unsigned k;
            if (k++ < 120) {
                uint8_t st = cpu_ldub_data(env, 0xD0003FC3u);   /* state machine */
                uint8_t rdy = cpu_ldub_data(env, 0xD0000055u);  /* bank-read-complete ready flag */
                uint8_t d53 = cpu_ldub_data(env, 0xD0000053u);  /* the transition field (prev validate) */
                uint8_t d4d = cpu_ldub_data(env, 0xD000004Du);
                uint8_t b6  = cpu_ldub_data(env, 0xD00040B6u);
                uint8_t ssc0 = cpu_ldub_data(env, 0xD000CDF4u);  /* SSC ch0 state machine */
                uint8_t qh = cpu_ldub_data(env, 0xD000CE0Au);    /* queue head idx */
                uint8_t qt = cpu_ldub_data(env, 0xD000CE0Bu);    /* queue tail idx */
                fprintf(stderr, "SEQ state=%u ready=%u d53=%u d4d=%u b6=%u | ssc0=%u q[h=%u t=%u]\n",
                        st, rdy, d53, d4d, b6, ssc0, qh, qt); fflush(stderr);
            }
        } else if (npc == 0x800801b4u) {            /* bank-read-complete -> ready=1 */
            static unsigned k; if (k++ < 30) { fprintf(stderr, "SEQ *** bank-read-COMPLETE (ready=1) ***\n"); fflush(stderr); }
        } else if (npc == 0x8007fc8cu || npc == 0x8007fc94u || npc == 0x8007fcecu) {  /* state2 advance/revert */
            static unsigned k; if (k++ < 60) {
                const char *w = npc==0x8007fc8cu ? "ADVANCE->3" : npc==0x8007fc94u ? "GATE jne d2" : "REVERT->2";
                fprintf(stderr, "ST2 %-12s d2=%u d0=%u | d18a8=%08x d18ac=%08x\n",
                        w, env->gpr_d[2], env->gpr_d[0],
                        cpu_ldl_le_data(env, 0xD00018A8u), cpu_ldl_le_data(env, 0xD00018ACu)); fflush(stderr);
            }
        } else if (npc == 0x8007fc38u || npc == 0x8007fc62u || npc == 0x8007fcb2u ||
                   npc == 0x8007fd9au || npc == 0x8007fe48u || npc == 0x8007ff54u || npc == 0x8008007au) {
            static unsigned k;
            if (k++ < 80) {
                int stnum = npc==0x8007fc38u?1: npc==0x8007fc62u?2: npc==0x8007fcb2u?3:
                            npc==0x8007fd9au?5: npc==0x8007fe48u?7: npc==0x8007ff54u?9: 13;
                uint8_t d53 = cpu_ldub_data(env, 0xD0000053u);
                uint8_t b6  = cpu_ldub_data(env, 0xD00040B6u);
                fprintf(stderr, "DCHK state%-2d b6=%u d53=%u\n", stnum, b6, d53); fflush(stderr);
            }
        } else if (npc == 0x80111d00u) {            /* SSC driver handshake */
            static unsigned k; if (k++ < 12) {
                uint8_t cdf4 = cpu_ldub_data(env, 0xD000CDF4u);
                /* dump the handshake cmd + data buffers (buf24=0xdf chan, buf28=<4 chan) */
                uint16_t cmd24 = cpu_lduw_le_data(env, 0xD000DFECu);
                uint16_t cmd28 = cpu_lduw_le_data(env, 0xD000DFEEu);
                uint16_t da24  = cpu_lduw_le_data(env, 0xD00037C0u);
                uint16_t db28  = cpu_lduw_le_data(env, 0xD00037BAu);
                fprintf(stderr, "SSCDRV cdf4=%u | cmd24=%04x cmd28=%04x | dataA(@37c0)=%04x dataB(@37ba)=%04x\n",
                        cdf4, cmd24, cmd28, da24, db28); fflush(stderr);
            }
        } else if (npc == 0x80111d44u && getenv("TC1797_CDF4LOG")) {  /* SSC channel-state (cdf4) write */
            static unsigned k; static uint8_t prev = 99;
            uint8_t nv = env->gpr_d[15] & 0xFFu;
            if (k++ < 400 && nv != prev) {           /* log cdf4 transitions */
                fprintf(stderr, "CDF4 %u -> %u\n", prev, nv); fflush(stderr);
            }
            prev = nv;
        } else if (npc == 0x800bf154u || npc == 0x800bf164u) {  /* FUN_800bf104 sub-check results */
            static unsigned k; if (k++ < 40) {
                const char *w = npc == 0x800bf154u ? "bVar2 SSC-xfer(FUN_80112dce)"
                                                   : "bVar3 reg0xb(FUN_800dece8)";
                fprintf(stderr, "BF104 %-30s = 0x%02x\n", w, env->gpr_d[2] & 0xFFu); fflush(stderr);
            }
        } else if ((npc == 0x800bf1a2u || npc == 0x800bf1aau) && getenv("TC1797_BANKLOG")) {  /* FUN_800bf19e state-3 sub-checks */
            static unsigned k; if (k++ < 60) {
                const char *w = npc == 0x800bf1a2u ? "iVar1 SSC-health(FUN_80112e52)"
                                                   : "iVar2 reg6(FUN_800ded70)";
                fprintf(stderr, "BF19E %-30s = 0x%02x\n", w, env->gpr_d[2] & 0xFFu); fflush(stderr);
            }
        } else if (npc == 0x800a5920u && getenv("TC1797_BANKLOG")) {  /* per-bank E2E validation result */
            static unsigned k; if (k++ < 80) {
                fprintf(stderr, "BANKVAL FUN_800a57f2 = 0x%02x  (0=ok) | d0019990=%08x d0019994=%08x\n",
                        env->gpr_d[2] & 0xFFu, env->gpr_d[5], env->gpr_d[6]); fflush(stderr);
            }
        } else if (npc == 0x800a58deu && getenv("TC1797_BANKLOG")) {  /* per-bank validator entry */
            static unsigned k; if (k++ < 80) {
                fprintf(stderr, "BANKDE entry param2(d5)=%u counter-region\n", env->gpr_d[5]); fflush(stderr);
            }
        } else if (npc == 0x800a57f2u && getenv("TC1797_BANKLOG")) {  /* E2E validator entry */
            static unsigned k; if (k++ < 80) {
                fprintf(stderr, "BANK57F2 entry: p1(d4)=%u p2(d5)=%08x p3(d6)=%08x\n",
                        env->gpr_d[4], env->gpr_d[5], env->gpr_d[6]); fflush(stderr);
            }
        } else if (npc == 0x8011630eu) {            /* FUN_8011630e: SSC descriptor enqueue */
            uint32_t p1 = env->gpr_d[4], p2 = env->gpr_d[5];
            uint8_t mode = cpu_ldub_data(env, 0xD00042ABu);
            if (mode == 1u && p2 == 0xbu) {         /* the failing reg-0xb gate, ring mode */
                static unsigned k; if (k++ < 20) {
                    uint16_t off = cpu_lduw_le_data(env, 0x80054B50u + p1 * 0x14u);
                    uint32_t desc = 0xD000CE60u + (p2 + off) * 0x20u;
                    uint32_t dw = cpu_ldl_le_data(env, desc);
                    uint32_t pi = cpu_ldl_le_data(env, 0x80054B44u + p1 * 0x14u);
                    uint32_t pj = cpu_ldl_le_data(env, 0x80054B48u + p1 * 0x14u);
                    uint32_t head = pi ? cpu_ldl_le_data(env, pi) : 0;
                    uint32_t tail = pj ? cpu_ldl_le_data(env, pj) : 0;
                    fprintf(stderr, "SSC630e-m1 ch=%u reg=%u desc=%08x busy=%u | headp=%08x tailp=%08x "
                            "head=%u tail=%u fill=%u %s\n", p1, p2, dw, dw & 1u, pi, pj, head, tail,
                            (tail - head) & 0xFFFFFFFFu,
                            ((tail - head) & 0xFFFFFFFFu) >= 0x20u ? "RING-FULL" : ""); fflush(stderr);
                }
            }
        } else if (npc == 0x8011a800u) {            /* SSC transfer engine: which buffer + mode */
            uint32_t buf = env->gpr_a[4];           /* param_1 = descriptor addr (pointer -> a4) */
            uint32_t nb = buf & ~0x20000000u;
            if (nb >= 0x80054490u && nb <= 0x80054540u) {     /* the HANDSHAKE descriptors */
                static unsigned k; if (k++ < 20) {
                    uint8_t mode = cpu_ldub_data(env, 0xD000402Fu);
                    fprintf(stderr, "XFER *** handshake-buf desc=%08x mode=%u (ring1 REACHED a800!) ***\n", nb, mode);
                    fflush(stderr);
                }
            } else {
                static unsigned long bankc; bankc++;
                if ((bankc & 0xFFFu) == 1) { fprintf(stderr, "XFER bank/other desc=%08x (traffic ~%lu)\n", nb, bankc); fflush(stderr); }
            }
        } else if (npc == 0x80115bb4u) {            /* sole caller of FUN_800c466c, gated on DAT_d0003642==3 */
            static unsigned k;
            uint8_t p642 = cpu_ldub_data(env, 0xD0003642u);
            uint8_t p643 = cpu_ldub_data(env, 0xD0003643u);
            fprintf(stderr, "GATE FUN_80115bb4 ENTRY #%u | DAT_d0003642=%u (need 3) DAT_d0003643(phase)=%u\n",
                    ++k, p642, p643); fflush(stderr);
        } else if (npc == 0x80115d82u) {            /* the gated dispatch actually taken */
            fprintf(stderr, "GATE *** FUN_800c466c() DISPATCH TAKEN (DAT_d0003642 was 3) ***\n"); fflush(stderr);
        } else if (npc == 0x800fbabcu || npc == 0x800fbbe2u || npc == 0x800fbafeu || npc == 0x800f76f0u) {
            static unsigned k;
            if (k++ < 60) {
                uint8_t a16f = cpu_ldub_data(env, 0xD000416Fu);
                uint8_t g938 = cpu_ldub_data(env, 0xD0000938u);
                const char *nm = npc == 0x800fbabcu ? "INIT1(416f=0)" :
                                 npc == 0x800fbbe2u ? "INIT2(416f=0)" :
                                 npc == 0x800fbafeu ? "GATE-0x3006   " : "ORCH";
                fprintf(stderr, "ADCLOG %s 416f=%u d938=%u\n", nm, a16f, g938); fflush(stderr);
            }
        } else if (npc == 0x800fdfa4u) {            /* phase driver indirect action-call: a15=fn, a13=&d642[level] */
            uint32_t a15 = env->gpr_a[15];          /* the check/action fn being called */
            uint32_t a13 = env->gpr_a[13];          /* &DAT_d0003642 + level */
            uint32_t level = a13 - 0xD0003642u;
            uint8_t  phase = cpu_ldub_data(env, a13);
            if (level <= 3) {                        /* sane levels only */
                static unsigned k; if (k++ < 200)
                    fprintf(stderr, "PHASEDRV calli fn=%08x level=%u phase=%u\n",
                            a15 & ~1u, level, phase); fflush(stderr);
            }
        } else if (npc == 0x8008034au) {            /* d4d bank check: which bank + value */
            static unsigned k;
            if (k++ < 40) {
                uint8_t b6 = cpu_ldub_data(env, 0xD00040B6u);
                uint16_t bk1 = cpu_lduw_le_data(env, 0xD00133E8u); /* bank1 block[3] */
                uint16_t bk2 = cpu_lduw_le_data(env, 0xD0013368u); /* bank2 block[6] */
                uint16_t bk3 = cpu_lduw_le_data(env, 0xD0013396u); /* bank3 block[2] */
                uint8_t d4d = cpu_ldub_data(env, 0xD000004Du);
                uint32_t param = env->gpr_d[4];                 /* param_1 (the expected) */
                fprintf(stderr, "MON CHECK b6=%u d4d=%u param=0x%x | bk1[3]=%04x bk2[6]=%04x bk3[2]=%04x\n",
                        b6, d4d, param, bk1, bk2, bk3);
                fflush(stderr);
            }
        } else if (npc == 0x8008008au) {
            fprintf(stderr, "MON *** BANK2-READ (step3) REACHED ***\n"); fflush(stderr);
        } else if (npc == 0x80080074u) {
            static unsigned k; if (k++ < 8) { fprintf(stderr, "MON bank1-error RE-READ (loop)\n"); fflush(stderr); }
        }
    }
    /* TC1797_CALSEED: keep the 4f6 sensor-status byte = 0xe7 (donor value, bits0+2) at every watched PC so
     * FUN_8014065a's 4041=1 branch stays taken -> eb1=1 -> e7e=1 -> stable phase 4. Proves the sensor route
     * drives os_running; the faithful fix is to model FUN_801291fa's ADC/diagnostic inputs to yield 0xe7. */
    {
        static int csf = -1;
        if (csf < 0) {
            csf = getenv("TC1797_F6FORCE") ? 1 : 0;   /* brute 4f6 force (separate env) */
        }
        if (csf) {
            uint8_t f6 = 0xe7;
            cpu_physical_memory_write(0xD00004F6u, &f6, 1);
        }
    }
    /* TC1797_CALSEED: reset the health counter DAT_c0003d00 (0xC0003D00) at FUN_801291fa entry -- the
     * faithful equivalent of the OSEK health-OK message 0x17 being delivered. Then FUN_801291fa computes
     * 4f6 with bit0 kept (counter not saturated) instead of recomputing the brute force away. */
    if (pc == 0x801291fau) {
        static int ch = -1;
        if (ch < 0) {
            ch = getenv("TC1797_CALSEED") ? 1 : 0;
        }
        if (ch) {
            uint8_t z = 0;
            cpu_physical_memory_write(0xC0003D00u, &z, 1);   /* health ctr reset (sim OSEK msg 0x17) */
            uint8_t st = 0x0a;                                /* engine/system state (donor value, sets 4f6 bit2) */
            cpu_physical_memory_write(0xD0003C09u, &st, 1);
        }
    }
    /* MARCH catcher (env TC1797_MARCHLOG): a PC below flash means the OSEK dispatcher calli'd a garbage
     * task body and the CPU is marching through non-flash memory. Log the first few march PCs + a11 (the
     * calli return-site in FUN_801255da) + the LIVE dispatch state read straight from guest RAM
     * (nextid/curid/slot[nextid]/cpu_src) -- this is the non-perturbing instruction-level capture of the
     * exact garbage dispatch (gdb halts make the march vanish). */
    if (pc < 0x80000000u) {
        static int ml = -1;
        if (ml < 0) {
            ml = getenv("TC1797_MARCHLOG") ? 1 : 0;
        }
        if (ml) {
            static unsigned mn;
            if (mn++ < 16) {
                uint8_t nid = 0, cid = 0;
                uint32_t slotbase = 0, srr = 0, slotv = 0;
                cpu_physical_memory_read(0xD0000988u, &nid, 1);
                cpu_physical_memory_read(0xD0000064u, &cid, 1);
                cpu_physical_memory_read(0xD000998Cu, &slotbase, 4);
                cpu_physical_memory_read(0xF7E0FFFCu, &srr, 4);
                if (slotbase >= 0xd0000000u && slotbase < 0xd0020000u) {
                    cpu_physical_memory_read(slotbase + (uint32_t)nid * 0x10u, &slotv, 4);
                }
                fprintf(stderr, "TCMARCH #%u pc=%08x a11=%08x nextid=%u curid=%u slot[nid]=%08x "
                        "cpu_src=%08x(SRR=%u)\n", mn, pc, env->gpr_a[11], nid, cid, slotv, srr,
                        (srr >> 13) & 1u);
                fflush(stderr);
            }
        }
        return;
    }
    /* TC1797_CALSEED: at each config reader (FUN_800e0856 d6f4 copy + FUN_800e006e/FUN_800dfe42 validators),
     * write the chip's REAL calibration (donor DSPR via TC1797_CALFILE) into ALL config blocks first, so
     * every reader sees valid 0x2a-magic data instead of QEMU's blank-DFLASH zeros -> validators pass, no
     * re-init. This is the donor ECU's actual calibration (the faithful equivalent of its DFLASH being
     * present), seeded at these exact PCs to beat the periodic re-load. */
    /* TC1797_CALSEED: at FUN_8014065a entry, seed the 4f6 status byte (DAT_d00004f4._2_1_, 0xD00004F6) to
     * the donor value 0xe7 (bits0+2 set) so FUN_8014065a takes the (4f6&4 && 4f6&1) branch -> 4041=1 ->
     * eb1=1 -> e7e=1 -> phase 4, bypassing the whole calibration/340e path. This is the value the chip's
     * FUN_80128d3c derives from healthy sensors -- the faithful fix is to model those ADC/diagnostic
     * inputs; this seed proves the route reaches phase 4. */
    if (pc == 0x8014065au) {
        static int cs3 = -1;
        if (cs3 < 0) {
            cs3 = getenv("TC1797_F6FORCE") ? 1 : 0;   /* brute 4f6 force (separate env) */
        }
        if (cs3) {
            uint8_t f6 = 0xe7;
            cpu_physical_memory_write(0xD00004F6u, &f6, 1);
        }
        return;
    }
    if (pc == 0x800e0856u || pc == 0x800e006eu || pc == 0x800dfe42u) {
        static int cs2 = -1;
        static gchar *cb = NULL;
        static gsize cl = 0;
        static int ld = 0;
        if (cs2 < 0) {
            cs2 = getenv("TC1797_CALSEED") ? 1 : 0;
        }
        if (cs2) {
            if (!ld) {
                ld = 1;
                const char *f = getenv("TC1797_CALFILE");
                if (f && !g_file_get_contents(f, &cb, &cl, NULL)) {
                    cb = NULL;
                }
            }
            if (cb && cl >= 0x14000u) {
                cpu_physical_memory_write(0xD0003560u, (uint8_t *)cb + 0x3560, 0x10);
                cpu_physical_memory_write(0xD0004080u, (uint8_t *)cb + 0x4080, 0x100);
                cpu_physical_memory_write(0xD0009880u, (uint8_t *)cb + 0x9880, 0x40);
                cpu_physical_memory_write(0xD000D700u, (uint8_t *)cb + 0xD700, 0xA0);
                cpu_physical_memory_write(0xD0013350u, (uint8_t *)cb + 0x13350, 0xB0);
            }
        }
        return;
    }
    /* MARCHLOG dispatch-calli sequence: log each FUN_801255da `calli a15` (0x801256c8) target +
     * nextid/curid/SRR, to see whether the OS tick keeps re-activating (nextid climbs back up) or the
     * dispatch just drains tasks to nextid=0 (then marches on the slot[0] guard). Falls through. */
    if (pc == 0x801256c8u) {
        static int ml2 = -1;
        if (ml2 < 0) {
            ml2 = getenv("TC1797_MARCHLOG") ? 1 : 0;
        }
        if (ml2) {
            static unsigned dn;
            if (dn++ < 120) {
                uint8_t nid = 0, cid = 0;
                uint32_t srr = 0, base = 0, slot0 = 0, slotn = 0;
                cpu_physical_memory_read(0xD0000988u, &nid, 1);
                cpu_physical_memory_read(0xD0000064u, &cid, 1);
                cpu_physical_memory_read(0xF7E0FFFCu, &srr, 4);
                cpu_physical_memory_read(0xD000998Cu, &base, 4);
                if (base >= 0xd0000000u && base < 0xd0020000u) {
                    cpu_physical_memory_read(base, &slot0, 4);            /* slot[0] guard */
                    cpu_physical_memory_read(base + (uint32_t)nid * 0x10u, &slotn, 4);
                }
                fprintf(stderr, "TCDISP #%u a15=%08x d15=%08x nextid=%u curid=%u SRR=%u slot0=%08x "
                        "slot[nid]=%08x\n", dn, env->gpr_a[15], env->gpr_d[15], nid, cid,
                        (srr >> 13) & 1u, slot0, slotn);
                fflush(stderr);
            }
        }
    }
    /* Non-perturbing fatal-handler log (env TC1797_FATLOG): FUN_800bf97c(cat=d4,
     * code=d5) is the firmware FATAL/reset handler; a11 at entry is the caller =
     * the failing check. Captures the FREE-RUN reset-loop driver (the crash-log
     * struct is not yet set up for the early fatals, so the SCU-reset DTCLOG path
     * cannot see them). */
    if (pc == 0x800bf97cu) {
        static int fl = -1;
        if (fl < 0) {
            fl = getenv("TC1797_FATLOG") ? 1 : 0;
        }
        if (fl) {
            static unsigned fn;
            if (fn++ < 300) {
                fprintf(stderr, "TCFATAL #%u cat=0x%02x code=0x%04x ra=%08x\n",
                        fn, env->gpr_d[4] & 0xff, env->gpr_d[5] & 0xffff,
                        env->gpr_a[11]);
                fflush(stderr);
            }
        }
        return;
    }
    /* Non-perturbing arming-path counters (env TC1797_ARMLOG): does FREE-RUN run
     * the CPU code that enables ch28/ch26's ADC SRCs (FUN_800fbce4 / FUN_800fbb46)
     * and the ch26-arm gate FUN_800f939e before the DTC-0x3006 gate (FUN_800fbafe,
     * armed by FUN_800f7540) resets? The breakpoint-based counts read 0 because
     * gdb halts steer the boot off the gate path -- this measures the real path. */
    if (pc == 0x800f7540u || pc == 0x800f7754u || pc == 0x800fbce4u
        || pc == 0x800f939eu || pc == 0x800fbb46u || pc == 0x800fbafeu) {
        static int al = -1;
        if (al < 0) {
            al = getenv("TC1797_ARMLOG") ? 1 : 0;
        }
        if (al) {
            static unsigned c7540, c7754, cbce4, c939e, cbb46, cbafe;
            unsigned *cnt = (pc == 0x800f7540u) ? &c7540 :
                            (pc == 0x800f7754u) ? &c7754 :
                            (pc == 0x800fbce4u) ? &cbce4 :
                            (pc == 0x800f939eu) ? &c939e :
                            (pc == 0x800fbb46u) ? &cbb46 : &cbafe;
            (*cnt)++;
            if (*cnt <= 3 || (*cnt % 16) == 0) {
                fprintf(stderr, "TCARM pc=%08x n=%u (7540=%u 7754=%u bce4=%u "
                        "939e=%u bb46=%u bafe=%u)\n", pc, *cnt,
                        c7540, c7754, cbce4, c939e, cbb46, cbafe);
                fflush(stderr);
            }
        }
        return;
    }
    /* Non-perturbing schedule-table / task-dispatch probe (env TC1797_TASKLOG):
     * is the ERCOSEK schedule table advancing past the startup table so the
     * periodic tasks that ENABLE ch28/ch26 (and drive the phase machine) get
     * dispatched? Counts entries to the relevant task bodies + the table-advance
     * (NextScheduleTable) + the phase-gated advancer fn_801199d4, and at the gate
     * reads the phase byte 0xD0003643 and pump-enable 0xD0004163. */
    if (pc == 0x80081a0eu || pc == 0x8008317au || pc == 0x80082492u
        || pc == 0x80082986u || pc == 0x80083156u || pc == 0x801199d4u
        || pc == 0x80125332u || pc == 0x800f76f0u || pc == 0x800fbafeu
        || pc == 0x800fbb24u || pc == 0x800f7368u || pc == 0x800bb400u) {
        static int tl = -1;
        if (tl < 0) {
            tl = getenv("TC1797_TASKLOG") ? 1 : 0;
        }
        if (tl) {
            static unsigned c_pump, c_ch28, c_master, c_mact, c_28act,
                            c_adv1, c_next, c_f76f0, c_gate, c_fatal, c_thunk,
                            c_pcpstart;
            unsigned *cnt =
                (pc == 0x80081a0eu) ? &c_pump  :
                (pc == 0x8008317au) ? &c_ch28  :
                (pc == 0x80082492u) ? &c_master:
                (pc == 0x80082986u) ? &c_mact  :
                (pc == 0x80083156u) ? &c_28act :
                (pc == 0x801199d4u) ? &c_adv1  :
                (pc == 0x80125332u) ? &c_next  :
                (pc == 0x800f76f0u) ? &c_f76f0 :
                (pc == 0x800fbafeu) ? &c_gate  :
                (pc == 0x800fbb24u) ? &c_fatal :
                (pc == 0x800f7368u) ? &c_thunk : &c_pcpstart;
            if (pc == 0x800bb400u && (c_pcpstart <= 3 || (c_pcpstart % 16) == 0)) {
                uint32_t pcp_cs = cpu_ldl_le_data(env, 0xF0043F10u);
                uint8_t flag = cpu_ldub_data(env, 0xD000416Fu);
                fprintf(stderr, "TCTASK PCP-START n=%u PCP_CS(F0043F10)=0x%08x "
                        "flag(D000416F)=0x%02x  (runs at idx3, BEFORE gate idx36)\n",
                        c_pcpstart, pcp_cs, flag);
            }
            if (pc == 0x800fbb24u && (c_fatal <= 3 || (c_fatal % 16) == 0)) {
                fprintf(stderr, "TCTASK GATE-FATAL-SITE n=%u gate_entry=%u thunk=%u "
                        "(gate reached: thunk-path vs pump-path)\n",
                        c_fatal + 1, c_gate, c_thunk);
            }
            (*cnt)++;
            /* at the gate: read phase byte + pump-enable + the gate flag */
            if (pc == 0x800fbafeu && (c_gate <= 3 || (c_gate % 16) == 0)) {
                uint8_t phase = cpu_ldub_data(env, 0xD0003643u);
                uint8_t en4163 = cpu_ldub_data(env, 0xD0004163u);
                uint8_t flag = cpu_ldub_data(env, 0xD000416Fu);
                fprintf(stderr, "TCTASK GATE n=%u phase(D0003643)=0x%02x "
                        "en(D0004163)=0x%02x flag(D000416F)=0x%02x\n",
                        c_gate, phase, en4163, flag);
            }
            if (pc == 0x801199d4u && (c_adv1 <= 3 || (c_adv1 % 16) == 0)) {
                uint8_t phase = cpu_ldub_data(env, 0xD0003643u);
                fprintf(stderr, "TCTASK ADV(fn_801199d4) n=%u phase(D0003643)=0x%02x"
                        " (advances table only if phase==4/3/5)\n", c_adv1, phase);
            }
            if (pc == 0x80125332u) {
                fprintf(stderr, "TCTASK NextScheduleTable n=%u  (TABLE ADVANCE!)\n",
                        c_next);
            }
            if (pc == 0x800f76f0u && (c_f76f0 <= 6 || (c_f76f0 % 16) == 0)) {
                uint8_t skip3c6c = cpu_ldub_data(env, 0xD0003C6Cu);
                uint8_t en4163 = cpu_ldub_data(env, 0xD0004163u);
                uint8_t phase = cpu_ldub_data(env, 0xD0003643u);
                fprintf(stderr, "TCTASK PUMP(f76f0) n=%u skip(D0003C6C)=0x%02x "
                        "en(D0004163)=0x%02x phase=0x%02x  (calls gate iff "
                        "skip&0x40==0 && en==1)\n",
                        c_f76f0, skip3c6c, en4163, phase);
            }
            if (c_pump + c_ch28 + c_master + c_mact + c_28act + c_next <= 5
                || (((*cnt) % 64) == 0)) {
                fprintf(stderr, "TCTASK pump=%u ch28=%u master=%u m_act=%u "
                        "ch28_act=%u adv1=%u NEXT=%u f76f0=%u gate=%u\n",
                        c_pump, c_ch28, c_master, c_mact, c_28act,
                        c_adv1, c_next, c_f76f0, c_gate);
            }
            fflush(stderr);
        }
        return;
    }
    /* Dispatch-crash diagnostic (env TC1797_DISP): the ERCOSEK scheduler enters
     * the next task via `calli a15` at 0x801256c8, where
     *   a15 = *( *(  (*0xD000998C) + nextid*0x10  + 4 ) ).
     * If nextid is out of range (table has `limit` entries) or an entry is
     * clobbered, a15 is garbage and the CPU runs away into unmapped space. Log
     * every dispatch + ALWAYS flag the out-of-code-range case. */
    if (pc == 0x801256c8u) {
        static int disp = -1;
        if (disp < 0) {
            disp = getenv("TC1797_DISP") ? 1 : 0;
        }
        if (disp) {
            uint32_t a15  = env->gpr_a[15];
            uint32_t base = cpu_ldl_le_data(env, 0xD000998Cu);
            uint32_t nid  = cpu_ldl_le_data(env, 0xD0000988u);
            uint32_t cid  = cpu_ldl_le_data(env, 0xD0000064u);
            uint32_t lim  = cpu_ldl_le_data(env, 0xD0000984u);
            bool bad = (a15 < 0x80000000u || a15 >= 0x80400000u);
            static unsigned dn;
            if (bad || dn++ < 60) {
                fprintf(stderr,
                    "TCDBG DISP%s a15=%08x base=%08x nextid=%u curid=%u limit=%u pc_ra=%08x\n",
                    bad ? "-BAD" : "", a15, base, nid, cid, lim, env->gpr_a[11]);
                fflush(stderr);
            }
        }
        return;
    }
    /* Experimental counter-object injection (TC1797_INJECT): the firmware's
     * broken init never sets DAT_d00009a4, so its StartCounter bails. Provide
     * the counter object at the firmware's own (unused) counter TCB slot and
     * let the real StartCounter arm it. Tests how far the chain comes alive. */
    static int inj = -1;
    if (inj < 0) {
        inj = getenv("TC1797_INJECT") ? 1 : 0;
    }
    if (inj && pc == 0x800bfaacu && cpu_ldl_le_data(env, 0xD00009A4) == 0) {
        uint32_t obj = 0xD00099E0u;          /* firmware's own counter TCB slot */
        cpu_stl_le_data(env, obj + 0, 0);              /* [0]  -> DAT_d00018c8 */
        cpu_stl_le_data(env, obj + 4, 0xD0009BD0u);    /* [4]  -> 18cc (TCB)   */
        cpu_stb_data(env,  obj + 8, 1);                /* [8]  autostart != 0  */
        cpu_stb_data(env,  obj + 9, 0);                /* [9]  -> 400c         */
        cpu_stb_data(env,  obj + 10, 0x20);            /* [10] mode (os-run)   */
        cpu_stl_le_data(env, obj + 12, 0);             /* [12] -> 18e0         */
        cpu_stl_le_data(env, obj + 16, 0);             /* [16] -> 18dc         */
        cpu_stl_le_data(env, 0xD00009A4u, obj);        /* DAT_d00009a4 = obj   */
        fprintf(stderr, "TCDBG INJECT: counter obj @%08x set into DAT_d00009a4\n", obj);
        fflush(stderr);
    }
    /* Periodic-task dispatch hijack: the OSEK dispatcher FUN_80124512 calls the
     * ready task's method via `calli a4` at 0x8012459a. The big periodic app
     * task FUN_80081faa (which runs the diag poll + UDS handling) never gets
     * activated because the counter/alarm init is gated. Every Nth dispatch,
     * redirect a4 to FUN_80081faa so the firmware's own calli runs it with a
     * correct context + return. */
    {
        static int hij = -1;
        if (hij < 0) {
            hij = getenv("TC1797_HIJACK") ? 1 : 0;   /* separate gate: dispatcher hijack */
        }
        if (!hij) {
            goto skip_hijack;
        }
    }
    if (pc == 0x8012459au) {
        static unsigned dc;
        uint8_t phase = cpu_ldub_data(env, 0xD0003643u);
        if (phase >= 4 && (dc++ % 50u) == 0u) {
            /* Run the diag dispatch task FUN_800dd0e8: drain diag MO + service
             * dispatch (FUN_8011c1fe) + response build/TX (FUN_8011c2e8). */
            env->gpr_a[4] = 0x80081faau;   /* real periodic task (proper calli/task
                                            * context) that calls FUN_800dd0e8 -> diag;
                                            * calli'ing the FUN_800dd0e8 subroutine
                                            * directly faulted before the drain. */
            cpu_stb_data(env, 0xD000400Fu, 1);   /* keep os_running asserted */
            { static unsigned ff; if (ff++ < 20) {
                fprintf(stderr, "TCDBG HIJACK#%u phase=%u\n", ff, phase); fflush(stderr); } }
        }
        return;
    }
 skip_hijack: ;
    if (pc == 0x800dd0e8u) {
        static unsigned dn, hits;
        uint32_t sess = cpu_ldl_le_data(env, 0xD0019A44u);
        uint32_t chan = (sess >= 0x80000000u && sess < 0x80400000u)
                        ? cpu_lduw_le_data(env, sess + 0x1au) : 0xFFFFFFFFu;
        uint32_t moidx = (chan < 256u) ? cpu_ldub_data(env, 0xD0014FE7u + chan) : 0xFFu;
        uint32_t mostat = (moidx < 128u)
            ? cpu_ldl_le_data(env, 0xF0005000u + moidx * 0x20u + 0x1Cu) : 0;
        uint32_t a8e = cpu_lduw_le_data(env, 0xD0019A8Eu);
        uint32_t a4c = cpu_ldl_le_data(env, 0xD0019A4Cu);
        uint32_t nd = (mostat >> 3) & 1u;
        if (nd) {
            hits++;                       /* count diag-task runs that SAW NEWDAT */
        }
        if (dn++ < 100000) {
            fprintf(stderr, "TCDBG DIAGTASK chan=%u mo=%u NEWDAT=%u a8e=%04x a4c=%08x newdat_hits=%u\n",
                    chan, moidx, nd, a8e, a4c, hits);
            fflush(stderr);
        }
        return;
    }
    if (pc == 0x800dc44cu && env->gpr_d[4] == 0x6eu) {   /* diag drain, channel 110 */
        static unsigned dr, drnd;
        uint32_t st = cpu_ldl_le_data(env, 0xF0005A9Cu);  /* MO84 MOSTAT (NEWDAT bit3) */
        dr++;
        if ((st >> 3) & 1u) {
            drnd++;
        }
        if (dr <= 100000) {
            fprintf(stderr, "TCDBG DRAIN110 #%u NEWDAT=%u saw_newdat_total=%u\n",
                    dr, (st >> 3) & 1u, drnd);
            fflush(stderr);
        }
        return;
    }
    {
        static int verbose = -1;
        if (verbose < 0) {
            verbose = getenv("TC1797_DBG") ? 1 : 0;
        }
        if (!verbose) {
            return;                      /* TC1797_DISP-only run: suppress tracer */
        }
    }
    if (n++ > 4000) {
        return;
    }
    uint32_t d9a4 = cpu_ldl_le_data(env, 0xD00009A4);
    uint32_t err  = cpu_ldl_le_data(env, 0xD0016CB0);
    uint32_t cnt0 = cpu_ldl_le_data(env, 0xD00099E0);
    fprintf(stderr,
        "TCDBG pc=%08x a2=%08x a4=%08x a5=%08x a11=%08x a15=%08x d4=%08x d15=%08x "
        "| DAT_d00009a4=%08x d0016cb0=%08x TCB[d00099e0]=%08x\n",
        pc, env->gpr_a[2], env->gpr_a[4], env->gpr_a[5], env->gpr_a[11],
        env->gpr_a[15], env->gpr_d[4], env->gpr_d[15], d9a4, err, cnt0);
    fflush(stderr);
}

/* Exception helpers */

static G_NORETURN
void raise_exception_sync_internal(CPUTriCoreState *env, uint32_t class, int tin,
                                   uintptr_t pc, uint32_t fcd_pc)
{
    CPUState *cs = env_cpu(env);
    uint64_t last_pc;

    /* in case we come from a helper-call we need to restore the PC */
    cpu_restore_state(cs, pc);
    last_pc = env->PC;
    uint32_t trap_old_a11 = (uint32_t)env->gpr_a[11];  /* caller RA before trap rewrites it */
    /* DIAGNOSTIC TEST (env TC1797_IDLESURVIVE): the ERCOSEK OS dispatches its idle
     * sentinel (task 0, ctx=0xFFFFFFFF) only when FULLY idle. On silicon it never goes
     * fully idle (the background task keeps it busy with key-on work), so the idle path
     * is a 'never reached' sentinel whose dispatch derefs 0xFFFFFFFF -> calli 0 -> this
     * MPX(execute@0) trap. Treat it as a benign idle spin: redirect to the real idle-loop
     * entry 0x801257e4 (= *DAT_d0000970) instead of vectoring to the firmware reset stub.
     * Tests whether the idle-sentinel trap is the SOLE phase-4 blocker. NOT a default fix. */
    if (class == 1 && tin == 4 && env->PC == 0 && getenv("TC1797_IDLESURVIVE")) {
        env->PC = 0x801257e4u;
        cpu_loop_exit(env_cpu(env));
    }

    /* Tin is loaded into d[15] */
    env->gpr_d[15] = tin;

    if (class == TRAPC_CTX_MNG && tin == TIN3_FCU) {
        /* upper context cannot be saved, if the context list is empty */
    } else {
        helper_svucx(env);
    }

    /* The return address in a[11] is updated */
    if (class == TRAPC_CTX_MNG && tin == TIN3_FCD) {
        env->SYSCON |= MASK_SYSCON_FCD_SF;
        /* when we run out of CSAs after saving a context a FCD trap is taken
           and the return address is the start of the trap handler which used
           the last CSA */
        env->gpr_a[11] = fcd_pc;
    } else if (class == TRAPC_SYSCALL) {
        env->gpr_a[11] = env->PC + 4;
    } else {
        env->gpr_a[11] = env->PC;
    }
    /* The stack pointer in A[10] is set to the Interrupt Stack Pointer (ISP)
       when the processor was not previously using the interrupt stack
       (in case of PSW.IS = 0). The stack pointer bit is set for using the
       interrupt stack: PSW.IS = 1. */
    if ((env->PSW & MASK_PSW_IS) == 0) {
        env->gpr_a[10] = env->ISP;
    }
    env->PSW |= MASK_PSW_IS;
    /* The I/O mode is set to Supervisor mode, which means all permissions
       are enabled: PSW.IO = 10 B .*/
    env->PSW |= (2 << 10);

    /*The current Protection Register Set is set to 0: PSW.PRS = 00 B .*/
    env->PSW &= ~MASK_PSW_PRS;

    /* The Call Depth Counter (CDC) is cleared, and the call depth limit is
       set for 64: PSW.CDC = 0000000 B .*/
    env->PSW &= ~MASK_PSW_CDC;

    /* Call Depth Counter is enabled, PSW.CDE = 1. */
    env->PSW |= MASK_PSW_CDE;

    /* Write permission to global registers A[0], A[1], A[8], A[9] is
       disabled: PSW.GW = 0. */
    env->PSW &= ~MASK_PSW_GW;

    /*The interrupt system is globally disabled: ICR.IE = 0. The ‘old’
      ICR.IE and ICR.CCPN are saved */

    /* PCXI.PIE = ICR.IE */
    pcxi_set_pie(env, icr_get_ie(env));

    /* PCXI.PCPN = ICR.CCPN */
    pcxi_set_pcpn(env, icr_get_ccpn(env));
    /* DIAGNOSTIC (env TC1797_TRAPLOG): identify CPU traps -- class/TIN/faulting-PC/BTV.
     * A trap whose BTV vectors to the firmware's reset stub (0x8001c7xx) is the phase-3
     * reset; class 4 = bus/peripheral error (unmapped access), class 2 = illegal opcode/
     * fetch (e.g. a jump to a null pointer), class 1 = protection. */
    if (getenv("TC1797_TRAPLOG")) {
        static unsigned tl;
        if (tl++ < 60) {
            uint8_t curid = 0, nextid = 0;
            uint32_t tbase = 0, t0c = 0, t0p = 0, spc = 0;
            cpu_physical_memory_read(0xD0000064u, &curid, 1);  /* OSEK curid */
            cpu_physical_memory_read(0xD0000988u, &nextid, 1); /* OSEK nextid */
            cpu_physical_memory_read(0xD000998Cu, &tbase, 4);  /* task-table base */
            if (tbase >= 0xC0000000u && tbase < 0xE0000000u) {
                cpu_physical_memory_read(tbase + nextid*0x10u, &t0c, 4);    /* task[next][0]=ctx */
                cpu_physical_memory_read(tbase + nextid*0x10u + 4, &t0p, 4);/* task[next][1] */
                if (t0p >= 0xC0000000u && t0p < 0xE0000000u) {
                    cpu_physical_memory_read(t0p, &spc, 4);                 /* *(task[next][1]) = saved PC */
                }
            }
            fprintf(stderr, "TRAP class=%u tin=%d faultPC=0x%08x callerRA=0x%08x curid=%u "
                    "nextid=%u | tbase=%08x t[n][0]=%08x t[n][1]=%08x *t[n][1]=%08x\n",
                    class, tin, (uint32_t)last_pc, trap_old_a11, curid, nextid,
                    tbase, t0c, t0p, spc);
            /* Walk the CSA chain (PCXI->prev) and print each frame's A11 (return
             * addr) to reveal WHO called FUN_801255da(0) -> idle dispatch. CSA EA =
             * (PCXI.PCXS<<28)|(PCXI.PCXO<<6); upper-ctx word[0]=PCXI, word[3]=A11. */
            uint32_t pcxi = env->PCXI;
            fprintf(stderr, "  CSA-chain:");
            for (int d = 0; d < 10 && (pcxi & 0xFFFFF); d++) {
                uint32_t ea = ((pcxi & 0x000F0000u) << 12) | ((pcxi & 0x0000FFFFu) << 6);
                uint32_t a11 = 0, prev = 0;
                if (ea >= 0xC0000000u && ea < 0xE0000000u) {
                    cpu_physical_memory_read(ea + 0xC, &a11, 4);
                    cpu_physical_memory_read(ea + 0x0, &prev, 4);
                }
                fprintf(stderr, " [%08x]ra=%08x", ea, a11);
                pcxi = prev;
            }
            /* Scheduler state: hi-coop, preemption-stack top + ptr + stack words. */
            uint32_t hicoop = 0, premtop = 0, premptr = 0, pw0 = 0, pw1 = 0;
            cpu_physical_memory_read(0xD0000984u, &hicoop, 4);
            cpu_physical_memory_read(0xD000098Cu, &premtop, 4);
            cpu_physical_memory_read(0xD00009B0u, &premptr, 4);
            if (premptr >= 0xC0000000u && premptr < 0xE0000000u) {
                cpu_physical_memory_read(premptr - 4, &pw0, 4);
                cpu_physical_memory_read(premptr - 8, &pw1, 4);
            }
            fprintf(stderr, "\n  hicoop=%u premtop=%08x premptr=%08x pstack[-1]=%08x [-2]=%08x\n",
                    hicoop, premtop, premptr, pw0, pw1);
            fflush(stderr);
        }
    }
    /* Update PC using the trap vector table */
    env->PC = env->BTV | (class << 5);

    qemu_plugin_vcpu_exception_cb(cs, last_pc);
    cpu_loop_exit(cs);
}

void helper_raise_exception_sync(CPUTriCoreState *env, uint32_t class,
                                 uint32_t tin)
{
    raise_exception_sync_internal(env, class, tin, 0, 0);
}

static void raise_exception_sync_helper(CPUTriCoreState *env, uint32_t class,
                                        uint32_t tin, uintptr_t pc)
{
    raise_exception_sync_internal(env, class, tin, pc, 0);
}

/* Range-based memory-protection trap (TRAPC_PROT, class 1). Called from the
 * softmmu fault path (tricore_cpu_tlb_fill) with the host return address so
 * cpu_restore_state recovers the guest PC of the faulting access. */
G_NORETURN void tricore_raise_protection_trap(CPUTriCoreState *env,
                                              uint32_t tin, uintptr_t pc)
{
    raise_exception_sync_internal(env, TRAPC_PROT, tin, pc, 0);
}

/* Addressing mode helper */

static uint16_t reverse16(uint16_t val)
{
    uint8_t high = (uint8_t)(val >> 8);
    uint8_t low  = (uint8_t)(val & 0xff);

    uint16_t rh, rl;

    rl = (uint16_t)((high * 0x0202020202ULL & 0x010884422010ULL) % 1023);
    rh = (uint16_t)((low * 0x0202020202ULL & 0x010884422010ULL) % 1023);

    return (rh << 8) | rl;
}

uint32_t helper_br_update(uint32_t reg)
{
    uint32_t index = reg & 0xffff;
    uint32_t incr  = reg >> 16;
    uint32_t new_index = reverse16(reverse16(index) + reverse16(incr));
    return reg - index + new_index;
}

uint32_t helper_circ_update(uint32_t reg, uint32_t off)
{
    uint32_t index = reg & 0xffff;
    uint32_t length = reg >> 16;
    int32_t new_index = index + off;
    if (new_index < 0) {
        new_index += length;
    } else {
        new_index %= length;
    }
    return reg - index + new_index;
}

static uint32_t ssov32(CPUTriCoreState *env, int64_t arg)
{
    uint32_t ret;
    int64_t max_pos = INT32_MAX;
    int64_t max_neg = INT32_MIN;
    if (arg > max_pos) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV = (1 << 31);
        ret = (uint32_t)max_pos;
    } else {
        if (arg < max_neg) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            ret = (uint32_t)max_neg;
        } else {
            env->PSW_USB_V = 0;
            ret = (uint32_t)arg;
        }
    }
    env->PSW_USB_AV = arg ^ arg * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;
    return ret;
}

static uint32_t suov32_pos(CPUTriCoreState *env, uint64_t arg)
{
    uint32_t ret;
    uint64_t max_pos = UINT32_MAX;
    if (arg > max_pos) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV = (1 << 31);
        ret = (uint32_t)max_pos;
    } else {
        env->PSW_USB_V = 0;
        ret = (uint32_t)arg;
     }
    env->PSW_USB_AV = arg ^ arg * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;
    return ret;
}

static uint32_t suov32_neg(CPUTriCoreState *env, int64_t arg)
{
    uint32_t ret;

    if (arg < 0) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV = (1 << 31);
        ret = 0;
    } else {
        env->PSW_USB_V = 0;
        ret = (uint32_t)arg;
    }
    env->PSW_USB_AV = arg ^ arg * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;
    return ret;
}

static uint32_t ssov16(CPUTriCoreState *env, int32_t hw0, int32_t hw1)
{
    int32_t max_pos = INT16_MAX;
    int32_t max_neg = INT16_MIN;
    int32_t av0, av1;

    env->PSW_USB_V = 0;
    av0 = hw0 ^ hw0 * 2u;
    if (hw0 > max_pos) {
        env->PSW_USB_V = (1 << 31);
        hw0 = max_pos;
    } else if (hw0 < max_neg) {
        env->PSW_USB_V = (1 << 31);
        hw0 = max_neg;
    }

    av1 = hw1 ^ hw1 * 2u;
    if (hw1 > max_pos) {
        env->PSW_USB_V = (1 << 31);
        hw1 = max_pos;
    } else if (hw1 < max_neg) {
        env->PSW_USB_V = (1 << 31);
        hw1 = max_neg;
    }

    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = (av0 | av1) << 16;
    env->PSW_USB_SAV |= env->PSW_USB_AV;
    return (hw0 & 0xffff) | (hw1 << 16);
}

static uint32_t suov16(CPUTriCoreState *env, int32_t hw0, int32_t hw1)
{
    int32_t max_pos = UINT16_MAX;
    int32_t av0, av1;

    env->PSW_USB_V = 0;
    av0 = hw0 ^ hw0 * 2u;
    if (hw0 > max_pos) {
        env->PSW_USB_V = (1 << 31);
        hw0 = max_pos;
    } else if (hw0 < 0) {
        env->PSW_USB_V = (1 << 31);
        hw0 = 0;
    }

    av1 = hw1 ^ hw1 * 2u;
    if (hw1 > max_pos) {
        env->PSW_USB_V = (1 << 31);
        hw1 = max_pos;
    } else if (hw1 < 0) {
        env->PSW_USB_V = (1 << 31);
        hw1 = 0;
    }

    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = (av0 | av1) << 16;
    env->PSW_USB_SAV |= env->PSW_USB_AV;
    return (hw0 & 0xffff) | (hw1 << 16);
}

uint32_t helper_add_ssov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t result = t1 + t2;
    return ssov32(env, result);
}

uint64_t helper_add64_ssov(CPUTriCoreState *env, uint64_t r1, uint64_t r2)
{
    uint64_t result;
    int64_t ovf;

    result = r1 + r2;
    ovf = (result ^ r1) & ~(r1 ^ r2);
    env->PSW_USB_AV = (result ^ result * 2u) >> 32;
    env->PSW_USB_SAV |= env->PSW_USB_AV;
    if (ovf < 0) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV = (1 << 31);
        /* ext_ret > MAX_INT */
        if ((int64_t)r1 >= 0) {
            result = INT64_MAX;
        /* ext_ret < MIN_INT */
        } else {
            result = INT64_MIN;
        }
    } else {
        env->PSW_USB_V = 0;
    }
    return result;
}

uint32_t helper_add_h_ssov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t ret_hw0, ret_hw1;

    ret_hw0 = sextract32(r1, 0, 16) + sextract32(r2, 0, 16);
    ret_hw1 = sextract32(r1, 16, 16) + sextract32(r2, 16, 16);
    return ssov16(env, ret_hw0, ret_hw1);
}

uint32_t helper_addr_h_ssov(CPUTriCoreState *env, uint64_t r1, uint32_t r2_l,
                            uint32_t r2_h)
{
    int64_t mul_res0 = sextract64(r1, 0, 32);
    int64_t mul_res1 = sextract64(r1, 32, 32);
    int64_t r2_low = sextract64(r2_l, 0, 32);
    int64_t r2_high = sextract64(r2_h, 0, 32);
    int64_t result0, result1;
    uint32_t ovf0, ovf1;
    uint32_t avf0, avf1;

    ovf0 = ovf1 = 0;

    result0 = r2_low + mul_res0 + 0x8000;
    result1 = r2_high + mul_res1 + 0x8000;

    avf0 = result0 * 2u;
    avf0 = result0 ^ avf0;
    avf1 = result1 * 2u;
    avf1 = result1 ^ avf1;

    if (result0 > INT32_MAX) {
        ovf0 = (1 << 31);
        result0 = INT32_MAX;
    } else if (result0 < INT32_MIN) {
        ovf0 = (1 << 31);
        result0 = INT32_MIN;
    }

    if (result1 > INT32_MAX) {
        ovf1 = (1 << 31);
        result1 = INT32_MAX;
    } else if (result1 < INT32_MIN) {
        ovf1 = (1 << 31);
        result1 = INT32_MIN;
    }

    env->PSW_USB_V = ovf0 | ovf1;
    env->PSW_USB_SV |= env->PSW_USB_V;

    env->PSW_USB_AV = avf0 | avf1;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return (result1 & 0xffff0000ULL) | ((result0 >> 16) & 0xffffULL);
}

uint32_t helper_addsur_h_ssov(CPUTriCoreState *env, uint64_t r1, uint32_t r2_l,
                              uint32_t r2_h)
{
    int64_t mul_res0 = sextract64(r1, 0, 32);
    int64_t mul_res1 = sextract64(r1, 32, 32);
    int64_t r2_low = sextract64(r2_l, 0, 32);
    int64_t r2_high = sextract64(r2_h, 0, 32);
    int64_t result0, result1;
    uint32_t ovf0, ovf1;
    uint32_t avf0, avf1;

    ovf0 = ovf1 = 0;

    result0 = r2_low - mul_res0 + 0x8000;
    result1 = r2_high + mul_res1 + 0x8000;

    avf0 = result0 * 2u;
    avf0 = result0 ^ avf0;
    avf1 = result1 * 2u;
    avf1 = result1 ^ avf1;

    if (result0 > INT32_MAX) {
        ovf0 = (1 << 31);
        result0 = INT32_MAX;
    } else if (result0 < INT32_MIN) {
        ovf0 = (1 << 31);
        result0 = INT32_MIN;
    }

    if (result1 > INT32_MAX) {
        ovf1 = (1 << 31);
        result1 = INT32_MAX;
    } else if (result1 < INT32_MIN) {
        ovf1 = (1 << 31);
        result1 = INT32_MIN;
    }

    env->PSW_USB_V = ovf0 | ovf1;
    env->PSW_USB_SV |= env->PSW_USB_V;

    env->PSW_USB_AV = avf0 | avf1;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return (result1 & 0xffff0000ULL) | ((result0 >> 16) & 0xffffULL);
}


uint32_t helper_add_suov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int64_t t1 = extract64(r1, 0, 32);
    int64_t t2 = extract64(r2, 0, 32);
    int64_t result = t1 + t2;
    return suov32_pos(env, result);
}

uint32_t helper_add_h_suov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t ret_hw0, ret_hw1;

    ret_hw0 = extract32(r1, 0, 16) + extract32(r2, 0, 16);
    ret_hw1 = extract32(r1, 16, 16) + extract32(r2, 16, 16);
    return suov16(env, ret_hw0, ret_hw1);
}

uint32_t helper_sub_ssov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t result = t1 - t2;
    return ssov32(env, result);
}

uint64_t helper_sub64_ssov(CPUTriCoreState *env, uint64_t r1, uint64_t r2)
{
    uint64_t result;
    int64_t ovf;

    result = r1 - r2;
    ovf = (result ^ r1) & (r1 ^ r2);
    env->PSW_USB_AV = (result ^ result * 2u) >> 32;
    env->PSW_USB_SAV |= env->PSW_USB_AV;
    if (ovf < 0) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV = (1 << 31);
        /* ext_ret > MAX_INT */
        if ((int64_t)r1 >= 0) {
            result = INT64_MAX;
        /* ext_ret < MIN_INT */
        } else {
            result = INT64_MIN;
        }
    } else {
        env->PSW_USB_V = 0;
    }
    return result;
}

uint32_t helper_sub_h_ssov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t ret_hw0, ret_hw1;

    ret_hw0 = sextract32(r1, 0, 16) - sextract32(r2, 0, 16);
    ret_hw1 = sextract32(r1, 16, 16) - sextract32(r2, 16, 16);
    return ssov16(env, ret_hw0, ret_hw1);
}

uint32_t helper_subr_h_ssov(CPUTriCoreState *env, uint64_t r1, uint32_t r2_l,
                            uint32_t r2_h)
{
    int64_t mul_res0 = sextract64(r1, 0, 32);
    int64_t mul_res1 = sextract64(r1, 32, 32);
    int64_t r2_low = sextract64(r2_l, 0, 32);
    int64_t r2_high = sextract64(r2_h, 0, 32);
    int64_t result0, result1;
    uint32_t ovf0, ovf1;
    uint32_t avf0, avf1;

    ovf0 = ovf1 = 0;

    result0 = r2_low - mul_res0 + 0x8000;
    result1 = r2_high - mul_res1 + 0x8000;

    avf0 = result0 * 2u;
    avf0 = result0 ^ avf0;
    avf1 = result1 * 2u;
    avf1 = result1 ^ avf1;

    if (result0 > INT32_MAX) {
        ovf0 = (1 << 31);
        result0 = INT32_MAX;
    } else if (result0 < INT32_MIN) {
        ovf0 = (1 << 31);
        result0 = INT32_MIN;
    }

    if (result1 > INT32_MAX) {
        ovf1 = (1 << 31);
        result1 = INT32_MAX;
    } else if (result1 < INT32_MIN) {
        ovf1 = (1 << 31);
        result1 = INT32_MIN;
    }

    env->PSW_USB_V = ovf0 | ovf1;
    env->PSW_USB_SV |= env->PSW_USB_V;

    env->PSW_USB_AV = avf0 | avf1;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return (result1 & 0xffff0000ULL) | ((result0 >> 16) & 0xffffULL);
}

uint32_t helper_subadr_h_ssov(CPUTriCoreState *env, uint64_t r1, uint32_t r2_l,
                              uint32_t r2_h)
{
    int64_t mul_res0 = sextract64(r1, 0, 32);
    int64_t mul_res1 = sextract64(r1, 32, 32);
    int64_t r2_low = sextract64(r2_l, 0, 32);
    int64_t r2_high = sextract64(r2_h, 0, 32);
    int64_t result0, result1;
    uint32_t ovf0, ovf1;
    uint32_t avf0, avf1;

    ovf0 = ovf1 = 0;

    result0 = r2_low + mul_res0 + 0x8000;
    result1 = r2_high - mul_res1 + 0x8000;

    avf0 = result0 * 2u;
    avf0 = result0 ^ avf0;
    avf1 = result1 * 2u;
    avf1 = result1 ^ avf1;

    if (result0 > INT32_MAX) {
        ovf0 = (1 << 31);
        result0 = INT32_MAX;
    } else if (result0 < INT32_MIN) {
        ovf0 = (1 << 31);
        result0 = INT32_MIN;
    }

    if (result1 > INT32_MAX) {
        ovf1 = (1 << 31);
        result1 = INT32_MAX;
    } else if (result1 < INT32_MIN) {
        ovf1 = (1 << 31);
        result1 = INT32_MIN;
    }

    env->PSW_USB_V = ovf0 | ovf1;
    env->PSW_USB_SV |= env->PSW_USB_V;

    env->PSW_USB_AV = avf0 | avf1;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return (result1 & 0xffff0000ULL) | ((result0 >> 16) & 0xffffULL);
}

uint32_t helper_sub_suov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int64_t t1 = extract64(r1, 0, 32);
    int64_t t2 = extract64(r2, 0, 32);
    int64_t result = t1 - t2;
    return suov32_neg(env, result);
}

uint32_t helper_sub_h_suov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t ret_hw0, ret_hw1;

    ret_hw0 = extract32(r1, 0, 16) - extract32(r2, 0, 16);
    ret_hw1 = extract32(r1, 16, 16) - extract32(r2, 16, 16);
    return suov16(env, ret_hw0, ret_hw1);
}

uint32_t helper_mul_ssov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t result = t1 * t2;
    return ssov32(env, result);
}

uint32_t helper_mul_suov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int64_t t1 = extract64(r1, 0, 32);
    int64_t t2 = extract64(r2, 0, 32);
    int64_t result = t1 * t2;

    return suov32_pos(env, result);
}

uint32_t helper_sha_ssov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int32_t t2 = sextract64(r2, 0, 6);
    int64_t result;
    if (t2 == 0) {
        result = t1;
    } else if (t2 > 0) {
        result = t1 << t2;
    } else {
        result = t1 >> -t2;
    }
    return ssov32(env, result);
}

uint32_t helper_abs_ssov(CPUTriCoreState *env, uint32_t r1)
{
    uint32_t result;
    result = ((int32_t)r1 >= 0) ? r1 : (0 - r1);
    return ssov32(env, result);
}

uint32_t helper_abs_h_ssov(CPUTriCoreState *env, uint32_t r1)
{
    int32_t ret_h0, ret_h1;

    ret_h0 = sextract32(r1, 0, 16);
    ret_h0 = (ret_h0 >= 0) ? ret_h0 : (0 - ret_h0);

    ret_h1 = sextract32(r1, 16, 16);
    ret_h1 = (ret_h1 >= 0) ? ret_h1 : (0 - ret_h1);

    return ssov16(env, ret_h0, ret_h1);
}

uint32_t helper_absdif_ssov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t result;

    if (t1 > t2) {
        result = t1 - t2;
    } else {
        result = t2 - t1;
    }
    return ssov32(env, result);
}

uint32_t helper_absdif_h_ssov(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t t1, t2;
    int32_t ret_h0, ret_h1;

    t1 = sextract32(r1, 0, 16);
    t2 = sextract32(r2, 0, 16);
    if (t1 > t2) {
        ret_h0 = t1 - t2;
    } else {
        ret_h0 = t2 - t1;
    }

    t1 = sextract32(r1, 16, 16);
    t2 = sextract32(r2, 16, 16);
    if (t1 > t2) {
        ret_h1 = t1 - t2;
    } else {
        ret_h1 = t2 - t1;
    }

    return ssov16(env, ret_h0, ret_h1);
}

uint32_t helper_madd32_ssov(CPUTriCoreState *env, uint32_t r1,
                            uint32_t r2, uint32_t r3)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t t3 = sextract64(r3, 0, 32);
    int64_t result;

    result = t2 + (t1 * t3);
    return ssov32(env, result);
}

uint32_t helper_madd32_suov(CPUTriCoreState *env, uint32_t r1,
                            uint32_t r2, uint32_t r3)
{
    uint64_t t1 = extract64(r1, 0, 32);
    uint64_t t2 = extract64(r2, 0, 32);
    uint64_t t3 = extract64(r3, 0, 32);
    int64_t result;

    result = t2 + (t1 * t3);
    return suov32_pos(env, result);
}

uint64_t helper_madd64_ssov(CPUTriCoreState *env, uint32_t r1,
                            uint64_t r2, uint32_t r3)
{
    uint64_t ret, ovf;
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t3 = sextract64(r3, 0, 32);
    int64_t mul;

    mul = t1 * t3;
    ret = mul + r2;
    ovf = (ret ^ mul) & ~(mul ^ r2);

    t1 = ret >> 32;
    env->PSW_USB_AV = t1 ^ t1 * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    if ((int64_t)ovf < 0) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV = (1 << 31);
        /* ext_ret > MAX_INT */
        if (mul >= 0) {
            ret = INT64_MAX;
        /* ext_ret < MIN_INT */
        } else {
            ret = INT64_MIN;
        }
    } else {
        env->PSW_USB_V = 0;
    }

    return ret;
}

uint32_t
helper_madd32_q_add_ssov(CPUTriCoreState *env, uint64_t r1, uint64_t r2)
{
    int64_t result;

    result = (r1 + r2);

    env->PSW_USB_AV = (result ^ result * 2u);
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    /* we do the saturation by hand, since we produce an overflow on the host
       if the mul before was (0x80000000 * 0x80000000) << 1). If this is the
       case, we flip the saturated value. */
    if (r2 == 0x8000000000000000LL) {
        if (result > 0x7fffffffLL) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            result = INT32_MIN;
        } else if (result < -0x80000000LL) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            result = INT32_MAX;
        } else {
            env->PSW_USB_V = 0;
        }
    } else {
        if (result > 0x7fffffffLL) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            result = INT32_MAX;
        } else if (result < -0x80000000LL) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            result = INT32_MIN;
        } else {
            env->PSW_USB_V = 0;
        }
    }
    return (uint32_t)result;
}

uint64_t helper_madd64_q_ssov(CPUTriCoreState *env, uint64_t r1, uint32_t r2,
                              uint32_t r3, uint32_t n)
{
    int64_t t1 = (int64_t)r1;
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t t3 = sextract64(r3, 0, 32);
    int64_t result, mul;
    int64_t ovf;

    mul = (t2 * t3) << n;
    result = mul + t1;

    env->PSW_USB_AV = (result ^ result * 2u) >> 32;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    ovf = (result ^ mul) & ~(mul ^ t1);
    /* we do the saturation by hand, since we produce an overflow on the host
       if the mul was (0x80000000 * 0x80000000) << 1). If this is the
       case, we flip the saturated value. */
    if ((r2 == 0x80000000) && (r3 == 0x80000000) && (n == 1)) {
        if (ovf >= 0) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            /* ext_ret > MAX_INT */
            if (mul < 0) {
                result = INT64_MAX;
            /* ext_ret < MIN_INT */
            } else {
               result = INT64_MIN;
            }
        } else {
            env->PSW_USB_V = 0;
        }
    } else {
        if (ovf < 0) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            /* ext_ret > MAX_INT */
            if (mul >= 0) {
                result = INT64_MAX;
            /* ext_ret < MIN_INT */
            } else {
               result = INT64_MIN;
            }
        } else {
            env->PSW_USB_V = 0;
        }
    }
    return (uint64_t)result;
}

uint32_t helper_maddr_q_ssov(CPUTriCoreState *env, uint32_t r1, uint32_t r2,
                             uint32_t r3, uint32_t n)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t t3 = sextract64(r3, 0, 32);
    int64_t mul, ret;

    if ((t2 == -0x8000ll) && (t3 == -0x8000ll) && (n == 1)) {
        mul = 0x7fffffff;
    } else {
        mul = (t2 * t3) << n;
    }

    ret = t1 + mul + 0x8000;

    env->PSW_USB_AV = ret ^ ret * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    if (ret > 0x7fffffffll) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV |= env->PSW_USB_V;
        ret = INT32_MAX;
    } else if (ret < -0x80000000ll) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV |= env->PSW_USB_V;
        ret = INT32_MIN;
    } else {
        env->PSW_USB_V = 0;
    }
    return ret & 0xffff0000ll;
}

uint64_t helper_madd64_suov(CPUTriCoreState *env, uint32_t r1,
                            uint64_t r2, uint32_t r3)
{
    uint64_t ret, mul;
    uint64_t t1 = extract64(r1, 0, 32);
    uint64_t t3 = extract64(r3, 0, 32);

    mul = t1 * t3;
    ret = mul + r2;

    t1 = ret >> 32;
    env->PSW_USB_AV = t1 ^ t1 * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    if (ret < r2) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV = (1 << 31);
        /* saturate */
        ret = UINT64_MAX;
    } else {
        env->PSW_USB_V = 0;
    }
    return ret;
}

uint32_t helper_msub32_ssov(CPUTriCoreState *env, uint32_t r1,
                            uint32_t r2, uint32_t r3)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t t3 = sextract64(r3, 0, 32);
    int64_t result;

    result = t2 - (t1 * t3);
    return ssov32(env, result);
}

uint32_t helper_msub32_suov(CPUTriCoreState *env, uint32_t r1,
                            uint32_t r2, uint32_t r3)
{
    uint64_t t1 = extract64(r1, 0, 32);
    uint64_t t2 = extract64(r2, 0, 32);
    uint64_t t3 = extract64(r3, 0, 32);
    uint64_t result;
    uint64_t mul;

    mul = (t1 * t3);
    result = t2 - mul;

    env->PSW_USB_AV = result ^ result * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;
    /* we calculate ovf by hand here, because the multiplication can overflow on
       the host, which would give false results if we compare to less than
       zero */
    if (mul > t2) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV = (1 << 31);
        result = 0;
    } else {
        env->PSW_USB_V = 0;
    }
    return result;
}

uint64_t helper_msub64_ssov(CPUTriCoreState *env, uint32_t r1,
                            uint64_t r2, uint32_t r3)
{
    uint64_t ret, ovf;
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t3 = sextract64(r3, 0, 32);
    int64_t mul;

    mul = t1 * t3;
    ret = r2 - mul;
    ovf = (ret ^ r2) & (mul ^ r2);

    t1 = ret >> 32;
    env->PSW_USB_AV = t1 ^ t1 * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    if ((int64_t)ovf < 0) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV = (1 << 31);
        /* ext_ret > MAX_INT */
        if (mul < 0) {
            ret = INT64_MAX;
        /* ext_ret < MIN_INT */
        } else {
            ret = INT64_MIN;
        }
    } else {
        env->PSW_USB_V = 0;
    }
    return ret;
}

uint64_t helper_msub64_suov(CPUTriCoreState *env, uint32_t r1,
                            uint64_t r2, uint32_t r3)
{
    uint64_t ret, mul;
    uint64_t t1 = extract64(r1, 0, 32);
    uint64_t t3 = extract64(r3, 0, 32);

    mul = t1 * t3;
    ret = r2 - mul;

    t1 = ret >> 32;
    env->PSW_USB_AV = t1 ^ t1 * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    if (ret > r2) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV = (1 << 31);
        /* saturate */
        ret = 0;
    } else {
        env->PSW_USB_V = 0;
    }
    return ret;
}

uint32_t
helper_msub32_q_sub_ssov(CPUTriCoreState *env, uint64_t r1, uint64_t r2)
{
    int64_t result;
    int64_t t1 = (int64_t)r1;
    int64_t t2 = (int64_t)r2;

    result = t1 - t2;

    env->PSW_USB_AV = (result ^ result * 2u);
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    /* we do the saturation by hand, since we produce an overflow on the host
       if the mul before was (0x80000000 * 0x80000000) << 1). If this is the
       case, we flip the saturated value. */
    if (r2 == 0x8000000000000000LL) {
        if (result > 0x7fffffffLL) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            result = INT32_MIN;
        } else if (result < -0x80000000LL) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            result = INT32_MAX;
        } else {
            env->PSW_USB_V = 0;
        }
    } else {
        if (result > 0x7fffffffLL) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            result = INT32_MAX;
        } else if (result < -0x80000000LL) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            result = INT32_MIN;
        } else {
            env->PSW_USB_V = 0;
        }
    }
    return (uint32_t)result;
}

uint64_t helper_msub64_q_ssov(CPUTriCoreState *env, uint64_t r1, uint32_t r2,
                              uint32_t r3, uint32_t n)
{
    int64_t t1 = (int64_t)r1;
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t t3 = sextract64(r3, 0, 32);
    int64_t result, mul;
    int64_t ovf;

    mul = (t2 * t3) << n;
    result = t1 - mul;

    env->PSW_USB_AV = (result ^ result * 2u) >> 32;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    ovf = (result ^ t1) & (t1 ^ mul);
    /* we do the saturation by hand, since we produce an overflow on the host
       if the mul before was (0x80000000 * 0x80000000) << 1). If this is the
       case, we flip the saturated value. */
    if (mul == 0x8000000000000000LL) {
        if (ovf >= 0) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            /* ext_ret > MAX_INT */
            if (mul >= 0) {
                result = INT64_MAX;
            /* ext_ret < MIN_INT */
            } else {
               result = INT64_MIN;
            }
        } else {
            env->PSW_USB_V = 0;
        }
    } else {
        if (ovf < 0) {
            env->PSW_USB_V = (1 << 31);
            env->PSW_USB_SV = (1 << 31);
            /* ext_ret > MAX_INT */
            if (mul < 0) {
                result = INT64_MAX;
            /* ext_ret < MIN_INT */
            } else {
               result = INT64_MIN;
            }
        } else {
            env->PSW_USB_V = 0;
        }
    }

    return (uint64_t)result;
}

uint32_t helper_msubr_q_ssov(CPUTriCoreState *env, uint32_t r1, uint32_t r2,
                             uint32_t r3, uint32_t n)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t t3 = sextract64(r3, 0, 32);
    int64_t mul, ret;

    if ((t2 == -0x8000ll) && (t3 == -0x8000ll) && (n == 1)) {
        mul = 0x7fffffff;
    } else {
        mul = (t2 * t3) << n;
    }

    ret = t1 - mul + 0x8000;

    env->PSW_USB_AV = ret ^ ret * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    if (ret > 0x7fffffffll) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV |= env->PSW_USB_V;
        ret = INT32_MAX;
    } else if (ret < -0x80000000ll) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV |= env->PSW_USB_V;
        ret = INT32_MIN;
    } else {
        env->PSW_USB_V = 0;
    }
    return ret & 0xffff0000ll;
}

uint32_t helper_abs_b(CPUTriCoreState *env, uint32_t arg)
{
    int32_t b, i;
    int32_t ovf = 0;
    int32_t avf = 0;
    int32_t ret = 0;

    for (i = 0; i < 4; i++) {
        b = sextract32(arg, i * 8, 8);
        b = (b >= 0) ? b : (0 - b);
        ovf |= (b > 0x7F) || (b < -0x80);
        avf |= b ^ b * 2u;
        ret |= (b & 0xff) << (i * 8);
    }

    env->PSW_USB_V = ovf << 31;
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = avf << 24;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return ret;
}

uint32_t helper_abs_h(CPUTriCoreState *env, uint32_t arg)
{
    int32_t h, i;
    int32_t ovf = 0;
    int32_t avf = 0;
    int32_t ret = 0;

    for (i = 0; i < 2; i++) {
        h = sextract32(arg, i * 16, 16);
        h = (h >= 0) ? h : (0 - h);
        ovf |= (h > 0x7FFF) || (h < -0x8000);
        avf |= h ^ h * 2u;
        ret |= (h & 0xffff) << (i * 16);
    }

    env->PSW_USB_V = ovf << 31;
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = avf << 16;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return ret;
}

uint32_t helper_absdif_b(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t b, i;
    int32_t extr_r2;
    int32_t ovf = 0;
    int32_t avf = 0;
    int32_t ret = 0;

    for (i = 0; i < 4; i++) {
        extr_r2 = sextract32(r2, i * 8, 8);
        b = sextract32(r1, i * 8, 8);
        b = (b > extr_r2) ? (b - extr_r2) : (extr_r2 - b);
        ovf |= (b > 0x7F) || (b < -0x80);
        avf |= b ^ b * 2u;
        ret |= (b & 0xff) << (i * 8);
    }

    env->PSW_USB_V = ovf << 31;
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = avf << 24;
    env->PSW_USB_SAV |= env->PSW_USB_AV;
    return ret;
}

uint32_t helper_absdif_h(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t h, i;
    int32_t extr_r2;
    int32_t ovf = 0;
    int32_t avf = 0;
    int32_t ret = 0;

    for (i = 0; i < 2; i++) {
        extr_r2 = sextract32(r2, i * 16, 16);
        h = sextract32(r1, i * 16, 16);
        h = (h > extr_r2) ? (h - extr_r2) : (extr_r2 - h);
        ovf |= (h > 0x7FFF) || (h < -0x8000);
        avf |= h ^ h * 2u;
        ret |= (h & 0xffff) << (i * 16);
    }

    env->PSW_USB_V = ovf << 31;
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = avf << 16;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return ret;
}

uint32_t helper_addr_h(CPUTriCoreState *env, uint64_t r1, uint32_t r2_l,
                       uint32_t r2_h)
{
    int64_t mul_res0 = sextract64(r1, 0, 32);
    int64_t mul_res1 = sextract64(r1, 32, 32);
    int64_t r2_low = sextract64(r2_l, 0, 32);
    int64_t r2_high = sextract64(r2_h, 0, 32);
    int64_t result0, result1;
    uint32_t ovf0, ovf1;
    uint32_t avf0, avf1;

    ovf0 = ovf1 = 0;

    result0 = r2_low + mul_res0 + 0x8000;
    result1 = r2_high + mul_res1 + 0x8000;

    if ((result0 > INT32_MAX) || (result0 < INT32_MIN)) {
        ovf0 = (1 << 31);
    }

    if ((result1 > INT32_MAX) || (result1 < INT32_MIN)) {
        ovf1 = (1 << 31);
    }

    env->PSW_USB_V = ovf0 | ovf1;
    env->PSW_USB_SV |= env->PSW_USB_V;

    avf0 = result0 * 2u;
    avf0 = result0 ^ avf0;
    avf1 = result1 * 2u;
    avf1 = result1 ^ avf1;

    env->PSW_USB_AV = avf0 | avf1;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return (result1 & 0xffff0000ULL) | ((result0 >> 16) & 0xffffULL);
}

uint32_t helper_addsur_h(CPUTriCoreState *env, uint64_t r1, uint32_t r2_l,
                         uint32_t r2_h)
{
    int64_t mul_res0 = sextract64(r1, 0, 32);
    int64_t mul_res1 = sextract64(r1, 32, 32);
    int64_t r2_low = sextract64(r2_l, 0, 32);
    int64_t r2_high = sextract64(r2_h, 0, 32);
    int64_t result0, result1;
    uint32_t ovf0, ovf1;
    uint32_t avf0, avf1;

    ovf0 = ovf1 = 0;

    result0 = r2_low - mul_res0 + 0x8000;
    result1 = r2_high + mul_res1 + 0x8000;

    if ((result0 > INT32_MAX) || (result0 < INT32_MIN)) {
        ovf0 = (1 << 31);
    }

    if ((result1 > INT32_MAX) || (result1 < INT32_MIN)) {
        ovf1 = (1 << 31);
    }

    env->PSW_USB_V = ovf0 | ovf1;
    env->PSW_USB_SV |= env->PSW_USB_V;

    avf0 = result0 * 2u;
    avf0 = result0 ^ avf0;
    avf1 = result1 * 2u;
    avf1 = result1 ^ avf1;

    env->PSW_USB_AV = avf0 | avf1;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return (result1 & 0xffff0000ULL) | ((result0 >> 16) & 0xffffULL);
}

uint32_t helper_maddr_q(CPUTriCoreState *env, uint32_t r1, uint32_t r2,
                        uint32_t r3, uint32_t n)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t t3 = sextract64(r3, 0, 32);
    int64_t mul, ret;

    if ((t2 == -0x8000ll) && (t3 == -0x8000ll) && (n == 1)) {
        mul = 0x7fffffff;
    } else {
        mul = (t2 * t3) << n;
    }

    ret = t1 + mul + 0x8000;

    if ((ret > 0x7fffffffll) || (ret < -0x80000000ll)) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV |= env->PSW_USB_V;
    } else {
        env->PSW_USB_V = 0;
    }
    env->PSW_USB_AV = ret ^ ret * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return ret & 0xffff0000ll;
}

uint32_t helper_add_b(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t b, i;
    int32_t extr_r1, extr_r2;
    int32_t ovf = 0;
    int32_t avf = 0;
    uint32_t ret = 0;

    for (i = 0; i < 4; i++) {
        extr_r1 = sextract32(r1, i * 8, 8);
        extr_r2 = sextract32(r2, i * 8, 8);

        b = extr_r1 + extr_r2;
        ovf |= ((b > 0x7f) || (b < -0x80));
        avf |= b ^ b * 2u;
        ret |= ((b & 0xff) << (i*8));
    }

    env->PSW_USB_V = (ovf << 31);
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = avf << 24;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return ret;
}

uint32_t helper_add_h(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t h, i;
    int32_t extr_r1, extr_r2;
    int32_t ovf = 0;
    int32_t avf = 0;
    int32_t ret = 0;

    for (i = 0; i < 2; i++) {
        extr_r1 = sextract32(r1, i * 16, 16);
        extr_r2 = sextract32(r2, i * 16, 16);
        h = extr_r1 + extr_r2;
        ovf |= ((h > 0x7fff) || (h < -0x8000));
        avf |= h ^ h * 2u;
        ret |= (h & 0xffff) << (i * 16);
    }

    env->PSW_USB_V = (ovf << 31);
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = (avf << 16);
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return ret;
}

uint32_t helper_subr_h(CPUTriCoreState *env, uint64_t r1, uint32_t r2_l,
                       uint32_t r2_h)
{
    int64_t mul_res0 = sextract64(r1, 0, 32);
    int64_t mul_res1 = sextract64(r1, 32, 32);
    int64_t r2_low = sextract64(r2_l, 0, 32);
    int64_t r2_high = sextract64(r2_h, 0, 32);
    int64_t result0, result1;
    uint32_t ovf0, ovf1;
    uint32_t avf0, avf1;

    ovf0 = ovf1 = 0;

    result0 = r2_low - mul_res0 + 0x8000;
    result1 = r2_high - mul_res1 + 0x8000;

    if ((result0 > INT32_MAX) || (result0 < INT32_MIN)) {
        ovf0 = (1 << 31);
    }

    if ((result1 > INT32_MAX) || (result1 < INT32_MIN)) {
        ovf1 = (1 << 31);
    }

    env->PSW_USB_V = ovf0 | ovf1;
    env->PSW_USB_SV |= env->PSW_USB_V;

    avf0 = result0 * 2u;
    avf0 = result0 ^ avf0;
    avf1 = result1 * 2u;
    avf1 = result1 ^ avf1;

    env->PSW_USB_AV = avf0 | avf1;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return (result1 & 0xffff0000ULL) | ((result0 >> 16) & 0xffffULL);
}

uint32_t helper_subadr_h(CPUTriCoreState *env, uint64_t r1, uint32_t r2_l,
                         uint32_t r2_h)
{
    int64_t mul_res0 = sextract64(r1, 0, 32);
    int64_t mul_res1 = sextract64(r1, 32, 32);
    int64_t r2_low = sextract64(r2_l, 0, 32);
    int64_t r2_high = sextract64(r2_h, 0, 32);
    int64_t result0, result1;
    uint32_t ovf0, ovf1;
    uint32_t avf0, avf1;

    ovf0 = ovf1 = 0;

    result0 = r2_low + mul_res0 + 0x8000;
    result1 = r2_high - mul_res1 + 0x8000;

    if ((result0 > INT32_MAX) || (result0 < INT32_MIN)) {
        ovf0 = (1 << 31);
    }

    if ((result1 > INT32_MAX) || (result1 < INT32_MIN)) {
        ovf1 = (1 << 31);
    }

    env->PSW_USB_V = ovf0 | ovf1;
    env->PSW_USB_SV |= env->PSW_USB_V;

    avf0 = result0 * 2u;
    avf0 = result0 ^ avf0;
    avf1 = result1 * 2u;
    avf1 = result1 ^ avf1;

    env->PSW_USB_AV = avf0 | avf1;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return (result1 & 0xffff0000ULL) | ((result0 >> 16) & 0xffffULL);
}

uint32_t helper_msubr_q(CPUTriCoreState *env, uint32_t r1, uint32_t r2,
                        uint32_t r3, uint32_t n)
{
    int64_t t1 = sextract64(r1, 0, 32);
    int64_t t2 = sextract64(r2, 0, 32);
    int64_t t3 = sextract64(r3, 0, 32);
    int64_t mul, ret;

    if ((t2 == -0x8000ll) && (t3 == -0x8000ll) && (n == 1)) {
        mul = 0x7fffffff;
    } else {
        mul = (t2 * t3) << n;
    }

    ret = t1 - mul + 0x8000;

    if ((ret > 0x7fffffffll) || (ret < -0x80000000ll)) {
        env->PSW_USB_V = (1 << 31);
        env->PSW_USB_SV |= env->PSW_USB_V;
    } else {
        env->PSW_USB_V = 0;
    }
    env->PSW_USB_AV = ret ^ ret * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return ret & 0xffff0000ll;
}

uint32_t helper_sub_b(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t b, i;
    int32_t extr_r1, extr_r2;
    int32_t ovf = 0;
    int32_t avf = 0;
    uint32_t ret = 0;

    for (i = 0; i < 4; i++) {
        extr_r1 = sextract32(r1, i * 8, 8);
        extr_r2 = sextract32(r2, i * 8, 8);

        b = extr_r1 - extr_r2;
        ovf |= ((b > 0x7f) || (b < -0x80));
        avf |= b ^ b * 2u;
        ret |= ((b & 0xff) << (i*8));
    }

    env->PSW_USB_V = (ovf << 31);
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = avf << 24;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return ret;
}

uint32_t helper_sub_h(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t h, i;
    int32_t extr_r1, extr_r2;
    int32_t ovf = 0;
    int32_t avf = 0;
    int32_t ret = 0;

    for (i = 0; i < 2; i++) {
        extr_r1 = sextract32(r1, i * 16, 16);
        extr_r2 = sextract32(r2, i * 16, 16);
        h = extr_r1 - extr_r2;
        ovf |= ((h > 0x7fff) || (h < -0x8000));
        avf |= h ^ h * 2u;
        ret |= (h & 0xffff) << (i * 16);
    }

    env->PSW_USB_V = (ovf << 31);
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = avf << 16;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return ret;
}

uint32_t helper_eq_b(uint32_t r1, uint32_t r2)
{
    uint32_t ret, msk;
    int32_t i;

    ret = 0;
    msk = 0xff;
    for (i = 0; i < 4; i++) {
        if ((r1 & msk) == (r2 & msk)) {
            ret |= msk;
        }
        msk = msk << 8;
    }

    return ret;
}

uint32_t helper_eq_h(uint32_t r1, uint32_t r2)
{
    int32_t ret = 0;

    if ((r1 & 0xffff) == (r2 & 0xffff)) {
        ret = 0xffff;
    }

    if ((r1 & 0xffff0000) == (r2 & 0xffff0000)) {
        ret |= 0xffff0000;
    }

    return ret;
}

uint32_t helper_eqany_b(uint32_t r1, uint32_t r2)
{
    int32_t i;
    uint32_t ret = 0;

    for (i = 0; i < 4; i++) {
        ret |= (sextract32(r1,  i * 8, 8) == sextract32(r2,  i * 8, 8));
    }

    return ret;
}

uint32_t helper_eqany_h(uint32_t r1, uint32_t r2)
{
    uint32_t ret;

    ret = (sextract32(r1, 0, 16) == sextract32(r2,  0, 16));
    ret |= (sextract32(r1, 16, 16) == sextract32(r2,  16, 16));

    return ret;
}

uint32_t helper_lt_b(uint32_t r1, uint32_t r2)
{
    int32_t i;
    uint32_t ret = 0;

    for (i = 0; i < 4; i++) {
        if (sextract32(r1,  i * 8, 8) < sextract32(r2,  i * 8, 8)) {
            ret |= (0xff << (i * 8));
        }
    }

    return ret;
}

uint32_t helper_lt_bu(uint32_t r1, uint32_t r2)
{
    int32_t i;
    uint32_t ret = 0;

    for (i = 0; i < 4; i++) {
        if (extract32(r1,  i * 8, 8) < extract32(r2,  i * 8, 8)) {
            ret |= (0xff << (i * 8));
        }
    }

    return ret;
}

uint32_t helper_lt_h(uint32_t r1, uint32_t r2)
{
    uint32_t ret = 0;

    if (sextract32(r1,  0, 16) < sextract32(r2,  0, 16)) {
        ret |= 0xffff;
    }

    if (sextract32(r1,  16, 16) < sextract32(r2,  16, 16)) {
        ret |= 0xffff0000;
    }

    return ret;
}

uint32_t helper_lt_hu(uint32_t r1, uint32_t r2)
{
    uint32_t ret = 0;

    if (extract32(r1,  0, 16) < extract32(r2,  0, 16)) {
        ret |= 0xffff;
    }

    if (extract32(r1,  16, 16) < extract32(r2,  16, 16)) {
        ret |= 0xffff0000;
    }

    return ret;
}

#define EXTREMA_H_B(name, op)                                 \
uint32_t helper_##name ##_b(uint32_t r1, uint32_t r2)         \
{                                                             \
    int32_t i, extr_r1, extr_r2;                              \
    uint32_t ret = 0;                                         \
                                                              \
    for (i = 0; i < 4; i++) {                                 \
        extr_r1 = sextract32(r1, i * 8, 8);                   \
        extr_r2 = sextract32(r2, i * 8, 8);                   \
        extr_r1 = (extr_r1 op extr_r2) ? extr_r1 : extr_r2;   \
        ret |= (extr_r1 & 0xff) << (i * 8);                   \
    }                                                         \
    return ret;                                               \
}                                                             \
                                                              \
uint32_t helper_##name ##_bu(uint32_t r1, uint32_t r2)        \
{                                                             \
    int32_t i;                                                \
    uint32_t extr_r1, extr_r2;                                \
    uint32_t ret = 0;                                         \
                                                              \
    for (i = 0; i < 4; i++) {                                 \
        extr_r1 = extract32(r1, i * 8, 8);                    \
        extr_r2 = extract32(r2, i * 8, 8);                    \
        extr_r1 = (extr_r1 op extr_r2) ? extr_r1 : extr_r2;   \
        ret |= (extr_r1 & 0xff) << (i * 8);                   \
    }                                                         \
    return ret;                                               \
}                                                             \
                                                              \
uint32_t helper_##name ##_h(uint32_t r1, uint32_t r2)         \
{                                                             \
    int32_t extr_r1, extr_r2;                                 \
    uint32_t ret = 0;                                         \
                                                              \
    extr_r1 = sextract32(r1, 0, 16);                          \
    extr_r2 = sextract32(r2, 0, 16);                          \
    ret = (extr_r1 op extr_r2) ? extr_r1 : extr_r2;           \
    ret = ret & 0xffff;                                       \
                                                              \
    extr_r1 = sextract32(r1, 16, 16);                         \
    extr_r2 = sextract32(r2, 16, 16);                         \
    extr_r1 = (extr_r1 op extr_r2) ? extr_r1 : extr_r2;       \
    ret |= extr_r1 << 16;                                     \
                                                              \
    return ret;                                               \
}                                                             \
                                                              \
uint32_t helper_##name ##_hu(uint32_t r1, uint32_t r2)        \
{                                                             \
    uint32_t extr_r1, extr_r2;                                \
    uint32_t ret = 0;                                         \
                                                              \
    extr_r1 = extract32(r1, 0, 16);                           \
    extr_r2 = extract32(r2, 0, 16);                           \
    ret = (extr_r1 op extr_r2) ? extr_r1 : extr_r2;           \
    ret = ret & 0xffff;                                       \
                                                              \
    extr_r1 = extract32(r1, 16, 16);                          \
    extr_r2 = extract32(r2, 16, 16);                          \
    extr_r1 = (extr_r1 op extr_r2) ? extr_r1 : extr_r2;       \
    ret |= extr_r1 << (16);                                   \
                                                              \
    return ret;                                               \
}                                                             \
                                                              \
uint64_t helper_ix##name(uint64_t r1, uint32_t r2)            \
{                                                             \
    int64_t r2l, r2h, r1hl;                                   \
    uint64_t ret = 0;                                         \
                                                              \
    ret = ((r1 + 2) & 0xffff);                                \
    r2l = sextract64(r2, 0, 16);                              \
    r2h = sextract64(r2, 16, 16);                             \
    r1hl = sextract64(r1, 32, 16);                            \
                                                              \
    if ((r2l op ## = r2h) && (r2l op r1hl)) {                 \
        ret |= (r2l & 0xffff) << 32;                          \
        ret |= extract64(r1, 0, 16) << 16;                    \
    } else if ((r2h op r2l) && (r2h op r1hl)) {               \
        ret |= extract64(r2, 16, 16) << 32;                   \
        ret |= extract64(r1 + 1, 0, 16) << 16;                \
    } else {                                                  \
        ret |= r1 & 0xffffffff0000ull;                        \
    }                                                         \
    return ret;                                               \
}                                                             \
                                                              \
uint64_t helper_ix##name ##_u(uint64_t r1, uint32_t r2)       \
{                                                             \
    int64_t r2l, r2h, r1hl;                                   \
    uint64_t ret = 0;                                         \
                                                              \
    ret = ((r1 + 2) & 0xffff);                                \
    r2l = extract64(r2, 0, 16);                               \
    r2h = extract64(r2, 16, 16);                              \
    r1hl = extract64(r1, 32, 16);                             \
                                                              \
    if ((r2l op ## = r2h) && (r2l op r1hl)) {                 \
        ret |= (r2l & 0xffff) << 32;                          \
        ret |= extract64(r1, 0, 16) << 16;                    \
    } else if ((r2h op r2l) && (r2h op r1hl)) {               \
        ret |= extract64(r2, 16, 16) << 32;                   \
        ret |= extract64(r1 + 1, 0, 16) << 16;                \
    } else {                                                  \
        ret |= r1 & 0xffffffff0000ull;                        \
    }                                                         \
    return ret;                                               \
}

EXTREMA_H_B(max, >)
EXTREMA_H_B(min, <)

#undef EXTREMA_H_B

uint32_t helper_clo_h(uint32_t r1)
{
    uint32_t ret_hw0 = extract32(r1, 0, 16);
    uint32_t ret_hw1 = extract32(r1, 16, 16);

    ret_hw0 = clo32(ret_hw0 << 16);
    ret_hw1 = clo32(ret_hw1 << 16);

    if (ret_hw0 > 16) {
        ret_hw0 = 16;
    }
    if (ret_hw1 > 16) {
        ret_hw1 = 16;
    }

    return ret_hw0 | (ret_hw1 << 16);
}

uint32_t helper_clz_h(uint32_t r1)
{
    uint32_t ret_hw0 = extract32(r1, 0, 16);
    uint32_t ret_hw1 = extract32(r1, 16, 16);

    ret_hw0 = clz32(ret_hw0 << 16);
    ret_hw1 = clz32(ret_hw1 << 16);

    if (ret_hw0 > 16) {
        ret_hw0 = 16;
    }
    if (ret_hw1 > 16) {
        ret_hw1 = 16;
    }

    return ret_hw0 | (ret_hw1 << 16);
}

uint32_t helper_cls_h(uint32_t r1)
{
    uint32_t ret_hw0 = extract32(r1, 0, 16);
    uint32_t ret_hw1 = extract32(r1, 16, 16);

    ret_hw0 = clrsb32(ret_hw0 << 16);
    ret_hw1 = clrsb32(ret_hw1 << 16);

    if (ret_hw0 > 15) {
        ret_hw0 = 15;
    }
    if (ret_hw1 > 15) {
        ret_hw1 = 15;
    }

    return ret_hw0 | (ret_hw1 << 16);
}

uint32_t helper_sh(uint32_t r1, uint32_t r2)
{
    int32_t shift_count = sextract32(r2, 0, 6);

    if (shift_count == -32) {
        return 0;
    } else if (shift_count < 0) {
        return r1 >> -shift_count;
    } else {
        return r1 << shift_count;
    }
}

uint32_t helper_sh_h(uint32_t r1, uint32_t r2)
{
    int32_t ret_hw0, ret_hw1;
    int32_t shift_count;

    shift_count = sextract32(r2, 0, 5);

    if (shift_count == -16) {
        return 0;
    } else if (shift_count < 0) {
        ret_hw0 = extract32(r1, 0, 16) >> -shift_count;
        ret_hw1 = extract32(r1, 16, 16) >> -shift_count;
        return (ret_hw0 & 0xffff) | (ret_hw1 << 16);
    } else {
        ret_hw0 = extract32(r1, 0, 16) << shift_count;
        ret_hw1 = extract32(r1, 16, 16) << shift_count;
        return (ret_hw0 & 0xffff) | (ret_hw1 << 16);
    }
}

uint32_t helper_sha(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t shift_count;
    int64_t result, t1;
    uint32_t ret;

    shift_count = sextract32(r2, 0, 6);
    t1 = sextract32(r1, 0, 32);

    if (shift_count == 0) {
        env->PSW_USB_C = env->PSW_USB_V = 0;
        ret = r1;
    } else if (shift_count == -32) {
        env->PSW_USB_C = r1;
        env->PSW_USB_V = 0;
        ret = t1 >> 31;
    } else if (shift_count > 0) {
        result = t1 << shift_count;
        /* calc carry */
        env->PSW_USB_C = ((result & 0xffffffff00000000ULL) != 0);
        /* calc v */
        env->PSW_USB_V = (((result > 0x7fffffffLL) ||
                           (result < -0x80000000LL)) << 31);
        /* calc sv */
        env->PSW_USB_SV |= env->PSW_USB_V;
        ret = (uint32_t)result;
    } else {
        env->PSW_USB_V = 0;
        env->PSW_USB_C = (r1 & ((1 << -shift_count) - 1));
        ret = t1 >> -shift_count;
    }

    env->PSW_USB_AV = ret ^ ret * 2u;
    env->PSW_USB_SAV |= env->PSW_USB_AV;

    return ret;
}

uint32_t helper_sha_h(uint32_t r1, uint32_t r2)
{
    int32_t shift_count;
    int32_t ret_hw0, ret_hw1;

    shift_count = sextract32(r2, 0, 5);

    if (shift_count == 0) {
        return r1;
    } else if (shift_count < 0) {
        ret_hw0 = sextract32(r1, 0, 16) >> -shift_count;
        ret_hw1 = sextract32(r1, 16, 16) >> -shift_count;
        return (ret_hw0 & 0xffff) | (ret_hw1 << 16);
    } else {
        ret_hw0 = sextract32(r1, 0, 16) << shift_count;
        ret_hw1 = sextract32(r1, 16, 16) << shift_count;
        return (ret_hw0 & 0xffff) | (ret_hw1 << 16);
    }
}

uint32_t helper_bmerge(uint32_t r1, uint32_t r2)
{
    uint32_t i, ret;

    ret = 0;
    for (i = 0; i < 16; i++) {
        ret |= (r1 & 1) << (2 * i + 1);
        ret |= (r2 & 1) << (2 * i);
        r1 = r1 >> 1;
        r2 = r2 >> 1;
    }
    return ret;
}

uint64_t helper_bsplit(uint32_t r1)
{
    int32_t i;
    uint64_t ret;

    ret = 0;
    for (i = 0; i < 32; i = i + 2) {
        /* even */
        ret |= (r1 & 1) << (i/2);
        r1 = r1 >> 1;
        /* odd */
        ret |= (uint64_t)(r1 & 1) << (i/2 + 32);
        r1 = r1 >> 1;
    }
    return ret;
}

uint32_t helper_parity(uint32_t r1)
{
    uint32_t ret;
    uint32_t nOnes, i;

    ret = 0;
    nOnes = 0;
    for (i = 0; i < 8; i++) {
        ret ^= (r1 & 1);
        r1 = r1 >> 1;
    }
    /* second byte */
    nOnes = 0;
    for (i = 0; i < 8; i++) {
        nOnes ^= (r1 & 1);
        r1 = r1 >> 1;
    }
    ret |= nOnes << 8;
    /* third byte */
    nOnes = 0;
    for (i = 0; i < 8; i++) {
        nOnes ^= (r1 & 1);
        r1 = r1 >> 1;
    }
    ret |= nOnes << 16;
    /* fourth byte */
    nOnes = 0;
    for (i = 0; i < 8; i++) {
        nOnes ^= (r1 & 1);
        r1 = r1 >> 1;
    }
    ret |= nOnes << 24;

    return ret;
}

uint32_t helper_pack(uint32_t carry, uint32_t r1_low, uint32_t r1_high,
                     uint32_t r2)
{
    uint32_t ret;
    int32_t fp_exp, fp_frac, temp_exp, fp_exp_frac;
    int32_t int_exp  = r1_high;
    int32_t int_mant = r1_low;
    uint32_t flag_rnd = (int_mant & (1 << 7)) && (
                        (int_mant & (1 << 8)) ||
                        (int_mant & 0x7f)     ||
                        (carry != 0));
    if (((int_mant & (1<<31)) == 0) && (int_exp == 255)) {
        fp_exp = 255;
        fp_frac = extract32(int_mant, 8, 23);
    } else if ((int_mant & (1<<31)) && (int_exp >= 127)) {
        fp_exp  = 255;
        fp_frac = 0;
    } else if ((int_mant & (1<<31)) && (int_exp <= -128)) {
        fp_exp  = 0;
        fp_frac = 0;
    } else if (int_mant == 0) {
        fp_exp  = 0;
        fp_frac = 0;
    } else {
        if (((int_mant & (1 << 31)) == 0)) {
            temp_exp = 0;
        } else {
            temp_exp = int_exp + 128;
        }
        fp_exp_frac = (((temp_exp & 0xff) << 23) |
                      extract32(int_mant, 8, 23))
                      + flag_rnd;
        fp_exp  = extract32(fp_exp_frac, 23, 8);
        fp_frac = extract32(fp_exp_frac, 0, 23);
    }
    ret = r2 & (1 << 31);
    ret = ret + (fp_exp << 23);
    ret = ret + (fp_frac & 0x7fffff);

    return ret;
}

uint64_t helper_unpack(uint32_t arg1)
{
    int32_t fp_exp  = extract32(arg1, 23, 8);
    int32_t fp_frac = extract32(arg1, 0, 23);
    uint64_t ret;
    int32_t int_exp, int_mant;

    if (fp_exp == 255) {
        int_exp = 255;
        int_mant = (fp_frac << 7);
    } else if ((fp_exp == 0) && (fp_frac == 0)) {
        int_exp  = -127;
        int_mant = 0;
    } else if ((fp_exp == 0) && (fp_frac != 0)) {
        int_exp  = -126;
        int_mant = (fp_frac << 7);
    } else {
        int_exp  = fp_exp - 127;
        int_mant = (fp_frac << 7);
        int_mant |= (1 << 30);
    }
    ret = int_exp;
    ret = ret << 32;
    ret |= int_mant;

    return ret;
}

uint64_t helper_dvinit_b_13(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    uint64_t ret;
    int32_t abs_sig_dividend, abs_divisor;

    ret = sextract32(r1, 0, 32);
    ret = ret << 24;
    if (!((r1 & 0x80000000) == (r2 & 0x80000000))) {
        ret |= 0xffffff;
    }

    abs_sig_dividend = abs((int32_t)r1) >> 8;
    abs_divisor = abs((int32_t)r2);
    /* calc overflow
       ofv if (a/b >= 255) <=> (a/255 >= b) */
    env->PSW_USB_V = (abs_sig_dividend >= abs_divisor) << 31;
    env->PSW_USB_V = env->PSW_USB_V << 31;
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = 0;

    return ret;
}

uint64_t helper_dvinit_b_131(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    uint64_t ret = sextract32(r1, 0, 32);

    ret = ret << 24;
    if (!((r1 & 0x80000000) == (r2 & 0x80000000))) {
        ret |= 0xffffff;
    }
    /* calc overflow */
    env->PSW_USB_V = ((r2 == 0) || ((r2 == 0xffffffff) && (r1 == 0xffffff80)));
    env->PSW_USB_V = env->PSW_USB_V << 31;
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = 0;

    return ret;
}

uint64_t helper_dvinit_h_13(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    uint64_t ret;
    int32_t abs_sig_dividend, abs_divisor;

    ret = sextract32(r1, 0, 32);
    ret = ret << 16;
    if (!((r1 & 0x80000000) == (r2 & 0x80000000))) {
        ret |= 0xffff;
    }

    abs_sig_dividend = abs((int32_t)r1) >> 16;
    abs_divisor = abs((int32_t)r2);
    /* calc overflow
       ofv if (a/b >= 0xffff) <=> (a/0xffff >= b) */
    env->PSW_USB_V = (abs_sig_dividend >= abs_divisor) << 31;
    env->PSW_USB_V = env->PSW_USB_V << 31;
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = 0;

    return ret;
}

uint64_t helper_dvinit_h_131(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    uint64_t ret = sextract32(r1, 0, 32);

    ret = ret << 16;
    if (!((r1 & 0x80000000) == (r2 & 0x80000000))) {
        ret |= 0xffff;
    }
    /* calc overflow */
    env->PSW_USB_V = ((r2 == 0) || ((r2 == 0xffffffff) && (r1 == 0xffff8000)));
    env->PSW_USB_V = env->PSW_USB_V << 31;
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = 0;

    return ret;
}

uint64_t helper_dvadj(uint64_t r1, uint32_t r2)
{
    int32_t x_sign = (r1 >> 63);
    int32_t q_sign = x_sign ^ (r2 >> 31);
    int32_t eq_pos = x_sign & ((r1 >> 32) == r2);
    int32_t eq_neg = x_sign & ((r1 >> 32) == -r2);
    uint32_t quotient;
    uint64_t remainder;

    if ((q_sign & ~eq_neg) | eq_pos) {
        quotient = (r1 + 1) & 0xffffffff;
    } else {
        quotient = r1 & 0xffffffff;
    }

    if (eq_pos | eq_neg) {
        remainder = 0;
    } else {
        remainder = (r1 & 0xffffffff00000000ull);
    }
    return remainder | quotient;
}

uint64_t helper_dvstep(uint64_t r1, uint32_t r2)
{
    int32_t dividend_sign = extract64(r1, 63, 1);
    int32_t divisor_sign = extract32(r2, 31, 1);
    int32_t quotient_sign = (dividend_sign != divisor_sign);
    int32_t addend, dividend_quotient, remainder;
    int32_t i, temp;

    if (quotient_sign) {
        addend = r2;
    } else {
        addend = -r2;
    }
    dividend_quotient = (int32_t)r1;
    remainder = (int32_t)(r1 >> 32);

    for (i = 0; i < 8; i++) {
        remainder = (remainder << 1) | extract32(dividend_quotient, 31, 1);
        dividend_quotient <<= 1;
        temp = remainder + addend;
        if ((temp < 0) == dividend_sign) {
            remainder = temp;
        }
        if (((temp < 0) == dividend_sign)) {
            dividend_quotient = dividend_quotient | !quotient_sign;
        } else {
            dividend_quotient = dividend_quotient | quotient_sign;
        }
    }
    return ((uint64_t)remainder << 32) | (uint32_t)dividend_quotient;
}

uint64_t helper_dvstep_u(uint64_t r1, uint32_t r2)
{
    int32_t dividend_quotient = extract64(r1, 0, 32);
    int64_t remainder = extract64(r1, 32, 32);
    int32_t i;
    int64_t temp;
    for (i = 0; i < 8; i++) {
        remainder = (remainder << 1) | extract32(dividend_quotient, 31, 1);
        dividend_quotient <<= 1;
        temp = (remainder & 0xffffffff) - r2;
        if (temp >= 0) {
            remainder = temp;
        }
        dividend_quotient = dividend_quotient | !(temp < 0);
    }
    return ((uint64_t)remainder << 32) | (uint32_t)dividend_quotient;
}

uint64_t helper_divide(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    int32_t quotient, remainder;
    int32_t dividend = (int32_t)r1;
    int32_t divisor = (int32_t)r2;

    if (divisor == 0) {
        if (dividend >= 0) {
            quotient = 0x7fffffff;
            remainder = 0;
        } else {
            quotient = 0x80000000;
            remainder = 0;
        }
        env->PSW_USB_V = (1 << 31);
    } else if ((divisor == 0xffffffff) && (dividend == 0x80000000)) {
        quotient = 0x7fffffff;
        remainder = 0;
        env->PSW_USB_V = (1 << 31);
    } else {
        remainder = dividend % divisor;
        quotient = (dividend - remainder)/divisor;
        env->PSW_USB_V = 0;
    }
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = 0;
    return ((uint64_t)remainder << 32) | (uint32_t)quotient;
}

uint64_t helper_divide_u(CPUTriCoreState *env, uint32_t r1, uint32_t r2)
{
    uint32_t quotient, remainder;
    uint32_t dividend = r1;
    uint32_t divisor = r2;

    if (divisor == 0) {
        quotient = 0xffffffff;
        remainder = 0;
        env->PSW_USB_V = (1 << 31);
    } else {
        remainder = dividend % divisor;
        quotient = (dividend - remainder)/divisor;
        env->PSW_USB_V = 0;
    }
    env->PSW_USB_SV |= env->PSW_USB_V;
    env->PSW_USB_AV = 0;
    return ((uint64_t)remainder << 32) | quotient;
}

uint64_t helper_mul_h(uint32_t arg00, uint32_t arg01,
                      uint32_t arg10, uint32_t arg11, uint32_t n)
{
    uint32_t result0, result1;

    int32_t sc1 = ((arg00 & 0xffff) == 0x8000) &&
                  ((arg10 & 0xffff) == 0x8000) && (n == 1);
    int32_t sc0 = ((arg01 & 0xffff) == 0x8000) &&
                  ((arg11 & 0xffff) == 0x8000) && (n == 1);
    if (sc1) {
        result1 = 0x7fffffff;
    } else {
        result1 = (((uint32_t)(arg00 * arg10)) << n);
    }
    if (sc0) {
        result0 = 0x7fffffff;
    } else {
        result0 = (((uint32_t)(arg01 * arg11)) << n);
    }
    return (((uint64_t)result1 << 32)) | result0;
}

uint64_t helper_mulm_h(uint32_t arg00, uint32_t arg01,
                       uint32_t arg10, uint32_t arg11, uint32_t n)
{
    uint64_t ret;
    int64_t result0, result1;

    int32_t sc1 = ((arg00 & 0xffff) == 0x8000) &&
                  ((arg10 & 0xffff) == 0x8000) && (n == 1);
    int32_t sc0 = ((arg01 & 0xffff) == 0x8000) &&
                  ((arg11 & 0xffff) == 0x8000) && (n == 1);

    if (sc1) {
        result1 = 0x7fffffff;
    } else {
        result1 = (((int32_t)arg00 * (int32_t)arg10) << n);
    }
    if (sc0) {
        result0 = 0x7fffffff;
    } else {
        result0 = (((int32_t)arg01 * (int32_t)arg11) << n);
    }
    ret = (result1 + result0);
    ret = ret << 16;
    return ret;
}
uint32_t helper_mulr_h(uint32_t arg00, uint32_t arg01,
                       uint32_t arg10, uint32_t arg11, uint32_t n)
{
    uint32_t result0, result1;

    int32_t sc1 = ((arg00 & 0xffff) == 0x8000) &&
                  ((arg10 & 0xffff) == 0x8000) && (n == 1);
    int32_t sc0 = ((arg01 & 0xffff) == 0x8000) &&
                  ((arg11 & 0xffff) == 0x8000) && (n == 1);

    if (sc1) {
        result1 = 0x7fffffff;
    } else {
        result1 = ((arg00 * arg10) << n) + 0x8000;
    }
    if (sc0) {
        result0 = 0x7fffffff;
    } else {
        result0 = ((arg01 * arg11) << n) + 0x8000;
    }
    return (result1 & 0xffff0000) | (result0 >> 16);
}

uint32_t helper_crc32b(uint32_t arg0, uint32_t arg1)
{
    uint8_t buf[1] = { arg0 & 0xff };

    return crc32(arg1, buf, 1);
}


uint32_t helper_crc32_be(uint32_t arg0, uint32_t arg1)
{
    uint8_t buf[4];
    stl_be_p(buf, arg0);

    return crc32(arg1, buf, 4);
}

uint32_t helper_crc32_le(uint32_t arg0, uint32_t arg1)
{
    uint8_t buf[4];
    stl_le_p(buf, arg0);

    return crc32(arg1, buf, 4);
}

static uint32_t crc_div(uint32_t crc_in, uint32_t data, uint32_t gen,
                        uint32_t n, uint32_t m)
{
    uint32_t i;

    data = data << n;
    for (i = 0; i < m; i++) {
        if (crc_in & (1u << (n - 1))) {
            crc_in <<= 1;
            if (data & (1u << (m - 1))) {
                crc_in++;
            }
            crc_in ^= gen;
        } else {
            crc_in <<= 1;
            if (data & (1u << (m - 1))) {
                crc_in++;
            }
        }
        data <<= 1;
    }

    return crc_in;
}

uint32_t helper_crcn(uint32_t arg0, uint32_t arg1, uint32_t arg2)
{
    uint32_t crc_out, crc_in;
    uint32_t n = extract32(arg0, 12, 4) + 1;
    uint32_t gen = extract32(arg0, 16, n);
    uint32_t inv = extract32(arg0, 9, 1);
    uint32_t le = extract32(arg0, 8, 1);
    uint32_t m = extract32(arg0, 0, 3) + 1;
    uint32_t data = extract32(arg1, 0, m);
    uint32_t seed = extract32(arg2, 0, n);

    if (le == 1) {
        if (m == 0) {
            data = 0;
        } else {
            data = revbit32(data) >> (32 - m);
        }
    }

    if (inv == 1) {
        seed = ~seed;
    }

    if (m > n) {
        crc_in = (data >> (m - n)) ^ seed;
    } else {
        crc_in = (data << (n - m)) ^ seed;
    }

    crc_out = crc_div(crc_in, data, gen, n, m);

    if (inv) {
        crc_out = ~crc_out;
    }

    return extract32(crc_out, 0, n);
}

uint32_t helper_shuffle(uint32_t arg0, uint32_t arg1)
{
    uint32_t resb;
    uint32_t byte_select;
    uint32_t res = 0;

    byte_select = arg1 & 0x3;
    resb = extract32(arg0, byte_select * 8, 8);
    res |= resb << 0;

    byte_select = (arg1 >> 2) & 0x3;
    resb = extract32(arg0, byte_select * 8, 8);
    res |= resb << 8;

    byte_select = (arg1 >> 4) & 0x3;
    resb = extract32(arg0, byte_select * 8, 8);
    res |= resb << 16;

    byte_select = (arg1 >> 6) & 0x3;
    resb = extract32(arg0, byte_select * 8, 8);
    res |= resb << 24;

    if (arg1 & 0x100) {
        /* Assign the correct nibble position.  */
        res = ((res & 0xf0f0f0f0) >> 4)
          | ((res & 0x0f0f0f0f) << 4);
        /* Assign the correct bit position.  */
        res = ((res & 0x88888888) >> 3)
          | ((res & 0x44444444) >> 1)
          | ((res & 0x22222222) << 1)
          | ((res & 0x11111111) << 3);
    }

    return res;
}

/* context save area (CSA) related helpers */

static int cdc_increment(uint32_t *psw)
{
    if ((*psw & MASK_PSW_CDC) == 0x7f) {
        return 0;
    }

    (*psw)++;
    /* check for overflow */
    int lo = clo32((*psw & MASK_PSW_CDC) << (32 - 7));
    int mask = (1u << (7 - lo)) - 1;
    int count = *psw & mask;
    if (count == 0) {
        (*psw)--;
        return 1;
    }
    return 0;
}

static int cdc_decrement(uint32_t *psw)
{
    if ((*psw & MASK_PSW_CDC) == 0x7f) {
        return 0;
    }
    /* check for underflow */
    int lo = clo32((*psw & MASK_PSW_CDC) << (32 - 7));
    int mask = (1u << (7 - lo)) - 1;
    int count = *psw & mask;
    if (count == 0) {
        return 1;
    }
    (*psw)--;
    return 0;
}

static bool cdc_zero(uint32_t *psw)
{
    int cdc = *psw & MASK_PSW_CDC;
    /* Returns TRUE if PSW.CDC.COUNT == 0 or if PSW.CDC ==
       7'b1111111, otherwise returns FALSE. */
    if (cdc == 0x7f) {
        return true;
    }
    /* find CDC.COUNT */
    int lo = clo32((*psw & MASK_PSW_CDC) << (32 - 7));
    int mask = (1u << (7 - lo)) - 1;
    int count = *psw & mask;
    return count == 0;
}

static void save_context_upper(CPUTriCoreState *env, uint32_t ea)
{
    cpu_stl_le_data(env, ea, env->PCXI);
    cpu_stl_le_data(env, ea + 4, psw_read(env));
    cpu_stl_le_data(env, ea + 8, env->gpr_a[10]);
    cpu_stl_le_data(env, ea + 12, env->gpr_a[11]);
    cpu_stl_le_data(env, ea + 16, env->gpr_d[8]);
    cpu_stl_le_data(env, ea + 20, env->gpr_d[9]);
    cpu_stl_le_data(env, ea + 24, env->gpr_d[10]);
    cpu_stl_le_data(env, ea + 28, env->gpr_d[11]);
    cpu_stl_le_data(env, ea + 32, env->gpr_a[12]);
    cpu_stl_le_data(env, ea + 36, env->gpr_a[13]);
    cpu_stl_le_data(env, ea + 40, env->gpr_a[14]);
    cpu_stl_le_data(env, ea + 44, env->gpr_a[15]);
    cpu_stl_le_data(env, ea + 48, env->gpr_d[12]);
    cpu_stl_le_data(env, ea + 52, env->gpr_d[13]);
    cpu_stl_le_data(env, ea + 56, env->gpr_d[14]);
    cpu_stl_le_data(env, ea + 60, env->gpr_d[15]);
}

static void save_context_lower(CPUTriCoreState *env, uint32_t ea)
{
    cpu_stl_le_data(env, ea, env->PCXI);
    cpu_stl_le_data(env, ea + 4, env->gpr_a[11]);
    cpu_stl_le_data(env, ea + 8, env->gpr_a[2]);
    cpu_stl_le_data(env, ea + 12, env->gpr_a[3]);
    cpu_stl_le_data(env, ea + 16, env->gpr_d[0]);
    cpu_stl_le_data(env, ea + 20, env->gpr_d[1]);
    cpu_stl_le_data(env, ea + 24, env->gpr_d[2]);
    cpu_stl_le_data(env, ea + 28, env->gpr_d[3]);
    cpu_stl_le_data(env, ea + 32, env->gpr_a[4]);
    cpu_stl_le_data(env, ea + 36, env->gpr_a[5]);
    cpu_stl_le_data(env, ea + 40, env->gpr_a[6]);
    cpu_stl_le_data(env, ea + 44, env->gpr_a[7]);
    cpu_stl_le_data(env, ea + 48, env->gpr_d[4]);
    cpu_stl_le_data(env, ea + 52, env->gpr_d[5]);
    cpu_stl_le_data(env, ea + 56, env->gpr_d[6]);
    cpu_stl_le_data(env, ea + 60, env->gpr_d[7]);
}

static void restore_context_upper(CPUTriCoreState *env, uint32_t ea,
                                  uint32_t *new_PCXI, uint32_t *new_PSW)
{
    *new_PCXI = cpu_ldl_le_data(env, ea);
    *new_PSW = cpu_ldl_le_data(env, ea + 4);
    env->gpr_a[10] = cpu_ldl_le_data(env, ea + 8);
    env->gpr_a[11] = cpu_ldl_le_data(env, ea + 12);
    env->gpr_d[8]  = cpu_ldl_le_data(env, ea + 16);
    env->gpr_d[9]  = cpu_ldl_le_data(env, ea + 20);
    env->gpr_d[10] = cpu_ldl_le_data(env, ea + 24);
    env->gpr_d[11] = cpu_ldl_le_data(env, ea + 28);
    env->gpr_a[12] = cpu_ldl_le_data(env, ea + 32);
    env->gpr_a[13] = cpu_ldl_le_data(env, ea + 36);
    env->gpr_a[14] = cpu_ldl_le_data(env, ea + 40);
    env->gpr_a[15] = cpu_ldl_le_data(env, ea + 44);
    env->gpr_d[12] = cpu_ldl_le_data(env, ea + 48);
    env->gpr_d[13] = cpu_ldl_le_data(env, ea + 52);
    env->gpr_d[14] = cpu_ldl_le_data(env, ea + 56);
    env->gpr_d[15] = cpu_ldl_le_data(env, ea + 60);
}

static void restore_context_lower(CPUTriCoreState *env, uint32_t ea,
                                  uint32_t *ra, uint32_t *pcxi)
{
    *pcxi = cpu_ldl_le_data(env, ea);
    *ra = cpu_ldl_le_data(env, ea + 4);
    env->gpr_a[2] = cpu_ldl_le_data(env, ea + 8);
    env->gpr_a[3] = cpu_ldl_le_data(env, ea + 12);
    env->gpr_d[0] = cpu_ldl_le_data(env, ea + 16);
    env->gpr_d[1] = cpu_ldl_le_data(env, ea + 20);
    env->gpr_d[2] = cpu_ldl_le_data(env, ea + 24);
    env->gpr_d[3] = cpu_ldl_le_data(env, ea + 28);
    env->gpr_a[4] = cpu_ldl_le_data(env, ea + 32);
    env->gpr_a[5] = cpu_ldl_le_data(env, ea + 36);
    env->gpr_a[6] = cpu_ldl_le_data(env, ea + 40);
    env->gpr_a[7] = cpu_ldl_le_data(env, ea + 44);
    env->gpr_d[4] = cpu_ldl_le_data(env, ea + 48);
    env->gpr_d[5] = cpu_ldl_le_data(env, ea + 52);
    env->gpr_d[6] = cpu_ldl_le_data(env, ea + 56);
    env->gpr_d[7] = cpu_ldl_le_data(env, ea + 60);
}

/*
 * TC1797 hardware-interrupt entry (ported from the e:fs QEMU TriCore fork,
 * adapted to mainline). Mainline stubs interrupts out; this implements the
 * canonical interrupt-entry sequence (Architecture Vol.1 §5): save upper
 * context to the CSA at FCX, build PCXI (UL=1, PIE=old IE, PCPN=old CCPN),
 * disable IE, switch to the interrupt stack, set supervisor mode, ICR.CCPN =
 * ICR.PIPN, A[11] = return PC, then vector to BIV + PIPN*(VSS?8:32). The board
 * raises an interrupt by setting ICR.PIPN + cpu_interrupt(CPU_INTERRUPT_HARD);
 * tricore_cpu_exec_interrupt gates on IE && PIPN>CCPN and calls this.
 */
void tricore_cpu_do_interrupt(CPUState *cs)
{
    TriCoreCPU *cpu = TRICORE_CPU(cs);
    CPUTriCoreState *env = &cpu->env;
    target_ulong ea, new_FCX, psw;
    uint32_t pipn;

    psw = psw_read(env);
    cs->exception_index = -1;

    if (env->PSW & MASK_PSW_CDE) {
        cdc_increment(&psw);            /* CDO trap path omitted (per e:fs) */
    }
    psw = (env->PSW & ~MASK_PSW_CDE) | (1u << 7);

    /* DIAGNOSTIC (env TC1797_DBG): interrupt entry saves the upper context into a
     * CSA from the free list but never checks FCX==0 (FCU) or FCX==LCX (FCD).
     * If the list is exhausted when an IRQ fires, ea becomes 0 and we corrupt
     * memory at address 0 + the PCXI chain -> non-deterministic boot. Log if it
     * ever happens during boot (capped). */
    {
        static int csa_dbg = -1;
        if (csa_dbg < 0) {
            csa_dbg = getenv("TC1797_DBG") ? 1 : 0;
        }
        if (csa_dbg && (env->FCX == 0
                        || (env->FCX & 0xfffffu) == (env->LCX & 0xfffffu))) {
            static unsigned fcu_n;
            if (fcu_n++ < 60) {
                fprintf(stderr, "TCDBG IRQ-CSA-EXHAUST FCX=%08x LCX=%08x PC=%08x pipn=%u\n",
                        env->FCX, env->LCX, env->PC, FIELD_EX32(env->ICR, ICR, PIPN));
                fflush(stderr);
            }
        }
    }

    /* Save upper context of the interrupted task into the CSA at FCX. */
    ea = ((env->FCX & MASK_FCX_FCXS) << 12) + ((env->FCX & MASK_FCX_FCXO) << 6);
    new_FCX = cpu_ldl_le_data(env, ea);
    helper_stucx(env, ea);

    env->PSW = psw;
    pcxi_set_ul(env, 1);
    pcxi_set_pie(env, icr_get_ie(env)); /* PCXI.PIE = old ICR.IE */
    icr_set_ie(env, 0);                 /* globally disable interrupts */
    pcxi_set_pcpn(env, icr_get_ccpn(env)); /* PCXI.PCPN = old ICR.CCPN */

    /* Switch to the interrupt stack if not already on it (PSW.IS). */
    if (((env->PSW & MASK_PSW_IS) >> 9) == 0) {
        env->gpr_a[10] = env->ISP;
        env->PSW = (env->PSW & ~MASK_PSW_IS) | (1u << 9);
    }
    env->PSW = (env->PSW & ~MASK_PSW_IO) | (0b10u << 10);  /* supervisor */
    env->PSW = (env->PSW & ~MASK_PSW_PRS);                 /* PRS = 0 */
    env->PSW = (env->PSW & ~MASK_PSW_CDC);                 /* CDC = 0 */
    env->PSW = (env->PSW & ~MASK_PSW_CDE) | (1u << 7);     /* CDE = 1 */
    env->PSW = (env->PSW & ~MASK_PSW_GW);                  /* GW = 0 */

    /* ICR.CCPN = ICR.PIPN (the interrupt now becomes the current priority). */
    pipn = FIELD_EX32(env->ICR, ICR, PIPN);
    env->ICR = FIELD_DP32(env->ICR, ICR, CCPN, pipn);

    env->gpr_a[11] = env->PC;           /* return address */
    /* New PC = BIV + PIPN * (BIV.VSS ? 8 : 32). */
    env->PC = (env->BIV & 0xFFFFFFFEu) | (pipn << ((env->BIV & 0x1u) ? 3 : 5));

    /* Advance the CSA free-list: PCXI[19:0] = FCX[19:0]; FCX = new_FCX. */
    env->PCXI = (env->PCXI & 0xfff00000u) | (env->FCX & 0xfffffu);
    env->FCX = (env->FCX & 0xfff00000u) | (new_FCX & 0xfffffu);
}

void helper_call(CPUTriCoreState *env, uint32_t next_pc)
{
    uint32_t tmp_FCX;
    uint32_t ea;
    uint32_t new_FCX;
    uint32_t psw;

    psw = psw_read(env);
    /* if (FCX == 0) trap(FCU); */
    if (env->FCX == 0) {
        /* FCU trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_FCU, GETPC());
    }
    /* if (PSW.CDE) then if (cdc_increment()) then trap(CDO); */
    if (psw & MASK_PSW_CDE) {
        if (cdc_increment(&psw)) {
            /* CDO trap */
            raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_CDO, GETPC());
        }
    }
    /* PSW.CDE = 1;*/
    psw |= MASK_PSW_CDE;
    /*
     * we need to save PSW.CDE and not PSW.CDC into the CSAs. psw already
     * contains the CDC from cdc_increment(), so we cannot call psw_write()
     * here.
     */
    env->PSW |= MASK_PSW_CDE;

    /* tmp_FCX = FCX; */
    tmp_FCX = env->FCX;
    /* EA = {FCX.FCXS, 6'b0, FCX.FCXO, 6'b0}; */
    ea = ((env->FCX & MASK_FCX_FCXS) << 12) +
         ((env->FCX & MASK_FCX_FCXO) << 6);
    /* new_FCX = M(EA, word); */
    new_FCX = cpu_ldl_le_data(env, ea);
    /* M(EA, 16 * word) = {PCXI, PSW, A[10], A[11], D[8], D[9], D[10], D[11],
                           A[12], A[13], A[14], A[15], D[12], D[13], D[14],
                           D[15]}; */
    save_context_upper(env, ea);

    /* PCXI.PCPN = ICR.CCPN; */
    pcxi_set_pcpn(env, icr_get_ccpn(env));
    /* PCXI.PIE = ICR.IE; */
    pcxi_set_pie(env, icr_get_ie(env));
    /* PCXI.UL = 1; */
    pcxi_set_ul(env, 1);

    /* PCXI[19: 0] = FCX[19: 0]; */
    env->PCXI = (env->PCXI & 0xfff00000) + (env->FCX & 0xfffff);
    /* FCX[19: 0] = new_FCX[19: 0]; */
    env->FCX = (env->FCX & 0xfff00000) + (new_FCX & 0xfffff);
    /* A[11] = next_pc[31: 0]; */
    env->gpr_a[11] = next_pc;

    /* if (tmp_FCX == LCX) trap(FCD);*/
    if (tmp_FCX == env->LCX) {
        /* FCD trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_FCD, GETPC());
    }
    psw_write(env, psw);
}

void helper_ret(CPUTriCoreState *env)
{
    uint32_t ea;
    uint32_t new_PCXI;
    uint32_t new_PSW, psw;

    psw = psw_read(env);
     /* if (PSW.CDE) then if (cdc_decrement()) then trap(CDU);*/
    if (psw & MASK_PSW_CDE) {
        if (cdc_decrement(&psw)) {
            /* CDU trap */
            psw_write(env, psw);
            raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_CDU, GETPC());
        }
    }
    /*   if (PCXI[19: 0] == 0) then trap(CSU); */
    if ((env->PCXI & 0xfffff) == 0) {
        /* CSU trap */
        psw_write(env, psw);
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_CSU, GETPC());
    }
    /* if (PCXI.UL == 0) then trap(CTYP); */
    if (pcxi_get_ul(env) == 0) {
        /* CTYP trap */
        cdc_increment(&psw); /* restore to the start of helper */
        psw_write(env, psw);
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_CTYP, GETPC());
    }
    /* PC = {A11 [31: 1], 1’b0}; */
    env->PC = env->gpr_a[11] & 0xfffffffe;

    /* EA = {PCXI.PCXS, 6'b0, PCXI.PCXO, 6'b0}; */
    ea = (pcxi_get_pcxs(env) << 28) |
         (pcxi_get_pcxo(env) << 6);
    /* {new_PCXI, new_PSW, A[10], A[11], D[8], D[9], D[10], D[11], A[12],
        A[13], A[14], A[15], D[12], D[13], D[14], D[15]} = M(EA, 16 * word); */
    restore_context_upper(env, ea, &new_PCXI, &new_PSW);
    /* M(EA, word) = FCX; */
    cpu_stl_le_data(env, ea, env->FCX);
    /* FCX[19: 0] = PCXI[19: 0]; */
    env->FCX = (env->FCX & 0xfff00000) + (env->PCXI & 0x000fffff);
    /* PCXI = new_PCXI; */
    env->PCXI = new_PCXI;

    if (tricore_has_feature(env, TRICORE_FEATURE_131)) {
        /* PSW = {new_PSW[31:26], PSW[25:24], new_PSW[23:0]}; */
        psw_write(env, (new_PSW & ~(0x3000000)) + (psw & (0x3000000)));
    } else { /* TRICORE_FEATURE_13 only */
        /* PSW = new_PSW */
        psw_write(env, new_PSW);
    }
}

void helper_bisr(CPUTriCoreState *env, uint32_t const9)
{
    uint32_t tmp_FCX;
    uint32_t ea;
    uint32_t new_FCX;

    if (env->FCX == 0) {
        /* FCU trap */
       raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_FCU, GETPC());
    }

    tmp_FCX = env->FCX;
    ea = ((env->FCX & 0xf0000) << 12) + ((env->FCX & 0xffff) << 6);

    /* new_FCX = M(EA, word); */
    new_FCX = cpu_ldl_le_data(env, ea);
    /* M(EA, 16 * word) = {PCXI, A[11], A[2], A[3], D[0], D[1], D[2], D[3], A[4]
                           , A[5], A[6], A[7], D[4], D[5], D[6], D[7]}; */
    save_context_lower(env, ea);


    /* PCXI.PCPN = ICR.CCPN */
    pcxi_set_pcpn(env, icr_get_ccpn(env));
    /* PCXI.PIE  = ICR.IE */
    pcxi_set_pie(env, icr_get_ie(env));
    /* PCXI.UL = 0 */
    pcxi_set_ul(env, 0);

    /* PCXI[19: 0] = FCX[19: 0] */
    env->PCXI = (env->PCXI & 0xfff00000) + (env->FCX & 0xfffff);
    /* FXC[19: 0] = new_FCX[19: 0] */
    env->FCX = (env->FCX & 0xfff00000) + (new_FCX & 0xfffff);

    /* ICR.IE = 1 */
    icr_set_ie(env, 1);

    icr_set_ccpn(env, const9);

    if (tmp_FCX == env->LCX) {
        /* FCD trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_FCD, GETPC());
    }
}

void helper_rfe(CPUTriCoreState *env)
{
    uint32_t ea;
    uint32_t new_PCXI;
    uint32_t new_PSW;
    /* if (PCXI[19: 0] == 0) then trap(CSU); */
    if ((env->PCXI & 0xfffff) == 0) {
        /* raise csu trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_CSU, GETPC());
    }
    /* if (PCXI.UL == 0) then trap(CTYP); */
    if (pcxi_get_ul(env) == 0) {
        /* raise CTYP trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_CTYP, GETPC());
    }
    /* if (!cdc_zero() AND PSW.CDE) then trap(NEST); */
    if (!cdc_zero(&(env->PSW)) && (env->PSW & MASK_PSW_CDE)) {
        /* raise NEST trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_NEST, GETPC());
    }
    env->PC = env->gpr_a[11] & ~0x1;
    /* ICR.IE = PCXI.PIE; */
    icr_set_ie(env, pcxi_get_pie(env));

    /* Capture the CCPN we are returning FROM (the ISR level), before the restore. */
    uint32_t rfe_old_ccpn = FIELD_EX32(env->ICR, ICR, CCPN);
    /* ICR.CCPN = PCXI.PCPN; */
    icr_set_ccpn(env, pcxi_get_pcpn(env));

    /* The return just LOWERED CCPN to the pre-interrupt level. Faithful TC1.3.1
     * ICU: re-arbitrate so the highest still-asserting SRN is presented as PIPN
     * against the new CCPN -- the level-held OSEK dispatch SRPN-1, pending the
     * whole STM tick ISR, then satisfies PIPN>CCPN and is taken in the idle gap
     * by the DISAS_EXIT re-check this RFE forces. SAFE to write env->ICR here:
     * helper_rfe is declared without TCG_CALL_NO_WG, so TCG syncs cpu_ICR to env
     * before the call and reloads it after -- the arbiter's PIPN write sticks,
     * like the icr_set_ccpn/icr_set_ie writes already in this helper.
     * GATE rfe_old_ccpn > 1: only re-present when returning from a HIGHER-priority
     * ISR (e.g. the SRPN-7/8 STM tick at CCPN 7/8), NOT from the SRPN-1 dispatcher
     * itself (CCPN 1). SRPN-1 is continuously pending (the firmware holds its SRR),
     * so re-arbitrating after the dispatcher's OWN rfe would re-present + re-take it
     * immediately -> infinite dispatch loop wedging the OS. Without the gate the
     * dispatcher runs ~once/115ms (only on a full nest unwind) -> watchdog 0x3045. */
    if (tricore_icu_rfe && rfe_old_ccpn > 1) {
        tricore_icu_rfe(tricore_icu_rfe_ctx);
    }

    /*EA = {PCXI.PCXS, 6'b0, PCXI.PCXO, 6'b0};*/
    ea = (pcxi_get_pcxs(env) << 28) |
         (pcxi_get_pcxo(env) << 6);

    /*{new_PCXI, PSW, A[10], A[11], D[8], D[9], D[10], D[11], A[12],
      A[13], A[14], A[15], D[12], D[13], D[14], D[15]} = M(EA, 16 * word); */
    restore_context_upper(env, ea, &new_PCXI, &new_PSW);
    /* M(EA, word) = FCX;*/
    cpu_stl_le_data(env, ea, env->FCX);
    /* FCX[19: 0] = PCXI[19: 0]; */
    env->FCX = (env->FCX & 0xfff00000) + (env->PCXI & 0x000fffff);
    /* PCXI = new_PCXI; */
    env->PCXI = new_PCXI;
    /* write psw */
    psw_write(env, new_PSW);
}

void helper_rfm(CPUTriCoreState *env)
{
    env->PC = (env->gpr_a[11] & ~0x1);
    /* ICR.IE = PCXI.PIE; */
    icr_set_ie(env, pcxi_get_pie(env));
    /* ICR.CCPN = PCXI.PCPN; */
    icr_set_ccpn(env, pcxi_get_pcpn(env));

    /* {PCXI, PSW, A[10], A[11]} = M(DCX, 4 * word); */
    env->PCXI = cpu_ldl_le_data(env, env->DCX);
    psw_write(env, cpu_ldl_le_data(env, env->DCX + 4));
    env->gpr_a[10] = cpu_ldl_le_data(env, env->DCX + 8);
    env->gpr_a[11] = cpu_ldl_le_data(env, env->DCX + 12);

    if (tricore_has_feature(env, TRICORE_FEATURE_131)) {
        env->DBGTCR = 0;
    }
}

void helper_ldlcx(CPUTriCoreState *env, uint32_t ea)
{
    uint32_t dummy;
    /* insn doesn't load PCXI and RA */
    restore_context_lower(env, ea, &dummy, &dummy);
}

void helper_lducx(CPUTriCoreState *env, uint32_t ea)
{
    uint32_t dummy;
    /* insn doesn't load PCXI and PSW */
    restore_context_upper(env, ea, &dummy, &dummy);
}

void helper_stlcx(CPUTriCoreState *env, uint32_t ea)
{
    save_context_lower(env, ea);
}

void helper_stucx(CPUTriCoreState *env, uint32_t ea)
{
    save_context_upper(env, ea);
}

void helper_svlcx(CPUTriCoreState *env)
{
    uint32_t tmp_FCX;
    uint32_t ea;
    uint32_t new_FCX;

    if (env->FCX == 0) {
        /* FCU trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_FCU, GETPC());
    }
    /* tmp_FCX = FCX; */
    tmp_FCX = env->FCX;
    /* EA = {FCX.FCXS, 6'b0, FCX.FCXO, 6'b0}; */
    ea = ((env->FCX & MASK_FCX_FCXS) << 12) +
         ((env->FCX & MASK_FCX_FCXO) << 6);
    /* new_FCX = M(EA, word); */
    new_FCX = cpu_ldl_le_data(env, ea);
    /* M(EA, 16 * word) = {PCXI, PSW, A[10], A[11], D[8], D[9], D[10], D[11],
                           A[12], A[13], A[14], A[15], D[12], D[13], D[14],
                           D[15]}; */
    save_context_lower(env, ea);

    /* PCXI.PCPN = ICR.CCPN; */
    pcxi_set_pcpn(env, icr_get_ccpn(env));

    /* PCXI.PIE = ICR.IE; */
    pcxi_set_pie(env, icr_get_ie(env));

    /* PCXI.UL = 0; */
    pcxi_set_ul(env, 0);

    /* PCXI[19: 0] = FCX[19: 0]; */
    env->PCXI = (env->PCXI & 0xfff00000) + (env->FCX & 0xfffff);
    /* FCX[19: 0] = new_FCX[19: 0]; */
    env->FCX = (env->FCX & 0xfff00000) + (new_FCX & 0xfffff);

    /* if (tmp_FCX == LCX) trap(FCD);*/
    if (tmp_FCX == env->LCX) {
        /* FCD trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_FCD, GETPC());
    }
}

void helper_svucx(CPUTriCoreState *env)
{
    uint32_t tmp_FCX;
    uint32_t ea;
    uint32_t new_FCX;

    if (env->FCX == 0) {
        /* FCU trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_FCU, GETPC());
    }
    /* tmp_FCX = FCX; */
    tmp_FCX = env->FCX;
    /* EA = {FCX.FCXS, 6'b0, FCX.FCXO, 6'b0}; */
    ea = ((env->FCX & MASK_FCX_FCXS) << 12) +
         ((env->FCX & MASK_FCX_FCXO) << 6);
    /* new_FCX = M(EA, word); */
    new_FCX = cpu_ldl_le_data(env, ea);
    /* M(EA, 16 * word) = {PCXI, PSW, A[10], A[11], D[8], D[9], D[10], D[11],
                           A[12], A[13], A[14], A[15], D[12], D[13], D[14],
                           D[15]}; */
    save_context_upper(env, ea);

    /* PCXI.PCPN = ICR.CCPN; */
    pcxi_set_pcpn(env, icr_get_ccpn(env));

    /* PCXI.PIE = ICR.IE; */
    pcxi_set_pie(env, icr_get_ie(env));

    /* PCXI.UL = 1; */
    pcxi_set_ul(env, 1);

    /* PCXI[19: 0] = FCX[19: 0]; */
    env->PCXI = (env->PCXI & 0xfff00000) + (env->FCX & 0xfffff);
    /* FCX[19: 0] = new_FCX[19: 0]; */
    env->FCX = (env->FCX & 0xfff00000) + (new_FCX & 0xfffff);

    /* if (tmp_FCX == LCX) trap(FCD);*/
    if (tmp_FCX == env->LCX) {
        /* FCD trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_FCD, GETPC());
    }
}

void helper_rslcx(CPUTriCoreState *env)
{
    uint32_t ea;
    uint32_t new_PCXI;
    /*   if (PCXI[19: 0] == 0) then trap(CSU); */
    if ((env->PCXI & 0xfffff) == 0) {
        /* CSU trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_CSU, GETPC());
    }
    /* if (PCXI.UL == 1) then trap(CTYP); */
    if (pcxi_get_ul(env) == 1) {
        /* CTYP trap */
        raise_exception_sync_helper(env, TRAPC_CTX_MNG, TIN3_CTYP, GETPC());
    }
    /* EA = {PCXI.PCXS, 6'b0, PCXI.PCXO, 6'b0}; */
    /* EA = {PCXI.PCXS, 6'b0, PCXI.PCXO, 6'b0}; */
    ea = (pcxi_get_pcxs(env) << 28) |
         (pcxi_get_pcxo(env) << 6);

    /* {new_PCXI, A[11], A[10], A[11], D[8], D[9], D[10], D[11], A[12],
        A[13], A[14], A[15], D[12], D[13], D[14], D[15]} = M(EA, 16 * word); */
    restore_context_lower(env, ea, &env->gpr_a[11], &new_PCXI);
    /* M(EA, word) = FCX; */
    cpu_stl_le_data(env, ea, env->FCX);
    /* M(EA, word) = FCX; */
    cpu_stl_le_data(env, ea, env->FCX);
    /* FCX[19: 0] = PCXI[19: 0]; */
    env->FCX = (env->FCX & 0xfff00000) + (env->PCXI & 0x000fffff);
    /* PCXI = new_PCXI; */
    env->PCXI = new_PCXI;
}

void helper_psw_write(CPUTriCoreState *env, uint32_t arg)
{
    psw_write(env, arg);
}

uint32_t helper_psw_read(CPUTriCoreState *env)
{
    return psw_read(env);
}
