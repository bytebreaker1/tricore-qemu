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
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/core/registerfields.h"
#include "cpu.h"
#include "exec/cputlb.h"
#include "accel/tcg/cpu-mmu-index.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "fpu/softfloat-helpers.h"
#include "qemu/qemu-print.h"

enum {
    TLBRET_DIRTY = -4,
    TLBRET_INVALID = -3,
    TLBRET_NOMATCH = -2,
    TLBRET_BADADDR = -1,
    TLBRET_MATCH = 0
};

/*
 * TC1.3.1 range-based memory protection (MPU).
 *
 * Enforced only when SYSCON.PROTEN == 1 (TriCore Arch Vol1 sec 10.3.1); when
 * disabled, every access is permitted and we keep the flat 1:1 model that the
 * rest of the board relies on. The active protection register set is PSW.PRS
 * (0..3). Per the TC1797 (UM Fig 2-9) each set has 4 data ranges (bounds
 * DPRn_rL/U, mode byte r of DPMn) and 2 code ranges (CPRn_rL/U, mode byte r of
 * CPMn). An access is permitted if its address lies inside ANY range whose
 * matching enable bit is set -- permissions OR across overlapping ranges
 * (Vol1 sec 10.2.1). Outside all enabled ranges => deny => MPR/MPW/MPX trap.
 *
 * DPMn/CPMn per-range mode byte (range r at bits [8r+7:8r]): RE=bit0, WE=bit1
 * for data; XE=bit0 for code. Decision is byte-granular; the TLB entry is
 * page-granular (correct for page-aligned ranges, the normal case). The
 * context-save/restore protection bypass (Vol1 sec 10.3.3) is not modeled --
 * only relevant under PROTEN, which this firmware leaves disabled.
 */
#define TC_DPM_RE  0x1u   /* data range Read Enable  */
#define TC_DPM_WE  0x2u   /* data range Write Enable */
#define TC_CPM_XE  0x1u   /* code range eXecute Enable */

/* The DPR/CPR bound registers and the DPM/CPM mode registers are individual
 * uint32_t fields generated (in order) from csfr.h.inc, so each group is
 * contiguous and can be indexed as a flat array. Validate that at build time. */
QEMU_BUILD_BUG_ON(offsetof(CPUTriCoreState, DPR0_1L) -
                  offsetof(CPUTriCoreState, DPR0_0L) != 2 * sizeof(uint32_t));
QEMU_BUILD_BUG_ON(offsetof(CPUTriCoreState, DPR1_0L) -
                  offsetof(CPUTriCoreState, DPR0_0L) != 8 * sizeof(uint32_t));
QEMU_BUILD_BUG_ON(offsetof(CPUTriCoreState, DPM1) -
                  offsetof(CPUTriCoreState, DPM0) != sizeof(uint32_t));
QEMU_BUILD_BUG_ON(offsetof(CPUTriCoreState, CPR1_0L) -
                  offsetof(CPUTriCoreState, CPR0_0L) != 8 * sizeof(uint32_t));
QEMU_BUILD_BUG_ON(offsetof(CPUTriCoreState, CPM1) -
                  offsetof(CPUTriCoreState, CPM0) != sizeof(uint32_t));

static bool tc_data_access_ok(CPUTriCoreState *env, unsigned set,
                              uint32_t addr, bool is_write)
{
    const uint32_t *dpr = &env->DPR0_0L + set * 8; /* 4 ranges * {L,U} per set */
    uint32_t dpm = (&env->DPM0)[set];

    for (int r = 0; r < 4; r++) {
        uint8_t mode = (dpm >> (r * 8)) & 0xff;
        bool en = is_write ? (mode & TC_DPM_WE) : (mode & TC_DPM_RE);
        if (en && addr >= dpr[r * 2] && addr < dpr[r * 2 + 1]) {
            return true;
        }
    }
    return false;
}

static bool tc_code_access_ok(CPUTriCoreState *env, unsigned set, uint32_t addr)
{
    const uint32_t *cpr = &env->CPR0_0L + set * 8;
    uint32_t cpm = (&env->CPM0)[set];

    for (int r = 0; r < 4; r++) {  /* TC1797 uses 2; spare ranges read disabled */
        uint8_t mode = (cpm >> (r * 8)) & 0xff;
        if ((mode & TC_CPM_XE) && addr >= cpr[r * 2] && addr < cpr[r * 2 + 1]) {
            return true;
        }
    }
    return false;
}

static int get_physical_address(CPUTriCoreState *env, hwaddr *physical,
                                int *prot, vaddr address,
                                MMUAccessType access_type, int mmu_idx)
{
    *physical = address & 0xFFFFFFFF;

    /* Protection disabled => flat, all-permitted (unchanged fast path). */
    if (!(env->SYSCON & MASK_SYSCON_PRO_TEN)) {
        *prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        return TLBRET_MATCH;
    }

    /* Protection enabled: evaluate the current set (PSW.PRS). */
    unsigned set = (env->PSW & MASK_PSW_PRS) >> 12;
    if (access_type == MMU_INST_FETCH) {
        if (!tc_code_access_ok(env, set, address)) {
            return TLBRET_NOMATCH;          /* -> MPX */
        }
        *prot = PAGE_READ | PAGE_EXEC;
    } else {
        bool re = tc_data_access_ok(env, set, address, false);
        bool we = tc_data_access_ok(env, set, address, true);
        if (access_type == MMU_DATA_STORE) {
            if (!we) {
                return TLBRET_NOMATCH;      /* -> MPW */
            }
        } else if (!re) {
            return TLBRET_NOMATCH;          /* -> MPR */
        }
        *prot = (re ? PAGE_READ : 0) | (we ? PAGE_WRITE : 0);
    }
    return TLBRET_MATCH;
}

hwaddr tricore_cpu_get_phys_addr_debug(CPUState *cs, vaddr addr)
{
    TriCoreCPU *cpu = TRICORE_CPU(cs);
    hwaddr phys_addr;
    int prot;
    int mmu_idx = cpu_mmu_index(cs, false);

    if (get_physical_address(&cpu->env, &phys_addr, &prot, addr,
                             MMU_DATA_LOAD, mmu_idx)) {
        return -1;
    }
    return phys_addr;
}

bool tricore_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                          MMUAccessType access_type, int mmu_idx,
                          bool probe, uintptr_t retaddr)
{
    CPUTriCoreState *env = cpu_env(cs);
    hwaddr physical;
    int prot = 0;   /* left unset by get_physical_address on a denied access */
    int ret = get_physical_address(env, &physical, &prot,
                                   address, access_type, mmu_idx);

    qemu_log_mask(CPU_LOG_MMU, "%s address=0x%" VADDR_PRIx " ret %d physical "
                  HWADDR_FMT_plx " prot %d\n",
                  __func__, address, ret, physical, prot);

    if (ret == TLBRET_MATCH) {
        /*
         * prot already encodes the permitted access types (full RWX when
         * protection is disabled), so do NOT force PAGE_EXEC here -- that
         * would let a data page satisfy an instruction fetch without an MPX
         * re-check when SYSCON.PROTEN is set.
         *
         * Granularity: TC1.3.1 protection ranges are byte-granular, but a TLB
         * page caches one permission set for a whole 4KB page. When protection
         * is enforced we therefore register a sub-page (1-byte) entry: QEMU
         * marks it TLB_INVALID and re-runs this fill -- re-evaluating the
         * ranges -- on every access, so a range boundary that falls mid-page
         * is honoured exactly. With protection disabled we cache the full page
         * for normal speed (the common path; unchanged behaviour).
         */
        vaddr page_size = (env->SYSCON & MASK_SYSCON_PRO_TEN) ? 1
                                                              : TARGET_PAGE_SIZE;
        tlb_set_page(cs, address & TARGET_PAGE_MASK,
                     physical & TARGET_PAGE_MASK, prot,
                     mmu_idx, page_size);
        return true;
    }

    assert(ret < 0);
    if (probe) {
        return false;
    }
    /* Range-based protection violation: class-1 trap, TIN by access kind. */
    uint32_t tin = (access_type == MMU_INST_FETCH) ? TIN1_MPX :
                   (access_type == MMU_DATA_STORE) ? TIN1_MPW : TIN1_MPR;
    tricore_raise_protection_trap(env, tin, retaddr);
}

/*
 * Range-based memory-protection logic self-test (opt-in: TC1797_MPUTEST=1).
 * Exercises get_physical_address() with crafted protection state to validate
 * the PROTEN gate, range bounds, RE/WE/XE decoding and PSW.PRS set selection.
 * The trap-delivery path itself is the shared raise_exception_sync_internal
 * used by every TriCore sync trap. Has no effect on a normal run.
 */
void tricore_mpu_selftest(void)
{
    CPUTriCoreState env;
    hwaddr phys;
    int prot, r;
    unsigned pass = 0, fail = 0;

#define CHK(cond) do {                                                       \
        if (cond) { pass++; }                                                \
        else { fail++; error_report("MPU selftest FAIL @%d", __LINE__); }    \
    } while (0)

    memset(&env, 0, sizeof(env));

    /* (1) PROTEN disabled -> flat RWX, always matches. */
    r = get_physical_address(&env, &phys, &prot, 0x12345678, MMU_DATA_LOAD, 0);
    CHK(r == TLBRET_MATCH && (prot & PAGE_READ) && (prot & PAGE_WRITE)
        && (prot & PAGE_EXEC));

    /* Enable protection; set 0 data range [0xD0000000,0xD0001000) RE+WE. */
    env.SYSCON = MASK_SYSCON_PRO_TEN;
    env.PSW = 0;                       /* PRS = 0 */
    env.DPR0_0L = 0xD0000000; env.DPR0_0U = 0xD0001000;
    env.DPM0 = TC_DPM_RE | TC_DPM_WE;  /* range-0 mode byte */

    /* (2) in-range load+store match; out-of-range deny. */
    r = get_physical_address(&env, &phys, &prot, 0xD0000500, MMU_DATA_LOAD, 0);
    CHK(r == TLBRET_MATCH && (prot & PAGE_READ));
    r = get_physical_address(&env, &phys, &prot, 0xD0000500, MMU_DATA_STORE, 0);
    CHK(r == TLBRET_MATCH && (prot & PAGE_WRITE));
    r = get_physical_address(&env, &phys, &prot, 0xD0002000, MMU_DATA_LOAD, 0);
    CHK(r == TLBRET_NOMATCH);          /* -> MPR */
    r = get_physical_address(&env, &phys, &prot, 0xD0002000, MMU_DATA_STORE, 0);
    CHK(r == TLBRET_NOMATCH);          /* -> MPW */

    /* (3) RE-only: load ok, store denied. */
    env.DPM0 = TC_DPM_RE;
    r = get_physical_address(&env, &phys, &prot, 0xD0000500, MMU_DATA_LOAD, 0);
    CHK(r == TLBRET_MATCH);
    r = get_physical_address(&env, &phys, &prot, 0xD0000500, MMU_DATA_STORE, 0);
    CHK(r == TLBRET_NOMATCH);          /* -> MPW */

    /* (4) code range / instruction fetch. */
    env.DPM0 = TC_DPM_RE | TC_DPM_WE;
    env.CPR0_0L = 0x80000000; env.CPR0_0U = 0x80100000;
    env.CPM0 = TC_CPM_XE;
    r = get_physical_address(&env, &phys, &prot, 0x80000100, MMU_INST_FETCH, 0);
    CHK(r == TLBRET_MATCH && (prot & PAGE_EXEC));
    r = get_physical_address(&env, &phys, &prot, 0x80200000, MMU_INST_FETCH, 0);
    CHK(r == TLBRET_NOMATCH);          /* -> MPX */

    /* (5) PSW.PRS set selection: configure set 1, switch PRS=1. */
    env.DPR1_0L = 0xC0000000; env.DPR1_0U = 0xC0000800; env.DPM1 = TC_DPM_RE;
    env.PSW = (1u << 12);              /* PRS = 1 */
    r = get_physical_address(&env, &phys, &prot, 0xC0000400, MMU_DATA_LOAD, 0);
    CHK(r == TLBRET_MATCH);            /* set-1 range active */
    r = get_physical_address(&env, &phys, &prot, 0xD0000500, MMU_DATA_LOAD, 0);
    CHK(r == TLBRET_NOMATCH);          /* set-0 range no longer selected */

    info_report("tc1797 MPU selftest: %u passed, %u failed", pass, fail);
#undef CHK
}

void fpu_set_state(CPUTriCoreState *env)
{
    switch (extract32(env->PSW, 24, 2)) {
    case 0:
        set_float_rounding_mode(float_round_nearest_even, &env->fp_status);
        break;
    case 1:
        set_float_rounding_mode(float_round_up, &env->fp_status);
        break;
    case 2:
        set_float_rounding_mode(float_round_down, &env->fp_status);
        break;
    case 3:
        set_float_rounding_mode(float_round_to_zero, &env->fp_status);
        break;
    }

    set_flush_inputs_to_zero(1, &env->fp_status);
    set_flush_to_zero(1, &env->fp_status);
    set_float_detect_tininess(float_tininess_before_rounding, &env->fp_status);
    set_float_ftz_detection(float_ftz_before_rounding, &env->fp_status);
    set_default_nan_mode(1, &env->fp_status);
    /* Default NaN pattern: sign bit clear, frac msb set */
    set_float_default_nan_pattern(0b01000000, &env->fp_status);
}

uint32_t psw_read(CPUTriCoreState *env)
{
    /* clear all USB bits */
    env->PSW &= 0x7ffffff;
    /* now set them from the cache */
    env->PSW |= ((env->PSW_USB_C != 0) << 31);
    env->PSW |= ((env->PSW_USB_V   & (1 << 31))  >> 1);
    env->PSW |= ((env->PSW_USB_SV  & (1 << 31))  >> 2);
    env->PSW |= ((env->PSW_USB_AV  & (1 << 31))  >> 3);
    env->PSW |= ((env->PSW_USB_SAV & (1 << 31))  >> 4);

    return env->PSW;
}

void psw_write(CPUTriCoreState *env, uint32_t val)
{
    env->PSW_USB_C = (val & MASK_USB_C);
    env->PSW_USB_V = (val & MASK_USB_V) << 1;
    env->PSW_USB_SV = (val & MASK_USB_SV) << 2;
    env->PSW_USB_AV = (val & MASK_USB_AV) << 3;
    env->PSW_USB_SAV = (val & MASK_USB_SAV) << 4;
    env->PSW = val;

    fpu_set_state(env);
}

#define FIELD_GETTER_WITH_FEATURE(NAME, REG, FIELD, FEATURE)     \
uint32_t NAME(CPUTriCoreState *env)                             \
{                                                                \
    if (tricore_has_feature(env, TRICORE_FEATURE_##FEATURE)) {   \
        return FIELD_EX32(env->REG, REG, FIELD ## _ ## FEATURE); \
    }                                                            \
    return FIELD_EX32(env->REG, REG, FIELD ## _13);              \
}

#define FIELD_GETTER(NAME, REG, FIELD)       \
uint32_t NAME(CPUTriCoreState *env)         \
{                                            \
    return FIELD_EX32(env->REG, REG, FIELD); \
}

#define FIELD_SETTER_WITH_FEATURE(NAME, REG, FIELD, FEATURE)              \
void NAME(CPUTriCoreState *env, uint32_t val)                            \
{                                                                         \
    if (tricore_has_feature(env, TRICORE_FEATURE_##FEATURE)) {            \
        env->REG = FIELD_DP32(env->REG, REG, FIELD ## _ ## FEATURE, val); \
    }                                                                     \
    env->REG = FIELD_DP32(env->REG, REG, FIELD ## _13, val);              \
}

#define FIELD_SETTER(NAME, REG, FIELD)                \
void NAME(CPUTriCoreState *env, uint32_t val)        \
{                                                     \
    env->REG = FIELD_DP32(env->REG, REG, FIELD, val); \
}

FIELD_GETTER_WITH_FEATURE(pcxi_get_pcpn, PCXI, PCPN, 161)
FIELD_SETTER_WITH_FEATURE(pcxi_set_pcpn, PCXI, PCPN, 161)
FIELD_GETTER_WITH_FEATURE(pcxi_get_pie, PCXI, PIE, 161)
FIELD_SETTER_WITH_FEATURE(pcxi_set_pie, PCXI, PIE, 161)
FIELD_GETTER_WITH_FEATURE(pcxi_get_ul, PCXI, UL, 161)
FIELD_SETTER_WITH_FEATURE(pcxi_set_ul, PCXI, UL, 161)
FIELD_GETTER(pcxi_get_pcxs, PCXI, PCXS)
FIELD_GETTER(pcxi_get_pcxo, PCXI, PCXO)

FIELD_GETTER_WITH_FEATURE(icr_get_ie, ICR, IE, 161)
FIELD_SETTER_WITH_FEATURE(icr_set_ie, ICR, IE, 161)
FIELD_GETTER(icr_get_ccpn, ICR, CCPN)
FIELD_SETTER(icr_set_ccpn, ICR, CCPN)
