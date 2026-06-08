/*
 * TC1797 ADC kernels — native result-register model (see tc1797_adc.h).
 * Faithful port of bridge/tc1797_adc.py.
 */
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "tc1797_adc.h"

#define ADC_STATUS   0x038u               /* GLOBSTR */
#define ADC_GLOBCTR  0x030u               /* GLOBCTR (ANON request, [9:8]) */
#define RESR_BASE    0x180u
#define RESRD_BASE   0x1C0u
#define RESULT_MASK  0x3FFFu
#define STATUS_READY 0x00000300u         /* kernel running, no pending conv */

void tc1797_adc_init(Tc1797Adc *a, uint32_t base0)
{
    memset(a, 0, sizeof(*a));
    a->base0 = base0;
    for (int ki = 0; ki < TC1797_ADC_NKERN; ki++) {
        a->k[ki].default_count = -1;
        a->k[ki].shadow[ADC_GLOBCTR >> 2] = 0x000000FFu;  /* GLOBCTR power-on reset (DAVE TC1797.REGS) */
        for (int n = 0; n < TC1797_ADC_NRES; n++) {
            a->k[ki].input[n] = -1;
        }
    }
}

static int adc_decode(uint32_t addr, uint32_t *off)
{
    for (int i = 0; i < TC1797_ADC_NKERN; i++) {
        uint32_t base = TC1797_ADC0_BASE + i * TC1797_ADC_KSIZE;
        if (addr >= base && addr < base + TC1797_ADC_KSIZE) {
            *off = addr - base;
            return i;
        }
    }
    return -1;
}

bool tc1797_adc_is_addr(uint32_t addr)
{
    uint32_t off;
    return adc_decode(addr, &off) >= 0;
}

static uint32_t resr_word(AdcKernel *k, int n)
{
    if (k->input[n] >= 0) {              /* continuous input: always valid */
        return (1u << 31) | ((k->chnr[n] & 0xF) << 24)
               | ((uint32_t)k->input[n] & RESULT_MASK);
    }
    if (k->vf[n] == 0 && k->default_count >= 0) {
        return (1u << 31) | ((k->chnr[n] & 0xF) << 24)
               | ((uint32_t)k->default_count & RESULT_MASK);
    }
    return ((uint32_t)k->vf[n] << 31) | ((k->chnr[n] & 0xF) << 24)
           | (k->result[n] & RESULT_MASK);
}

uint32_t tc1797_adc_read(Tc1797Adc *a, uint32_t addr)
{
    addr = TC1797_ADC0_BASE + (addr - a->base0);   /* relocate to IP-relative space */
    uint32_t off;
    int ki = adc_decode(addr, &off);
    if (ki < 0) {
        return 0;
    }
    AdcKernel *k = &a->k[ki];
    off &= ~0x3u;

    if (off >= RESR_BASE && off < RESR_BASE + TC1797_ADC_NRES * 4) {
        int n = (off - RESR_BASE) >> 2;
        uint32_t w = resr_word(k, n);
        if (k->input[n] < 0 && !(k->vf[n] == 0 && k->default_count >= 0)) {
            k->vf[n] = 0;                /* one-shot: reading RESRn clears VF */
        }
        return w;
    }
    if (off >= RESRD_BASE && off < RESRD_BASE + TC1797_ADC_NRES * 4) {
        return resr_word(k, (off - RESRD_BASE) >> 2);   /* debug: VF preserved */
    }
    if (off == ADC_STATUS) {
        /*
         * GLOBSTR.ANON[9:8] (analog-part status) follows the ANON request the
         * firmware wrote into GLOBCTR[9:8]: 0 at reset (analog off), 0b11 once
         * the bring-up enables the kernel (it writes 0x8712 to GLOBCTR -> ANON=3,
         * then polls GLOBSTR for [9:8]==3). Faithful per DAVE TC1797.REGS
         * (GLOBSTR reset=0, GLOBCTR reset=0xFF, ANON=[9:8]); models an
         * instantaneous analog settle (no startup-time counter). Other status
         * bits (CHNR[6:3], SAMPLE/BUSY[13:11]) read 0 -- no conversion pending.
         */
        return k->shadow[ADC_GLOBCTR >> 2] & 0x00000300u;
    }
    return k->shadow[off >> 2];
}

void tc1797_adc_write(Tc1797Adc *a, uint32_t addr, uint32_t val)
{
    addr = TC1797_ADC0_BASE + (addr - a->base0);
    uint32_t off;
    int ki = adc_decode(addr, &off);
    if (ki >= 0) {
        a->k[ki].shadow[(off & ~0x3u) >> 2] = val;
    }
}

void tc1797_adc_set_input(Tc1797Adc *a, int kernel, int n, int count, int chnr)
{
    if (kernel < 0 || kernel >= TC1797_ADC_NKERN
        || n < 0 || n >= TC1797_ADC_NRES) {
        return;
    }
    a->k[kernel].input[n] = count < 0 ? -1 : (count & RESULT_MASK);
    if (chnr >= 0) {
        a->k[kernel].chnr[n] = chnr & 0xF;
    }
}

void tc1797_adc_set_default(Tc1797Adc *a, int count)
{
    for (int ki = 0; ki < TC1797_ADC_NKERN; ki++) {
        a->k[ki].default_count = count < 0 ? -1 : (count & RESULT_MASK);
    }
}

/* ───────────────────────── opt-in self-test ─────────────────────────────── */
void tc1797_adc_selftest(void)
{
    Tc1797Adc a;
    tc1797_adc_init(&a, TC1797_ADC0_BASE);
    unsigned pass = 0, fail = 0;
#define CHK(c) do { if (c) pass++; else { fail++; \
        error_report("ADC selftest FAIL @%d", __LINE__); } } while (0)
    uint32_t base = TC1797_ADC0_BASE;

    /* GLOBSTR.ANON status follows the GLOBCTR ANON request (DAVE TC1797.REGS):
     * analog off (0) at reset, 0x300 after the kernel-enable write sets ANON=3. */
    CHK(tc1797_adc_read(&a, base + ADC_STATUS) == 0);
    tc1797_adc_write(&a, base + ADC_GLOBCTR, 0x00008712u);
    CHK(tc1797_adc_read(&a, base + ADC_STATUS) == STATUS_READY);

    /* persistent input on RESR5: valid + value, survives repeated reads */
    tc1797_adc_set_input(&a, 0, 5, 0xABC, 5);
    uint32_t r1 = tc1797_adc_read(&a, base + RESR_BASE + 5 * 4);
    uint32_t r2 = tc1797_adc_read(&a, base + RESR_BASE + 5 * 4);
    CHK((r1 & (1u << 31)) && (r1 & RESULT_MASK) == 0xABC
        && ((r1 >> 24) & 0xF) == 5 && r2 == r1);

    /* one-shot result on RESR6: valid once, VF clears on read */
    a.k[0].result[6] = 0x123; a.k[0].vf[6] = 1;
    uint32_t s1 = tc1797_adc_read(&a, base + RESR_BASE + 6 * 4);
    uint32_t s2 = tc1797_adc_read(&a, base + RESR_BASE + 6 * 4);
    CHK((s1 & (1u << 31)) && (s1 & RESULT_MASK) == 0x123 && !(s2 & (1u << 31)));

    /* idle baseline: an un-set channel reads valid at the default count */
    tc1797_adc_set_default(&a, 0x800);
    uint32_t d = tc1797_adc_read(&a, base + RESR_BASE + 9 * 4);
    CHK((d & (1u << 31)) && (d & RESULT_MASK) == 0x800);

    /* RESRD debug read does not clear VF */
    a.k[1].result[2] = 0x55; a.k[1].vf[2] = 1;
    uint32_t b = TC1797_ADC0_BASE + 1 * TC1797_ADC_KSIZE;
    uint32_t e1 = tc1797_adc_read(&a, b + RESRD_BASE + 2 * 4);
    uint32_t e2 = tc1797_adc_read(&a, b + RESRD_BASE + 2 * 4);
    CHK((e1 & (1u << 31)) && (e2 & (1u << 31)) && e1 == e2);

    info_report("tc1797 ADC selftest: %u passed, %u failed", pass, fail);
#undef CHK
}
