/*
 * TC1797 Fast ADC (FADC) — native result-register model (see tc1797_fadc.h).
 */
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "tc1797_fadc.h"

#define FADC_CLC     0x00u
#define FADC_ID      0x08u
#define FADC_RES     0x100u              /* per-channel result block base */
#define FADC_VF      (1u << 31)
#define FADC_RESMASK 0xFFFu              /* 12-bit result (matches firmware scaling) */
#define FADC_MODID   0x00B1C001u         /* module-ID shape (UM value best-effort) */

void tc1797_fadc_init(Tc1797Fadc *f, uint32_t base)
{
    memset(f, 0, sizeof(*f));
    f->base = base;
    f->default_count = -1;
    for (int i = 0; i < TC1797_FADC_NCH; i++) {
        f->input[i] = -1;
    }
}

static uint32_t fadc_res_word(Tc1797Fadc *f, int ch)
{
    int32_t c = f->input[ch] >= 0 ? f->input[ch] : f->default_count;
    if (c < 0) {
        return 0;                        /* no input -> silicon reset value 0 */
    }
    return FADC_VF | ((uint32_t)c & FADC_RESMASK);
}

uint32_t tc1797_fadc_read(Tc1797Fadc *f, uint32_t addr)
{
    uint32_t off = (addr - f->base) & ~0x3u;
    if (off >= FADC_RES && off < FADC_RES + TC1797_FADC_NCH * 4) {
        return fadc_res_word(f, (off - FADC_RES) >> 2);
    }
    if (off == FADC_ID) {
        return FADC_MODID;
    }
    if (off < TC1797_FADC_SIZE) {
        return f->regs[off >> 2];         /* CLC + coherent config read-back */
    }
    return 0;
}

void tc1797_fadc_write(Tc1797Fadc *f, uint32_t addr, uint32_t val)
{
    uint32_t off = (addr - f->base) & ~0x3u;
    if (off >= TC1797_FADC_SIZE) {
        return;
    }
    if (off == FADC_CLC) {
        /* CLC: DISS (bit1) mirrors DISR (bit0) so module enable/disable resolves. */
        f->regs[FADC_CLC >> 2] = (val & ~0x2u) | ((val & 1u) << 1);
        return;
    }
    if (off == FADC_ID) {
        return;                          /* module ID is read-only */
    }
    f->regs[off >> 2] = val;
}

void tc1797_fadc_set_input(Tc1797Fadc *f, int ch, int count)
{
    if (ch >= 0 && ch < TC1797_FADC_NCH) {
        f->input[ch] = count < 0 ? -1 : (count & FADC_RESMASK);
    }
}

void tc1797_fadc_set_default(Tc1797Fadc *f, int count)
{
    f->default_count = count < 0 ? -1 : (count & FADC_RESMASK);
}

/* ───────────────────────── opt-in self-test ─────────────────────────────── */
void tc1797_fadc_selftest(void)
{
    Tc1797Fadc f;
    tc1797_fadc_init(&f, TC1797_FADC_BASE);
    unsigned pass = 0, fail = 0;
#define CHK(c) do { if (c) pass++; else { fail++; \
        error_report("FADC selftest FAIL @%d", __LINE__); } } while (0)
    uint32_t base = TC1797_FADC_BASE;

    /* silicon reset: un-set channel reads 0 (no boot perturbation) */
    CHK(tc1797_fadc_read(&f, base + FADC_RES) == 0);

    /* persistent channel input: VF set + value, survives repeated reads */
    tc1797_fadc_set_input(&f, 2, 0x3AB);
    uint32_t r1 = tc1797_fadc_read(&f, base + FADC_RES + 2 * 4);
    uint32_t r2 = tc1797_fadc_read(&f, base + FADC_RES + 2 * 4);
    CHK((r1 & FADC_VF) && (r1 & FADC_RESMASK) == 0x3AB && r2 == r1);

    /* idle baseline applies to an un-set channel */
    tc1797_fadc_set_default(&f, 0x100);
    uint32_t d = tc1797_fadc_read(&f, base + FADC_RES + 5 * 4);
    CHK((d & FADC_VF) && (d & FADC_RESMASK) == 0x100);

    /* CLC DISR->DISS mirror (module enable/disable) */
    tc1797_fadc_write(&f, base + FADC_CLC, 1);          /* DISR=1 -> disabled */
    CHK((tc1797_fadc_read(&f, base + FADC_CLC) & 0x3) == 0x3);
    tc1797_fadc_write(&f, base + FADC_CLC, 0);          /* DISR=0 -> enabled  */
    CHK((tc1797_fadc_read(&f, base + FADC_CLC) & 0x3) == 0x0);

    /* coherent config read-back */
    tc1797_fadc_write(&f, base + 0x20, 0xCAFEF00D);
    CHK(tc1797_fadc_read(&f, base + 0x20) == 0xCAFEF00D);

    /* module ID is read-only */
    tc1797_fadc_write(&f, base + FADC_ID, 0xDEAD);
    CHK(tc1797_fadc_read(&f, base + FADC_ID) == FADC_MODID);

    info_report("tc1797 FADC selftest: %u passed, %u failed", pass, fail);
#undef CHK
}
