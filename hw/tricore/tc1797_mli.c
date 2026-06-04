/*
 * TC1797 MLI0/MLI1 — Micro Link Interface model (see tc1797_mli.h).
 */
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "tc1797_mli.h"

#define MLI_CLC  0x00u
#define MLI_ID   0x08u

void tc1797_mli_init(Tc1797Mli *m, uint32_t base, uint32_t id)
{
    memset(m, 0, sizeof(*m));
    m->base = base;
    m->id = id;
}

uint32_t tc1797_mli_read(Tc1797Mli *m, uint32_t addr)
{
    uint32_t off = (addr - m->base) & (TC1797_MLI_SIZE - 1) & ~0x3u;
    if (off == MLI_ID) {
        return m->id;
    }
    return m->regs[off >> 2];             /* CLC + coherent config read-back */
}

void tc1797_mli_write(Tc1797Mli *m, uint32_t addr, uint32_t val)
{
    uint32_t off = (addr - m->base) & (TC1797_MLI_SIZE - 1) & ~0x3u;
    if (off == MLI_CLC) {
        /* CLC: DISS (bit1) mirrors DISR (bit0) so an enable/disable poll resolves. */
        m->regs[MLI_CLC >> 2] = (val & ~0x2u) | ((val & 1u) << 1);
        return;
    }
    if (off == MLI_ID) {
        return;                           /* module ID is read-only */
    }
    m->regs[off >> 2] = val;
}

/* ───────────────────────── opt-in self-test ─────────────────────────────── */
void tc1797_mli_selftest(void)
{
    Tc1797Mli m0, m1;
    tc1797_mli_init(&m0, TC1797_MLI0_BASE, 0x0024C001u);
    tc1797_mli_init(&m1, TC1797_MLI1_BASE, 0x0025C001u);
    unsigned pass = 0, fail = 0;
#define CHK(c) do { if (c) pass++; else { fail++; \
        error_report("MLI selftest FAIL @%d", __LINE__); } } while (0)

    /* module IDs (distinct per instance) */
    CHK(tc1797_mli_read(&m0, TC1797_MLI0_BASE + MLI_ID) == 0x0024C001u);
    CHK(tc1797_mli_read(&m1, TC1797_MLI1_BASE + MLI_ID) == 0x0025C001u);

    /* CLC DISR->DISS mirror */
    tc1797_mli_write(&m0, TC1797_MLI0_BASE + MLI_CLC, 1);     /* DISR=1 -> disabled */
    CHK((tc1797_mli_read(&m0, TC1797_MLI0_BASE + MLI_CLC) & 0x3) == 0x3);
    tc1797_mli_write(&m0, TC1797_MLI0_BASE + MLI_CLC, 0);     /* DISR=0 -> enabled  */
    CHK((tc1797_mli_read(&m0, TC1797_MLI0_BASE + MLI_CLC) & 0x3) == 0x0);

    /* coherent config read-back, isolated per instance */
    tc1797_mli_write(&m0, TC1797_MLI0_BASE + 0x20, 0x12345678u);
    tc1797_mli_write(&m1, TC1797_MLI1_BASE + 0x20, 0xA5A5A5A5u);
    CHK(tc1797_mli_read(&m0, TC1797_MLI0_BASE + 0x20) == 0x12345678u);
    CHK(tc1797_mli_read(&m1, TC1797_MLI1_BASE + 0x20) == 0xA5A5A5A5u);

    /* module ID is read-only */
    tc1797_mli_write(&m0, TC1797_MLI0_BASE + MLI_ID, 0xDEADu);
    CHK(tc1797_mli_read(&m0, TC1797_MLI0_BASE + MLI_ID) == 0x0024C001u);

    info_report("tc1797 MLI selftest: %u passed, %u failed", pass, fail);
#undef CHK
}
