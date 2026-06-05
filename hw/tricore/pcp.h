/*
 * Infineon PCP2 (Peripheral Control Processor, version 2) — native model.
 *
 * The TC1797 carries a PCP2: a second, independent processor with its OWN
 * instruction set (NOT TriCore) that runs short "channel programs" to service
 * peripherals without disturbing the TriCore CPU. This is a from-scratch
 * interpreter + a channel/IRQ engine that runs on its OWN host thread, sharing
 * the guest address space (PRAM/CMEM/SFRs) with the TriCore vCPU.
 *
 * Ported from the validated bridge reference (bridge/pcp.py + pcp_engine.py).
 * Ground truth: TC1797 UM v1.1 ch.10 + Ghidra tricore.pcp.sinc.
 */
#ifndef HW_TRICORE_PCP_H
#define HW_TRICORE_PCP_H

#include "qemu/thread.h"

/* TC1797 PCP memory map (UM Table 10-19 / segment-15 map). */
#define PCP_PRAM_BASE  0xF0050000u   /* channel contexts + data, 16 KB */
#define PCP_PRAM_SIZE  0x4000u
#define PCP_CMEM_BASE  0xF0060000u   /* channel programs, 32 KB           */
#define PCP_CMEM_SIZE  0x8000u

/* EXIT.INT -> raise a TriCore service request at this SRPN (host callback,
 * invoked by the PCP thread while holding the BQL). */
typedef void (*PcpIrqFn)(void *opaque, uint8_t srpn);

/* A PCP2 execution core: 8 GPRs + a separate half-word PC. */
typedef struct PcpCore {
    uint32_t R[8];          /* R0..R7 */
    uint16_t pc;            /* program counter, in 16-bit instruction units */
    uint8_t  cur_srpn;
    bool     exited;        /* set by EXIT; unwinds run_channel */
    uint8_t  ex_st, ex_int, ex_ep, ex_ec;   /* EXIT operands */
    uint64_t insn_count;
    bool     trace;
} PcpCore;

/* The engine: owns a core and runs triggered channel programs on a worker
 * thread, serialising shared-memory access against the vCPU via the BQL. */
typedef struct PcpEngine {
    PcpCore   core;
    QemuThread thread;
    QemuMutex mutex;        /* protects queue + stats + running */
    QemuCond  cond;
    uint8_t   queue[256];
    unsigned  qhead, qtail, qcount;
    bool      running;
    bool      started;

    /* configuration */
    uint32_t  csa_base;     /* PRAM base of the per-SRPN context save areas */
    bool      rcb;          /* PCP_CS.RCB: 1 => entry table (PC=2*SRPN)     */
    int       context_model;/* PCP_CS.CS: 0=Full 1=Small 2=Minimum         */
    uint32_t  max_insns;    /* runaway guard per channel                    */
    uint64_t  f_pcp_hz;     /* PCP own clock (fPCP); set from the SoC clock tree.
                             * Carried here so the channel engine can pace its
                             * execution to PCP-cycles under a deterministic clock
                             * (currently informational; pacing is the icount step). */
    PcpIrqFn  irq_fn;
    void     *irq_opaque;

    /* statistics (mutex-protected) */
    uint64_t  st_channels, st_insns, st_exits, st_errors, st_irqs;
} PcpEngine;

void pcp_engine_init(PcpEngine *e, uint32_t csa_base, int context_model,
                     bool rcb, PcpIrqFn irq_fn, void *opaque);
void pcp_engine_start(PcpEngine *e);
void pcp_engine_stop(PcpEngine *e);
/* CPU -> PCP: request channel `srpn` (1..255). Async; returns immediately. */
void pcp_engine_trigger(PcpEngine *e, uint8_t srpn);

/* Run one channel inline on the caller thread (unit tests / single-thread).
 * Returns the instruction count; *out_exit_int reports EXIT.INT if non-NULL.
 * The caller must hold the BQL if the channel can touch MMIO. */
unsigned pcp_core_run_channel(PcpCore *c, uint8_t srpn, uint32_t csa_base,
                              bool rcb, int model, uint32_t max_insns,
                              bool *out_exit_int);

/* Opt-in self-test (TC1797_PCPTEST=1): runs a hand-assembled channel through
 * real guest PRAM/CMEM and checks the decode/execute/context/EXIT path. */
void pcp_selftest(void);

/* Opt-in OWN-THREAD self-test (TC1797_PCPTHREADTEST=1): proves the full
 * trigger -> queue -> worker thread -> run -> writeback -> IRQ chain on the
 * real engine worker thread, using an isolated engine instance so it never
 * touches live firmware PCP state. */
void pcp_thread_selftest(void);

#endif /* HW_TRICORE_PCP_H */
