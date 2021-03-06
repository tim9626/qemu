/*
 * QEMU AVR CPU
 *
 * Copyright (c) 2016 Michael Rolnik
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
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

#if !defined(CPU_Z80_H)
#define CPU_Z80_H

#include "qemu-common.h"

#define TARGET_LONG_BITS 32
#define CPU_RESOLVING_TYPE TYPE_Z80_CPU

#define CPUArchState struct CPUZ80State

#include "exec/cpu-defs.h"
#include "fpu/softfloat.h"

/*
 *  TARGET_PAGE_BITS cannot be more than 8 bits because
 *  1.  all IO registers occupy [0x0000 .. 0x00ff] address range, and they
 *      should be implemented as a device and not memory
 *  2.  SRAM starts at the address 0x0100
 */
#define TARGET_PAGE_BITS 8
#define TARGET_PHYS_ADDR_SPACE_BITS 24
#define TARGET_VIRT_ADDR_SPACE_BITS 24
#define NB_MMU_MODES 2

#define HF_INHIBIT_IRQ_SHIFT 3
#define HF_RF_SHIFT         16 /* must be same as eflags */
#define HF_SVMI_SHIFT       21 /* SVM intercepts are active */

#define HF_INHIBIT_IRQ_MASK  (1 << HF_INHIBIT_IRQ_SHIFT)
#define HF_RF_MASK           (1 << HF_RF_SHIFT)
#define HF_SVMI_MASK         (1 << HF_SVMI_SHIFT)

#define TF_MASK                 0x00000100
#define RF_MASK                 0x00010000

#define EXCP01_DB	1
#define EXCP08_DBLE	8
#define EXCP0E_PAGE	14

#define DR6_BS          (1 << 14)

#define DR7_TYPE_SHIFT  16
#define DR7_MAX_BP           4
#define DR7_TYPE_BP_INST     0x0
#define DR7_TYPE_DATA_WR     0x1
#define DR7_TYPE_IO_RW       0x2
#define DR7_TYPE_DATA_RW     0x3

/*
 *  AVR has two memory spaces, data & code.
 *  e.g. both have 0 address
 *  ST/LD instructions access data space
 *  LPM/SPM and instruction fetching access code memory space
 */
#define MMU_CODE_IDX 0
#define MMU_DATA_IDX 1

#define EXCP_RESET 1
#define EXCP_INT(n) (EXCP_RESET + (n) + 1)

#define PHYS_ADDR_MASK 0xfff00000

#define PHYS_BASE_CODE 0x00000000
#define PHYS_BASE_DATA 0x00800000
#define PHYS_BASE_REGS 0x10000000

#define VIRT_BASE_CODE 0x00000000
#define VIRT_BASE_DATA 0x00000000
#define VIRT_BASE_REGS 0x00000000

/*
 *  there are two groups of registers
 *  1. CPU regs     - accessible by LD/ST and CPU itself
 *  2. CPU IO regs  - accessible by LD/ST and IN/OUT
 */
#define Z80_CPU_REGS 0x0020
#define Z80_CPU_IO_REGS 0x0040
#define Z80_REGS (Z80_CPU_IO_REGS + Z80_CPU_REGS)

#define Z80_CPU_REGS_BASE 0x0000
#define Z80_CPU_IO_REGS_BASE (Z80_CPU_REGS_BASE + Z80_CPU_REGS)

#define Z80_CPU_REGS_LAST (Z80_CPU_REGS_BASE + Z80_CPU_REGS - 1)
#define Z80_CPU_IO_REGS_LAST (Z80_CPU_IO_REGS_BASE + Z80_CPU_IO_REGS - 1)

#define SVM_EXIT_SHUTDOWN	0x07f
#define SVM_EXIT_EXCP_BASE      0x040
#define SVM_EXIT_SWINT		0x075

typedef enum {
    CC_OP_DYNAMIC, /* must use dynamic code to get cc_op */
    CC_OP_EFLAGS,  /* all cc are explicitly computed, CC_SRC = flags */

    CC_OP_MULB, /* modify all flags, C, O = (CC_SRC != 0) */
    CC_OP_MULW,
    CC_OP_MULL,
    CC_OP_MULQ,

    CC_OP_ADDB, /* modify all flags, CC_DST = res, CC_SRC = src1 */
    CC_OP_ADDW,
    CC_OP_ADDL,
    CC_OP_ADDQ,

    CC_OP_ADCB, /* modify all flags, CC_DST = res, CC_SRC = src1 */
    CC_OP_ADCW,
    CC_OP_ADCL,
    CC_OP_ADCQ,

    CC_OP_SUBB, /* modify all flags, CC_DST = res, CC_SRC = src1 */
    CC_OP_SUBW,
    CC_OP_SUBL,
    CC_OP_SUBQ,

    CC_OP_SBBB, /* modify all flags, CC_DST = res, CC_SRC = src1 */
    CC_OP_SBBW,
    CC_OP_SBBL,
    CC_OP_SBBQ,

    CC_OP_LOGICB, /* modify all flags, CC_DST = res */
    CC_OP_LOGICW,
    CC_OP_LOGICL,
    CC_OP_LOGICQ,

    CC_OP_INCB, /* modify all flags except, CC_DST = res, CC_SRC = C */
    CC_OP_INCW,
    CC_OP_INCL,
    CC_OP_INCQ,

    CC_OP_DECB, /* modify all flags except, CC_DST = res, CC_SRC = C  */
    CC_OP_DECW,
    CC_OP_DECL,
    CC_OP_DECQ,

    CC_OP_SHLB, /* modify all flags, CC_DST = res, CC_SRC.msb = C */
    CC_OP_SHLW,
    CC_OP_SHLL,
    CC_OP_SHLQ,

    CC_OP_SARB, /* modify all flags, CC_DST = res, CC_SRC.lsb = C */
    CC_OP_SARW,
    CC_OP_SARL,
    CC_OP_SARQ,

    CC_OP_BMILGB, /* Z,S via CC_DST, C = SRC==0; O=0; P,A undefined */
    CC_OP_BMILGW,
    CC_OP_BMILGL,
    CC_OP_BMILGQ,

    CC_OP_ADCX, /* CC_DST = C, CC_SRC = rest.  */
    CC_OP_ADOX, /* CC_DST = O, CC_SRC = rest.  */
    CC_OP_ADCOX, /* CC_DST = C, CC_SRC2 = O, CC_SRC = rest.  */

    CC_OP_CLR, /* Z set, all other flags clear.  */
    CC_OP_POPCNT, /* Z via CC_SRC, all other flags clear.  */

    CC_OP_NB,
} CCOp;

enum z80_features {
    Z80_FEATURE_SRAM,

    Z80_FEATURE_1_BYTE_PC,
    Z80_FEATURE_2_BYTE_PC,
    Z80_FEATURE_3_BYTE_PC,

    Z80_FEATURE_1_BYTE_SP,
    Z80_FEATURE_2_BYTE_SP,

    Z80_FEATURE_BREAK,
    Z80_FEATURE_DES,
    Z80_FEATURE_RMW, /* Read Modify Write - XCH LAC LAS LAT */

    Z80_FEATURE_EIJMP_EICALL,
    Z80_FEATURE_IJMP_ICALL,
    Z80_FEATURE_JMP_CALL,

    Z80_FEATURE_ADIW_SBIW,

    Z80_FEATURE_SPM,
    Z80_FEATURE_SPMX,

    Z80_FEATURE_ELPMX,
    Z80_FEATURE_ELPM,
    Z80_FEATURE_LPMX,
    Z80_FEATURE_LPM,

    Z80_FEATURE_MOVW,
    Z80_FEATURE_MUL,
    Z80_FEATURE_RAMPD,
    Z80_FEATURE_RAMPX,
    Z80_FEATURE_RAMPY,
    Z80_FEATURE_RAMPZ,
};

typedef struct CPUZ80State CPUZ80State;

struct CPUZ80State {
    uint32_t pc_w; /* 0x003fffff up to 22 bits */

    target_ulong eflags; /* eflags register. During CPU emulation, CC
                            flags and DF are set to zero because they are
                            stored elsewhere */
    uint32_t hflags; /* TB flags, see HF_xxx constants. These flags
                           are known at translation time. */


    /* exception/interrupt handling */
    int error_code;
    int exception_is_int;
    target_ulong exception_next_eip;
    target_ulong dr[8]; /* debug registers; note dr4 and dr5 are unused */
	union {
		struct CPUBreakpoint *cpu_breakpoint[4];
		struct CPUWatchpoint *cpu_watchpoint[4];
	}; /* break/watchpoints for dr[0..3] */

	int old_exception;  /* exception in flight */

    target_ulong eip;

    uint32_t sregC; /* 0x00000001 1 bits */
    uint32_t sregZ; /* 0x0000ffff 16 bits, negative logic */
    uint32_t sregN; /* 0x00000001 1 bits */
    uint32_t sregV; /* 0x00000001 1 bits */
    uint32_t sregS; /* 0x00000001 1 bits */
    uint32_t sregH; /* 0x00000001 1 bits */
    uint32_t sregT; /* 0x00000001 1 bits */
    uint32_t sregI; /* 0x00000001 1 bits */

    uint32_t rampD; /* 0x00ff0000 8 bits */
    uint32_t rampX; /* 0x00ff0000 8 bits */
    uint32_t rampY; /* 0x00ff0000 8 bits */
    uint32_t rampZ; /* 0x00ff0000 8 bits */
    uint32_t eind; /* 0x00ff0000 8 bits */

    uint32_t r[Z80_CPU_REGS];
                     /* 8 bits each */
    uint32_t sp; /* 16 bits */

    uint64_t intsrc; /* interrupt sources */
    bool fullacc;/* CPU/MEM if true MEM only otherwise */

    uint32_t features;

    /* Those resources are used only in QEMU core */
    CPU_COMMON
};

static inline int z80_feature(CPUZ80State *env, int feature)
{
    return (env->features & (1U << feature)) != 0;
}

static inline void z80_set_feature(CPUZ80State *env, int feature)
{
    env->features |= (1U << feature);
}

void cpu_svm_check_intercept_param(CPUZ80State *env1, uint32_t type,
                                   uint64_t param, uintptr_t retaddr);

void cpu_vmexit(CPUZ80State *nenv, uint32_t exit_code, uint64_t exit_info_1,
                uintptr_t retaddr);

void helper_single_step(CPUZ80State *env);
void helper_rechecking_single_step(CPUZ80State *env);
void helper_reset_rf(CPUZ80State *env);

void QEMU_NORETURN raise_exception(CPUZ80State *env, int exception_index);

#define Z80_CPU_TYPE_SUFFIX "-" TYPE_Z80_CPU
#define Z80_CPU_TYPE_NAME(model) model Z80_CPU_TYPE_SUFFIX
#define CPU_RESOLVING_TYPE TYPE_Z80_CPU

#define cpu_list z80_cpu_list
#define cpu_signal_handler cpu_z80_signal_handler

#include "exec/cpu-all.h"
#include "cpu-qom.h"

static inline int cpu_mmu_index(CPUZ80State *env, bool ifetch)
{
    return ifetch ? MMU_CODE_IDX : MMU_DATA_IDX;
}

void z80_translate_init(void);



void z80_cpu_list(FILE *f, fprintf_function cpu_fprintf);
int cpu_z80_exec(CPUState *cpu);
int cpu_z80_signal_handler(int host_signum, void *pinfo, void *puc);
int z80_cpu_handle_mmu_fault(CPUState *cpu, vaddr address, int rw,
                                int mmu_idx);
int z80_cpu_memory_rw_debug(CPUState *cs, vaddr address, uint8_t *buf,
                                int len, bool is_write);

enum {
    TB_FLAGS_FULL_ACCESS = 1,
};

static inline void cpu_get_tb_cpu_state(CPUZ80State *env, target_ulong *pc,
                                target_ulong *cs_base, uint32_t *pflags)
{
    uint32_t flags = 0;

    *pc = env->pc_w * 2;
    *cs_base = 0;

    if (env->fullacc) {
        flags |= TB_FLAGS_FULL_ACCESS;
    }

    *pflags = flags;
}

static inline int cpu_interrupts_enabled(CPUZ80State *env)
{
    return env->sregI != 0;
}

static inline uint8_t cpu_get_sreg(CPUZ80State *env)
{
    uint8_t sreg;
    sreg = (env->sregC & 0x01) << 0
         | (env->sregZ == 0 ? 1 : 0) << 1
         | (env->sregN) << 2
         | (env->sregV) << 3
         | (env->sregS) << 4
         | (env->sregH) << 5
         | (env->sregT) << 6
         | (env->sregI) << 7;
    return sreg;
}

static inline void cpu_set_sreg(CPUZ80State *env, uint8_t sreg)
{
    env->sregC = (sreg >> 0) & 0x01;
    env->sregZ = (sreg >> 1) & 0x01 ? 0 : 1;
    env->sregN = (sreg >> 2) & 0x01;
    env->sregV = (sreg >> 3) & 0x01;
    env->sregS = (sreg >> 4) & 0x01;
    env->sregH = (sreg >> 5) & 0x01;
    env->sregT = (sreg >> 6) & 0x01;
    env->sregI = (sreg >> 7) & 0x01;
}

#include "exec/exec-all.h"

#endif /* !defined (CPU_Z80_H) */
