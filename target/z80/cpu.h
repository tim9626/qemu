/*
 * Z80 virtual CPU header
 *
 *  Copyright (c) 2007 Stuart Brady
 *  Copyright (c) 2003 Fabrice Bellard
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef CPU_Z80_H
#define CPU_Z80_H

#include "qemu-common.h"
#include "qemu/bswap.h"
#include "cpu-qom.h"


#define TARGET_LONG_BITS 32

/* target supports implicit self modifying code */
#define TARGET_HAS_SMC
/* support for self modifying code even if the modified instruction is
   close to the modifying instruction */
#define TARGET_HAS_PRECISE_SMC

#define TARGET_HAS_ICE 1

#define ELF_MACHINE	EM_NONE

#define CPUArchState struct CPUZ80State

#include "exec/cpu-defs.h"

#define TARGET_PAGE_BITS 16

#define TARGET_PHYS_ADDR_SPACE_BITS 16
#ifdef CONFIG_USER_ONLY
# define TARGET_VIRT_ADDR_SPACE_BITS 16
#else
# define TARGET_VIRT_ADDR_SPACE_BITS 16
#endif

#define SR_MD 30
#define SR_RB 29
#define SR_BL 28
#define SR_FD 15
#define SR_M  9
#define SR_Q  8
#define SR_I3 7
#define SR_I2 6
#define SR_I1 5
#define SR_I0 4
#define SR_S  1
#define SR_T  0

/* Z80 registers */

#define R_A     0
#define R_F     1

#define R_BC    2
#define R_DE    3
#define R_HL    4
#define R_IX    5
#define R_IY    6
#define R_SP    7

#define R_I     8
#define R_R     9

#define R_AFX   10
#define R_BCX   11
#define R_DEX   12
#define R_HLX   13

/* flags masks */
#define CC_C   	0x0001
#define CC_N    0x0002
#define CC_P 	0x0004
#define CC_X 	0x0008
#define CC_H	0x0010
#define CC_Y 	0x0020
#define CC_Z	0x0040
#define CC_S    0x0080

/* hidden flags - used internally by qemu to represent additionnal cpu
   states. Only the CPL, INHIBIT_IRQ and HALTED are not redundant. We avoid
   using the IOPL_MASK, TF_MASK and VM_MASK bit position to ease oring
   with eflags. */
/* current cpl */
#define HF_CPL_SHIFT         0
/* true if soft mmu is being used */
#define HF_SOFTMMU_SHIFT     2
/* true if hardware interrupts must be disabled for next instruction */
#define HF_INHIBIT_IRQ_SHIFT 3
/* 16 or 32 segments */
#define HF_CS32_SHIFT        4
#define HF_SS32_SHIFT        5
/* zero base for DS, ES and SS : can be '0' only in 32 bit CS segment */
#define HF_ADDSEG_SHIFT      6
/* copy of CR0.PE (protected mode) */
#define HF_PE_SHIFT          7
#define HF_TF_SHIFT          8 /* must be same as eflags */
#define HF_MP_SHIFT          9 /* the order must be MP, EM, TS */
#define HF_EM_SHIFT         10
#define HF_TS_SHIFT         11
#define HF_IOPL_SHIFT       12 /* must be same as eflags */
#define HF_LMA_SHIFT        14 /* only used on x86_64: long mode active */
#define HF_CS64_SHIFT       15 /* only used on x86_64: 64 bit code segment  */
#define HF_OSFXSR_SHIFT     16 /* CR4.OSFXSR */
#define HF_VM_SHIFT         17 /* must be same as eflags */
#define HF_HALTED_SHIFT     18 /* CPU halted */
#define HF_SMM_SHIFT        19 /* CPU in SMM mode */

#define HF_CPL_MASK          (3 << HF_CPL_SHIFT)
#define HF_SOFTMMU_MASK      (1 << HF_SOFTMMU_SHIFT)
#define HF_INHIBIT_IRQ_MASK  (1 << HF_INHIBIT_IRQ_SHIFT)
#define HF_CS32_MASK         (1 << HF_CS32_SHIFT)
#define HF_SS32_MASK         (1 << HF_SS32_SHIFT)
#define HF_ADDSEG_MASK       (1 << HF_ADDSEG_SHIFT)
#define HF_PE_MASK           (1 << HF_PE_SHIFT)
#define HF_TF_MASK           (1 << HF_TF_SHIFT)
#define HF_MP_MASK           (1 << HF_MP_SHIFT)
#define HF_EM_MASK           (1 << HF_EM_SHIFT)
#define HF_TS_MASK           (1 << HF_TS_SHIFT)
#define HF_LMA_MASK          (1 << HF_LMA_SHIFT)
#define HF_CS64_MASK         (1 << HF_CS64_SHIFT)
#define HF_OSFXSR_MASK       (1 << HF_OSFXSR_SHIFT)
#define HF_HALTED_MASK       (1 << HF_HALTED_SHIFT)
#define HF_SMM_MASK          (1 << HF_SMM_SHIFT)

#define EXCP00_DIVZ	0
#define EXCP01_SSTP	1
#define EXCP02_NMI	2
#define EXCP03_INT3	3
#define EXCP04_INTO	4
#define EXCP05_BOUND	5
#define EXCP06_ILLOP	6
#define EXCP07_PREX	7
#define EXCP08_DBLE	8
#define EXCP09_XERR	9
#define EXCP0A_TSS	10
#define EXCP0B_NOSEG	11
#define EXCP0C_STACK	12
#define EXCP0D_GPF	13
#define EXCP0E_PAGE	14
#define EXCP10_COPR	16
#define EXCP11_ALGN	17
#define EXCP12_MCHK	18

#define DELAY_SLOT_MASK        0x7
#define DELAY_SLOT             (1 << 0)
#define DELAY_SLOT_CONDITIONAL (1 << 1)
#define DELAY_SLOT_RTE         (1 << 2)

#define TB_FLAG_PENDING_MOVCA  (1 << 3)

enum {
    CC_OP_DYNAMIC, /* must use dynamic code to get cc_op */
    CC_OP_EFLAGS,  /* all cc are explicitely computed, CC_SRC = flags */

    CC_OP_NB,
};

#define CPU_NB_REGS 14

#define NB_MMU_MODES 2

/* TLB size */
enum {
    TLB_SIZE = 128,
    TLB_MASK = TLB_SIZE - 1,
};

typedef struct Z80TLBEntry {
    uint32_t mr;
    uint32_t tr;
} Z80TLBEntry;

typedef struct CPUZ80TLBContext {
    Z80TLBEntry itlb[TLB_SIZE];
    Z80TLBEntry dtlb[TLB_SIZE];

    int (*cpu_z80_map_address_code)(struct Z80CPU *cpu,
                                         hwaddr *physical,
                                         int *prot,
                                         target_ulong address, int rw);
    int (*cpu_z80_map_address_data)(struct Z80CPU *cpu,
                                         hwaddr *physical,
                                         int *prot,
                                         target_ulong address, int rw);
} CPUZ80TLBContext;

struct z80_def_t {
    const char *name;
    target_ulong iu_version;
    uint32_t fpu_version;
    uint32_t mmu_version;
    uint32_t mmu_bm;
    uint32_t mmu_ctpr_mask;
    uint32_t mmu_cxr_mask;
    uint32_t mmu_sfsr_mask;
    uint32_t mmu_trcr_mask;
    uint32_t mxcc_version;
    uint32_t features;
    uint32_t nwindows;
    uint32_t maxtl;
};

#define FPSCR_MASK             (0x003fffff)
#define FPSCR_RM_MASK          (0x03 << 0)
#define FPSCR_RM_NEAREST       (0 << 0)
#define FPSCR_RM_ZERO          (1 << 0)
#define FPSCR_FR               (1 << 21)
#define FPSCR_SZ               (1 << 20)
#define FPSCR_PR               (1 << 19)
#define FPSCR_DN               (1 << 18)
#define FPSCR_CAUSE_MASK       (0x3f << 12)
#define FPSCR_CAUSE_SHIFT      (12)
#define FPSCR_CAUSE_E          (1 << 17)
#define FPSCR_CAUSE_V          (1 << 16)
#define FPSCR_CAUSE_Z          (1 << 15)
#define FPSCR_CAUSE_O          (1 << 14)
#define FPSCR_CAUSE_U          (1 << 13)
#define FPSCR_CAUSE_I          (1 << 12)
#define FPSCR_ENABLE_MASK      (0x1f << 7)
#define FPSCR_ENABLE_SHIFT     (7)
#define FPSCR_ENABLE_V         (1 << 11)
#define FPSCR_ENABLE_Z         (1 << 10)
#define FPSCR_ENABLE_O         (1 << 9)
#define FPSCR_ENABLE_U         (1 << 8)
#define FPSCR_ENABLE_I         (1 << 7)
#define FPSCR_FLAG_MASK        (0x1f << 2)
#define FPSCR_FLAG_SHIFT       (2)
#define FPSCR_FLAG_V           (1 << 6)
#define FPSCR_FLAG_Z           (1 << 5)
#define FPSCR_FLAG_O           (1 << 4)
#define FPSCR_FLAG_U           (1 << 3)
#define FPSCR_FLAG_I           (1 << 2)
#define DELAY_SLOT_MASK        0x7
#define GUSA_SHIFT             4
#ifdef CONFIG_USER_ONLY
#define GUSA_EXCLUSIVE         (1 << 12)
#define GUSA_MASK              ((0xff << GUSA_SHIFT) | GUSA_EXCLUSIVE)
#else
/* Provide dummy versions of the above to allow tests against tbflags
   to be elided while avoiding ifdefs.  */
#define GUSA_EXCLUSIVE         0
#define GUSA_MASK              0
#endif
#define TB_FLAG_ENVFLAGS_MASK  (DELAY_SLOT_MASK | GUSA_MASK)
#define UTLB_SIZE 64
#define ITLB_SIZE 4

typedef struct tlb_t {
    uint32_t vpn;		/* virtual page number */
    uint32_t ppn;		/* physical page number */
    uint32_t size;		/* mapped page size in bytes */
    uint8_t asid;		/* address space identifier */
    uint8_t v:1;		/* validity */
    uint8_t sz:2;		/* page size */
    uint8_t sh:1;		/* share status */
    uint8_t c:1;		/* cacheability */
    uint8_t pr:2;		/* protection key */
    uint8_t d:1;		/* dirty */
    uint8_t wt:1;		/* write through */
    uint8_t sa:3;		/* space attribute (PCMCIA) */
    uint8_t tc:1;		/* timing control */
} tlb_t;

typedef struct memory_content {
    uint32_t address;
    uint32_t value;
    struct memory_content *next;
} memory_content;

typedef struct CPUZ80State CPUZ80State;

struct CPUZ80State {
#if TARGET_LONG_BITS > HOST_LONG_BITS
    /* temporaries if we cannot store them in host registers */
    target_ulong t0, t1, t2;
#endif

    target_ulong shadow_gpr[16][32];

    /* Z80 registers */
    uint32_t pc;
    /* not sure if this is messy: */
    target_ulong regs[CPU_NB_REGS];

	uint32_t sr; /* status register (with T split out) */
	uint32_t sr_m; /* M bit of status register */
	uint32_t sr_q; /* Q bit of status register */
	uint32_t sr_t; /* T bit of status register */

    int iff1;
    int iff2;
    int imode;

    int ir;

    /* standard registers */
    target_ulong eflags; /* eflags register. During CPU emulation, CC
                        flags are set to zero because they are
                        stored elsewhere */

    /* emulator internal eflags handling */
    target_ulong cc_src;
    target_ulong cc_dst;
    uint32_t cc_op;
    uint32_t hflags; /* hidden flags, see HF_xxx constants */

    target_ulong cr[5]; /* NOTE: cr1 is unused */

    /* sysenter registers */
    uint64_t efer;
    uint64_t star;

    uint64_t pat;

    /* exception/interrupt handling */
    jmp_buf jmp_env;
    int exception_index;
    int error_code;
    int exception_is_int;
    target_ulong exception_next_pc;
    target_ulong dr[8]; /* debug registers */
    uint32_t smbase;
    int interrupt_request;
    int user_mode_only; /* user mode only simulation */
    int halted;
    struct {} end_reset_fields;
    uint32_t ssr;		/* saved status register */
	uint32_t spc; /* saved program counter */
	uint32_t gbr; /* global base register */
	uint32_t vbr; /* vector base register */
	uint32_t sgr; /* saved global register 15 */
	uint32_t dbr; /* debug base register */
	void *intc_handle;
    uint32_t expevt;		/* exception event register */
    uint32_t intevt;		/* interrupt event register */
    uint32_t mmucr;		/* MMU control register */
	uint32_t pteh; /* page table entry high register */
	uint32_t ptel; /* page table entry low register */
	uint32_t ptea; /* page table entry assistance register */
	uint32_t ttb; /* tranlation table base register */
	uint32_t tea; /* TLB exception address register */
	tlb_t utlb[UTLB_SIZE];
	tlb_t itlb[ITLB_SIZE];
	/* LDST = LOCK_ADDR != -1.  */
	uint32_t lock_addr;
	uint32_t lock_value;
	uint32_t pr;		/* procedure register */
	uint32_t fpul;		/* floating point communication register */
	uint32_t flags;		/* general execution flags */
	uint32_t delayed_pc;        /* target of delayed branch */
	uint32_t delayed_cond;      /* condition of delayed branch */
	/* The features that we should emulate. See sh_features above.  */
	uint32_t features;

    CPU_COMMON

	int in_sleep;
    uint32_t tra;
    uint32_t mach;		/* multiply and accumulate high */
    uint32_t macl;		/* multiply and accumulate low */
    memory_content *movcal_backup;
    memory_content **movcal_backup_tail;
	uint32_t fpscr; /* floating point status/control register */
	/* float point status register */
	float_status fp_status;
	uint32_t gregs[24];		/* general registers */
	float32 fregs[32];		/* floating point registers */

    /* in order to simplify APIC support, we leave this pointer to the
       user */
    struct APICState *apic_state;
};

struct Z80CPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    CPUZ80State env;
};

static inline struct Z80CPU *z80_env_get_cpu(const CPUZ80State *env) {
    return container_of(env, Z80CPU, env);
}

Z80CPU* cpu_z80_init(const char* cpu_model);

#define ENV_GET_CPU(e) CPU(z80_env_get_cpu(e))

#define ENV_OFFSET offsetof(Z80CPU, env)

void z80_cpu_do_interrupt(CPUState *cpu);
bool z80_cpu_exec_interrupt(CPUState *cpu, int int_req);
void z80_cpu_dump_state(CPUState *cpu, FILE *f,
                           fprintf_function cpu_fprintf, int flags);
hwaddr z80_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
int z80_cpu_gdb_read_register(CPUState *cpu, uint8_t *buf, int reg);
int z80_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);
void z80_cpu_do_unaligned_access(CPUState *cpu, vaddr addr,
                                    MMUAccessType access_type,
                                    int mmu_idx, uintptr_t retaddr);

void z80_translate_init(void);
int cpu_z80_signal_handler(int host_signum, void *pinfo,
                           void *puc);
int z80_cpu_handle_mmu_fault(CPUState *cpu, vaddr address, int size, int rw,
                                int mmu_idx);

void z80_cpu_list(FILE *f, fprintf_function cpu_fprintf);

#if !defined(CONFIG_USER_ONLY)
void cpu_z80_invalidate_tlb(CPUZ80State *s);
uint32_t cpu_z80_read_mmaped_itlb_addr(CPUZ80State *s,
                                       hwaddr addr);
void cpu_z80_write_mmaped_itlb_addr(CPUZ80State *s, hwaddr addr,
                                    uint32_t mem_value);
uint32_t cpu_z80_read_mmaped_itlb_data(CPUZ80State *s,
                                       hwaddr addr);
void cpu_z80_write_mmaped_itlb_data(CPUZ80State *s, hwaddr addr,
                                    uint32_t mem_value);
uint32_t cpu_z80_read_mmaped_utlb_addr(CPUZ80State *s,
                                       hwaddr addr);
void cpu_z80_write_mmaped_utlb_addr(CPUZ80State *s, hwaddr addr,
                                    uint32_t mem_value);
uint32_t cpu_z80_read_mmaped_utlb_data(CPUZ80State *s,
                                       hwaddr addr);
void cpu_z80_write_mmaped_utlb_data(CPUZ80State *s, hwaddr addr,
                                    uint32_t mem_value);
#endif

int cpu_z80_is_cached(CPUZ80State * env, target_ulong addr);

void cpu_load_tlb(CPUZ80State * env);

#define Z80_CPU_TYPE_SUFFIX "-" TYPE_Z80_CPU
#define Z80_CPU_TYPE_NAME(model) model Z80_CPU_TYPE_SUFFIX
#define CPU_RESOLVING_TYPE TYPE_Z80_CPU

#define cpu_signal_handler cpu_z80_signal_handler
#define cpu_list z80_cpu_list

/* MMU modes definitions */
#define MMU_MODE0_SUFFIX _kernel
#define MMU_MODE1_SUFFIX _user
#define MMU_USER_IDX 1
static inline int cpu_mmu_index (CPUZ80State *env, bool ifetch)
{
	/* return (env->hflags & HF_CPL_MASK) == 3 ? 1 : 0; */
	return 0;
}

#include "exec/cpu-all.h"

/* Memory access type */
enum {
    /* Privilege */
    ACCESS_PRIV = 0x01,
    /* Direction */
    ACCESS_WRITE = 0x02,
    /* Type of instruction */
    ACCESS_CODE = 0x10,
    ACCESS_INT = 0x20
};

/* MMU control register */
#define MMUCR    0x1F000010
#define MMUCR_AT (1<<0)
#define MMUCR_TI (1<<2)
#define MMUCR_SV (1<<8)
#define MMUCR_URC_BITS (6)
#define MMUCR_URC_OFFSET (10)
#define MMUCR_URC_SIZE (1 << MMUCR_URC_BITS)
#define MMUCR_URC_MASK (((MMUCR_URC_SIZE) - 1) << MMUCR_URC_OFFSET)
static inline int cpu_mmucr_urc (uint32_t mmucr)
{
    return ((mmucr & MMUCR_URC_MASK) >> MMUCR_URC_OFFSET);
}

/* PTEH : Page Translation Entry High register */
#define PTEH_ASID_BITS (8)
#define PTEH_ASID_SIZE (1 << PTEH_ASID_BITS)
#define PTEH_ASID_MASK (PTEH_ASID_SIZE - 1)
#define cpu_pteh_asid(pteh) ((pteh) & PTEH_ASID_MASK)
#define PTEH_VPN_BITS (22)
#define PTEH_VPN_OFFSET (10)
#define PTEH_VPN_SIZE (1 << PTEH_VPN_BITS)
#define PTEH_VPN_MASK (((PTEH_VPN_SIZE) - 1) << PTEH_VPN_OFFSET)
static inline int cpu_pteh_vpn (uint32_t pteh)
{
    return ((pteh & PTEH_VPN_MASK) >> PTEH_VPN_OFFSET);
}

/* PTEL : Page Translation Entry Low register */
#define PTEL_V        (1 << 8)
#define cpu_ptel_v(ptel) (((ptel) & PTEL_V) >> 8)
#define PTEL_C        (1 << 3)
#define cpu_ptel_c(ptel) (((ptel) & PTEL_C) >> 3)
#define PTEL_D        (1 << 2)
#define cpu_ptel_d(ptel) (((ptel) & PTEL_D) >> 2)
#define PTEL_SH       (1 << 1)
#define cpu_ptel_sh(ptel)(((ptel) & PTEL_SH) >> 1)
#define PTEL_WT       (1 << 0)
#define cpu_ptel_wt(ptel) ((ptel) & PTEL_WT)

#define PTEL_SZ_HIGH_OFFSET  (7)
#define PTEL_SZ_HIGH  (1 << PTEL_SZ_HIGH_OFFSET)
#define PTEL_SZ_LOW_OFFSET   (4)
#define PTEL_SZ_LOW   (1 << PTEL_SZ_LOW_OFFSET)
static inline int cpu_ptel_sz (uint32_t ptel)
{
    int sz;
    sz = (ptel & PTEL_SZ_HIGH) >> PTEL_SZ_HIGH_OFFSET;
    sz <<= 1;
    sz |= (ptel & PTEL_SZ_LOW) >> PTEL_SZ_LOW_OFFSET;
    return sz;
}

#define PTEL_PPN_BITS (19)
#define PTEL_PPN_OFFSET (10)
#define PTEL_PPN_SIZE (1 << PTEL_PPN_BITS)
#define PTEL_PPN_MASK (((PTEL_PPN_SIZE) - 1) << PTEL_PPN_OFFSET)
static inline int cpu_ptel_ppn (uint32_t ptel)
{
    return ((ptel & PTEL_PPN_MASK) >> PTEL_PPN_OFFSET);
}

#define PTEL_PR_BITS   (2)
#define PTEL_PR_OFFSET (5)
#define PTEL_PR_SIZE (1 << PTEL_PR_BITS)
#define PTEL_PR_MASK (((PTEL_PR_SIZE) - 1) << PTEL_PR_OFFSET)
static inline int cpu_ptel_pr (uint32_t ptel)
{
    return ((ptel & PTEL_PR_MASK) >> PTEL_PR_OFFSET);
}

/* PTEA : Page Translation Entry Assistance register */
#define PTEA_SA_BITS (3)
#define PTEA_SA_SIZE (1 << PTEA_SA_BITS)
#define PTEA_SA_MASK (PTEA_SA_SIZE - 1)
#define cpu_ptea_sa(ptea) ((ptea) & PTEA_SA_MASK)
#define PTEA_TC        (1 << 3)
#define cpu_ptea_tc(ptea) (((ptea) & PTEA_TC) >> 3)

static inline target_ulong cpu_read_sr(CPUZ80State *env)
{
    return env->sr | (env->sr_m << SR_M) |
                     (env->sr_q << SR_Q) |
                     (env->sr_t << SR_T);
}

static inline void cpu_write_sr(CPUZ80State *env, target_ulong sr)
{
    env->sr_m = (sr >> SR_M) & 1;
    env->sr_q = (sr >> SR_Q) & 1;
    env->sr_t = (sr >> SR_T) & 1;
    env->sr = sr & ~((1u << SR_M) | (1u << SR_Q) | (1u << SR_T));
}

static inline void cpu_get_tb_cpu_state(CPUZ80State *env, target_ulong *pc,
                                        target_ulong *cs_base, uint32_t *flags)
{
	*pc = env->pc;
	*cs_base = 0;
	*flags = env->hflags;
}

#endif /* CPU_Z80_H */
