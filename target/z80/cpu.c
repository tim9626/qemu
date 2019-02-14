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

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "qemu-common.h"
#include "migration/vmstate.h"

static void z80_cpu_set_pc(CPUState *cs, vaddr value)
{
    Z80CPU *cpu = Z80_CPU(cs);

    cpu->env.pc_w = value / 2; /* internally PC points to words */
}

static bool z80_cpu_has_work(CPUState *cs)
{
    Z80CPU *cpu = Z80_CPU(cs);
    CPUZ80State *env = &cpu->env;

    return (cs->interrupt_request & (CPU_INTERRUPT_HARD | CPU_INTERRUPT_RESET))
            && cpu_interrupts_enabled(env);
}

static void z80_cpu_synchronize_from_tb(CPUState *cs, TranslationBlock *tb)
{
    Z80CPU *cpu = Z80_CPU(cs);
    CPUZ80State *env = &cpu->env;

    env->pc_w = tb->pc / 2; /* internally PC points to words */
}

static void z80_cpu_reset(CPUState *s)
{
    Z80CPU *cpu = Z80_CPU(s);
    Z80CPUClass *mcc = Z80_CPU_GET_CLASS(cpu);
    CPUZ80State *env = &cpu->env;

    mcc->parent_reset(s);

    env->pc_w = 0;
    env->sregI = 1;
    env->sregC = 0;
    env->sregZ = 0;
    env->sregN = 0;
    env->sregV = 0;
    env->sregS = 0;
    env->sregH = 0;
    env->sregT = 0;

    env->rampD = 0;
    env->rampX = 0;
    env->rampY = 0;
    env->rampZ = 0;
    env->eind = 0;
    env->sp = 0;

    memset(env->r, 0, sizeof(env->r));

    tlb_flush(s);
}

static void z80_cpu_disas_set_info(CPUState *cpu, disassemble_info *info)
{
    //info->mach = bfd_arch_z80;
    //info->print_insn = print_insn_z80;
}

static void z80_cpu_realizefn(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    Z80CPUClass *mcc = Z80_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }
    qemu_init_vcpu(cs);
    cpu_reset(cs);

    mcc->parent_realize(dev, errp);
}

static void z80_cpu_set_int(void *opaque, int irq, int level)
{
    Z80CPU *cpu = opaque;
    CPUZ80State *env = &cpu->env;
    CPUState *cs = CPU(cpu);

    uint64_t mask = (1ull << irq);
    if (level) {
        env->intsrc |= mask;
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        env->intsrc &= ~mask;
        if (env->intsrc == 0) {
            cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
        }
    }
}

static void z80_cpu_initfn(Object *obj)
{
    CPUState *cs = CPU(obj);
    Z80CPU *cpu = Z80_CPU(obj);

    cs->env_ptr = &cpu->env;

#ifndef CONFIG_USER_ONLY
    qdev_init_gpio_in(DEVICE(cpu), z80_cpu_set_int, 37);
#endif

    if (tcg_enabled()) {
        z80_translate_init();
    }
}

static ObjectClass *z80_cpu_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;
    char *typename;
    char **cpuname;

    if (!cpu_model) {
        return NULL;
    }

    cpuname = g_strsplit(cpu_model, ",", 1);
    typename = g_strdup_printf("%s-" TYPE_Z80_CPU, cpuname[0]);
    oc = object_class_by_name(typename);

    g_strfreev(cpuname);
    g_free(typename);

    if (!oc
        || !object_class_dynamic_cast(oc, TYPE_Z80_CPU)
        || object_class_is_abstract(oc)) {
        return NULL;
    }

    return oc;
}

static void z80_cpu_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    Z80CPUClass *mcc = Z80_CPU_CLASS(oc);

    mcc->parent_realize = dc->realize;
    dc->realize = z80_cpu_realizefn;

    mcc->parent_reset = cc->reset;
    cc->reset = z80_cpu_reset;

    cc->class_by_name = z80_cpu_class_by_name;

    cc->has_work = z80_cpu_has_work;
    cc->do_interrupt = z80_cpu_do_interrupt;
    cc->cpu_exec_interrupt = z80_cpu_exec_interrupt;
    cc->dump_state = z80_cpu_dump_state;
    cc->set_pc = z80_cpu_set_pc;
#if !defined(CONFIG_USER_ONLY)
    cc->memory_rw_debug = z80_cpu_memory_rw_debug;
#endif
#ifdef CONFIG_USER_ONLY
    cc->handle_mmu_fault = z80_cpu_handle_mmu_fault;
#else
    cc->get_phys_page_debug = z80_cpu_get_phys_page_debug;
    cc->vmsd = &vms_z80_cpu;
#endif
    cc->disas_set_info = z80_cpu_disas_set_info;
    cc->synchronize_from_tb = z80_cpu_synchronize_from_tb;
    cc->gdb_read_register = z80_cpu_gdb_read_register;
    cc->gdb_write_register = z80_cpu_gdb_write_register;
    cc->gdb_num_core_regs = 35;
    cc->tcg_initialize = z80_translate_init;
}

static void z80_avr1_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
}

static void z80_avr2_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
}

static void z80_avr25_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
}

static void z80_avr3_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
}

static void z80_avr31_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_RAMPZ);
    z80_set_feature(env, Z80_FEATURE_ELPM);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
}

static void z80_avr35_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
}

static void z80_avr4_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
    z80_set_feature(env, Z80_FEATURE_MUL);
}

static void z80_avr5_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
    z80_set_feature(env, Z80_FEATURE_MUL);
}

static void z80_avr51_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_RAMPZ);
    z80_set_feature(env, Z80_FEATURE_ELPMX);
    z80_set_feature(env, Z80_FEATURE_ELPM);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
    z80_set_feature(env, Z80_FEATURE_MUL);
}

static void z80_avr6_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_3_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_RAMPZ);
    z80_set_feature(env, Z80_FEATURE_EIJMP_EICALL);
    z80_set_feature(env, Z80_FEATURE_ELPMX);
    z80_set_feature(env, Z80_FEATURE_ELPM);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
    z80_set_feature(env, Z80_FEATURE_MUL);
}

static void z80_xmega2_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
    z80_set_feature(env, Z80_FEATURE_MUL);
    z80_set_feature(env, Z80_FEATURE_RMW);
}

static void z80_xmega4_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_RAMPZ);
    z80_set_feature(env, Z80_FEATURE_ELPMX);
    z80_set_feature(env, Z80_FEATURE_ELPM);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
    z80_set_feature(env, Z80_FEATURE_MUL);
    z80_set_feature(env, Z80_FEATURE_RMW);
}

static void z80_xmega5_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_2_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_RAMPD);
    z80_set_feature(env, Z80_FEATURE_RAMPX);
    z80_set_feature(env, Z80_FEATURE_RAMPY);
    z80_set_feature(env, Z80_FEATURE_RAMPZ);
    z80_set_feature(env, Z80_FEATURE_ELPMX);
    z80_set_feature(env, Z80_FEATURE_ELPM);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
    z80_set_feature(env, Z80_FEATURE_MUL);
    z80_set_feature(env, Z80_FEATURE_RMW);
}

static void z80_xmega6_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_3_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_RAMPZ);
    z80_set_feature(env, Z80_FEATURE_EIJMP_EICALL);
    z80_set_feature(env, Z80_FEATURE_ELPMX);
    z80_set_feature(env, Z80_FEATURE_ELPM);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
    z80_set_feature(env, Z80_FEATURE_MUL);
    z80_set_feature(env, Z80_FEATURE_RMW);
}

static void z80_xmega7_initfn(Object *obj)
{
    Z80CPU *cpu = Z80_CPU(obj);
    CPUZ80State *env = &cpu->env;

    z80_set_feature(env, Z80_FEATURE_LPM);
    z80_set_feature(env, Z80_FEATURE_IJMP_ICALL);
    z80_set_feature(env, Z80_FEATURE_ADIW_SBIW);
    z80_set_feature(env, Z80_FEATURE_SRAM);
    z80_set_feature(env, Z80_FEATURE_BREAK);

    z80_set_feature(env, Z80_FEATURE_3_BYTE_PC);
    z80_set_feature(env, Z80_FEATURE_2_BYTE_SP);
    z80_set_feature(env, Z80_FEATURE_RAMPD);
    z80_set_feature(env, Z80_FEATURE_RAMPX);
    z80_set_feature(env, Z80_FEATURE_RAMPY);
    z80_set_feature(env, Z80_FEATURE_RAMPZ);
    z80_set_feature(env, Z80_FEATURE_EIJMP_EICALL);
    z80_set_feature(env, Z80_FEATURE_ELPMX);
    z80_set_feature(env, Z80_FEATURE_ELPM);
    z80_set_feature(env, Z80_FEATURE_JMP_CALL);
    z80_set_feature(env, Z80_FEATURE_LPMX);
    z80_set_feature(env, Z80_FEATURE_MOVW);
    z80_set_feature(env, Z80_FEATURE_MUL);
    z80_set_feature(env, Z80_FEATURE_RMW);
}

typedef struct Z80CPUInfo {
    const char *name;
    void (*initfn)(Object *obj);
} Z80CPUInfo;

static const Z80CPUInfo z80_cpus[] = {
    {.name = "avr1", .initfn = z80_avr1_initfn},
    {.name = "avr2", .initfn = z80_avr2_initfn},
    {.name = "avr25", .initfn = z80_avr25_initfn},
    {.name = "avr3", .initfn = z80_avr3_initfn},
    {.name = "avr31", .initfn = z80_avr31_initfn},
    {.name = "avr35", .initfn = z80_avr35_initfn},
    {.name = "avr4", .initfn = z80_avr4_initfn},
    {.name = "avr5", .initfn = z80_avr5_initfn},
    {.name = "avr51", .initfn = z80_avr51_initfn},
    {.name = "avr6", .initfn = z80_avr6_initfn},
    {.name = "xmega2", .initfn = z80_xmega2_initfn},
    {.name = "xmega4", .initfn = z80_xmega4_initfn},
    {.name = "xmega5", .initfn = z80_xmega5_initfn},
    {.name = "xmega6", .initfn = z80_xmega6_initfn},
    {.name = "xmega7", .initfn = z80_xmega7_initfn},
};

static gint z80_cpu_list_compare(gconstpointer a, gconstpointer b)
{
    ObjectClass *class_a = (ObjectClass *)a;
    ObjectClass *class_b = (ObjectClass *)b;
    const char *name_a;
    const char *name_b;

    name_a = object_class_get_name(class_a);
    name_b = object_class_get_name(class_b);

    return strcmp(name_a, name_b);
}

static void z80_cpu_list_entry(gpointer data, gpointer user_data)
{
    ObjectClass *oc = data;
    CPUListState *s = user_data;
    const char *typename;
    char *name;

    typename = object_class_get_name(oc);
    name = g_strndup(typename, strlen(typename) - strlen("-" TYPE_Z80_CPU));
    (*s->cpu_fprintf)(s->file, "  %s\n", name);
    g_free(name);
}

void z80_cpu_list(FILE *f, fprintf_function cpu_fprintf)
{
    CPUListState s = {
        .file = f,
        .cpu_fprintf = cpu_fprintf,
    };
    GSList *list;

    list = object_class_get_list(TYPE_Z80_CPU, false);
    list = g_slist_sort(list, z80_cpu_list_compare);
    (*cpu_fprintf)(f, "Available CPUs:\n");
    g_slist_foreach(list, z80_cpu_list_entry, &s);
    g_slist_free(list);
}

static void cpu_register(const Z80CPUInfo *info)
{
    TypeInfo type_info = {
        .parent = TYPE_Z80_CPU,
        .instance_size = sizeof(Z80CPU),
        .instance_init = info->initfn,
        .class_size = sizeof(Z80CPUClass),
    };

    type_info.name = g_strdup_printf("%s-" TYPE_Z80_CPU, info->name);
    type_register(&type_info);
    g_free((void *)type_info.name);
}

static const TypeInfo z80_cpu_type_info = {
    .name = TYPE_Z80_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(Z80CPU),
    .instance_init = z80_cpu_initfn,
    .class_size = sizeof(Z80CPUClass),
    .class_init = z80_cpu_class_init,
    .abstract = false,
};

static void z80_cpu_register_types(void)
{
    int i;
    type_register_static(&z80_cpu_type_info);

    for (i = 0; i < ARRAY_SIZE(z80_cpus); i++) {
        cpu_register(&z80_cpus[i]);
    }
}

type_init(z80_cpu_register_types)
