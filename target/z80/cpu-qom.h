/*
 * QEMU Z80 CPU
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

#ifndef QEMU_Z80_CPU_QOM_H
#define QEMU_Z80_CPU_QOM_H

#include "qom/cpu.h"

#define TYPE_Z80_CPU "avr"

#define Z80_CPU_CLASS(klass) \
                    OBJECT_CLASS_CHECK(Z80CPUClass, (klass), TYPE_Z80_CPU)
#define Z80_CPU(obj) \
                    OBJECT_CHECK(Z80CPU, (obj), TYPE_Z80_CPU)
#define Z80_CPU_GET_CLASS(obj) \
                    OBJECT_GET_CLASS(Z80CPUClass, (obj), TYPE_Z80_CPU)

/**
 *  Z80CPUClass:
 *  @parent_realize: The parent class' realize handler.
 *  @parent_reset: The parent class' reset handler.
 *  @vr: Version Register value.
 *
 *  A Z80 CPU model.
 */
typedef struct Z80CPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    void (*parent_reset)(CPUState *cpu);
} Z80CPUClass;

/**
 *  Z80CPU:
 *  @env: #CPUZ80State
 *
 *  A Z80 CPU.
 */
typedef struct Z80CPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    CPUZ80State env;
} Z80CPU;

static inline Z80CPU *z80_env_get_cpu(CPUZ80State *env)
{
    return container_of(env, Z80CPU, env);
}

#define ENV_GET_CPU(e) CPU(z80_env_get_cpu(e))
#define ENV_OFFSET offsetof(Z80CPU, env)

#ifndef CONFIG_USER_ONLY
extern const struct VMStateDescription vms_z80_cpu;
#endif

void z80_cpu_do_interrupt(CPUState *cpu);
bool z80_cpu_exec_interrupt(CPUState *cpu, int int_req);
void z80_cpu_dump_state(CPUState *cs, FILE *f,
                            fprintf_function cpu_fprintf, int flags);
hwaddr z80_cpu_get_phys_page_debug(CPUState *cpu, vaddr addr);
int z80_cpu_gdb_read_register(CPUState *cpu, uint8_t *buf, int reg);
int z80_cpu_gdb_write_register(CPUState *cpu, uint8_t *buf, int reg);

#endif
