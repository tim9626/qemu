/*
 * OpenRISC Machine
 *
 * Copyright (c) 2011-2012 Jia Liu <proljc@gmail.com>
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
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "migration/cpu.h"

/*static const VMStateDescription vmstate_tlb_entry = {
    .name = "tlb_entry",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINTTL(mr, Z80TLBEntry),
        VMSTATE_UINTTL(tr, Z80TLBEntry),
        VMSTATE_END_OF_LIST()
    }
};*/

/*static const VMStateDescription vmstate_cpu_tlb = {
    .name = "cpu_tlb",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(itlb, CPUZ80TLBContext, TLB_SIZE, 0,
                             vmstate_tlb_entry, Z80TLBEntry),
        VMSTATE_STRUCT_ARRAY(dtlb, CPUZ80TLBContext, TLB_SIZE, 0,
                             vmstate_tlb_entry, Z80TLBEntry),
        VMSTATE_END_OF_LIST()
    }
};*/

static int get_sr(QEMUFile *f, void *opaque, size_t size, VMStateField *field)
{
    CPUZ80State *env = opaque;
    cpu_write_sr(env, qemu_get_be32(f));
    return 0;
}

static int put_sr(QEMUFile *f, void *opaque, size_t size,
                  VMStateField *field, QJSON *vmdesc)
{
    CPUZ80State *env = opaque;
    qemu_put_be32(f, cpu_read_sr(env));
    return 0;
}

static const VMStateInfo vmstate_sr = {
    .name = "sr",
    .get = get_sr,
    .put = put_sr,
};

static const VMStateDescription vmstate_env = {
    .name = "env",
    .version_id = 6,
    .minimum_version_id = 6,
    .fields = (VMStateField[]) {
        VMSTATE_UINTTL_2DARRAY(shadow_gpr, CPUZ80State, 16, 32),
        VMSTATE_UINTTL(pc, CPUZ80State),
        VMSTATE_UINTTL(lock_addr, CPUZ80State),
        VMSTATE_UINTTL(lock_value, CPUZ80State),
        /* Save the architecture value of the SR, not the internally
           expanded version.  Since this architecture value does not
           exist in memory to be stored, this requires a but of hoop
           jumping.  We want OFFSET=0 so that we effectively pass ENV
           to the helper functions, and we need to fill in the name by
           hand since there's no field of that name.  */
        {
            .name = "sr",
            .version_id = 0,
            .size = sizeof(uint32_t),
            .info = &vmstate_sr,
            .flags = VMS_SINGLE,
            .offset = 0
        },





        VMSTATE_END_OF_LIST()
    }
};

const VMStateDescription vmstate_z80_cpu = {
    .name = "cpu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_CPU(),
        VMSTATE_STRUCT(env, Z80CPU, 1, vmstate_env, CPUZ80State),
        VMSTATE_END_OF_LIST()
    }
};
