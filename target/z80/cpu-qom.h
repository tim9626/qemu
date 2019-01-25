#ifndef QEMU_SPARC_CPU_QOM_H
#define QEMU_SPARC_CPU_QOM_H

#include "qom/cpu.h"

#define TYPE_Z80_CPU "z80-cpu"

#define Z80_CPU_CLASS(klass) \
    OBJECT_CLASS_CHECK(Z80CPUClass, (klass), TYPE_Z80_CPU)
#define Z80_CPU(obj) \
    OBJECT_CHECK(Z80CPU, (obj), TYPE_Z80_CPU)
#define Z80_CPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(Z80CPUClass, (obj), TYPE_Z80_CPU)

typedef struct z80_def_t z80_def_t;

typedef struct Z80CPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/

    DeviceRealize parent_realize;
    void (*parent_reset)(CPUState *cpu);
    z80_def_t *cpu_def;
} Z80CPUClass;

typedef struct Z80CPU Z80CPU;

#endif
