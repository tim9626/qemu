/*
 * QEMU ZX Spectrum Emulator
 *
 * Copyright (c) 2007-2009 Stuart Brady <stuart.brady@gmail.com>
 * Copyright (c) 2007 Ulrich Hecht
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "target/z80/cpu.h"
#include "hw/hw.h"
#include "zx_defs.h"
#include "exec/cpu-defs.h"
#include "exec/cpu-all.h"
#include "exec/exec-all.h"
#include "qemu/timer.h"
//#include "console.h"
//#include "isa.h"
#include "sysemu/sysemu.h"
#include "zx_video.h"
#include "zx_keyboard.h"
#include "hw/boards.h"


#ifdef CONFIG_LIBSPECTRUM
#include <libspectrum.h>
#endif

#define ROM_FILENAME_48  "zx-rom.bin"
#define ROM_FILENAME_128 "zx-rom128.bin"

static int page_tab[4];

// #define IOPIPE_ENABLED
#ifdef IOPIPE_ENABLED
/* LLL -- IO port to pipe hack */
#define IOPIPE_READ_PATH_DFL  "/tmp/qemu_z80_rd"
#define IOPIPE_WRITE_PATH_DFL "/tmp/qemu_z80_wr"
typedef struct IOPipe {
    int rdfd;
    int wrfd;
    char *rd_path;
    char *wr_path;
} IOPipe;
static IOPipe iopipe = {0, 0, IOPIPE_READ_PATH_DFL, IOPIPE_WRITE_PATH_DFL};
static uint8_t iopipe_read(IOPipe *iop);
static int iopipe_write(IOPipe *iop, uint8_t data);
#endif

/* arrays with IO ports values. 
  
   io_in_ports will be loaded with the contents of 'io_in_file' which
   specifies the port and value to be read anytime a IN instruction is executed.

   io_out_ports will be written out to 'io_out_file' anytime an OUT instruction is executed.
 */
static uint8_t io_input_ports[256]={0};
static uint8_t io_output_ports[256]={0};

/* set from cli options */
extern const char *io_input_file;
extern const char *io_output_file;
extern const char *io_output_log_file;

// io_output_log_file fd for appending all OUT instructions
static FILE *io_output_log_fd = NULL;
 


static uint32_t io_spectrum_read(void *opaque, uint32_t addr)
{
    if ((addr & 1) == 0) {
#ifdef IOPIPE_ENABLED
        return iopipe_read(&iopipe);
#else
        return zx_keyboard_read(opaque, addr);
#endif
    } else {
        return 0xff;
    }
}

static void io_spectrum_write(void *opaque, uint32_t addr, uint32_t data)
{
    if ((addr & 1) == 0) {
#ifdef IOPIPE_ENABLED
        iopipe_write(&iopipe, (uint8_t)data);
#else
        zx_video_set_border(data & 0x7);
#endif
    }
}

static void io_page_write(void *opaque, uint32_t addr, uint32_t data)
{
    int newrom, newram;
    int changed = 0;

    newrom = 8 + !!(data & 0x10);
    newram = data & 0x7;
    if (page_tab[0] != newrom) {
        page_tab[0] = newrom;
        changed = 1;
    }
    if (page_tab[3] != newram) {
        page_tab[3] = newram;
        changed = 1;
    }
    if (changed) {
        cpu_interrupt(first_cpu, CPU_INTERRUPT_EXITTB);
        tlb_flush(first_cpu);
    }
}

static target_ulong zx_mapaddr_128k(target_ulong addr) {
    return (page_tab[addr >> 14] << 14) | (addr & ~(~0 << 14));
}

static void main_cpu_reset(void *opaque)
{
    CPUState *env = opaque;
    cpu_reset(env);
}

static QEMUTimer *zx_ula_timer;

static void zx_50hz_timer(void *opaque)
{
    int64_t next_time;

    CPUState *env = opaque;
    cpu_interrupt(env, CPU_INTERRUPT_HARD);

    /* FIXME: not exactly 50 Hz */
    next_time = cpu_get_clock() + muldiv64(1, NANOSECONDS_PER_SECOND, 50);
    timer_mod(zx_ula_timer, next_time);

    zx_video_do_retrace();
}

static CPUState *zx_env;

static void zx_timer_init(void)
{
    int64_t t = cpu_get_clock();
    zx_ula_timer = timer_new_ns(rtc_clock, zx_50hz_timer, zx_env);
    timer_mod(zx_ula_timer, t);
}

static const uint8_t halthack_oldip[16] =
    {253, 203, 1,110, 200, 58, 8, 92, 253, 203, 1, 174};
static const uint8_t halthack_newip[16] =
    {33, 59, 92, 118, 203, 110, 200, 58, 8, 92, 203, 174};


#ifdef IOPIPE_ENABLED
static uint8_t iopipe_read(IOPipe *iop)
{
    uint8_t val;
    // fprintf(stderr, "%s: about to read from pipe\n", __PRETTY_FUNCTION__);

    int ret; 
    // read 1 byte from the pipe, we BLOCK if there's nothing to read.
    while ((ret=read(iop->rdfd, &val, 1)) < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    // fprintf(stderr, "%s: read from pipe would block (ret=%d), waiting...\n", __PRETTY_FUNCTION__, ret);
                    usleep(5*1000);
                }
            else
                break;
        }
    
    if (ret > 0){
        // fprintf(stderr, "%s: read byte %02X from io pipe\n", __PRETTY_FUNCTION__, val);
        return val;
    }
    else {
        perror(__PRETTY_FUNCTION__);
        fprintf(stderr, "%s: error while reading from io pipe\n", __PRETTY_FUNCTION__);
        return 0xff;
    }
}

static int iopipe_write(IOPipe *iop, uint8_t data)
{
    int ret = write(iop->wrfd, &data, 1); // write 1 byte to the pipe
    if (ret > 0){
        // fprintf(stderr, "%s: wrote byte %02X to io pipe\n", __PRETTY_FUNCTION__, data);
        return 0;
    }
    else {
        fprintf(stderr, "%s: error while writing to io pipe\n", __PRETTY_FUNCTION__);
        return -1;
    }
}

static int iopipe_init(IOPipe *iop)
{
    int rdfd, wrfd;

    rdfd = open(iop->rd_path, O_RDONLY | O_NONBLOCK);
    if (rdfd == -1){
        fprintf(stderr, "%s: failed to open read pipe %s\n", __PRETTY_FUNCTION__, iop->rd_path);
        return -1;
    }
    
    wrfd = open(iop->wr_path, O_WRONLY | O_NONBLOCK);
    if (wrfd == -1){
        fprintf(stderr, "%s: failed to open write pipe %s\n", __PRETTY_FUNCTION__, iop->wr_path);
        perror(__PRETTY_FUNCTION__);
        return -2;
    }

    iop->rdfd = rdfd;
    iop->wrfd = wrfd;
    return 0;
}
#endif 

static uint32_t io_file_read(void *opaque, uint32_t addr)
{
    int p;

    /* default value for unspecified input ports readings */
    uint32_t deflt = 0x00; //TODO: maybe rand() % 256

    for (p=0; p<256; p++) 
        io_input_ports[p]=(uint8_t) deflt;

    // fprintf(stderr, "%s: port %x value %x\n", __PRETTY_FUNCTION__, (uint8_t)addr, (uint8_t)deflt);
    
    FILE *input_file_fd;
    input_file_fd = fopen(io_input_file, "r");

    if (input_file_fd) {
        char line[255];
        int lineno = 0;
        unsigned int port, value;

        while (fgets(line, 255, input_file_fd)) {
            lineno++;
            if (sscanf(line, "0x%02X: 0x%02X \n", &port, &value) >= 2)
                io_input_ports[(uint8_t)port]=(uint8_t)value;
            else 
                fprintf(stderr, "WARNING -- ignoring malformed input port specification line: %s (%s:%d).\n", line, io_input_file, lineno);


        }
    }
    else {
        fprintf(stderr, "ERROR at %s: failed to open file %s for reading.\n", __PRETTY_FUNCTION__, io_input_file);
        perror(__PRETTY_FUNCTION__);
    }

    /* finally, close the file */
    if (input_file_fd) {
        fclose(input_file_fd);
    }

    /* return the value read */
    return (uint32_t) io_input_ports[(uint8_t)addr];
}


static void io_file_write(void *opaque, uint32_t addr, uint32_t data)
{
    // fprintf(stderr, "%s: about to write to io_output_file: %s\n", __PRETTY_FUNCTION__, io_output_file);
    
    /* get current PC from cpu state */
    //uint32_t pc = zx_env->pc;
    //    fprintf(stderr, "INFO -- %s:    PC: 0x%04X  OUT port=0x%02X  value=0x%02X\n", __PRETTY_FUNCTION__, pc, (uint8_t)addr, (uint8_t)data);

    fprintf(stderr, "INFO -- %s:    OUT port=0x%02X  value=0x%02X\n", __PRETTY_FUNCTION__, (uint8_t)addr, (uint8_t)data);

    uint8_t port = (uint8_t)addr;
    io_output_ports[port] = (uint8_t)data;

    if (io_output_file)	{
	FILE *output_file_fd;
	output_file_fd = fopen(io_output_file, "w+");
    
	if (output_file_fd) {
	    int p;
	    const char *linefmt = "OUT Port 0x%02X:   0x%02X\r\n"; // ie: OUT Port 0x0A:\t 0xFF
	    for (p=0; p < 256; p++) {
		fprintf(output_file_fd, linefmt, p, io_output_ports[p]);
	    }
	}
	else {
	    fprintf(stderr, "ERROR at %s: failed to open file %s for writing.\n", __PRETTY_FUNCTION__, io_output_file);
	    perror(__PRETTY_FUNCTION__);
	}

	/* finally, close the file */
	if (output_file_fd) {
	    fclose(output_file_fd);
	}
    }

    if (io_output_log_file) {
	
	/* first, open the log file if neeeded */
	if (!io_output_log_fd) {
//	    fprintf(stderr, "%s: about to open io_output_log_file: %s\n",
//		    __PRETTY_FUNCTION__, 
//		    io_output_log_file);

	    io_output_log_fd = fopen(io_output_log_file, "w");
	    if (io_output_log_fd) {
		//TODO: log info
	    }
	    else {
		fprintf(stderr, "ERROR at %s: failed to open file %s for writing.\n",
			__PRETTY_FUNCTION__,
			io_output_log_file);
		perror(__PRETTY_FUNCTION__);
	    }
	}

	/* append to the output log file */
	if (io_output_log_fd){             // sanity check
	    const char *linefmt = "OUT(0x%02X):    0x%02X\r\n";


	    
	    /* write to the file */
	    fprintf(io_output_log_fd, linefmt, port, io_output_ports[port]);
	    fflush(io_output_log_fd);
	}
	else {
	    fprintf(stderr, "ERROR -- cannot write to io-output-log file %s. fopen failed?", io_output_log_file);
	}
    }
}


/* ZX Spectrum initialisation */
static void zx_spectrum_common_init(ram_addr_t ram_size,
                                    const char *boot_device,
                                    const char *kernel_filename,
                                    const char *cpu_model,
                                    int is_128k)
{
    char *filename;
    uint8_t halthack_curip[12];
    int ret;
    ram_addr_t ram_offset, rom_offset;
    int rom_size;
    int ram_base, rom_base;
    CPUState *env;
    // int port, pagebyte;
    int haltaddr;

    /* init CPUs */
    if (!cpu_model) {
        cpu_model = "z80";
    }
    env = (CPUState*) cpu_z80_init(cpu_model);
    zx_env = env; // XXX
    register_savevm("cpu", 0, 4, cpu_save, cpu_load, env);
    qemu_register_reset(main_cpu_reset, 0, env);
    main_cpu_reset(env);

    if (is_128k) {
        env->mapaddr = zx_mapaddr_128k;
    } else {
        env->mapaddr = NULL;
    }

    if (is_128k) {
        /* RAM first, then ROM */
        ram_size = 0x20000;
        ram_base = 0;
    } else {
        /* ROM first, then RAM, matching the 48K's memory map */
        ram_size = 0xc000;
        ram_base = 0x4000;
    }

    /* allocate RAM */
    ram_offset = qemu_ram_alloc(ram_size);
    cpu_register_physical_memory(ram_base, ram_size, ram_offset | IO_MEM_RAM);

    /* ROM load */
    if (bios_name == NULL) {
        if (is_128k) {
            bios_name = ROM_FILENAME_128;
        } else {
            bios_name = ROM_FILENAME_48;
        }
    }
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        rom_size = get_image_size(filename);
    } else {
        rom_size = -1;
    }
    if (rom_size <= 0 ||
        (rom_size % 0x4000) != 0) {
        goto rom_error;
    }
    if (is_128k) {
        rom_base = ram_size;
    } else {
        rom_base = 0;
    }
    rom_offset = qemu_ram_alloc(rom_size);
    cpu_register_physical_memory(rom_base, rom_size, rom_offset | IO_MEM_ROM);
    ret = load_image_targphys(filename, rom_base, rom_size);
    if (ret != rom_size) {
    rom_error:
        fprintf(stderr, "qemu: could not load ZX Spectrum ROM '%s'\n",
                        bios_name);
        exit(1);
    }
    if (filename) {
        qemu_free(filename);
    }

    /* hack from xz80 adding HALT to the keyboard input loop to save CPU */
    haltaddr = rom_base + 0x10b0 + 0x4000;
    cpu_physical_memory_read(haltaddr, halthack_curip, 12);
    if (!memcmp(halthack_curip, halthack_oldip, 12)) {
        cpu_physical_memory_write_rom(haltaddr, halthack_newip, 12);
    }

    /* map entire I/O space */
    if (io_output_file || io_output_log_file) 
        /* redirect OUTs to file if necessary*/
        register_ioport_write(0, 0x10000, 1, io_file_write, NULL);
    else 
        register_ioport_write(0, 0x10000, 1, io_spectrum_write, NULL);

    if (io_input_file)
        register_ioport_read (0, 0x10000, 1, io_file_read,  NULL);
    else
        register_ioport_read (0, 0x10000, 1, io_spectrum_read, NULL);

    zx_video_init(ram_offset, is_128k);
    zx_keyboard_init();
    zx_timer_init();

#ifdef IOPIPE_ENABLED
    iopipe_init(&iopipe);
#endif

    if (is_128k) {
        page_tab[0] = 8;
        page_tab[1] = 5;
        page_tab[2] = 2;
        page_tab[3] = 0;
    }

#ifdef CONFIG_LIBSPECTRUM
    /* load a snapshot */
    if (kernel_filename) {
        libspectrum_id_t type;
        libspectrum_class_t cls;
        libspectrum_snap* snap;
        uint8_t* snapmem;
        libspectrum_byte* page;
        int length;
        int i, p;
        const int pages_48k[3] = { 5, 2, 0 };

        if (libspectrum_init() != LIBSPECTRUM_ERROR_NONE ||
            libspectrum_identify_file(&type, kernel_filename, NULL, 0) != LIBSPECTRUM_ERROR_NONE ||
            libspectrum_identify_class(&cls, type) != LIBSPECTRUM_ERROR_NONE) {
            fprintf(stderr, "%s: libspectrum error\n", __FUNCTION__);
            exit(1);
        }
        snap = libspectrum_snap_alloc();
        if (cls != LIBSPECTRUM_CLASS_SNAPSHOT) {
            fprintf(stderr, "%s: %s is not a snapshot\n", __FUNCTION__, kernel_filename);
            exit(1);
        }
        snapmem = qemu_mallocz(0x10000);
        length = load_image(kernel_filename, snapmem);
        //printf("loaded %d bytes from %s\n",length, kernel_filename);
        if (libspectrum_snap_read(snap, snapmem, length, type, NULL) != LIBSPECTRUM_ERROR_NONE) {
            fprintf(stderr, "%s: failed to load snapshot %s\n", __FUNCTION__, kernel_filename);
            exit(1);
        }
        //printf("snap pc = %d\n",libspectrum_snap_pc(snap));

        /* fill memory */
        if (is_128k) {
            for (p = 0; p < 8; p++) {
                page = libspectrum_snap_pages(snap, p);
                if (page) {
                    for (i = 0x0000; i < 0x4000; i++) {
                        stb_phys(i + (p << 14), page[i]);
                    }
                }
            }
        } else {
            for (p = 0; p < 3; p++) {
                page = libspectrum_snap_pages(snap, pages_48k[p]);
                if (page) {
                    for (i = 0x0000; i < 0x4000; i++) {
                        stb_phys(i + ((p + 1) << 14), page[i]);
                    }
                }
            }
        }

        if (is_128k) {
            if ((libspectrum_machine_capabilities(libspectrum_snap_machine(snap) &
                 LIBSPECTRUM_MACHINE_CAPABILITY_128_MEMORY))) {
                /* restore paging state */
                pagebyte = libspectrum_snap_out_128_memoryport(snap);
                page_tab[0] = 8 + !!(pagebyte & 0x10);
                page_tab[3] = pagebyte & 0x7;
            } else {
                /* page in 48K ROM */
                page_tab[0] = 9;
                page_tab[3] = 0;
            }
        } else {
            if ((libspectrum_machine_capabilities(libspectrum_snap_machine(snap) &
                 LIBSPECTRUM_MACHINE_CAPABILITY_128_MEMORY))) {
                fprintf(stderr, "qemu: can't load 128K snap under 48K machine\n");
            }
        }

        /* set the border */
        zx_video_set_border(libspectrum_snap_out_ula(snap) & 0x7);

        /* restore registers */
        env->regs[R_A] = libspectrum_snap_a(snap);
        env->regs[R_F] = libspectrum_snap_f(snap);
        env->regs[R_BC] = libspectrum_snap_bc(snap);
        env->regs[R_DE] = libspectrum_snap_de(snap);
        env->regs[R_HL] = libspectrum_snap_hl(snap);
        env->regs[R_AX] = libspectrum_snap_a_(snap);
        env->regs[R_FX] = libspectrum_snap_f_(snap);
        env->regs[R_BCX] = libspectrum_snap_bc_(snap);
        env->regs[R_DEX] = libspectrum_snap_de_(snap);
        env->regs[R_HLX] = libspectrum_snap_hl_(snap);
        env->regs[R_IX] = libspectrum_snap_ix(snap);
        env->regs[R_IY] = libspectrum_snap_iy(snap);
        env->regs[R_I] = libspectrum_snap_i(snap);
        env->regs[R_R] = libspectrum_snap_r(snap);
        env->regs[R_SP] = libspectrum_snap_sp(snap);
        env->pc = libspectrum_snap_pc(snap);
        env->iff1 = libspectrum_snap_iff1(snap);
        env->iff2 = libspectrum_snap_iff2(snap);
        env->imode = libspectrum_snap_im(snap);

        qemu_free(snapmem);
    }
#endif
}

static void zx_spectrum48_init(ram_addr_t ram_size,
                               const char *boot_device,
                               const char *kernel_filename,
                               const char *kernel_cmdline,
                               const char *initrd_filename,
                               const char *cpu_model)
{
    zx_spectrum_common_init(ram_size, boot_device, kernel_filename,
                            cpu_model, 0);
}

static void zx_spectrum128_init(ram_addr_t ram_size,
                                const char *boot_device,
                                const char *kernel_filename,
                                const char *kernel_cmdline,
                                const char *initrd_filename,
                                const char *cpu_model)
{
    zx_spectrum_common_init(ram_size, boot_device, kernel_filename,
                            cpu_model, 1);
}

static QEMUMachine zxspec48_machine = {
    .name = "zxspec48",
    .desc = "ZX Spectrum 48K",
    .init = zx_spectrum48_init,
    .is_default = 1,
};

static QEMUMachine zxspec128_machine = {
    .name = "zxspec128",
    .desc = "ZX Spectrum 48K",
    .init = zx_spectrum128_init,
    .is_default = 0,
};

static void zxspec_machine_init(void) {
    qemu_register_machine(&zxspec48_machine);
    qemu_register_machine(&zxspec128_machine);
}

machine_init(zxspec_machine_init);
