/*
 * ZX Spectrum Keyboard Emulation
 *
 * Copyright (c) 2007-2009 Stuart Brady <stuart.brady@gmail.com>
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
#include "hw.h"
#include "qemu-timer.h"
#include "console.h"
#include "isa.h"
#include "sysemu.h"
#include "zx_keyboard.h"
#include "boards.h"

//#define DEBUG_ZX_KEYBOARD

#ifdef DEBUG_ZX_KEYBOARD
#define DPRINTF(fmt, ...) printf(fmt, ## __VA_ARGS__)
#else
#define DPRINTF(fmt, ...)
#endif

static int keystate[8];

uint32_t zx_keyboard_read(void *opaque, uint32_t addr)
{
    int r = 0;
    uint8_t colbits = 0xff;

    uint32_t rowbits = ((addr >> 8) & 0xff);

    for (r = 0; r < 8; r++) {
        if (!(rowbits & (1 << r))) {
            colbits &= keystate[r];
        }
    }
    return colbits;
}

typedef struct {
    int row;
    int column;
} ZXKeypos;

#define DEF_ZX_KEY(name, row, column) ZX_KEY_ ## name,
enum zx_keys {
#include "zx_key_template.h"
ZX_MAX_KEYS
};

#define DEF_ZX_KEY(name, row, column) [ZX_KEY_ ## name] = {row, column},
static const ZXKeypos keypos[ZX_MAX_KEYS] = {
#include "zx_key_template.h"
};

static int zx_keypressed[ZX_MAX_KEYS];
static int qemu_keypressed[0x100];

static const int map[0x100][2] = {
    [0 ... 0xff] = {-1, -1}, /* Unmapped by default */

    [0x01] = {ZX_KEY_CAPSSHIFT, ZX_KEY_SPACE}, /* Escape */

    [0x02] = {ZX_KEY_1, -1},
    [0x03] = {ZX_KEY_2, -1},
    [0x04] = {ZX_KEY_3, -1},
    [0x05] = {ZX_KEY_4, -1},
    [0x06] = {ZX_KEY_5, -1},
    [0x07] = {ZX_KEY_6, -1},
    [0x08] = {ZX_KEY_7, -1},
    [0x09] = {ZX_KEY_8, -1},
    [0x0a] = {ZX_KEY_9, -1},
    [0x0b] = {ZX_KEY_0, -1},

    [0x0c] = {ZX_KEY_SYMBSHIFT, ZX_KEY_J}, /* Minus */

    [0x0e] = {ZX_KEY_CAPSSHIFT, ZX_KEY_0}, /* Backspace */

    [0x10] = {ZX_KEY_Q, -1},
    [0x11] = {ZX_KEY_W, -1},
    [0x12] = {ZX_KEY_E, -1},
    [0x13] = {ZX_KEY_R, -1},
    [0x14] = {ZX_KEY_T, -1},
    [0x15] = {ZX_KEY_Y, -1},
    [0x16] = {ZX_KEY_U, -1},
    [0x17] = {ZX_KEY_I, -1},
    [0x18] = {ZX_KEY_O, -1},
    [0x19] = {ZX_KEY_P, -1},

    [0x0d] = {ZX_KEY_SYMBSHIFT, ZX_KEY_L}, /* Equals */
    [0x0f] = {ZX_KEY_CAPSSHIFT, ZX_KEY_1}, /* Tab */

    [0x1c] = {ZX_KEY_ENTER,     -1},

    [0x1d] = {ZX_KEY_SYMBSHIFT, -1}, /* Left Control */

    [0x1e] = {ZX_KEY_A, -1},
    [0x1f] = {ZX_KEY_S, -1},
    [0x20] = {ZX_KEY_D, -1},
    [0x21] = {ZX_KEY_F, -1},
    [0x22] = {ZX_KEY_G, -1},
    [0x23] = {ZX_KEY_H, -1},
    [0x24] = {ZX_KEY_J, -1},
    [0x25] = {ZX_KEY_K, -1},
    [0x26] = {ZX_KEY_L, -1},

    [0x27] = {ZX_KEY_SYMBSHIFT, ZX_KEY_O}, /* Semicolon */
    [0x28] = {ZX_KEY_SYMBSHIFT, ZX_KEY_7}, /* Apostrophe */

    [0x2a] = {ZX_KEY_CAPSSHIFT, -1}, /* Left Shift */

    [0x2b] = {ZX_KEY_SYMBSHIFT, ZX_KEY_3}, /* Hash */

    [0x2c] = {ZX_KEY_Z, -1},
    [0x2d] = {ZX_KEY_X, -1},
    [0x2e] = {ZX_KEY_C, -1},
    [0x2f] = {ZX_KEY_V, -1},
    [0x30] = {ZX_KEY_B, -1},
    [0x31] = {ZX_KEY_N, -1},
    [0x32] = {ZX_KEY_M, -1},

    [0x33] = {ZX_KEY_SYMBSHIFT, ZX_KEY_N}, /* Comma */
    [0x34] = {ZX_KEY_SYMBSHIFT, ZX_KEY_M}, /* Period */
    [0x35] = {ZX_KEY_SYMBSHIFT, ZX_KEY_V}, /* Slash */

    [0x36] = {ZX_KEY_CAPSSHIFT, -1}, /* Right Shift */
    [0x37] = {ZX_KEY_SYMBSHIFT, ZX_KEY_B}, /* * (Numpad) */
    [0x38] = {ZX_KEY_SYMBSHIFT, -1}, /* Left Alt */
    [0x39] = {ZX_KEY_SPACE,     -1}, /* Space Bar */

    [0x47] = {ZX_KEY_7,         -1}, /* 7 (Numpad) */
    [0x48] = {ZX_KEY_8,         -1}, /* 8 (Numpad) */
    [0x49] = {ZX_KEY_9,         -1}, /* 9 (Numpad) */
    [0x4a] = {ZX_KEY_SYMBSHIFT, ZX_KEY_J}, /* Minus (Numpad) */
    [0x4b] = {ZX_KEY_4,         -1}, /* 4 (Numpad) */
    [0x4c] = {ZX_KEY_5,         -1}, /* 5 (Numpad) */
    [0x4d] = {ZX_KEY_6,         -1}, /* 6 (Numpad) */
    [0x4e] = {ZX_KEY_SYMBSHIFT, ZX_KEY_K}, /* Plus (Numpad) */
    [0x4f] = {ZX_KEY_1,         -1}, /* 1 (Numpad) */
    [0x50] = {ZX_KEY_2,         -1}, /* 2 (Numpad) */
    [0x51] = {ZX_KEY_3,         -1}, /* 3 (Numpad) */
    [0x52] = {ZX_KEY_0,         -1}, /* 0 (Numpad) */
    [0x53] = {ZX_KEY_SYMBSHIFT, ZX_KEY_M}, /* Period (Numpad) */

    [0x9c] = {ZX_KEY_SYMBSHIFT, -1}, /* Enter (Numpad) */
    [0x9d] = {ZX_KEY_SYMBSHIFT, -1}, /* Right Control */
    [0xb5] = {ZX_KEY_SYMBSHIFT, ZX_KEY_V}, /* Slash (Numpad) */
    [0xb8] = {ZX_KEY_SYMBSHIFT, -1}, /* Right Alt */

    [0xc8] = {ZX_KEY_CAPSSHIFT, ZX_KEY_7}, /* Up Arrow */
    [0xcb] = {ZX_KEY_CAPSSHIFT, ZX_KEY_5}, /* Left Arrow */
    [0xcd] = {ZX_KEY_CAPSSHIFT, ZX_KEY_8}, /* Right Arrow */
    [0xd0] = {ZX_KEY_CAPSSHIFT, ZX_KEY_6}, /* Down Arrow */

    [0xdb] = {ZX_KEY_CAPSSHIFT, ZX_KEY_SYMBSHIFT}, /* Left Meta */
    [0xdc] = {ZX_KEY_CAPSSHIFT, ZX_KEY_SYMBSHIFT}, /* Menu */
    [0xdd] = {ZX_KEY_CAPSSHIFT, ZX_KEY_SYMBSHIFT}, /* Right Meta */
};

/* FIXME:
 *   Need to mappings from stepping on each other...
 *   or at least make them step on one another in a consistent manner?
 *   Could use separate state arrays for surpressing/adding keys
 *   and allow only one change to the modifier keys at a time...
 *
 * Also need to implement shifted mappings.
 */

static void zx_put_keycode(void *opaque, int keycode)
{
    int release = keycode & 0x80;
    int key, row, col;
    static int ext_keycode = 0;
    int i;
    int valid;

    if (keycode == 0xe0) {
        ext_keycode = 1;
    } else {
        if (ext_keycode) {
            keycode |= 0x80;
        } else {
            keycode &= 0x7f;
        }
        ext_keycode = 0;

        DPRINTF("Keycode 0x%02x (%s)\n", keycode, release ? "release" : "press");

        if (release && qemu_keypressed[keycode]) {
            valid = 1;
            qemu_keypressed[keycode] = 0;
        } else if (!release && !qemu_keypressed[keycode]) {
            valid = 1;
            qemu_keypressed[keycode] = 1;
        } else {
            valid = 0;
        }

        if (valid) {
            for (i = 0; i < 2; i++) {
                key = map[keycode][i];
                if (key != -1) {
                    row = keypos[key].row;
                    col = keypos[key].column;
                    if (release) {
                        if (--zx_keypressed[key] <= 0) {
                            DPRINTF("Releasing 0x%02x\n", key);
                            zx_keypressed[key] = 0;
                            keystate[row] |= 1 << col;
                        }
                    } else {
                        DPRINTF("Pressing 0x%02x\n", key);
                        zx_keypressed[key]++;
                        keystate[row] &= ~(1 << col);
                    }
                }
            }
        }
    }
}

void zx_keyboard_init(void)
{
    int i;
    for (i=0; i<8; i++) {
        keystate[i] = 0xff;
    }
    memset(zx_keypressed, 0, sizeof(zx_keypressed));
    memset(qemu_keypressed, 0, sizeof(qemu_keypressed));
    qemu_add_kbd_event_handler(zx_put_keycode, NULL);
}
