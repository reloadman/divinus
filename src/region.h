#pragma once

#include <ifaddrs.h>
#include <linux/if_link.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include "app_config.h"
#include "hal/support.h"
#include "lib/spng.h"
#include "text.h"

#define DEF_COLOR 0xFFFF
#define DEF_FONT "UbuntuMono-Regular"
#define DEF_OPAL 255
#define DEF_OUTL 0x8000
#define DEF_POSX 16
#define DEF_POSY 16
#define DEF_SIZE 32.0f
// Default outline thickness in pixels. Non-zero greatly improves readability on bright scenes.
#define DEF_THICK 2.0f
// Background box (behind text). 0 = disabled; otherwise ARGB1555 color.
// Minimal "premium" look is a semi-transparent black box with small padding.
// NOTE: For HiSilicon v4 (ARGB1555 overlays), bgAlpha and fgAlpha can be set separately.
// We treat bg_opal=0 as "disabled".
#define DEF_BG 0x0000         // RGB555 color (0..0x7FFF); 0 is black
#define DEF_BGOPAL 144        // 0..255; (~56% opacity) for background box
#define DEF_PAD 6
#define DEF_TIMEFMT "%Y/%m/%d %H:%M:%S"
#define MAX_OSD 10

extern char keepRunning;

typedef struct {
    unsigned int size;
    unsigned short reserved1;
    unsigned short reserved2;
    unsigned int offBits;
} bitmapfile;

typedef struct {
    unsigned short size;
    unsigned int width;
    int height;
    unsigned short planes;
    unsigned short bitCount;
    unsigned int compression;
    unsigned int sizeImage;
    unsigned int xPerMeter;
    unsigned int yPerMeter;
    unsigned int clrUsed;
    unsigned int clrImportant;
} bitmapinfo;

typedef struct {
    unsigned int redMask;
    unsigned int greenMask;
    unsigned int blueMask;
    unsigned int alphaMask;
    unsigned char clrSpace[4];
    unsigned char csEndpoints[24];
    unsigned int redGamma;
    unsigned int greenGamma;
    unsigned int blueGamma;
} bitmapfields;

typedef struct {
    double size;
    int hand, color;
    short opal, posx, posy;
    char updt;
    // Persist this region into config when saving (divinus.yaml).
    // Auto-generated runtime overlays (e.g. isp_debug) set this to 0.
    unsigned char persist;
    // Font name (legacy) or full font file path (preferred).
    // Keep this large enough to hold absolute paths.
    char font[256];
    char text[80];
    char img[64];
    int outl;
    double thick;
    int bg;        // RGB555 background color (0..0x7FFF). Enable via bgopal>0.
    short bgopal;  // background opacity (0..255). 0 disables background box.
    short pad;     // padding (px) around text when background box is enabled
} osd;

extern osd osds[MAX_OSD];
extern char timefmt[64];

int start_region_handler();
void stop_region_handler();