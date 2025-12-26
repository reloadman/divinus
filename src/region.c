#include "region.h"

#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include "media.h"
#include "night.h"

osd osds[MAX_OSD];
pthread_t regionPid = 0;
char timefmt[64];
unsigned int rxb_l, txb_l, cpu_l[6];

// Avoid frequent region recreation due to tiny bitmap size changes.
// Strategy: grow immediately; shrink only if new area is <= 90% of current.
// (Used on platforms where region recreation is expensive/noisy: hisi/v4, star/i6,i6c,m6.)
static unsigned short osd_rgn_w[MAX_OSD] = {0};
static unsigned short osd_rgn_h[MAX_OSD] = {0};
static char osd_font_logged[MAX_OSD][512] = {{0}};
static unsigned short osd_logged_w[MAX_OSD] = {0};
static unsigned short osd_logged_h[MAX_OSD] = {0};
static short osd_logged_x[MAX_OSD] = {0};
static short osd_logged_y[MAX_OSD] = {0};
static short osd_logged_bgopal[MAX_OSD] = {0};
static short osd_logged_opal[MAX_OSD] = {0};
static bool osd_attached[MAX_OSD] = {0};

static inline unsigned short u16_min(unsigned short a, unsigned short b) { return (a < b) ? a : b; }

static hal_bitmap region_pad_bitmap(const hal_bitmap *src, unsigned short target_w, unsigned short target_h, bool *out_allocated) {
    hal_bitmap out = *src;
    if (out_allocated) *out_allocated = false;
    if (!src || !src->data) return out;
    if (src->dim.width == target_w && src->dim.height == target_h) return out;

    size_t size = (size_t)target_w * (size_t)target_h * 2;
    void *buf = malloc(size);
    if (!buf) return out;

    // Fill with fully transparent pixels (alpha-bit 0).
    memset(buf, 0, size);

    const unsigned short sw = src->dim.width;
    const unsigned short sh = src->dim.height;
    const unsigned short cw = u16_min(sw, target_w);
    const unsigned short ch = u16_min(sh, target_h);

    const unsigned short *s = (const unsigned short *)src->data;
    unsigned short *d = (unsigned short *)buf;
    for (unsigned short y = 0; y < ch; y++) {
        memcpy(d + (size_t)y * target_w, s + (size_t)y * sw, (size_t)cw * 2);
    }

    out.data = buf;
    out.dim.width = target_w;
    out.dim.height = target_h;
    if (out_allocated) *out_allocated = true;
    return out;
}

// Sleep until the next second boundary (best-effort), in small chunks so shutdown is responsive.
static void region_sleep_to_next_second(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        // Fallback.
        usleep(1000 * 1000);
        return;
    }
    long ns_to_next = 1000000000L - ts.tv_nsec;
    if (ns_to_next < 0) ns_to_next = 0;
    // Cap to 1s.
    if (ns_to_next > 1000000000L) ns_to_next = 1000000000L;
    unsigned int us = (unsigned int)(ns_to_next / 1000L);
    while (keepRunning && us) {
        unsigned int step = (us > 50000u) ? 50000u : us; // 50ms
        usleep(step);
        us -= step;
    }
}

static bool region_font_has_supported_ext(const char *s) {
    if (!s) return false;
    const char *ext = strrchr(s, '.');
    if (!ext) return false;
    return strcasecmp(ext, ".ttf") == 0 ||
           strcasecmp(ext, ".otf") == 0 ||
           strcasecmp(ext, ".ttc") == 0;
}

// Resolves configured font string into a filesystem path usable by `sft_loadfile()`.
// - If `cfg` contains '/', treat it as a path (absolute or relative) and use as-is.
// - Otherwise treat it as a font name (legacy) and search common directories.
// Returns 0 on success, -1 on failure.
static int region_resolve_font_path(const char *cfg, char *out, size_t out_sz) {
    if (!out || out_sz == 0) return -1;
    out[0] = '\0';

    if (EMPTY(cfg))
        cfg = DEF_FONT;

    // Explicit path: allow both absolute and relative paths.
    if (strchr(cfg, '/')) {
        if (!access(cfg, F_OK)) {
            snprintf(out, out_sz, "%s", cfg);
            return 0;
        }
        return -1;
    }

    const char *dirs[] = {
        ".",
        "/oem/usr/share",
        "/usr/local/share/fonts",
        "/usr/share/fonts/truetype",
        "/usr/share/fonts",
        NULL
    };

    const bool has_ext = region_font_has_supported_ext(cfg);
    for (const char **dir = dirs; *dir; dir++) {
        // If caller specified filename with extension (e.g. "Ubuntu.ttf"), try directly.
        if (has_ext) {
            snprintf(out, out_sz, "%s/%s", *dir, cfg);
            if (!access(out, F_OK)) return 0;
        }

        // Legacy lookup: name without extension (e.g. "UbuntuMono-Regular").
        snprintf(out, out_sz, "%s/%s.ttf", *dir, cfg);
        if (!access(out, F_OK)) return 0;
        snprintf(out, out_sz, "%s/%s2.ttf", *dir, cfg);
        if (!access(out, F_OK)) return 0;
        snprintf(out, out_sz, "%s/%s.otf", *dir, cfg);
        if (!access(out, F_OK)) return 0;
        snprintf(out, out_sz, "%s/%s.ttc", *dir, cfg);
        if (!access(out, F_OK)) return 0;
    }

    out[0] = '\0';
    return -1;
}

static int region_guess_frame_height(void) {
    // Best-effort: use configured main stream dimensions if present.
    if (app_config.mp4_height > 0) return (int)app_config.mp4_height;
    if (app_config.jpeg_height > 0) return (int)app_config.jpeg_height;
    return 1080;
}

static int region_find_free_osd_slot_from_end(void) {
    for (int id = MAX_OSD - 1; id >= 0; id--) {
        if (EMPTY(osds[id].text) && EMPTY(osds[id].img))
            return id;
    }
    return -1;
}

static void region_setup_isp_debug_osd(void) {
    // These are the reserved slots for ISP debug overlay.
    const int id1 = 2; // top line
    const int id2 = 3; // bottom line

    if (!app_config.osd_enable)
        return;

    // If ISP debug is disabled, but the reserved regions still contain $I macros
    // (e.g. persisted from a previous enable), clear them so they don't show up.
    if (!app_config.osd_isp_debug) {
        // Only clear if these were auto-generated (transient).
        if (id1 >= 0 && id1 < MAX_OSD && osds[id1].persist == 0) {
            osds[id1].text[0] = '\0';
            osds[id1].img[0] = '\0';
            osds[id1].persist = 1;
            osds[id1].updt = 1;
        }
        if (id2 >= 0 && id2 < MAX_OSD && osds[id2].persist == 0) {
            osds[id2].text[0] = '\0';
            osds[id2].img[0] = '\0';
            osds[id2].persist = 1;
            osds[id2].updt = 1;
        }
        return;
    }

    // HiSilicon v4 often supports only a small number of regions reliably.
    // User requested fixed slots: use regions 2 and 3.
    if (id1 < 0 || id2 < 0 || id1 >= MAX_OSD || id2 >= MAX_OSD) {
        HAL_WARNING("region", "OSD ISP debug: requested slots out of range (MAX_OSD=%d)\n", MAX_OSD);
        return;
    }

    const int h = region_guess_frame_height();
    const int margin = 16;

    // Use a bit smaller font than the default to keep the overlay compact.
    const double size = 24.0;
    const int line_h = (int)(size * 3.0 / 2.0);

    // Make ISP debug overlay look consistent with the time overlay:
    // if any user-configured region contains `$t`/`$tu`, inherit its font.
    const char *inherit_font = DEF_FONT;
    for (int id = 0; id < MAX_OSD; id++) {
        if (EMPTY(osds[id].text)) continue;
        if (strstr(osds[id].text, "$t") == NULL) continue;
        if (!EMPTY(osds[id].font)) {
            inherit_font = osds[id].font;
            break;
        }
    }

    // Configure top line (id1) and bottom line (id2).
    const struct {
        int id;
        const char *macro;
        int posy;
    } lines[2] = {
        { id1, "$I1", h - margin - 2 * line_h },
        { id2, "$I2", h - margin - 1 * line_h },
    };

    for (int i = 0; i < 2; i++) {
        int id = lines[i].id;

        // Don't override user-configured regions.
        if (osds[id].persist != 0 && (!EMPTY(osds[id].text) || !EMPTY(osds[id].img))) {
            HAL_WARNING("region",
                "OSD ISP debug requested, but reg%d is already configured; skipping auto ISP debug overlay\n",
                id);
            continue;
        }

        osds[id].hand = -1;
        osds[id].color = DEF_COLOR;
        osds[id].outl = DEF_OUTL;
        osds[id].thick = DEF_THICK;
        osds[id].opal = DEF_OPAL;
        osds[id].size = size;
        osds[id].posx = DEF_POSX;
        osds[id].posy = (short)MAX(0, lines[i].posy);
        strncpy(osds[id].font, inherit_font, sizeof(osds[id].font) - 1);
        osds[id].font[sizeof(osds[id].font) - 1] = '\0';
        strncpy(osds[id].text, lines[i].macro, sizeof(osds[id].text) - 1);
        osds[id].text[sizeof(osds[id].text) - 1] = '\0';
        osds[id].img[0] = '\0';
        osds[id].persist = 0; // do not write into config
        osds[id].updt = 1;
    }
}

static void region_format_isp_debug_line(int line_no, char *dst, size_t dst_sz) {
    if (!dst || dst_sz == 0) return;
    dst[0] = '\0';

    unsigned int iso = 0, exptime = 0, again = 0, dgain = 0, ispdgain = 0;
    int ismax = 0;
    if (get_isp_exposure_info(&iso, &exptime, &again, &dgain, &ispdgain, &ismax) != EXIT_SUCCESS) {
        snprintf(dst, dst_sz, "ISP: n/a");
        return;
    }

    if (line_no == 1) {
        snprintf(dst, dst_sz, "ISO=%u T=%uus AG=%u DG=%u ISP=%u",
            iso, exptime, again, dgain, ispdgain);
        return;
    }

    unsigned char lum_u8 = 0;
    int have_lum = (get_isp_avelum(&lum_u8) == EXIT_SUCCESS);

    unsigned int drc = 0;
    int have_drc = (get_isp_drc_strength(&drc) == EXIT_SUCCESS);

    int ll = 0;
    int have_ll = (get_iq_lowlight_state(iso, exptime, &ll) == EXIT_SUCCESS);

    unsigned int comp = 0, expmax = 0, sysgainmax = 0;
    int have_ae = (get_isp_ae_auto_params(&comp, &expmax, &sysgainmax) == EXIT_SUCCESS);

    const char *mode = night_mode_on() ? "IR" : "DAY";

    // Keep it compact to fit into the 80-char OSD text buffer.
    if (have_lum && have_drc && have_ll && have_ae) {
        snprintf(dst, dst_sz, "Lum=%u Max=%d %s LL=%d DRC=%u C=%u EM=%u",
            (unsigned)lum_u8, ismax, mode, ll, drc, comp, expmax);
    } else if (have_lum && have_drc && have_ll) {
        snprintf(dst, dst_sz, "Lum=%u Max=%d %s LL=%d DRC=%u",
            (unsigned)lum_u8, ismax, mode, ll, drc);
    } else if (have_lum && have_drc) {
        snprintf(dst, dst_sz, "Lum=%u Max=%d %s DRC=%u",
            (unsigned)lum_u8, ismax, mode, drc);
    } else if (have_lum) {
        snprintf(dst, dst_sz, "Lum=%u Max=%d %s",
            (unsigned)lum_u8, ismax, mode);
    } else {
        snprintf(dst, dst_sz, "Max=%d %s", ismax, mode);
    }
}

void region_fill_formatted(char* str) {
    char out[80] = "";
    char param = 0;
    int ipos = 0, opos = 0;

    while(str[ipos] != 0)
    {
        if (str[ipos] != '$')
        {
            strncat(out, str + ipos, 1);
            opos++;
        }
        else if (str[ipos + 1] == 'B')
        {
            char ifname[16] = {0};
            struct ifaddrs *ifaddr, *ifa;

            ipos++;

            if (str[ipos + 1] == ':') {
                int j = 0;
                ipos++;
                while (str[ipos + 1] && str[ipos + 1] != '$' && str[ipos + 1] != ' ' && j < (sizeof(ifname) - 1))
                    ifname[j++] = str[++ipos];
                ifname[j] = '\0';
            }

            if (getifaddrs(&ifaddr) == -1) continue;

            for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
            { 
                if (EQUALS(ifa->ifa_name, "lo")) continue;
                if (ifname[0] && !EQUALS(ifa->ifa_name, ifname)) continue;
                if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_PACKET) continue;
                if (!ifa->ifa_data) continue;

                struct rtnl_link_stats *stats = ifa->ifa_data;
                char b[32];
                sprintf(b, "R:%dKbps S:%dKbps", 
                    (stats->rx_bytes - rxb_l) / 1024, (stats->tx_bytes - txb_l) / 1024);
                strcat(out, b);
                opos += strlen(b);
                rxb_l = stats->rx_bytes;
                txb_l = stats->tx_bytes;
                break;
            }
            
            freeifaddrs(ifaddr);
        }
        else if (str[ipos + 1] == 'C')
        {
            ipos++;
            char tmp[6];
            unsigned int cpu[6];
            FILE *stat = fopen("/proc/stat", "r");
            fscanf(stat, "%s %u %u %u %u %u %u",
                tmp, &cpu[0], &cpu[1], &cpu[2], &cpu[3], &cpu[4], &cpu[5]);
            fclose(stat);

            char c[5];
            char avg = 100 - (cpu[3] - cpu_l[3]) / sysconf(_SC_NPROCESSORS_ONLN);
            sprintf(c, "%d%%", avg);
            strcat(out, c);
            opos += strlen(c);
            for (int i = 0; i < sizeof(cpu) / sizeof(cpu[0]); i++)
                cpu_l[i] = cpu[i];
        }
        else if (str[ipos + 1] == 'M')
        {
            ipos++;
            struct sysinfo si;
            sysinfo(&si);

            char m[16];
            short used = (si.freeram + si.bufferram) / 1024 / 1024;
            short total = si.totalram / 1024 / 1024;
            sprintf(m, "%d/%dMB", used, total);
            strcat(out, m);
            opos += strlen(m);
        }
        else if (str[ipos + 1] == 'T')
        {
            ipos++;
            char s[8];
            float t = hal_temperature_read();
            sprintf(s, "%.1f", t);
            strcat(out, s);
            opos += strlen(s);
        }
        else if (str[ipos + 1] == 't')
        {
            ipos++;
            char s[64];
            time_t t = time(NULL);
            struct tm tm_buf, *tm_info;
            if (str[ipos + 1] == 'u') {
                ipos++;
                tm_info = gmtime(&t);
            } else tm_info = localtime_r(&t, &tm_buf);
            strftime(s, 64, timefmt, tm_info);
            strcat(out, s);
            opos += strlen(s);
        }
        else if (str[ipos + 1] == 'I')
        {
            ipos++;
            int line_no = 1;
            if (isdigit((unsigned char)str[ipos + 1])) {
                line_no = str[ipos + 1] - '0';
                ipos++;
            }
            char s[80];
            region_format_isp_debug_line(line_no, s, sizeof(s));
            strcat(out, s);
            opos += strlen(s);
        }
        else if (str[ipos + 1] == '$') {
            ipos++;
            strcat(out, "$");
            opos++;
        }
        ipos++; 
    }
    strncpy(str, out, 80);
}

static inline int region_open_bitmap(char *path, FILE **file) {
    unsigned short type;

    if (!path)
        HAL_ERROR("region", "Filename is empty!\n");
    if (!(*file = fopen(path, "rb")))
        HAL_ERROR("region", "Opening the bitmap failed!\n");
    if (fread(&type, 1, sizeof(type), *file) != sizeof(type))
        HAL_ERROR("region", "Reading the bitmap failed!\n");
    if (type != 0x4d42)
        HAL_ERROR("region", "Only bitmap files are currently supported!\n");

    return EXIT_SUCCESS;
}

int region_parse_bitmap(FILE **file, bitmapfile *bmpFile, bitmapinfo *bmpInfo) {
    if (fread(bmpFile, 1, sizeof(bitmapfile), *file) != sizeof(bitmapfile))
        HAL_ERROR("region", "Extracting the bitmap file header failed!\n");
    if (fread(bmpInfo, 1, sizeof(bitmapinfo), *file) != sizeof(bitmapinfo))
        HAL_ERROR("region", "Extracting the bitmap info failed!\n");
    if (bmpInfo->bitCount < 24)
        HAL_ERROR("region", "Indexed or <3bpp bitmaps are not supported!\n");
    if (bmpInfo->compression != 0 && !(bmpInfo->compression == 3 && (bmpInfo->bitCount == 16 || bmpInfo->bitCount == 32)))
        HAL_ERROR("region", "Compressed modes are not supported!\n");

    return EXIT_SUCCESS;
}

int region_prepare_image(char *path, hal_bitmap *bitmap) {
    FILE *file;
    unsigned char *bitmapdata, *bitmapout;
    unsigned short *dest;

    if (!path)
        HAL_ERROR("region", "Filename is empty!\n");
    if (!(file = fopen(path, "rb")))
        HAL_ERROR("region", "Opening the bitmap failed!\n");

    spng_ctx *ctx = spng_ctx_new(0);
    if (!ctx) {
        HAL_DANGER("server", "Constructing the PNG decoder context failed!\n");
        goto png_error;
    }

    spng_set_crc_action(ctx, SPNG_CRC_USE, SPNG_CRC_USE);
    spng_set_chunk_limits(ctx,
        app_config.web_server_thread_stack_size,
        app_config.web_server_thread_stack_size);
    spng_set_png_file(ctx, file);

    struct spng_ihdr ihdr;
    int err = spng_get_ihdr(ctx, &ihdr);
    if (err) {
        HAL_DANGER("server", "Parsing the PNG IHDR chunk failed!\nError: %s\n", spng_strerror(err));
        goto png_error;
    }
#ifdef DEBUG_IMAGE
    printf("[image] (PNG) width: %u, height: %u, depth: %u, color type: %u -> %s\n",
        ihdr.width, ihdr.height, ihdr.bit_depth, ihdr.color_type, color_type_str(ihdr.color_type));
    printf("        compress: %u, filter: %u, interl: %u\n",
        ihdr.compression_method, ihdr.filter_method, ihdr.interlace_method);
#endif

    struct spng_plte plte = {0};
    err = spng_get_plte(ctx, &plte);
    if (err && err != SPNG_ECHUNKAVAIL) {
        HAL_DANGER("server", "Parsing the PNG PLTE chunk failed!\nError: %s\n", spng_strerror(err));
        goto png_error;
    }
#ifdef DEBUG_IMAGE
    printf("        palette: %u\n", plte.n_entries);
#endif

    size_t bitmapsize;
    err = spng_decoded_image_size(ctx, SPNG_FMT_RGBA8, &bitmapsize);
    if (err) {
        HAL_DANGER("server", "Recovering the resulting image size failed!\nError: %s\n", spng_strerror(err));
        goto png_error;
    }

    bitmapdata = malloc(bitmapsize);
    if (!bitmapdata) {
        HAL_DANGER("server", "Allocating the PNG bitmap input buffer for size %u failed!\n", bitmapsize);
        goto png_error;
    }

    bitmapout = malloc(bitmapsize / 2);
    if (!bitmapout) {
        HAL_DANGER("server", "Allocating the PNG bitmap output buffer for size %u failed!\n", bitmapsize / 2);
        goto png_error;
    }

    err = spng_decode_image(ctx, bitmapdata, bitmapsize, SPNG_FMT_RGBA8, 0);
    if (!bitmapdata) {
        HAL_DANGER("server", "Decoding the PNG image failed!\nError: %s\n", spng_strerror(err));
        goto png_error;
    }

    dest = (unsigned short*)bitmapout;
    for (int i = 0; i < bitmapsize; i += 4) {
        *dest = ((bitmapdata[i + 3] & 0x80) << 8) | ((bitmapdata[i + 0] & 0xF8) << 7) |
            ((bitmapdata[i + 1] & 0xF8) << 2) | ((bitmapdata[i + 2] & 0xF8) >> 3);
        dest++;
    }
    free(bitmapdata);

    fclose(file);

    bitmap->data = bitmapout;
    bitmap->dim.width = ihdr.width;
    bitmap->dim.height = abs(ihdr.height);

    return EXIT_SUCCESS;

png_error:
    fclose(file);
    spng_ctx_free(ctx);
    free(bitmapdata);
    free(bitmapout);

    return EXIT_FAILURE;
}

int region_prepare_bitmap(char *path, hal_bitmap *bitmap) {
    bitmapfile bmpFile;
    bitmapinfo bmpInfo;
    bitmapfields bmpFields;
    static FILE *file;
    void *buffer, *start;
    unsigned int size;
    unsigned short *dest;
    unsigned char bpp, alpha, red, green, blue;
    char pos;

    if (region_open_bitmap(path, &file))
        return EXIT_FAILURE;

    if (region_parse_bitmap(&file, &bmpFile, &bmpInfo))
        HAL_ERROR("region", "Bitmap file \"%s\" cannot be processed!\n", path);

    if (bmpInfo.compression == 3 && fread(&bmpFields, 1, sizeof(bitmapfields), file) != sizeof(bitmapfields))
        HAL_ERROR("region", "Extracting the bitmap fields failed!\n");
    
    bpp = bmpInfo.bitCount / 8;
    size = bmpInfo.width * abs(bmpInfo.height);

    if (fseek(file, bmpFile.offBits, 0))
        HAL_ERROR("region", "Navigating to the bitmap image data failed!\n");
    if (!(buffer = malloc(size * bpp)))
        HAL_ERROR("region", "Allocating the bitmap input memory failed!\n");

    if (fread(buffer, 1, (unsigned int)(size * bpp), file) != 
        (unsigned int)(size * bpp))
        HAL_ERROR("region", "Reading the bitmap image data failed!\n");

    if (bmpInfo.height >= 0) {
        char *new  = malloc(size * bpp);
        int stride = bmpInfo.width * bpp;
        if (!new)
            HAL_ERROR("region", "Allocating the flipped bitmap memory failed!\n");
        for (int h = 0; h < bmpInfo.height; h++)
            memcpy(new + (h * stride),
                buffer + ((bmpInfo.height - 1) * stride) - (h * stride),
                stride);
        free(buffer);
        buffer = new;
    }

    if (!(bitmap->data = malloc(size * 2)))
        HAL_ERROR("region", "Allocating the destination buffer failed!\n");

    start = buffer;
    dest = bitmap->data;

    if (bmpInfo.compression != 3) {
        for (int i = 0; i < size; i++) {
            if ((pos = bmpInfo.bitCount) == 24)
                alpha = 0xFF;
            else
                alpha = (*((unsigned int*)start) >> (pos -= 8)) & 0xFF;
            red = (*((unsigned int*)start) >> (pos -= 8)) & 0xFF;
            green = (*((unsigned int*)start) >> (pos -= 8)) & 0xFF;
            blue = (*((unsigned int*)start) >> (pos -= 8)) & 0xFF;
            *dest = ((alpha & 0x80) << 8) | ((red & 0xF8) << 7) | ((green & 0xF8) << 2) | ((blue & 0xF8) >> 3);
            start += bpp;
            dest++;
        }
    } else {
        for (int i = 0; i < size; i++) {
            alpha = (*((unsigned int*)start) & bmpFields.alphaMask) >> __builtin_ctz(bmpFields.alphaMask);
            red = (*((unsigned int*)start) & bmpFields.redMask) >> __builtin_ctz(bmpFields.redMask);
            green = (*((unsigned int*)start) & bmpFields.greenMask) >> __builtin_ctz(bmpFields.greenMask);
            blue = (*((unsigned int*)start) & bmpFields.blueMask) >> __builtin_ctz(bmpFields.blueMask);
            *dest = ((alpha & 0x80) << 8) | ((red & 0xF8) << 7) | ((green & 0xF8) << 2) | ((blue & 0xF8) >> 3);
            start += bpp;
            dest++;
        }
    }
    free(buffer);

    fclose(file);

    bitmap->dim.width = bmpInfo.width;
    bitmap->dim.height = abs(bmpInfo.height);

    return EXIT_SUCCESS;
}

void *region_thread(void) {
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_region_init(); break;
        case HAL_PLATFORM_I6C: i6c_region_init(); break;
        case HAL_PLATFORM_M6:  m6_region_init(); break;
#endif
    }

    for (char id = 0; id < MAX_OSD; id++)
    {
        if (!EMPTY(osds[id].text) || !EMPTY(osds[id].img)) continue;
        osds[id].hand = -1;
        osds[id].color = DEF_COLOR;
        osds[id].opal = DEF_OPAL;
        osds[id].size = DEF_SIZE;
        osds[id].posx = DEF_POSX;
        osds[id].posy = DEF_POSY + (DEF_SIZE * 3 / 2) * id;
        osds[id].updt = 0;
        osds[id].persist = 1;
        strncpy(osds[id].font, DEF_FONT, sizeof(osds[id].font) - 1);
        osds[id].text[0] = '\0';
        osds[id].img[0] = '\0';
        osds[id].outl = DEF_OUTL;
        osds[id].thick = DEF_THICK;
        osds[id].bg = DEF_BG;
        osds[id].bgopal = 0;
        osds[id].pad = DEF_PAD;
    }

    // Optionally auto-add ISP debug overlay if enabled in config.
    // This runs after defaults/config have been applied and only uses free slots.
    region_setup_isp_debug_osd();

    while (keepRunning) {
        for (char id = 0; id < MAX_OSD; id++) {
            if (!EMPTY(osds[id].text))
            {
                char out[80];
                strncpy(out, osds[id].text, sizeof(out) - 1);
                if (strstr(out, "$"))
                {
                    region_fill_formatted(out);
                    osds[id].updt = 1;
                }

                if (osds[id].updt) {
                    char font_path[512];
                    if (region_resolve_font_path(osds[id].font, font_path, sizeof(font_path)) != 0) {
                        HAL_DANGER("region", "Font \"%s\" not found!\n", osds[id].font);
                        continue;
                    }
                    // Log resolved font path once per region when it changes (helps field debugging).
                    if (strcmp(osd_font_logged[(unsigned char)id], font_path) != 0) {
                        HAL_INFO("region", "reg%d font resolved: %s -> %s\n",
                            id, osds[id].font, font_path);
                        snprintf(osd_font_logged[(unsigned char)id],
                            sizeof(osd_font_logged[(unsigned char)id]), "%s", font_path);
                    }
                    // Background box is implemented where ARGB1555 overlay supports
                    // separate bgAlpha/fgAlpha (e.g. HiSilicon v4, SigmaStar i6c).
                    const int bg_enable = (osds[id].bgopal > 0) &&
                        (plat == HAL_PLATFORM_V4 ||
                         plat == HAL_PLATFORM_I6C ||
                         plat == HAL_PLATFORM_I6 ||
                         plat == HAL_PLATFORM_M6);
                    hal_bitmap bitmap = text_create_rendered(
                        font_path, osds[id].size, out,
                        osds[id].color, osds[id].outl, osds[id].thick,
                        osds[id].bg, osds[id].pad, bg_enable);
                    hal_rect rect = { .height = bitmap.dim.height, .width = bitmap.dim.width,
                        .x = osds[id].posx, .y = osds[id].posy };

                    // Size hysteresis: keep region size stable to avoid constant recreate spam.
                    hal_bitmap send_bmp = bitmap;
                    bool send_bmp_alloc = false;
                    if (plat == HAL_PLATFORM_V4 ||
                        plat == HAL_PLATFORM_I6 ||
                        plat == HAL_PLATFORM_I6C ||
                        plat == HAL_PLATFORM_M6) {
                        unsigned short cw = osd_rgn_w[(unsigned char)id];
                        unsigned short ch = osd_rgn_h[(unsigned char)id];
                        if (cw == 0 || ch == 0) {
                            osd_rgn_w[(unsigned char)id] = bitmap.dim.width;
                            osd_rgn_h[(unsigned char)id] = bitmap.dim.height;
                        } else {
                            const unsigned int cur_area = (unsigned int)cw * (unsigned int)ch;
                            const unsigned int new_area = (unsigned int)bitmap.dim.width * (unsigned int)bitmap.dim.height;
                            const bool need_grow = (bitmap.dim.width > cw) || (bitmap.dim.height > ch);
                            // On SigmaStar (i6/i6c/m6) dynamic overlays (ISP debug etc) can vary
                            // width noticeably, which would otherwise trigger frequent region
                            // recreation. Prefer "grow-only" sizing there and pad the bitmap.
                            const bool allow_shrink =
                                !(plat == HAL_PLATFORM_I6 ||
                                  plat == HAL_PLATFORM_I6C ||
                                  plat == HAL_PLATFORM_M6);
                            const bool shrink_big = allow_shrink && (new_area * 100u) <= (cur_area * 90u);
                            if (need_grow || shrink_big) {
                                osd_rgn_w[(unsigned char)id] = bitmap.dim.width;
                                osd_rgn_h[(unsigned char)id] = bitmap.dim.height;
                            } else {
                                // Keep existing region size and pad the bitmap to match.
                                send_bmp = region_pad_bitmap(&bitmap, cw, ch, &send_bmp_alloc);
                                rect.width = cw;
                                rect.height = ch;
                            }
                        }
                    }
                    hal_bitmap *bmp_to_use = &send_bmp;
                    if (!send_bmp.data)
                        bmp_to_use = &bitmap;

                    // Decide whether we need to (re)attach or just update bitmap.
                    bool need_attach =
                        !osd_attached[(unsigned char)id] ||
                        osd_logged_w[(unsigned char)id] != rect.width ||
                        osd_logged_h[(unsigned char)id] != rect.height ||
                        osd_logged_x[(unsigned char)id] != rect.x ||
                        osd_logged_y[(unsigned char)id] != rect.y ||
                        osd_logged_bgopal[(unsigned char)id] != osds[id].bgopal ||
                        osd_logged_opal[(unsigned char)id] != osds[id].opal;

                    // Log size/position changes once per change to help field debugging.
                    if (need_attach) {
                        HAL_INFO("region", "reg%d pos=%d,%d size=%ux%u bgopal=%d opal=%d",
                            id, rect.x, rect.y, rect.width, rect.height,
                            (int)osds[id].bgopal, (int)osds[id].opal);
                    }

                    int rgn_rc = 0;
                    if (need_attach) {
                    switch (plat) {
#if defined(__ARM_PCS_VFP)
                        case HAL_PLATFORM_I6:
                            rgn_rc = i6_region_create_ex(id, rect, osds[id].opal,
                                (osds[id].bgopal > 0) ? osds[id].bgopal : 0);
                            break;
                        case HAL_PLATFORM_I6C:
                            rgn_rc = i6c_region_create_ex(id, rect, osds[id].opal,
                                (osds[id].bgopal > 0) ? osds[id].bgopal : 0);
                            break;
                        case HAL_PLATFORM_M6:
                            rgn_rc = m6_region_create_ex(id, rect, osds[id].opal,
                                (osds[id].bgopal > 0) ? osds[id].bgopal : 0);
                            break;
                        case HAL_PLATFORM_RK:
                            rgn_rc = rk_region_create(id, rect, osds[id].opal);
                            break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
                        case HAL_PLATFORM_GM:
                            rgn_rc = gm_region_create(id, rect, osds[id].opal);
                            break;
                        case HAL_PLATFORM_V1:
                            rgn_rc = v1_region_create(id, rect, osds[id].opal);
                            break;
                        case HAL_PLATFORM_V2:
                            rgn_rc = v2_region_create(id, rect, osds[id].opal);
                            break;
                        case HAL_PLATFORM_V3:
                            rgn_rc = v3_region_create(id, rect, osds[id].opal);
                            break;
                        case HAL_PLATFORM_V4:
                            // For HiSilicon v4: allow semi-transparent background box via bgAlpha.
                            rgn_rc = v4_region_create_ex(id, rect, osds[id].opal,
                                (osds[id].bgopal > 0) ? osds[id].bgopal : 0);
                            break;
#elif defined(__mips__)
                        case HAL_PLATFORM_T31:
                            rgn_rc = t31_region_create(&osds[id].hand, rect, osds[id].opal);
                            break;
#endif
                        }
                        if (rgn_rc) {
                            HAL_ERROR("region", "reg%d attach/create failed (rc=%d) plat=%d\n",
                                id, rgn_rc, (int)plat);
                        } else {
                            osd_attached[(unsigned char)id] = true;
                            osd_logged_w[(unsigned char)id] = rect.width;
                            osd_logged_h[(unsigned char)id] = rect.height;
                            osd_logged_x[(unsigned char)id] = rect.x;
                            osd_logged_y[(unsigned char)id] = rect.y;
                            osd_logged_bgopal[(unsigned char)id] = osds[id].bgopal;
                            osd_logged_opal[(unsigned char)id] = osds[id].opal;
                        }
                    }

                    // If attached (either already or just now), update bitmap only.
                    if (!rgn_rc) {
                        switch (plat) {
#if defined(__ARM_PCS_VFP)
                            case HAL_PLATFORM_I6:
                                rgn_rc = i6_region_setbitmap(id, bmp_to_use);
                                break;
                            case HAL_PLATFORM_I6C:
                                rgn_rc = i6c_region_setbitmap(id, bmp_to_use);
                                break;
                            case HAL_PLATFORM_M6:
                                rgn_rc = m6_region_setbitmap(id, bmp_to_use);
                                break;
                            case HAL_PLATFORM_RK:
                                rgn_rc = rk_region_setbitmap(id, bmp_to_use);
                                break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
                            case HAL_PLATFORM_GM:
                                rgn_rc = gm_region_setbitmap(id, bmp_to_use);
                                break;
                            case HAL_PLATFORM_V1:
                                rgn_rc = v1_region_setbitmap(id, bmp_to_use);
                                break;
                            case HAL_PLATFORM_V2:
                                rgn_rc = v2_region_setbitmap(id, bmp_to_use);
                                break;
                            case HAL_PLATFORM_V3:
                                rgn_rc = v3_region_setbitmap(id, bmp_to_use);
                                break;
                            case HAL_PLATFORM_V4:
                                rgn_rc = v4_region_setbitmap(id, bmp_to_use);
                                break;
#elif defined(__mips__)
                            case HAL_PLATFORM_T31:
                                rgn_rc = t31_region_setbitmap(&osds[id].hand, bmp_to_use);
                                break;
#endif
                        }
                    }

                    if (rgn_rc) {
                        osd_attached[(unsigned char)id] = false;
                        HAL_ERROR("region", "reg%d set failed (rc=%d) plat=%d\n",
                            id, rgn_rc, (int)plat);
                    }
                    if (send_bmp_alloc && send_bmp.data && send_bmp.data != bitmap.data)
                        free(send_bmp.data);
                    if (bitmap.data)
                        free(bitmap.data);
                }
            }
            else if (EMPTY(osds[id].text) && osds[id].updt)
            {
                char img[64];
                if (EMPTY(osds[id].img))
                    sprintf(img, "/tmp/osd%d.bmp", id);
                else strncpy(img, osds[id].img, sizeof(osds[id].img) - 1);
                if (!access(img, F_OK))
                {
                    hal_bitmap bitmap;
                    int ret;
                    if (ENDS_WITH(img, ".png"))
                        ret = region_prepare_image(img, &bitmap);
                    else
                        ret = region_prepare_bitmap(img, &bitmap);
                    if (!ret)
                    {
                        hal_rect rect = { .height = bitmap.dim.height, .width = bitmap.dim.width,
                            .x = osds[id].posx, .y = osds[id].posy };
                        switch (plat) {
#if defined(__ARM_PCS_VFP)
                            case HAL_PLATFORM_I6:
                                i6_region_create_ex(id, rect, osds[id].opal,
                                    (osds[id].bgopal > 0) ? osds[id].bgopal : 0);
                                i6_region_setbitmap(id, &bitmap);
                                break;
                            case HAL_PLATFORM_I6C:
                                i6c_region_create_ex(id, rect, osds[id].opal,
                                    (osds[id].bgopal > 0) ? osds[id].bgopal : 0);
                                i6c_region_setbitmap(id, &bitmap);
                                break;
                            case HAL_PLATFORM_M6:
                                m6_region_create_ex(id, rect, osds[id].opal,
                                    (osds[id].bgopal > 0) ? osds[id].bgopal : 0);
                                m6_region_setbitmap(id, &bitmap);
                                break;
                            case HAL_PLATFORM_RK:
                                rk_region_create(id, rect, osds[id].opal);
                                rk_region_setbitmap(id, &bitmap);
                                break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
                            case HAL_PLATFORM_GM:
                                gm_region_create(id, rect, osds[id].opal);
                                gm_region_setbitmap(id, &bitmap);
                                break;
                            case HAL_PLATFORM_V1:
                                v1_region_create(id, rect, osds[id].opal);
                                v1_region_setbitmap(id, &bitmap);
                                break;
                            case HAL_PLATFORM_V2:
                                v2_region_create(id, rect, osds[id].opal);
                                v2_region_setbitmap(id, &bitmap);
                                break;
                            case HAL_PLATFORM_V3:
                                v3_region_create(id, rect, osds[id].opal);
                                v3_region_setbitmap(id, &bitmap);
                                break;
                            case HAL_PLATFORM_V4:
                            v4_region_create_ex(id, rect, osds[id].opal,
                                (osds[id].bgopal > 0) ? osds[id].bgopal : 0);
                                v4_region_setbitmap(id, &bitmap);
                                break;
#elif defined(__mips__)
                            case HAL_PLATFORM_T31:
                                t31_region_create(&osds[id].hand, rect, osds[id].opal);
                                t31_region_setbitmap(&osds[id].hand, &bitmap);
                                break;
#endif
                        }
                        free(bitmap.data);
                    }
                }
                else
                    switch (plat) {
#if defined(__ARM_PCS_VFP)
                        case HAL_PLATFORM_I6:  i6_region_destroy(id); break;
                        case HAL_PLATFORM_I6C: i6c_region_destroy(id); break;
                        case HAL_PLATFORM_M6:  m6_region_destroy(id); break;
                        case HAL_PLATFORM_RK:  rk_region_destroy(id); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
                        case HAL_PLATFORM_GM:  gm_region_destroy(id); break;
                        case HAL_PLATFORM_V1:  v1_region_destroy(id); break;
                        case HAL_PLATFORM_V2:  v2_region_destroy(id); break;
                        case HAL_PLATFORM_V3:  v3_region_destroy(id); break;
                        case HAL_PLATFORM_V4:  v4_region_destroy(id); break;
#elif defined(__mips__)
                        case HAL_PLATFORM_T31: t31_region_destroy(&osds[id].hand); break;
#endif
                    }
            }
            osds[id].updt = 0;
        }
        region_sleep_to_next_second();
    }

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_region_deinit(); break;
        case HAL_PLATFORM_I6C: i6c_region_deinit(); break;
        case HAL_PLATFORM_M6:  m6_region_deinit(); break;
#endif
    }
}

int start_region_handler() {
    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    size_t stacksize;
    pthread_attr_getstacksize(&thread_attr, &stacksize);
    size_t new_stacksize = 320 * 1024;
    if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
        HAL_DANGER("region", "Can't set stack size %zu\n", new_stacksize);
    if (pthread_create(
            &regionPid, &thread_attr, (void *(*)(void *))region_thread, NULL))
        HAL_DANGER("region", "Starting the handler thread failed!\n");
    if (pthread_attr_setstacksize(&thread_attr, stacksize))
        HAL_DANGER("region", "Can't set stack size %zu\n", stacksize);
    pthread_attr_destroy(&thread_attr);
}

void stop_region_handler() {
    pthread_join(regionPid, NULL);
}