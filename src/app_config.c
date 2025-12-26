#include "app_config.h"
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <strings.h>
#include <libfyaml.h>

struct AppConfig app_config;
// Canonical copy of time_format to persist into config; runtime `timefmt`
// may be manipulated elsewhere (OSD formatting), so we keep a clean copy here.
static char timefmt_cfg[sizeof(timefmt)] = DEF_TIMEFMT;

// Minimal path lookup for our config schema.
// We intentionally avoid `fy_node_by_path()` to keep dependencies smaller
// (it pulls in ypath/walker support). Supported forms:
//   "/a/b/c" (mapping keys only, no escapes, no indexes)
static inline struct fy_node *yaml_node(struct fy_document *fyd, const char *path) {
    if (!fyd || !path || *path != '/')
        return NULL;

    struct fy_node *cur = fy_document_root(fyd);
    if (!cur)
        return NULL;

    const char *p = path;
    while (*p == '/')
        p++;

    while (*p) {
        const char *seg = p;
        while (*p && *p != '/')
            p++;
        size_t seg_len = (size_t)(p - seg);
        while (*p == '/')
            p++;

        if (seg_len == 0)
            continue;

        if (fy_node_get_type(cur) != FYNT_MAPPING)
            return NULL;

        cur = fy_node_mapping_lookup_value_by_simple_key(cur, seg, seg_len);
        if (!cur)
            return NULL;
    }

    return cur;
}

static enum ConfigError yaml_get_scalar0(struct fy_document *fyd, const char *path, const char **out) {
    if (!out)
        return CONFIG_PARAM_INVALID_FORMAT;
    *out = NULL;

    struct fy_node *n = yaml_node(fyd, path);
    if (!n)
        return CONFIG_PARAM_NOT_FOUND;
    if (fy_node_get_type(n) != FYNT_SCALAR)
        return CONFIG_PARAM_INVALID_FORMAT;

    const char *s = fy_node_get_scalar0(n);
    if (!s)
        return CONFIG_PARAM_INVALID_FORMAT;

    *out = s;
    return CONFIG_OK;
}

static enum ConfigError yaml_get_string(struct fy_document *fyd, const char *path, char *dst, size_t dst_sz) {
    const char *s = NULL;
    enum ConfigError err = yaml_get_scalar0(fyd, path, &s);
    if (err != CONFIG_OK)
        return err;
    if (!dst || dst_sz == 0)
        return CONFIG_PARAM_INVALID_FORMAT;
    strncpy(dst, s, dst_sz - 1);
    dst[dst_sz - 1] = '\0';
    return CONFIG_OK;
}

static enum ConfigError yaml_get_bool(struct fy_document *fyd, const char *path, bool *out) {
    const char *s = NULL;
    enum ConfigError err = yaml_get_scalar0(fyd, path, &s);
    if (err != CONFIG_OK)
        return err;
    if (!out)
        return CONFIG_PARAM_INVALID_FORMAT;

    if (!strcasecmp(s, "true") || !strcasecmp(s, "yes") || !strcasecmp(s, "y") || !strcmp(s, "1")) {
        *out = true;
        return CONFIG_OK;
    }
    if (!strcasecmp(s, "false") || !strcasecmp(s, "no") || !strcasecmp(s, "n") || !strcmp(s, "0")) {
        *out = false;
        return CONFIG_OK;
    }
    return CONFIG_PARAM_ISNT_NUMBER;
}

static enum ConfigError yaml_get_int(struct fy_document *fyd, const char *path, int min, int max, int *out) {
    const char *s = NULL;
    enum ConfigError err = yaml_get_scalar0(fyd, path, &s);
    if (err != CONFIG_OK)
        return err;
    if (!out)
        return CONFIG_PARAM_INVALID_FORMAT;

    char *end = NULL;
    long v = strtol(s, &end, 0);
    if (end == s || (end && *end)) {
        // Accept bool-ish strings for integer values.
        if (!strcasecmp(s, "true")) v = 1;
        else if (!strcasecmp(s, "false")) v = 0;
        else return CONFIG_PARAM_ISNT_NUMBER;
    }
    if (v < min || v > max)
        return CONFIG_PARAM_ISNT_IN_RANGE;
    *out = (int)v;
    return CONFIG_OK;
}

static enum ConfigError yaml_get_uint(struct fy_document *fyd, const char *path, unsigned int min, unsigned int max, unsigned int *out) {
    const char *s = NULL;
    enum ConfigError err = yaml_get_scalar0(fyd, path, &s);
    if (err != CONFIG_OK)
        return err;
    if (!out)
        return CONFIG_PARAM_INVALID_FORMAT;

    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (end == s || (end && *end)) {
        if (!strcasecmp(s, "true")) v = 1;
        else if (!strcasecmp(s, "false")) v = 0;
        else return CONFIG_PARAM_ISNT_NUMBER;
    }
    if (v < min || v > max)
        return CONFIG_PARAM_ISNT_IN_RANGE;
    *out = (unsigned int)v;
    return CONFIG_OK;
}

// Decode a configured GPIO pin value.
// Supported encodings:
// - 999: disabled
// - N (0..pin_max): GPIO number
// - any negative value: treated as disabled (legacy configs sometimes used negatives)
static bool decode_cfg_pin(int cfg, int pin_max, int *pin_out) {
    if (pin_out) *pin_out = 0;
    if (cfg == 999)
        return false;
    if (cfg < 0)
        return false;
    if (cfg > pin_max)
        return false;
    if (pin_out) *pin_out = cfg;
    return true;
}

static enum ConfigError yaml_get_double(struct fy_document *fyd, const char *path, double min, double max, double *out) {
    const char *s = NULL;
    enum ConfigError err = yaml_get_scalar0(fyd, path, &s);
    if (err != CONFIG_OK)
        return err;
    if (!out)
        return CONFIG_PARAM_INVALID_FORMAT;

    char *end = NULL;
    double v = strtod(s, &end);
    if (end == s || (end && *end))
        return CONFIG_PARAM_ISNT_FLOAT;
    if (v < min || v > max)
        return CONFIG_PARAM_ISNT_IN_RANGE;
    *out = v;
    return CONFIG_OK;
}

static enum ConfigError yaml_get_string_list(struct fy_document *fyd, const char *path,
                                            char entries[][256], unsigned int max_entries,
                                            unsigned int *count_out) {
    if (count_out)
        *count_out = 0;
    struct fy_node *n = yaml_node(fyd, path);
    if (!n)
        return CONFIG_PARAM_NOT_FOUND;
    if (fy_node_get_type(n) != FYNT_SEQUENCE)
        return CONFIG_PARAM_INVALID_FORMAT;

    void *iter = NULL;
    struct fy_node *item;
    unsigned int count = 0;
    while ((item = fy_node_sequence_iterate(n, &iter)) != NULL) {
        if (count >= max_entries)
        break;
        if (fy_node_get_type(item) != FYNT_SCALAR)
            return CONFIG_PARAM_INVALID_FORMAT;
        const char *s = fy_node_get_scalar0(item);
        if (!s)
            return CONFIG_PARAM_INVALID_FORMAT;
        strncpy(entries[count], s, 255);
        entries[count][255] = '\0';
        count++;
    }
    if (count_out)
        *count_out = count;
    return CONFIG_OK;
}

static int yaml_map_add(struct fy_document *fyd, struct fy_node *map, const char *key, struct fy_node *value) {
    if (!fyd || !map || !key || !value)
        return -1;
    struct fy_node *k = fy_node_create_scalar_copy(fyd, key, FY_NT);
    if (!k)
        return -1;
    return fy_node_mapping_append(map, k, value);
}

static int yaml_map_add_str(struct fy_document *fyd, struct fy_node *map, const char *key, const char *val) {
    if (!val)
        val = "";
    struct fy_node *v = fy_node_create_scalar_copy(fyd, val, FY_NT);
    if (!v)
        return -1;
    return yaml_map_add(fyd, map, key, v);
}

// Create a double-quoted scalar node to keep values that start with special
// characters (e.g. '%') YAML-safe when saving configs.
static int yaml_map_add_quoted_str(struct fy_document *fyd, struct fy_node *map, const char *key, const char *val) {
    if (!val)
        val = "";

    // Worst case each character is escaped, plus surrounding quotes and NUL.
    char buf[sizeof(timefmt) * 2 + 3];
    size_t pos = 0;

    buf[pos++] = '"';
    for (const char *s = val; *s && pos + 2 < sizeof(buf); s++) {
        if (*s == '"' || *s == '\\') {
            buf[pos++] = '\\';
            if (pos + 1 >= sizeof(buf))
                break;
        }
        buf[pos++] = *s;
    }
    buf[pos++] = '"';
    buf[pos] = '\0';

    struct fy_node *v = fy_node_build_from_string(fyd, buf, FY_NT);
    if (!v)
        return -1;
    return yaml_map_add(fyd, map, key, v);
}

static int yaml_map_add_scalarf(struct fy_document *fyd, struct fy_node *map, const char *key, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    struct fy_node *v = fy_node_create_vscalarf(fyd, fmt, ap);
    va_end(ap);
    if (!v)
        return -1;
    return yaml_map_add(fyd, map, key, v);
}

// Ensure timefmt stays printable and null-terminated before persisting.
// Returns true if the input contained any invalid bytes or was empty (and got defaulted).
static bool sanitize_timefmt(const char *src, char *dst) {
    const char *input = src ? src : "";
    const size_t max_len = sizeof(timefmt) - 1;
    size_t raw_len = strnlen(input, max_len);

    bool changed = false;
    size_t out = 0;
    for (size_t i = 0; i < raw_len && out + 1 < sizeof(timefmt); i++) {
        unsigned char c = (unsigned char)input[i];
        if (c >= 0x20 && c <= 0x7e) {
            dst[out++] = (char)c;
        } else {
            changed = true;
        }
    }
    dst[out] = '\0';

    if (out == 0) {
        strncpy(dst, DEF_TIMEFMT, sizeof(timefmt) - 1);
        dst[sizeof(timefmt) - 1] = '\0';
        changed = true;
    }

    if (!changed && raw_len != out)
        changed = true;

    return changed;
}

// Update both runtime (timefmt) and canonical (timefmt_cfg) copies.
// Returns true if the provided value was altered during sanitization.
bool timefmt_set(const char *src) {
    char clean[sizeof(timefmt)];
    bool cleaned = sanitize_timefmt(src, clean);
    strncpy(timefmt, clean, sizeof(timefmt) - 1);
    timefmt[sizeof(timefmt) - 1] = '\0';
    strncpy(timefmt_cfg, clean, sizeof(timefmt_cfg) - 1);
    timefmt_cfg[sizeof(timefmt_cfg) - 1] = '\0';
    return cleaned;
}

// Ensure runtime buffer is printable; if not, restore from canonical copy.
void timefmt_repair_runtime(void) {
    char clean[sizeof(timefmt)];
    bool broken = sanitize_timefmt(timefmt, clean);
    if (broken) {
        // Prefer canonical clean copy; fallback to sanitized runtime.
        const char *src = (!EMPTY(timefmt_cfg)) ? timefmt_cfg : clean;
        strncpy(timefmt, src, sizeof(timefmt) - 1);
        timefmt[sizeof(timefmt) - 1] = '\0';
    }
}

// Best-effort repair for configs that became unparseable due to malformed UTF-8,
// typically caused by writing garbage into a double-quoted scalar (time_format).
// This rewrites the file to ASCII (replacing non-ASCII bytes with '?') and forces
// time_format to a sane default. Returns 0 on success, -1 on failure.
static int repair_divinus_yaml_time_format(const char *conf_path) {
    if (!conf_path) return -1;

    FILE *in = fopen(conf_path, "rb");
    if (!in) return -1;

    if (fseek(in, 0, SEEK_END) != 0) { fclose(in); return -1; }
    long sz_l = ftell(in);
    if (sz_l < 0 || sz_l > (1024 * 1024)) { fclose(in); return -1; } // 1MB safety cap
    size_t sz = (size_t)sz_l;
    if (fseek(in, 0, SEEK_SET) != 0) { fclose(in); return -1; }

    unsigned char *buf = (unsigned char *)malloc(sz ? sz : 1);
    if (!buf) { fclose(in); return -1; }
    if (sz && fread(buf, 1, sz, in) != sz) { free(buf); fclose(in); return -1; }
    fclose(in);

    // Output can be slightly larger due to replacement line.
    unsigned char *out = (unsigned char *)malloc(sz + 256);
    if (!out) { free(buf); return -1; }
    size_t o = 0;

    size_t i = 0;
    while (i < sz) {
        size_t line_start = i;
        size_t line_end = i;
        while (line_end < sz && buf[line_end] != '\n') line_end++;
        size_t nl_end = (line_end < sz && buf[line_end] == '\n') ? (line_end + 1) : line_end;

        // Find key at line start (after indent).
        size_t k = line_start;
        while (k < line_end && (buf[k] == ' ' || buf[k] == '\t')) k++;

        const char key[] = "time_format";
        const size_t key_len = sizeof(key) - 1;
        bool is_timefmt = false;
        if (k + key_len < line_end && memcmp(buf + k, key, key_len) == 0) {
            size_t p = k + key_len;
            while (p < line_end && (buf[p] == ' ' || buf[p] == '\t')) p++;
            if (p < line_end && buf[p] == ':')
                is_timefmt = true;
        }

        if (is_timefmt) {
            // Preserve indentation (ASCII-only).
            for (size_t j = line_start; j < k && o + 1 < sz + 256; j++) {
                unsigned char c = buf[j];
                if (c == '\t' || c == ' ')
                    out[o++] = c;
                else
                    out[o++] = ' ';
            }

            // Write fixed line.
            const char *fixed = "time_format: \"" DEF_TIMEFMT "\"";
            for (const char *s = fixed; *s && o + 1 < sz + 256; s++)
                out[o++] = (unsigned char)*s;
            if (o + 1 < sz + 256)
                out[o++] = '\n';

            // Skip original scalar; handle broken multi-line quoted values.
            size_t p = k + key_len;
            while (p < line_end && buf[p] != ':') p++;
            if (p < line_end) p++; // after ':'
            while (p < line_end && (buf[p] == ' ' || buf[p] == '\t')) p++;

            bool started_quote = (p < line_end && buf[p] == '"');
            if (started_quote) {
                // Look for closing quote on this line.
                bool esc = false, closed = false;
                for (size_t q = p + 1; q < line_end; q++) {
                    unsigned char c = buf[q];
                    if (esc) { esc = false; continue; }
                    if (c == '\\') { esc = true; continue; }
                    if (c == '"') { closed = true; break; }
                }

                if (!closed) {
                    // Skip following lines until we find an unescaped closing quote.
                    i = nl_end;
                    bool esc2 = false;
                    while (i < sz) {
                        unsigned char c = buf[i++];
                        if (c == '\n') esc2 = false; // reset at line boundaries
                        if (esc2) { esc2 = false; continue; }
                        if (c == '\\') { esc2 = true; continue; }
                        if (c == '"') {
                            // Skip until end of line after closing quote.
                            while (i < sz && buf[i] != '\n') i++;
                            if (i < sz && buf[i] == '\n') i++;
                            break;
                        }
                    }
                    continue;
                }
            }

            // Normal (single-line) case: just skip this line.
            i = nl_end;
            continue;
        }

        // Copy line, but force ASCII to avoid malformed UTF-8 elsewhere.
        for (size_t j = line_start; j < nl_end && o + 1 < sz + 256; j++) {
            unsigned char c = buf[j];
            if (c == '\n' || c == '\r' || c == '\t') {
                out[o++] = c;
            } else if (c >= 0x20 && c < 0x80) {
                out[o++] = c;
            } else {
                out[o++] = '?';
            }
        }

        i = nl_end;
    }

    free(buf);

    // Atomic replace.
    char tmp_template[PATH_MAX];
    snprintf(tmp_template, sizeof(tmp_template), "%s.fixXXXXXX", conf_path);
    int tfd = mkstemp(tmp_template);
    if (tfd < 0) { free(out); return -1; }
    FILE *of = fdopen(tfd, "wb");
    if (!of) { close(tfd); remove(tmp_template); free(out); return -1; }

    size_t wrote = fwrite(out, 1, o, of);
    free(out);
    int flush_rc = fflush(of);
    int fsync_rc = fsync(fileno(of));
    int close_rc = fclose(of);
    if (wrote != o || flush_rc || fsync_rc || close_rc) {
        remove(tmp_template);
        return -1;
    }

    if (rename(tmp_template, conf_path) != 0) {
        remove(tmp_template);
        return -1;
    }

    return 0;
}

int save_app_config(void) {
    const char *conf_path = DIVINUS_CONFIG_PATH;

    // If runtime buffer was corrupted, restore from the clean copy to avoid
    // persisting junk into config.
    timefmt_repair_runtime();
    if (EMPTY(timefmt_cfg))
        timefmt_set(DEF_TIMEFMT);
    // Even if memory corruption touched timefmt_cfg, re-sanitize right before saving.
    timefmt_set(timefmt_cfg);

    // Atomic save: write into a temp file in the same directory, then rename over target.
    // This avoids partially-written configs and removes the need for persistent *.bak files.
    char tmp_template[PATH_MAX];
    snprintf(tmp_template, sizeof(tmp_template), "%s.tmpXXXXXX", conf_path);
    int tfd = mkstemp(tmp_template);
    if (tfd < 0)
        HAL_ERROR("app_config", "Can't create temp config '%s': %s\n", tmp_template, strerror(errno));

    FILE *file = fdopen(tfd, "w");
    if (!file) {
        close(tfd);
        remove(tmp_template);
        HAL_ERROR("app_config", "Can't open temp config '%s': %s\n", tmp_template, strerror(errno));
    }

    struct fy_document *fyd = fy_document_create(NULL);
    if (!fyd) {
        fclose(file);
        remove(tmp_template);
        HAL_ERROR("app_config", "Can't create YAML document\n");
    }

    struct fy_node *root = fy_node_create_mapping(fyd);
    if (!root || fy_document_set_root(fyd, root)) {
        fy_document_destroy(fyd);
        fclose(file);
        remove(tmp_template);
        HAL_ERROR("app_config", "Can't build YAML root mapping\n");
    }

    // system
    struct fy_node *system = fy_node_create_mapping(fyd);
    if (!system || yaml_map_add(fyd, root, "system", system)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, system, "sensor_config", app_config.sensor_config)) goto EMIT_FAIL;
    if (!EMPTY(app_config.iq_config))
        if (yaml_map_add_str(fyd, system, "iq_config", app_config.iq_config)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, system, "web_port", "%u", (unsigned int)app_config.web_port)) goto EMIT_FAIL;
    if (!EMPTY(app_config.web_bind))
        if (yaml_map_add_str(fyd, system, "web_bind", app_config.web_bind)) goto EMIT_FAIL;
    {
        struct fy_node *wl = fy_node_create_sequence(fyd);
        bool any = false;
        for (int i = 0; i < (int)(sizeof(app_config.web_whitelist) / sizeof(app_config.web_whitelist[0])); i++) {
            if (EMPTY(app_config.web_whitelist[i])) continue;
            any = true;
            if (fy_node_sequence_append(wl, fy_node_create_scalar_copy(fyd, app_config.web_whitelist[i], FY_NT)))
                goto EMIT_FAIL;
        }
        if (any) {
            if (yaml_map_add(fyd, system, "web_whitelist", wl)) goto EMIT_FAIL;
        }
    }
    if (yaml_map_add_str(fyd, system, "web_enable_auth", app_config.web_enable_auth ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, system, "web_auth_user", app_config.web_auth_user)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, system, "web_auth_pass", app_config.web_auth_pass)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, system, "web_auth_skiplocal", app_config.web_auth_skiplocal ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, system, "web_enable_static", app_config.web_enable_static ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, system, "isp_thread_stack_size", "%u", app_config.isp_thread_stack_size)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, system, "venc_stream_thread_stack_size", "%u", app_config.venc_stream_thread_stack_size)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, system, "web_server_thread_stack_size", "%u", app_config.web_server_thread_stack_size)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, system, "night_thread_stack_size", "%u", app_config.night_thread_stack_size)) goto EMIT_FAIL;
    // Use canonical copy to persist (runtime buffer may be touched elsewhere).
    if (!EMPTY(timefmt_cfg))
        if (yaml_map_add_quoted_str(fyd, system, "time_format", timefmt_cfg)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, system, "watchdog", "%u", app_config.watchdog)) goto EMIT_FAIL;

    // night_mode
    struct fy_node *night_mode = fy_node_create_mapping(fyd);
    if (!night_mode || yaml_map_add(fyd, root, "night_mode", night_mode)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, night_mode, "enable", app_config.night_mode_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, night_mode, "manual", app_config.night_mode_manual ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, night_mode, "grayscale", app_config.night_mode_grayscale ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, night_mode, "ir_sensor_pin", "%d", app_config.ir_sensor_pin)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, night_mode, "check_interval_s", "%u", app_config.check_interval_s)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, night_mode, "ir_cut_pin1", "%d", app_config.ir_cut_pin1)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, night_mode, "ir_cut_pin2", "%d", app_config.ir_cut_pin2)) goto EMIT_FAIL;
    if (app_config.isp_lum_low >= 0)
        if (yaml_map_add_scalarf(fyd, night_mode, "isp_lum_low", "%d", app_config.isp_lum_low)) goto EMIT_FAIL;
    if (app_config.isp_lum_hi >= 0)
        if (yaml_map_add_scalarf(fyd, night_mode, "isp_lum_hi", "%d", app_config.isp_lum_hi)) goto EMIT_FAIL;
    if (app_config.isp_iso_low >= 0)
        if (yaml_map_add_scalarf(fyd, night_mode, "isp_iso_low", "%d", app_config.isp_iso_low)) goto EMIT_FAIL;
    if (app_config.isp_iso_hi >= 0)
        if (yaml_map_add_scalarf(fyd, night_mode, "isp_iso_hi", "%d", app_config.isp_iso_hi)) goto EMIT_FAIL;
    if (app_config.isp_exptime_low >= 0)
        if (yaml_map_add_scalarf(fyd, night_mode, "isp_exptime_low", "%d", app_config.isp_exptime_low)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, night_mode, "isp_switch_lockout_s", "%u", app_config.isp_switch_lockout_s)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, night_mode, "ir_led_pin", "%d", app_config.ir_led_pin)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, night_mode, "white_led_pin", "%d", app_config.white_led_pin)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, night_mode, "pin_switch_delay_us", "%u", app_config.pin_switch_delay_us)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, night_mode, "adc_device", app_config.adc_device)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, night_mode, "adc_threshold", "%d", app_config.adc_threshold)) goto EMIT_FAIL;

    // isp
    struct fy_node *isp = fy_node_create_mapping(fyd);
    if (!isp || yaml_map_add(fyd, root, "isp", isp)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, isp, "sensor_mirror", app_config.sensor_mirror ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, isp, "sensor_flip", app_config.sensor_flip ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, isp, "mirror", app_config.mirror ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, isp, "flip", app_config.flip ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, isp, "antiflicker", "%d", app_config.antiflicker)) goto EMIT_FAIL;

    // mdns
    struct fy_node *mdns = fy_node_create_mapping(fyd);
    if (!mdns || yaml_map_add(fyd, root, "mdns", mdns)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, mdns, "enable", app_config.mdns_enable ? "true" : "false")) goto EMIT_FAIL;

    // onvif
    struct fy_node *onvif = fy_node_create_mapping(fyd);
    if (!onvif || yaml_map_add(fyd, root, "onvif", onvif)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, onvif, "enable", app_config.onvif_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, onvif, "enable_auth", app_config.onvif_enable_auth ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, onvif, "auth_user", app_config.onvif_auth_user)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, onvif, "auth_pass", app_config.onvif_auth_pass)) goto EMIT_FAIL;

    // rtsp
    struct fy_node *rtsp = fy_node_create_mapping(fyd);
    if (!rtsp || yaml_map_add(fyd, root, "rtsp", rtsp)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, rtsp, "enable", app_config.rtsp_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, rtsp, "port", "%d", app_config.rtsp_port)) goto EMIT_FAIL;
    if (!EMPTY(app_config.rtsp_bind))
        if (yaml_map_add_str(fyd, rtsp, "bind", app_config.rtsp_bind)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, rtsp, "enable_auth", app_config.rtsp_enable_auth ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, rtsp, "auth_user", app_config.rtsp_auth_user)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, rtsp, "auth_pass", app_config.rtsp_auth_pass)) goto EMIT_FAIL;

    // record
    struct fy_node *record = fy_node_create_mapping(fyd);
    if (!record || yaml_map_add(fyd, root, "record", record)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, record, "enable", app_config.record_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, record, "continuous", app_config.record_continuous ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, record, "path", app_config.record_path)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, record, "filename", app_config.record_filename)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, record, "segment_duration", "%d", app_config.record_segment_duration)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, record, "segment_size", "%d", app_config.record_segment_size)) goto EMIT_FAIL;

    // stream
    struct fy_node *stream = fy_node_create_mapping(fyd);
    if (!stream || yaml_map_add(fyd, root, "stream", stream)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, stream, "enable", app_config.stream_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, stream, "udp_srcport", "%u", (unsigned int)app_config.stream_udp_srcport)) goto EMIT_FAIL;
    {
        struct fy_node *d = fy_node_create_sequence(fyd);
        bool any = false;
        for (int i = 0; i < (int)(sizeof(app_config.stream_dests) / sizeof(app_config.stream_dests[0])); i++) {
            if (EMPTY(app_config.stream_dests[i])) continue;
            any = true;
            if (fy_node_sequence_append(d, fy_node_create_scalar_copy(fyd, app_config.stream_dests[i], FY_NT)))
                goto EMIT_FAIL;
        }
        if (any) {
            if (yaml_map_add(fyd, stream, "dest", d)) goto EMIT_FAIL;
        }
    }

    // audio
    struct fy_node *audio = fy_node_create_mapping(fyd);
    if (!audio || yaml_map_add(fyd, root, "audio", audio)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, audio, "enable", app_config.audio_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, audio, "mute", app_config.audio_mute ? "true" : "false")) goto EMIT_FAIL;
    // MP3 support removed; keep config AAC-only.
    if (yaml_map_add_str(fyd, audio, "codec", "AAC")) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "bitrate", "%u", app_config.audio_bitrate)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "gain", "%d", app_config.audio_gain)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "srate", "%u", app_config.audio_srate)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "channels", "%u", (unsigned int)app_config.audio_channels)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "aac_quantqual", "%u", app_config.audio_aac_quantqual)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "aac_bandwidth", "%u", app_config.audio_aac_bandwidth)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, audio, "aac_tns", app_config.audio_aac_tns ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, audio, "speex_enable", app_config.audio_speex_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, audio, "speex_denoise", app_config.audio_speex_denoise ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, audio, "speex_agc", app_config.audio_speex_agc ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, audio, "speex_vad", app_config.audio_speex_vad ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, audio, "speex_dereverb", app_config.audio_speex_dereverb ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "speex_frame_size", "%u", app_config.audio_speex_frame_size)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "speex_noise_suppress_db", "%d", app_config.audio_speex_noise_suppress_db)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "speex_agc_level", "%d", app_config.audio_speex_agc_level)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "speex_agc_increment", "%d", app_config.audio_speex_agc_increment)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "speex_agc_decrement", "%d", app_config.audio_speex_agc_decrement)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "speex_agc_max_gain_db", "%d", app_config.audio_speex_agc_max_gain_db)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "speex_vad_prob_start", "%d", app_config.audio_speex_vad_prob_start)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, audio, "speex_vad_prob_continue", "%d", app_config.audio_speex_vad_prob_continue)) goto EMIT_FAIL;

    // mp4
    struct fy_node *mp4 = fy_node_create_mapping(fyd);
    if (!mp4 || yaml_map_add(fyd, root, "mp4", mp4)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, mp4, "enable", app_config.mp4_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, mp4, "codec", app_config.mp4_codecH265 ? "H.265" :
            (app_config.mp4_h264_plus ? "H.264+" : "H.264"))) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, mp4, "mode", "%u", app_config.mp4_mode)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, mp4, "width", "%u", app_config.mp4_width)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, mp4, "height", "%u", app_config.mp4_height)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, mp4, "fps", "%u", app_config.mp4_fps)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, mp4, "gop", "%u", app_config.mp4_gop)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, mp4, "profile", "%u", app_config.mp4_profile)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, mp4, "bitrate", "%u", app_config.mp4_bitrate)) goto EMIT_FAIL;

    // osd
    struct fy_node *osd = fy_node_create_mapping(fyd);
    if (!osd || yaml_map_add(fyd, root, "osd", osd)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, osd, "enable", app_config.osd_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, osd, "isp_debug", app_config.osd_isp_debug ? "true" : "false")) goto EMIT_FAIL;
    for (char i = 0; i < MAX_OSD; i++) {
        // Skip runtime-only overlays (e.g. auto ISP debug lines).
        if (!osds[i].persist) continue;
        char imgEmpty = EMPTY(osds[i].img);
        char textEmpty = EMPTY(osds[i].text);
        if (imgEmpty && textEmpty) continue;
    
        char key[32];
        if (!imgEmpty) {
            snprintf(key, sizeof(key), "reg%d_img", i);
            if (yaml_map_add_str(fyd, osd, key, osds[i].img)) goto EMIT_FAIL;
        }
        if (!textEmpty) {
            snprintf(key, sizeof(key), "reg%d_text", i);
            if (yaml_map_add_str(fyd, osd, key, osds[i].text)) goto EMIT_FAIL;
        }
        snprintf(key, sizeof(key), "reg%d_font", i);
        if (yaml_map_add_str(fyd, osd, key, osds[i].font)) goto EMIT_FAIL;
        snprintf(key, sizeof(key), "reg%d_opal", i);
        if (yaml_map_add_scalarf(fyd, osd, key, "%u", (unsigned int)osds[i].opal)) goto EMIT_FAIL;
        snprintf(key, sizeof(key), "reg%d_posx", i);
        if (yaml_map_add_scalarf(fyd, osd, key, "%d", (int)osds[i].posx)) goto EMIT_FAIL;
        snprintf(key, sizeof(key), "reg%d_posy", i);
        if (yaml_map_add_scalarf(fyd, osd, key, "%d", (int)osds[i].posy)) goto EMIT_FAIL;
        snprintf(key, sizeof(key), "reg%d_size", i);
        if (yaml_map_add_scalarf(fyd, osd, key, "%.1f", osds[i].size)) goto EMIT_FAIL;
        snprintf(key, sizeof(key), "reg%d_color", i);
        if (yaml_map_add_scalarf(fyd, osd, key, "0x%04x", (unsigned int)osds[i].color)) goto EMIT_FAIL;
        snprintf(key, sizeof(key), "reg%d_outl", i);
        if (yaml_map_add_scalarf(fyd, osd, key, "0x%04x", (unsigned int)osds[i].outl)) goto EMIT_FAIL;
        snprintf(key, sizeof(key), "reg%d_thick", i);
        if (yaml_map_add_scalarf(fyd, osd, key, "%.1f", osds[i].thick)) goto EMIT_FAIL;
        if (osds[i].bgopal > 0) {
            snprintf(key, sizeof(key), "reg%d_bg", i);
            if (yaml_map_add_scalarf(fyd, osd, key, "0x%04x", (unsigned int)(osds[i].bg & 0x7FFF))) goto EMIT_FAIL;
            snprintf(key, sizeof(key), "reg%d_bgopal", i);
            if (yaml_map_add_scalarf(fyd, osd, key, "%d", (int)osds[i].bgopal)) goto EMIT_FAIL;
            snprintf(key, sizeof(key), "reg%d_pad", i);
            if (yaml_map_add_scalarf(fyd, osd, key, "%d", (int)osds[i].pad)) goto EMIT_FAIL;
        }
    }

    // jpeg
    struct fy_node *jpeg = fy_node_create_mapping(fyd);
    if (!jpeg || yaml_map_add(fyd, root, "jpeg", jpeg)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, jpeg, "enable", app_config.jpeg_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, jpeg, "osd_enable", app_config.jpeg_osd_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, jpeg, "grayscale_night", app_config.jpeg_grayscale_night ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, jpeg, "mode", "%u", app_config.jpeg_mode)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, jpeg, "width", "%u", app_config.jpeg_width)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, jpeg, "height", "%u", app_config.jpeg_height)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, jpeg, "fps", "%u", app_config.jpeg_fps)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, jpeg, "qfactor", "%u", app_config.jpeg_qfactor)) goto EMIT_FAIL;

    // http_post
    struct fy_node *http_post = fy_node_create_mapping(fyd);
    if (!http_post || yaml_map_add(fyd, root, "http_post", http_post)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, http_post, "enable", app_config.http_post_enable ? "true" : "false")) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, http_post, "host", app_config.http_post_host)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, http_post, "url", app_config.http_post_url)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, http_post, "login", app_config.http_post_login)) goto EMIT_FAIL;
    if (yaml_map_add_str(fyd, http_post, "password", app_config.http_post_password)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, http_post, "width", "%u", app_config.http_post_width)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, http_post, "height", "%u", app_config.http_post_height)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, http_post, "interval", "%u", app_config.http_post_interval)) goto EMIT_FAIL;
    if (yaml_map_add_scalarf(fyd, http_post, "qfactor", "%u", app_config.http_post_qfactor)) goto EMIT_FAIL;

    if (fy_emit_document_to_fp(fyd, FYECF_MODE_BLOCK | FYECF_INDENT_2 | FYECF_WIDTH_INF, file)) {
        goto EMIT_FAIL;
    }
    fy_document_destroy(fyd);

    // Ensure content hits storage before we swap the file name.
    int flush_rc = fflush(file);
    int fsync_rc = fsync(fileno(file));
    int close_rc = fclose(file);
    if (flush_rc || fsync_rc || close_rc) {
        remove(tmp_template);
        HAL_ERROR("app_config", "Failed to write config (flush=%d fsync=%d close=%d)\n",
            flush_rc, fsync_rc, close_rc);
    }

    // Atomic replacement (POSIX rename).
    if (rename(tmp_template, conf_path)) {
        remove(tmp_template);
        HAL_ERROR("app_config", "Failed to replace config '%s': %s\n", conf_path, strerror(errno));
    }

    return EXIT_SUCCESS;

EMIT_FAIL:
    fy_document_destroy(fyd);
    fclose(file);
    remove(tmp_template);
    HAL_ERROR("app_config", "Failed to build/emit YAML config\n");
}

enum ConfigError parse_app_config(void) {
    memset(&app_config, 0, sizeof(struct AppConfig));
    bool timefmt_cleaned = false;

    // Initialize OSD defaults early so partial configs (e.g. only reg0_text)
    // still render with sane font/color/position/outline.
    for (char i = 0; i < MAX_OSD; i++) {
        osds[i].hand = -1;
        osds[i].color = DEF_COLOR;
        osds[i].opal = DEF_OPAL;
        osds[i].size = DEF_SIZE;
        osds[i].posx = DEF_POSX;
        osds[i].posy = DEF_POSY + (DEF_SIZE * 3 / 2) * i;
        osds[i].updt = 0;
        osds[i].persist = 1;
        strncpy(osds[i].font, DEF_FONT, sizeof(osds[i].font) - 1);
        osds[i].font[sizeof(osds[i].font) - 1] = '\0';
        osds[i].text[0] = '\0';
        osds[i].img[0] = '\0';
        osds[i].outl = DEF_OUTL;
        osds[i].thick = DEF_THICK;
        osds[i].bg = DEF_BG;
        osds[i].bgopal = 0;
        osds[i].pad = DEF_PAD;
    }

    app_config.web_port = 8080;
    app_config.web_bind[0] = '\0';
    *app_config.web_whitelist[0] = '\0';
    app_config.web_enable_auth = false;
    app_config.web_auth_skiplocal = false;
    app_config.web_enable_static = false;
    app_config.isp_thread_stack_size = 16 * 1024;
    app_config.venc_stream_thread_stack_size = 16 * 1024;
    app_config.web_server_thread_stack_size = 32 * 1024;
    // Night thread can call into ISP/IQ reload logic which tends to be stack-hungry
    // on some SDKs (e.g. hisi/v4). Use a safer default than 16KB.
    app_config.night_thread_stack_size = 64 * 1024;
    app_config.watchdog = 0;

    app_config.mdns_enable = false;

    app_config.osd_enable = false;
    app_config.osd_isp_debug = false;

    app_config.onvif_enable = false;
    app_config.onvif_enable_auth = false;
    app_config.onvif_auth_user[0] = '\0';
    app_config.onvif_auth_pass[0] = '\0';

    app_config.rtsp_enable = false;
    app_config.rtsp_port = 554;
    app_config.rtsp_bind[0] = '\0';
    app_config.rtsp_enable_auth = false;
    app_config.rtsp_auth_user[0] = '\0';
    app_config.rtsp_auth_pass[0] = '\0';

    app_config.record_enable = false;
    app_config.record_continuous = false;
    app_config.record_filename[0] = '\0';
    strcpy(app_config.record_path, "/mnt/sdcard/recordings");
    app_config.record_segment_duration = 0;
    app_config.record_segment_size = 0;

    app_config.stream_enable = false;
    app_config.stream_udp_srcport = 0;
    *app_config.stream_dests[0] = '\0';

    app_config.sensor_config[0] = 0;
    app_config.iq_config[0] = 0;
    app_config.audio_enable = false;
    app_config.audio_mute = false;
    // MP3 support removed; AAC-only.
    app_config.audio_codec = HAL_AUDCODEC_AAC;
    app_config.audio_bitrate = 128;
    // Default audio gain:
    // - hisi/v4 uses 0..100 scale, where 50 is unity gain
    // - other platforms interpret it as dB and use 0 as unity gain
    app_config.audio_gain = (plat == HAL_PLATFORM_V4) ? 50 : 0;
    app_config.audio_srate = 48000;
    app_config.audio_channels = 1;
    // AAC (FAAC) advanced options (ignored unless codec=AAC)
    app_config.audio_aac_quantqual = 0;   // 0 = disabled (use bitrate mode)
    app_config.audio_aac_bandwidth = 0;   // 0 = auto
    app_config.audio_aac_tns = false;
    // SpeexDSP preprocess defaults (used only when codec=AAC)
    app_config.audio_speex_enable = true;
    app_config.audio_speex_denoise = true;
    app_config.audio_speex_agc = true;
    app_config.audio_speex_vad = true;
    app_config.audio_speex_dereverb = false; // heavy / experimental in upstream
    // 0 = auto (computed at runtime as srate/50 i.e. 20ms).
    app_config.audio_speex_frame_size = 0;
    app_config.audio_speex_noise_suppress_db = -20;
    app_config.audio_speex_agc_level = 24000;
    app_config.audio_speex_agc_increment = 12;
    app_config.audio_speex_agc_decrement = 40;
    app_config.audio_speex_agc_max_gain_db = 30;
    app_config.audio_speex_vad_prob_start = 60;
    app_config.audio_speex_vad_prob_continue = 45;
    app_config.mp4_enable = false;

    // JPEG/MJPEG stream (multipart/x-mixed-replace). Snapshots use last MJPEG frame.
    app_config.jpeg_enable = false;
    app_config.jpeg_osd_enable = true;
    // Prefer grayscale OSD at night; this is independent from runtime timefmt handling.
    // (Field renamed by user; keep defaults aligned.)
    app_config.jpeg_grayscale_night = true;
    app_config.jpeg_fps = 15;
    app_config.jpeg_width = 640;
    app_config.jpeg_height = 480;
    app_config.jpeg_mode = HAL_VIDMODE_QP;
    app_config.jpeg_qfactor = 80;

    app_config.sensor_mirror = false;
    app_config.sensor_flip = false;
    app_config.mirror = false;
    app_config.flip = false;
    app_config.antiflicker = 0;

    app_config.night_mode_enable = false;
    app_config.night_mode_manual = false;
    app_config.night_mode_grayscale = false;
    app_config.ir_sensor_pin = 999;
    app_config.ir_cut_pin1 = 999;
    app_config.ir_cut_pin2 = 999;
    app_config.ir_led_pin = 999;
    app_config.white_led_pin = 999;
    app_config.pin_switch_delay_us = 250;
    app_config.check_interval_s = 10;
    app_config.adc_device[0] = 0;
    app_config.adc_threshold = 128;
    app_config.isp_lum_low = -1;
    app_config.isp_lum_hi = -1;
    app_config.isp_iso_low = -1;
    app_config.isp_iso_hi = -1;
    app_config.isp_exptime_low = -1;
    app_config.isp_switch_lockout_s = 15;

    const char *conf_path = DIVINUS_CONFIG_PATH;
    struct fy_document *fyd = fy_document_build_from_file(NULL, conf_path);
    if (!fyd) {
        // Config may be corrupted (e.g. malformed UTF-8 in time_format). Try to repair and retry once.
        HAL_WARNING("app_config", "Failed to parse YAML config '%s' - attempting repair\n", conf_path);
        if (repair_divinus_yaml_time_format(conf_path) == 0) {
            fyd = fy_document_build_from_file(NULL, conf_path);
        }
        if (!fyd) {
            HAL_ERROR("app_config", "Failed to parse YAML config '%s'\n", conf_path);
        return -1;
        }
    }

    enum ConfigError err = CONFIG_OK;

    if (plat != HAL_PLATFORM_GM && plat != HAL_PLATFORM_RK) {
        err = yaml_get_string(fyd, "/system/sensor_config", app_config.sensor_config, sizeof(app_config.sensor_config));
        if (err != CONFIG_OK && (plat == HAL_PLATFORM_AK ||
             plat == HAL_PLATFORM_V1 || plat == HAL_PLATFORM_V2 ||
             plat == HAL_PLATFORM_V3 || plat == HAL_PLATFORM_V4))
            goto RET_ERR_YAML;
    }
    // Optional per-platform ISP/IQ config (e.g. Goke/HiSilicon v4 "scene_auto" IQ ini)
    yaml_get_string(fyd, "/system/iq_config", app_config.iq_config, sizeof(app_config.iq_config));

    {
        int port = 0;
        err = yaml_get_int(fyd, "/system/web_port", 0, USHRT_MAX, &port);
    if (err != CONFIG_OK)
            goto RET_ERR_YAML;
    app_config.web_port = (unsigned short)port;
    }
    {
        err = yaml_get_string(fyd, "/system/web_bind", app_config.web_bind, sizeof(app_config.web_bind));
        if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
            goto RET_ERR_YAML;
    }
    {
        unsigned int count = 0;
        // Reset to empty before parsing.
        for (int i = 0; i < (int)(sizeof(app_config.web_whitelist) / sizeof(app_config.web_whitelist[0])); i++)
            app_config.web_whitelist[i][0] = '\0';
        yaml_get_string_list(fyd, "/system/web_whitelist",
            app_config.web_whitelist,
            (unsigned int)(sizeof(app_config.web_whitelist) / sizeof(app_config.web_whitelist[0])),
            &count);
        (void)count;
    }
    yaml_get_bool(fyd, "/system/web_enable_auth", &app_config.web_enable_auth);
    yaml_get_string(fyd, "/system/web_auth_user", app_config.web_auth_user, sizeof(app_config.web_auth_user));
    yaml_get_string(fyd, "/system/web_auth_pass", app_config.web_auth_pass, sizeof(app_config.web_auth_pass));
    yaml_get_bool(fyd, "/system/web_auth_skiplocal", &app_config.web_auth_skiplocal);
    err = yaml_get_bool(fyd, "/system/web_enable_static", &app_config.web_enable_static);
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    err = yaml_get_uint(fyd, "/system/isp_thread_stack_size", 16 * 1024, UINT_MAX, &app_config.isp_thread_stack_size);
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    err = yaml_get_uint(fyd, "/system/venc_stream_thread_stack_size", 16 * 1024, UINT_MAX, &app_config.venc_stream_thread_stack_size);
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    err = yaml_get_uint(fyd, "/system/web_server_thread_stack_size", 16 * 1024, UINT_MAX, &app_config.web_server_thread_stack_size);
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    err = yaml_get_uint(fyd, "/system/night_thread_stack_size", 16 * 1024, UINT_MAX, &app_config.night_thread_stack_size);
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    yaml_get_string(fyd, "/system/time_format", timefmt, sizeof(timefmt));
    timefmt_cleaned |= timefmt_set(timefmt);
    yaml_get_uint(fyd, "/system/watchdog", 0, UINT_MAX, &app_config.watchdog);

    err = yaml_get_bool(fyd, "/night_mode/enable", &app_config.night_mode_enable);
    #define PIN_MAX 95
    #define PIN_SENTINEL 999
    #define PIN_CFG_MAX 999
    {
        // Parse night_mode fields regardless of `enable`, because some features
        // (e.g. IR-cut exercise on startup) need pin definitions even when the
        // background night mode thread is disabled.
        int lum;
        yaml_get_bool(fyd, "/night_mode/manual", &app_config.night_mode_manual);
        yaml_get_bool(fyd, "/night_mode/grayscale", &app_config.night_mode_grayscale);
        // Pins: allow sentinel 999 ("disabled") and negative values ("inverted") in config.
        yaml_get_int(fyd, "/night_mode/ir_sensor_pin", -PIN_CFG_MAX, PIN_CFG_MAX, &app_config.ir_sensor_pin);
        yaml_get_uint(fyd, "/night_mode/check_interval_s", 0, 600, &app_config.check_interval_s);
        yaml_get_int(fyd, "/night_mode/ir_cut_pin1", -PIN_CFG_MAX, PIN_CFG_MAX, &app_config.ir_cut_pin1);
        yaml_get_int(fyd, "/night_mode/ir_cut_pin2", -PIN_CFG_MAX, PIN_CFG_MAX, &app_config.ir_cut_pin2);
        yaml_get_int(fyd, "/night_mode/ir_led_pin", -PIN_CFG_MAX, PIN_CFG_MAX, &app_config.ir_led_pin);
        yaml_get_int(fyd, "/night_mode/white_led_pin", -PIN_CFG_MAX, PIN_CFG_MAX, &app_config.white_led_pin);
        yaml_get_uint(fyd, "/night_mode/pin_switch_delay_us", 0, 1000, &app_config.pin_switch_delay_us);
        yaml_get_string(fyd, "/night_mode/adc_device", app_config.adc_device, sizeof(app_config.adc_device));
        yaml_get_int(fyd, "/night_mode/adc_threshold", INT_MIN, INT_MAX, &app_config.adc_threshold);

        // Normalize pins: values outside the supported range are treated as "disabled".
        {
            int pin = 0;
            if (app_config.ir_sensor_pin != PIN_SENTINEL &&
                !decode_cfg_pin(app_config.ir_sensor_pin, PIN_MAX, &pin)) {
                HAL_WARNING("app_config", "night_mode.ir_sensor_pin=%d invalid, disabling\n", app_config.ir_sensor_pin);
                app_config.ir_sensor_pin = PIN_SENTINEL;
            }
            if (app_config.ir_cut_pin1 != PIN_SENTINEL &&
                !decode_cfg_pin(app_config.ir_cut_pin1, PIN_MAX, &pin)) {
                HAL_WARNING("app_config", "night_mode.ir_cut_pin1=%d invalid, disabling\n", app_config.ir_cut_pin1);
                app_config.ir_cut_pin1 = PIN_SENTINEL;
            }
            if (app_config.ir_cut_pin2 != PIN_SENTINEL &&
                !decode_cfg_pin(app_config.ir_cut_pin2, PIN_MAX, &pin)) {
                HAL_WARNING("app_config", "night_mode.ir_cut_pin2=%d invalid, disabling\n", app_config.ir_cut_pin2);
                app_config.ir_cut_pin2 = PIN_SENTINEL;
            }
            if (app_config.ir_led_pin != PIN_SENTINEL &&
                !decode_cfg_pin(app_config.ir_led_pin, PIN_MAX, &pin)) {
                HAL_WARNING("app_config", "night_mode.ir_led_pin=%d invalid, disabling\n", app_config.ir_led_pin);
                app_config.ir_led_pin = PIN_SENTINEL;
            }
            if (app_config.white_led_pin != PIN_SENTINEL &&
                !decode_cfg_pin(app_config.white_led_pin, PIN_MAX, &pin)) {
                HAL_WARNING("app_config", "night_mode.white_led_pin=%d invalid, disabling\n", app_config.white_led_pin);
                app_config.white_led_pin = PIN_SENTINEL;
            }
        }
        // Optional ISP-derived day/night hysteresis thresholds (hisi/v4 only).
        if (yaml_get_int(fyd, "/night_mode/isp_lum_low", 0, 255, &lum) == CONFIG_OK)
            app_config.isp_lum_low = lum;
        if (yaml_get_int(fyd, "/night_mode/isp_lum_hi", 0, 255, &lum) == CONFIG_OK)
            app_config.isp_lum_hi = lum;
        {
            int v;
            if (yaml_get_int(fyd, "/night_mode/isp_iso_low", 0, INT_MAX, &v) == CONFIG_OK)
                app_config.isp_iso_low = v;
            if (yaml_get_int(fyd, "/night_mode/isp_iso_hi", 0, INT_MAX, &v) == CONFIG_OK)
                app_config.isp_iso_hi = v;
            if (yaml_get_int(fyd, "/night_mode/isp_exptime_low", 0, INT_MAX, &v) == CONFIG_OK)
                app_config.isp_exptime_low = v;
        }
        {
            int lock_s = 0;
            if (yaml_get_int(fyd, "/night_mode/isp_switch_lockout_s", 0, 3600, &lock_s) == CONFIG_OK)
                app_config.isp_switch_lockout_s = (unsigned int)lock_s;
        }
    }
    // Only hisi/v4 supports ISP luminance source today.
    if (plat != HAL_PLATFORM_V4) {
        app_config.isp_lum_low = -1;
        app_config.isp_lum_hi = -1;
        app_config.isp_iso_low = -1;
        app_config.isp_iso_hi = -1;
        app_config.isp_exptime_low = -1;
    }

    err = yaml_get_bool(fyd, "/isp/sensor_mirror", &app_config.sensor_mirror);
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    err = yaml_get_bool(fyd, "/isp/sensor_flip", &app_config.sensor_flip);
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    err = yaml_get_bool(fyd, "/isp/mirror", &app_config.mirror);
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    err = yaml_get_bool(fyd, "/isp/flip", &app_config.flip);
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    yaml_get_int(fyd, "/isp/antiflicker", -1, 60, &app_config.antiflicker);

    yaml_get_bool(fyd, "/mdns/enable", &app_config.mdns_enable);

    yaml_get_bool(fyd, "/osd/enable", &app_config.osd_enable);
    yaml_get_bool(fyd, "/osd/isp_debug", &app_config.osd_isp_debug);
    if (app_config.osd_enable) {
        for (char i = 0; i < MAX_OSD; i++) {
            char param[16];
            int val;
            sprintf(param, "/osd/reg%d_img", i);
            yaml_get_string(fyd, param, osds[i].img, sizeof(osds[i].img));
            sprintf(param, "/osd/reg%d_text", i);
            yaml_get_string(fyd, param, osds[i].text, sizeof(osds[i].text));
            sprintf(param, "/osd/reg%d_font", i);
            yaml_get_string(fyd, param, osds[i].font, sizeof(osds[i].font));
            sprintf(param, "/osd/reg%d_opal", i);
            err = yaml_get_int(fyd, param, 0, UCHAR_MAX, &val);
            if (err == CONFIG_OK) osds[i].opal = (unsigned char)val;
            sprintf(param, "/osd/reg%d_posx", i);
            err = yaml_get_int(fyd, param, 0, SHRT_MAX, &val);
            if (err == CONFIG_OK) osds[i].posx = (short)val;
            sprintf(param, "/osd/reg%d_posy", i);
            err = yaml_get_int(fyd, param, 0, SHRT_MAX, &val);
            if (err == CONFIG_OK) osds[i].posy = (short)val;
            sprintf(param, "/osd/reg%d_size", i);
            yaml_get_double(fyd, param, 0, INT_MAX, &osds[i].size);
            sprintf(param, "/osd/reg%d_color", i);
            yaml_get_int(fyd, param, 0, USHRT_MAX, &osds[i].color);
            sprintf(param, "/osd/reg%d_outl", i);
            yaml_get_int(fyd, param, 0, USHRT_MAX, &osds[i].outl);
            sprintf(param, "/osd/reg%d_thick", i);
            yaml_get_double(fyd, param, 0, UCHAR_MAX, &osds[i].thick);
            sprintf(param, "/osd/reg%d_bg", i);
            {
                int c = 0;
                if (yaml_get_int(fyd, param, 0, 0x7FFF, &c) == CONFIG_OK)
                    osds[i].bg = c & 0x7FFF;
            }
            sprintf(param, "/osd/reg%d_bgopal", i);
            {
                int a = 0;
                if (yaml_get_int(fyd, param, 0, 255, &a) == CONFIG_OK)
                    osds[i].bgopal = (short)a;
            }
            sprintf(param, "/osd/reg%d_pad", i);
            {
                int p = 0;
                if (yaml_get_int(fyd, param, 0, 64, &p) == CONFIG_OK)
                    osds[i].pad = (short)p;
            }
            osds[i].updt = 1;
        }
    }

    yaml_get_bool(fyd, "/onvif/enable", &app_config.onvif_enable);
    if (app_config.onvif_enable) {
        yaml_get_bool(fyd, "/onvif/enable_auth", &app_config.onvif_enable_auth);
        yaml_get_string(fyd, "/onvif/auth_user", app_config.onvif_auth_user, sizeof(app_config.onvif_auth_user));
        yaml_get_string(fyd, "/onvif/auth_pass", app_config.onvif_auth_pass, sizeof(app_config.onvif_auth_pass));
    }

    yaml_get_bool(fyd, "/record/enable", &app_config.record_enable);
    yaml_get_bool(fyd, "/record/continuous", &app_config.record_continuous);
    yaml_get_string(fyd, "/record/path", app_config.record_path, sizeof(app_config.record_path));
    yaml_get_string(fyd, "/record/filename", app_config.record_filename, sizeof(app_config.record_filename));
    yaml_get_int(fyd, "/record/segment_duration", 0, INT_MAX, &app_config.record_segment_duration);
    yaml_get_int(fyd, "/record/segment_size", 0, INT_MAX, &app_config.record_segment_size);

    yaml_get_bool(fyd, "/rtsp/enable", &app_config.rtsp_enable);
    yaml_get_int(fyd, "/rtsp/port", 0, USHRT_MAX, &app_config.rtsp_port);
    err = yaml_get_string(fyd, "/rtsp/bind", app_config.rtsp_bind, sizeof(app_config.rtsp_bind));
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    if (app_config.rtsp_enable) {
        yaml_get_bool(fyd, "/rtsp/enable_auth", &app_config.rtsp_enable_auth);
        yaml_get_string(fyd, "/rtsp/auth_user", app_config.rtsp_auth_user, sizeof(app_config.rtsp_auth_user));
        yaml_get_string(fyd, "/rtsp/auth_pass", app_config.rtsp_auth_pass, sizeof(app_config.rtsp_auth_pass));
    }

    yaml_get_bool(fyd, "/stream/enable", &app_config.stream_enable);
    if (app_config.stream_enable) {
        int val = 0;
        if (yaml_get_int(fyd, "/stream/udp_srcport", 0, USHRT_MAX, &val) == CONFIG_OK)
            app_config.stream_udp_srcport = (unsigned short)val;

        for (int i = 0; i < (int)(sizeof(app_config.stream_dests) / sizeof(app_config.stream_dests[0])); i++)
            app_config.stream_dests[i][0] = '\0';
        yaml_get_string_list(fyd, "/stream/dest",
            app_config.stream_dests,
            (unsigned int)(sizeof(app_config.stream_dests) / sizeof(app_config.stream_dests[0])),
            NULL);
    }

    yaml_get_bool(fyd, "/audio/enable", &app_config.audio_enable);
    // Persisted mute flag (keep audio/RTSP track alive but send silence).
    // Parse regardless of enable; it will be applied when/if audio starts.
    yaml_get_bool(fyd, "/audio/mute", &app_config.audio_mute);
    if (app_config.audio_enable) {
        {
            // MP3 support removed; accept only AAC/AAC-LC (or ignore).
            char codec_raw[32] = {0};
            if (yaml_get_string(fyd, "/audio/codec", codec_raw, sizeof(codec_raw)) == CONFIG_OK) {
                if (!strcasecmp(codec_raw, "AAC") || !strcasecmp(codec_raw, "AAC-LC")) {
                    app_config.audio_codec = HAL_AUDCODEC_AAC;
                } else if (!strcasecmp(codec_raw, "MP3")) {
                    HAL_WARNING("app_config", "audio.codec=MP3 is no longer supported; forcing AAC\n");
                    app_config.audio_codec = HAL_AUDCODEC_AAC;
            }
        }
        }
        yaml_get_uint(fyd, "/audio/bitrate", 32, 320, &app_config.audio_bitrate);
        // `audio.gain` semantics are platform-specific.
        // - hisi/v4: user-friendly 0..100 (50 = unity gain, 0 = quieter, 100 = louder)
        // - others: legacy dB level range (-60..+30) used by platform HALs
        if (plat == HAL_PLATFORM_V4)
            yaml_get_int(fyd, "/audio/gain", 0, 100, &app_config.audio_gain);
        else
            yaml_get_int(fyd, "/audio/gain", -60, 30, &app_config.audio_gain);
        err = yaml_get_uint(fyd, "/audio/srate", 8000, 96000, &app_config.audio_srate);
        if (err != CONFIG_OK)
            goto RET_ERR_YAML;
        {
            int ch = 0;
            if (yaml_get_int(fyd, "/audio/channels", 1, 2, &ch) == CONFIG_OK)
                app_config.audio_channels = (unsigned char)ch;
        }

        // AAC-only advanced options (FAAC).
        // `aac_quantqual` enables quality/VBR mode when > 0 (range 10..5000 in our FAAC build).
        // In this mode `bitrate` is ignored by FAAC (bitRate=0).
        yaml_get_uint(fyd, "/audio/aac_quantqual", 0, 5000, &app_config.audio_aac_quantqual);
        // Encoder bandwidth in Hz; 0 lets FAAC choose defaults.
        yaml_get_uint(fyd, "/audio/aac_bandwidth", 0, 96000, &app_config.audio_aac_bandwidth);
        // Temporal Noise Shaping (TNS).
        yaml_get_bool(fyd, "/audio/aac_tns", &app_config.audio_aac_tns);

        // SpeexDSP preprocess options (used only when codec=AAC).
        yaml_get_bool(fyd, "/audio/speex_enable", &app_config.audio_speex_enable);
        yaml_get_bool(fyd, "/audio/speex_denoise", &app_config.audio_speex_denoise);
        yaml_get_bool(fyd, "/audio/speex_agc", &app_config.audio_speex_agc);
        yaml_get_bool(fyd, "/audio/speex_vad", &app_config.audio_speex_vad);
        yaml_get_bool(fyd, "/audio/speex_dereverb", &app_config.audio_speex_dereverb);
        {
            int fs = 0;
            // 0 or missing = auto (srate/50). Values < 80 are clamped at runtime.
            if (yaml_get_int(fyd, "/audio/speex_frame_size", 0, 8192, &fs) == CONFIG_OK)
                app_config.audio_speex_frame_size = (unsigned int)fs;
        }
        yaml_get_int(fyd, "/audio/speex_noise_suppress_db", -60, 0, &app_config.audio_speex_noise_suppress_db);
        yaml_get_int(fyd, "/audio/speex_agc_level", 1, 32768, &app_config.audio_speex_agc_level);
        yaml_get_int(fyd, "/audio/speex_agc_increment", 0, 100, &app_config.audio_speex_agc_increment);
        yaml_get_int(fyd, "/audio/speex_agc_decrement", 0, 100, &app_config.audio_speex_agc_decrement);
        yaml_get_int(fyd, "/audio/speex_agc_max_gain_db", 0, 60, &app_config.audio_speex_agc_max_gain_db);
        yaml_get_int(fyd, "/audio/speex_vad_prob_start", 0, 100, &app_config.audio_speex_vad_prob_start);
        yaml_get_int(fyd, "/audio/speex_vad_prob_continue", 0, 100, &app_config.audio_speex_vad_prob_continue);
    }

    yaml_get_bool(fyd, "/mp4/enable", &app_config.mp4_enable);
    if (app_config.mp4_enable) {
        {
            // Accept: H.264/H264/AVC, H.265/H265/HEVC, and "+" variants: H.264+/H264+/AVC+.
            // Do NOT rely on enum index parity (e.g. "H264+" used to map to H.265 by accident).
            char codec_raw[32] = {0};
            if (yaml_get_string(fyd, "/mp4/codec", codec_raw, sizeof(codec_raw)) == CONFIG_OK) {
                char norm[32] = {0};
                int j = 0;
                for (int i = 0; codec_raw[i] && j < (int)sizeof(norm) - 1; i++) {
                    unsigned char c = (unsigned char)codec_raw[i];
                    if (c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '.')
                        continue;
                    norm[j++] = (char)toupper(c);
                }
                norm[j] = '\0';

                const bool plus = (strchr(norm, '+') != NULL);
                const bool is_h265 = (CONTAINS(norm, "H265") || CONTAINS(norm, "HEVC") || CONTAINS(norm, "265"));
                const bool is_h264 = (CONTAINS(norm, "H264") || CONTAINS(norm, "AVC") || CONTAINS(norm, "264"));

                if (is_h265) {
                    app_config.mp4_codecH265 = true;
                } else if (is_h264) {
                    app_config.mp4_codecH265 = false;
                } else {
                    // Default: H.264 (backwards compatible).
                    app_config.mp4_codecH265 = false;
                }

                // "H.264+" is only meaningful for H.264 streams.
                app_config.mp4_h264_plus = (plus && !app_config.mp4_codecH265);
            } else {
                app_config.mp4_codecH265 = false;
                app_config.mp4_h264_plus = false;
            }
        }
        {
            int val = 0;
            yaml_get_int(fyd, "/mp4/mode", 0, INT_MAX, &val);
            app_config.mp4_mode = (unsigned int)val;
        }
        err = yaml_get_uint(fyd, "/mp4/width", 160, UINT_MAX, &app_config.mp4_width);
        if (err != CONFIG_OK)
            goto RET_ERR_YAML;
        err = yaml_get_uint(fyd, "/mp4/height", 120, UINT_MAX, &app_config.mp4_height);
        if (err != CONFIG_OK)
            goto RET_ERR_YAML;
        err = yaml_get_uint(fyd, "/mp4/fps", 1, UINT_MAX, &app_config.mp4_fps);
        if (err != CONFIG_OK)
            goto RET_ERR_YAML;
        app_config.mp4_gop = app_config.mp4_fps;
        yaml_get_uint(fyd, "/mp4/gop", 1, UINT_MAX, &app_config.mp4_gop);
        {
            int val = 0;
            yaml_get_int(fyd, "/mp4/profile", 0, INT_MAX, &val);
            app_config.mp4_profile = (unsigned int)val;
        }
        yaml_get_uint(fyd, "/mp4/profile", 0, 2, &app_config.mp4_profile);

        err = yaml_get_uint(fyd, "/mp4/bitrate", 32, UINT_MAX, &app_config.mp4_bitrate);
        if (err != CONFIG_OK)
            goto RET_ERR_YAML;
    }

    // JPEG section represents the MJPEG (multipart JPEG) stream.
    err = yaml_get_bool(fyd, "/jpeg/enable", &app_config.jpeg_enable);
    if (err != CONFIG_OK && err != CONFIG_PARAM_NOT_FOUND)
        goto RET_ERR_YAML;
    yaml_get_bool(fyd, "/jpeg/osd_enable", &app_config.jpeg_osd_enable);
    yaml_get_bool(fyd, "/jpeg/grayscale_night", &app_config.jpeg_grayscale_night);
    if (app_config.jpeg_enable) {
        int mode = 0;
        yaml_get_int(fyd, "/jpeg/mode", 0, INT_MAX, &mode);
        app_config.jpeg_mode = (unsigned int)mode;
        err = yaml_get_uint(fyd, "/jpeg/width", 160, UINT_MAX, &app_config.jpeg_width);
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        err = yaml_get_uint(fyd, "/jpeg/height", 120, UINT_MAX, &app_config.jpeg_height);
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        err = yaml_get_uint(fyd, "/jpeg/fps", 1, UINT_MAX, &app_config.jpeg_fps);
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        err = yaml_get_uint(fyd, "/jpeg/qfactor", 1, 99, &app_config.jpeg_qfactor);
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        // MJPEG bitrate control is deprecated; always operate in QP mode.
        app_config.jpeg_mode = HAL_VIDMODE_QP;
    }

    yaml_get_bool(fyd, "/http_post/enable", &app_config.http_post_enable);
    if (app_config.http_post_enable) {
        err = yaml_get_string(fyd, "/http_post/host", app_config.http_post_host, sizeof(app_config.http_post_host));
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        err = yaml_get_string(fyd, "/http_post/url", app_config.http_post_url, sizeof(app_config.http_post_url));
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        err = yaml_get_string(fyd, "/http_post/login", app_config.http_post_login, sizeof(app_config.http_post_login));
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        err = yaml_get_string(fyd, "/http_post/password", app_config.http_post_password, sizeof(app_config.http_post_password));
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        err = yaml_get_uint(fyd, "/http_post/width", 160, UINT_MAX, &app_config.http_post_width);
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        err = yaml_get_uint(fyd, "/http_post/height", 120, UINT_MAX, &app_config.http_post_height);
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        err = yaml_get_uint(fyd, "/http_post/interval", 1, UINT_MAX, &app_config.http_post_interval);
        if (err != CONFIG_OK) goto RET_ERR_YAML;
        err = yaml_get_uint(fyd, "/http_post/qfactor", 1, 99, &app_config.http_post_qfactor);
        if (err != CONFIG_OK) goto RET_ERR_YAML;
    }

    fy_document_destroy(fyd);

    // If time_format contained junk and we cleaned it, persist the fixed value
    // so subsequent runs start from a sane config.
    if (timefmt_cleaned) {
        HAL_WARNING("app_config", "time_format contained invalid bytes; resetting to '%s'\n", timefmt_cfg);
        int sr = save_app_config();
        if (sr != 0)
            HAL_WARNING("app_config", "Failed to persist cleaned time_format (ret=%d)\n", sr);
    }

    return CONFIG_OK;
RET_ERR_YAML:
    fy_document_destroy(fyd);
    return err;
}