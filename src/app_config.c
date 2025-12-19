#include "app_config.h"
#include <ctype.h>

const char *appconf_paths[] = {"./divinus.yaml", "/etc/divinus.yaml"};

struct AppConfig app_config;

static inline void open_app_config(FILE **file, const char *flags) {
    const char **path = appconf_paths;
    char conf_path[PATH_MAX], exe_path[PATH_MAX];
    *file = NULL;

    ssize_t exe_len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (exe_len != -1) {
        char *dir = dirname(exe_path);
        exe_path[exe_len] = '\0';
        snprintf(conf_path, sizeof(conf_path), "%s/divinus.yaml", dir);
        if (!access(conf_path, F_OK)) {
            if (*flags == 'w') {
                char bak_path[PATH_MAX];
                sprintf(bak_path, "%s.bak", conf_path);
                remove(bak_path);
                rename(conf_path, bak_path);
            }
            *file = fopen(conf_path, flags);
            return;
        }
    }

    while (*path) {
        if (access(*path++, F_OK)) continue;
        if (*flags == 'w') {
            char bak_path[PATH_MAX];
            sprintf(bak_path, "%s.bak", *(path - 1));
            remove(bak_path);
            rename(*(path - 1), bak_path);
        }
        *file = fopen(*(path - 1), flags);
        break;
    }
}

void restore_app_config(void) {
    char conf_path[PATH_MAX], exe_path[PATH_MAX];

    ssize_t exe_len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (exe_len != -1) {
        char bak_path[PATH_MAX], *dir = dirname(exe_path);
        exe_path[exe_len] = '\0';
        snprintf(conf_path, sizeof(conf_path), "%s/divinus.yaml", dir);
        sprintf(bak_path, "%s.bak", conf_path);
        if (!access(bak_path, F_OK)) {
            remove(conf_path);
            rename(bak_path, conf_path);
            return;
        }
    }

    const char **path = appconf_paths;
    while (*path) {
        char bak_path[PATH_MAX];
        sprintf(bak_path, "%s.bak", *path);
        if (!access(bak_path, F_OK)) {
            remove(*path);
            rename(bak_path, *path);
        }
        path++;
    }
}

int save_app_config(void) {
    FILE *file;

    open_app_config(&file, "w");
    if (!file)
        HAL_ERROR("app_config", "Can't open config file for writing\n");

    fprintf(file, "system:\n");
    fprintf(file, "  sensor_config: %s\n", app_config.sensor_config);
    if (!EMPTY(app_config.iq_config))
        fprintf(file, "  iq_config: %s\n", app_config.iq_config);
    fprintf(file, "  web_port: %d\n", app_config.web_port);
    if (!EMPTY(*app_config.web_whitelist)) {
        fprintf(file, "  web_whitelist: ");
        for (int i = 0; app_config.web_whitelist[i] && *app_config.web_whitelist[i]; i++) {
            fprintf(file, "    - %s\n", app_config.web_whitelist[i]);
        }
    }
    fprintf(file, "  web_enable_auth: %s\n", app_config.web_enable_auth ? "true" : "false");
    fprintf(file, "  web_auth_user: %s\n", app_config.web_auth_user);
    fprintf(file, "  web_auth_pass: %s\n", app_config.web_auth_pass);
    fprintf(file, "  web_auth_skiplocal: %s\n", app_config.web_auth_skiplocal ? "true" : "false");
    fprintf(file, "  web_enable_static: %s\n", app_config.web_enable_static ? "true" : "false");
    fprintf(file, "  isp_thread_stack_size: %d\n", app_config.isp_thread_stack_size);
    fprintf(file, "  venc_stream_thread_stack_size: %d\n", app_config.venc_stream_thread_stack_size);
    fprintf(file, "  web_server_thread_stack_size: %d\n", app_config.web_server_thread_stack_size);
    if (!EMPTY(timefmt) || EQUALS(timefmt, DEF_TIMEFMT))
        fprintf(file, "  time_format: %s\n", timefmt);
    fprintf(file, "  watchdog: %d\n", app_config.watchdog);

    fprintf(file, "night_mode:\n");
    fprintf(file, "  enable: %s\n", app_config.night_mode_enable ? "true" : "false");
    fprintf(file, "  grayscale: %s\n", app_config.night_mode_grayscale ? "true" : "false");
    fprintf(file, "  ir_sensor_pin: %d\n", app_config.ir_sensor_pin);
    fprintf(file, "  check_interval_s: %d\n", app_config.check_interval_s);
    fprintf(file, "  ir_cut_pin1: %d\n", app_config.ir_cut_pin1);
    fprintf(file, "  ir_cut_pin2: %d\n", app_config.ir_cut_pin2);
    if (app_config.isp_lum_low >= 0)
        fprintf(file, "  isp_lum_low: %d\n", app_config.isp_lum_low);
    if (app_config.isp_lum_hi >= 0)
        fprintf(file, "  isp_lum_hi: %d\n", app_config.isp_lum_hi);
    if (app_config.isp_iso_low >= 0)
        fprintf(file, "  isp_iso_low: %d\n", app_config.isp_iso_low);
    if (app_config.isp_iso_hi >= 0)
        fprintf(file, "  isp_iso_hi: %d\n", app_config.isp_iso_hi);
    if (app_config.isp_exptime_low >= 0)
        fprintf(file, "  isp_exptime_low: %d\n", app_config.isp_exptime_low);
    fprintf(file, "  isp_switch_lockout_s: %u\n", app_config.isp_switch_lockout_s);
    fprintf(file, "  ir_led_pin: %d\n", app_config.ir_led_pin);
    fprintf(file, "  pin_switch_delay_us: %d\n", app_config.pin_switch_delay_us);
    fprintf(file, "  adc_device: %s\n", app_config.adc_device);
    fprintf(file, "  adc_threshold: %d\n", app_config.adc_threshold);

    fprintf(file, "isp:\n");
    fprintf(file, "  mirror: %s\n", app_config.mirror ? "true" : "false");
    fprintf(file, "  flip: %s\n", app_config.flip ? "true" : "false");
    fprintf(file, "  antiflicker: %d\n", app_config.antiflicker);

    fprintf(file, "mdns:\n");
    fprintf(file, "  enable: %s\n", app_config.mdns_enable ? "true" : "false");

    fprintf(file, "onvif:\n");
    fprintf(file, "  enable: %s\n", app_config.onvif_enable ? "true" : "false");
    fprintf(file, "  enable_auth: %s\n", app_config.onvif_enable_auth ? "true" : "false");
    fprintf(file, "  auth_user: %s\n", app_config.onvif_auth_user);
    fprintf(file, "  auth_pass: %s\n", app_config.onvif_auth_pass);

    fprintf(file, "rtsp:\n");
    fprintf(file, "  enable: %s\n", app_config.rtsp_enable ? "true" : "false");
    fprintf(file, "  port: %d\n", app_config.rtsp_port);
    fprintf(file, "  enable_auth: %s\n", app_config.rtsp_enable_auth ? "true" : "false");
    fprintf(file, "  auth_user: %s\n", app_config.rtsp_auth_user);
    fprintf(file, "  auth_pass: %s\n", app_config.rtsp_auth_pass);

    fprintf(file, "record:\n");
    fprintf(file, "  enable: %s\n", app_config.record_enable ? "true" : "false");
    fprintf(file, "  continuous: %s\n", app_config.record_continuous ? "true" : "false");
    fprintf(file, "  path: %s\n", app_config.record_path);
    fprintf(file, "  filename: %s\n", app_config.record_filename);
    fprintf(file, "  segment_duration: %d\n", app_config.record_segment_duration);
    fprintf(file, "  segment_size: %d\n", app_config.record_segment_size);

    fprintf(file, "stream:\n");
    fprintf(file, "  enable: %s\n", app_config.stream_enable ? "true" : "false");
    fprintf(file, "  udp_srcport: %d\n", app_config.stream_udp_srcport);
    if (!EMPTY(*app_config.stream_dests)) {
        fprintf(file, "  dests: ");
        for (int i = 0; app_config.stream_dests[i] && *app_config.stream_dests[i]; i++) {
            fprintf(file, "    - %s\n", app_config.stream_dests[i]);
        }
    }

    fprintf(file, "audio:\n");
    fprintf(file, "  enable: %s\n", app_config.audio_enable ? "true" : "false");
    fprintf(file, "  codec: %s\n",
        app_config.audio_codec == HAL_AUDCODEC_AAC ? "AAC" : "MP3");
    fprintf(file, "  bitrate: %d\n", app_config.audio_bitrate);
    fprintf(file, "  gain: %d\n", app_config.audio_gain);
    fprintf(file, "  srate: %d\n", app_config.audio_srate);
    fprintf(file, "  channels: %d\n", app_config.audio_channels);
    // AAC-only tuning (FAAC). Safe to keep in config even if codec=MP3.
    fprintf(file, "  aac_quantqual: %u\n", app_config.audio_aac_quantqual);
    fprintf(file, "  aac_bandwidth: %u\n", app_config.audio_aac_bandwidth);
    fprintf(file, "  aac_tns: %s\n", app_config.audio_aac_tns ? "true" : "false");
    // SpeexDSP preprocess (used only when codec=AAC). Safe to keep in config even if codec=MP3.
    fprintf(file, "  speex_enable: %s\n", app_config.audio_speex_enable ? "true" : "false");
    fprintf(file, "  speex_denoise: %s\n", app_config.audio_speex_denoise ? "true" : "false");
    fprintf(file, "  speex_agc: %s\n", app_config.audio_speex_agc ? "true" : "false");
    fprintf(file, "  speex_vad: %s\n", app_config.audio_speex_vad ? "true" : "false");
    fprintf(file, "  speex_dereverb: %s\n", app_config.audio_speex_dereverb ? "true" : "false");
    fprintf(file, "  speex_frame_size: %u\n", app_config.audio_speex_frame_size);
    fprintf(file, "  speex_noise_suppress_db: %d\n", app_config.audio_speex_noise_suppress_db);
    fprintf(file, "  speex_agc_level: %d\n", app_config.audio_speex_agc_level);
    fprintf(file, "  speex_agc_increment: %d\n", app_config.audio_speex_agc_increment);
    fprintf(file, "  speex_agc_decrement: %d\n", app_config.audio_speex_agc_decrement);
    fprintf(file, "  speex_agc_max_gain_db: %d\n", app_config.audio_speex_agc_max_gain_db);
    fprintf(file, "  speex_vad_prob_start: %d\n", app_config.audio_speex_vad_prob_start);
    fprintf(file, "  speex_vad_prob_continue: %d\n", app_config.audio_speex_vad_prob_continue);

    fprintf(file, "mp4:\n");
    fprintf(file, "  enable: %s\n", app_config.mp4_enable ? "true" : "false");
    fprintf(file, "  codec: %s\n", app_config.mp4_codecH265 ? "H.265" :
        (app_config.mp4_h264_plus ? "H.264+" : "H.264"));
    fprintf(file, "  mode: %d\n", app_config.mp4_mode);
    fprintf(file, "  width: %d\n", app_config.mp4_width);
    fprintf(file, "  height: %d\n", app_config.mp4_height);
    fprintf(file, "  fps: %d\n", app_config.mp4_fps);
    fprintf(file, "  gop: %d\n", app_config.mp4_gop);
    fprintf(file, "  profile: %d\n", app_config.mp4_profile);
    fprintf(file, "  bitrate: %d\n", app_config.mp4_bitrate);

    fprintf(file, "osd:\n");
    fprintf(file, "  enable: %s\n", app_config.osd_enable ? "true" : "false");
    for (char i = 0; i < MAX_OSD; i++) {
        char imgEmpty = EMPTY(osds[i].img);
        char textEmpty = EMPTY(osds[i].text);
        if (imgEmpty && textEmpty) continue;
    
        if (!imgEmpty)
            fprintf(file, "    reg%d_img: %s\n", i, osds[i].img);
        if (!textEmpty)
            fprintf(file, "    reg%d_text: %s\n", i, osds[i].text);
        fprintf(file, "    reg%d_font: %s\n", i, osds[i].font);
        fprintf(file, "    reg%d_opal: %d\n", i, osds[i].opal);
        fprintf(file, "    reg%d_posx: %d\n", i, osds[i].posx);
        fprintf(file, "    reg%d_posy: %d\n", i, osds[i].posy);
        fprintf(file, "    reg%d_size: %.1f\n", i, osds[i].size);
        fprintf(file, "    reg%d_color: %#04x\n", i, osds[i].color);
        fprintf(file, "    reg%d_outl: %#04x\n", i, osds[i].outl);
        fprintf(file, "    reg%d_thick: %.1f\n", i, osds[i].thick);
    }

    // NOTE: "jpeg" section represents the MJPEG (multipart JPEG) stream.
    // Snapshots (/image.jpg) are served from the last MJPEG frame.
    fprintf(file, "jpeg:\n");
    fprintf(file, "  enable: %s\n", app_config.jpeg_enable ? "true" : "false");
    fprintf(file, "  osd_enable: %s\n", app_config.jpeg_osd_enable ? "true" : "false");
    fprintf(file, "  mode: %d\n", app_config.jpeg_mode);
    fprintf(file, "  width: %d\n", app_config.jpeg_width);
    fprintf(file, "  height: %d\n", app_config.jpeg_height);
    fprintf(file, "  fps: %d\n", app_config.jpeg_fps);
    fprintf(file, "  qfactor: %d\n", app_config.jpeg_qfactor);

    fprintf(file, "http_post:\n");
    fprintf(file, "  enable: %s\n", app_config.http_post_enable ? "true" : "false");
    fprintf(file, "  host: %s\n", app_config.http_post_host);
    fprintf(file, "  url: %s\n", app_config.http_post_url);
    fprintf(file, "  login: %s\n", app_config.http_post_login);
    fprintf(file, "  password: %s\n", app_config.http_post_password);
    fprintf(file, "  width: %d\n", app_config.http_post_width);
    fprintf(file, "  height: %d\n", app_config.http_post_height);
    fprintf(file, "  interval: %d\n", app_config.http_post_interval);
    fprintf(file, "  qfactor: %d\n", app_config.http_post_qfactor);

    fclose(file);
    return EXIT_SUCCESS;
}

enum ConfigError parse_app_config(void) {
    memset(&app_config, 0, sizeof(struct AppConfig));

    app_config.web_port = 8080;
    *app_config.web_whitelist[0] = '\0';
    app_config.web_enable_auth = false;
    app_config.web_auth_skiplocal = false;
    app_config.web_enable_static = false;
    app_config.isp_thread_stack_size = 16 * 1024;
    app_config.venc_stream_thread_stack_size = 16 * 1024;
    app_config.web_server_thread_stack_size = 32 * 1024;
    app_config.watchdog = 0;

    app_config.mdns_enable = false;

    app_config.osd_enable = false;

    app_config.onvif_enable = false;
    app_config.onvif_enable_auth = false;
    app_config.onvif_auth_user[0] = '\0';
    app_config.onvif_auth_pass[0] = '\0';

    app_config.rtsp_enable = false;
    app_config.rtsp_port = 554;
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
    app_config.audio_codec = HAL_AUDCODEC_MP3;
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
    app_config.jpeg_fps = 15;
    app_config.jpeg_width = 640;
    app_config.jpeg_height = 480;
    app_config.jpeg_mode = HAL_VIDMODE_QP;
    app_config.jpeg_qfactor = 80;

    app_config.mirror = false;
    app_config.flip = false;
    app_config.antiflicker = 0;

    app_config.night_mode_enable = false;
    app_config.night_mode_grayscale = false;
    app_config.ir_sensor_pin = 999;
    app_config.ir_cut_pin1 = 999;
    app_config.ir_cut_pin2 = 999;
    app_config.ir_led_pin = 999;
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

    struct IniConfig ini;
    memset(&ini, 0, sizeof(struct IniConfig));

    FILE *file;
    open_app_config(&file, "r");
    if (!open_config(&ini, &file))  {
        printf("Can't find config divinus.yaml in:\n"
            "    ./divinus.yaml\n    /etc/divinus.yaml\n");
        return -1;
    }

    enum ConfigError err;
    find_sections(&ini);

    if (plat != HAL_PLATFORM_GM && plat != HAL_PLATFORM_RK) {
        err = parse_param_value(&ini, "system", "sensor_config", app_config.sensor_config);
        if (err != CONFIG_OK && (plat == HAL_PLATFORM_AK ||
             plat == HAL_PLATFORM_V1 || plat == HAL_PLATFORM_V2 ||
             plat == HAL_PLATFORM_V3 || plat == HAL_PLATFORM_V4))
            goto RET_ERR;
    }
    // Optional per-platform ISP/IQ config (e.g. Goke/HiSilicon v4 "scene_auto" IQ ini)
    parse_param_value(&ini, "system", "iq_config", app_config.iq_config);
    int port, count;
    err = parse_int(&ini, "system", "web_port", 0, USHRT_MAX, &port);
    if (err != CONFIG_OK)
        goto RET_ERR;
    app_config.web_port = (unsigned short)port;
    parse_list(&ini, "system", "web_whitelist",
        sizeof(app_config.web_whitelist) / sizeof(*app_config.web_whitelist),
        &count, app_config.web_whitelist);
    *app_config.web_whitelist[count] = '\0';
    parse_bool(&ini, "system", "web_enable_auth", &app_config.web_enable_auth);
    parse_param_value(
        &ini, "system", "web_auth_user", app_config.web_auth_user);
    parse_param_value(
        &ini, "system", "web_auth_pass", app_config.web_auth_pass);
    parse_bool(&ini, "system", "web_auth_skiplocal", &app_config.web_auth_skiplocal);
    err = parse_bool(
        &ini, "system", "web_enable_static", &app_config.web_enable_static);
    if (err != CONFIG_OK)
        goto RET_ERR;
    err = parse_int(
        &ini, "system", "isp_thread_stack_size", 16 * 1024, INT_MAX,
        &app_config.isp_thread_stack_size);
    if (err != CONFIG_OK)
        goto RET_ERR;
    err = parse_int(
        &ini, "system", "venc_stream_thread_stack_size", 16 * 1024, INT_MAX,
        &app_config.venc_stream_thread_stack_size);
    if (err != CONFIG_OK)
        goto RET_ERR;
    err = parse_int(
        &ini, "system", "web_server_thread_stack_size", 16 * 1024, INT_MAX,
        &app_config.web_server_thread_stack_size);
    if (err != CONFIG_OK)
        goto RET_ERR;
    parse_param_value(&ini, "system", "time_format", timefmt);
    if (EMPTY(timefmt))
        strncpy(timefmt, DEF_TIMEFMT, sizeof(timefmt) - 1);
    parse_int(&ini, "system", "watchdog", 0, INT_MAX, &app_config.watchdog);

    err =
        parse_bool(&ini, "night_mode", "enable", &app_config.night_mode_enable);
    #define PIN_MAX 95
    {
        // Parse night_mode fields regardless of `enable`, because some features
        // (e.g. IR-cut exercise on startup) need pin definitions even when the
        // background night mode thread is disabled.
        int lum;
        parse_bool(&ini, "night_mode", "grayscale", &app_config.night_mode_grayscale);
        parse_int(&ini, "night_mode", "ir_sensor_pin", 0, PIN_MAX, &app_config.ir_sensor_pin);
        parse_int(&ini, "night_mode", "check_interval_s", 0, 600, &app_config.check_interval_s);
        parse_int(&ini, "night_mode", "ir_cut_pin1", 0, PIN_MAX, &app_config.ir_cut_pin1);
        parse_int(&ini, "night_mode", "ir_cut_pin2", 0, PIN_MAX, &app_config.ir_cut_pin2);
        parse_int(&ini, "night_mode", "ir_led_pin", 0, PIN_MAX, &app_config.ir_led_pin);
        parse_int(&ini, "night_mode", "pin_switch_delay_us", 0, 1000, &app_config.pin_switch_delay_us);
        parse_param_value(&ini, "night_mode", "adc_device", app_config.adc_device);
        parse_int(&ini, "night_mode", "adc_threshold", INT_MIN, INT_MAX, &app_config.adc_threshold);
        // Optional ISP-derived day/night hysteresis thresholds (hisi/v4 only).
        if (parse_int(&ini, "night_mode", "isp_lum_low", 0, 255, &lum) == CONFIG_OK)
            app_config.isp_lum_low = lum;
        if (parse_int(&ini, "night_mode", "isp_lum_hi", 0, 255, &lum) == CONFIG_OK)
            app_config.isp_lum_hi = lum;
        {
            int v;
            if (parse_int(&ini, "night_mode", "isp_iso_low", 0, INT_MAX, &v) == CONFIG_OK)
                app_config.isp_iso_low = v;
            if (parse_int(&ini, "night_mode", "isp_iso_hi", 0, INT_MAX, &v) == CONFIG_OK)
                app_config.isp_iso_hi = v;
            if (parse_int(&ini, "night_mode", "isp_exptime_low", 0, INT_MAX, &v) == CONFIG_OK)
                app_config.isp_exptime_low = v;
        }
        {
            int lock_s = 0;
            if (parse_int(&ini, "night_mode", "isp_switch_lockout_s", 0, 3600, &lock_s) == CONFIG_OK)
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

    err = parse_bool(&ini, "isp", "mirror", &app_config.mirror);
    if (err != CONFIG_OK)
        goto RET_ERR;
    err = parse_bool(&ini, "isp", "flip", &app_config.flip);
    if (err != CONFIG_OK)
        goto RET_ERR;
    parse_int(&ini, "isp", "antiflicker", -1, 60, &app_config.antiflicker);

    parse_bool(&ini, "mdns", "enable", &app_config.mdns_enable);

    parse_bool(&ini, "osd", "enable", &app_config.osd_enable);
    if (app_config.osd_enable) {
        for (char i = 0; i < MAX_OSD; i++) {
            char param[16];
            int val;
            sprintf(param, "reg%d_img", i);
            parse_param_value(&ini, "osd", param, osds[i].img);
            sprintf(param, "reg%d_text", i);
            parse_param_value(&ini, "osd", param, osds[i].text);
            sprintf(param, "reg%d_font", i);
            parse_param_value(&ini, "osd", param, osds[i].font);
            sprintf(param, "reg%d_opal", i);
            err = parse_int(&ini, "osd", param, 0, UCHAR_MAX, &val);
            if (err == CONFIG_OK) osds[i].opal = (unsigned char)val;
            sprintf(param, "reg%d_posx", i);
            err = parse_int(&ini, "osd", param, 0, SHRT_MAX, &val);
            if (err == CONFIG_OK) osds[i].posx = (short)val;
            sprintf(param, "reg%d_posy", i);
            err = parse_int(&ini, "osd", param, 0, SHRT_MAX, &val);
            if (err == CONFIG_OK) osds[i].posy = (short)val;
            sprintf(param, "reg%d_size", i);
            parse_double(&ini, "osd", param, 0, INT_MAX, &osds[i].size);
            sprintf(param, "reg%d_color", i);
            parse_int(&ini, "osd", param, 0, USHRT_MAX, &osds[i].color);
            sprintf(param, "reg%d_outl", i);
            parse_int(&ini, "osd", param, 0, USHRT_MAX, &osds[i].outl);
            sprintf(param, "reg%d_thick", i);
            parse_double(&ini, "osd", param, 0, UCHAR_MAX, &osds[i].thick);
            osds[i].updt = 1;
        }
    }

    parse_bool(&ini, "onvif", "enable", &app_config.onvif_enable);
    if (app_config.onvif_enable) {
        parse_bool(&ini, "onvif", "enable_auth", &app_config.onvif_enable_auth);
        parse_param_value(
            &ini, "onvif", "auth_user", app_config.onvif_auth_user);
        parse_param_value(
            &ini, "onvif", "auth_pass", app_config.onvif_auth_pass);
    }

    parse_bool(&ini, "record", "enable", &app_config.record_enable);
    parse_bool(&ini, "record", "continuous", &app_config.record_continuous);
    parse_param_value(
        &ini, "record", "path", app_config.record_path);
    parse_param_value(
        &ini, "record", "filename", app_config.record_filename);
    parse_int(&ini, "record", "segment_duration", 0, INT_MAX,
        &app_config.record_segment_duration);
    parse_int(&ini, "record", "segment_size", 0, INT_MAX,
        &app_config.record_segment_size);

    parse_bool(&ini, "rtsp", "enable", &app_config.rtsp_enable);
    parse_int(&ini, "rtsp", "port", 0, USHRT_MAX, &app_config.rtsp_port);
    if (app_config.rtsp_enable) {
        parse_bool(&ini, "rtsp", "enable_auth", &app_config.rtsp_enable_auth);
        parse_param_value(
            &ini, "rtsp", "auth_user", app_config.rtsp_auth_user);
        parse_param_value(
            &ini, "rtsp", "auth_pass", app_config.rtsp_auth_pass);
    }

    parse_bool(&ini, "stream", "enable", &app_config.stream_enable);
    if (app_config.stream_enable) {
        int count, val;
        parse_int(&ini, "stream", "udp_srcport", 0, USHRT_MAX, &val);
        if (err != CONFIG_OK) app_config.stream_udp_srcport = (unsigned short)val;
        err = parse_list(&ini, "stream", "dest",
            sizeof(app_config.stream_dests) / sizeof(*app_config.stream_dests),
            &count, app_config.stream_dests);
        *app_config.stream_dests[count] = '\0';
        if (err != CONFIG_OK)
            goto RET_ERR;
    }

    parse_bool(&ini, "audio", "enable", &app_config.audio_enable);
    if (app_config.audio_enable) {
        {
            const char *possible_values[] = {"MP3", "AAC", "AAC-LC"};
            const int count = sizeof(possible_values) / sizeof(const char *);
            int val = 0;
            if (parse_enum(&ini, "audio", "codec", (void *)&val,
                    possible_values, count, 0) == CONFIG_OK) {
                if (val == 0)
                    app_config.audio_codec = HAL_AUDCODEC_MP3;
                else
                    app_config.audio_codec = HAL_AUDCODEC_AAC;
            }
        }
        parse_int(&ini, "audio", "bitrate", 32, 320, &app_config.audio_bitrate);
        // `audio.gain` semantics are platform-specific.
        // - hisi/v4: user-friendly 0..100 (50 = unity gain, 0 = quieter, 100 = louder)
        // - others: legacy dB level range (-60..+30) used by platform HALs
        if (plat == HAL_PLATFORM_V4)
            parse_int(&ini, "audio", "gain", 0, 100, &app_config.audio_gain);
        else
            parse_int(&ini, "audio", "gain", -60, 30, &app_config.audio_gain);
        err = parse_int(&ini, "audio", "srate", 8000, 96000, 
            &app_config.audio_srate);
        if (err != CONFIG_OK)
            goto RET_ERR;
        parse_int(&ini, "audio", "channels", 1, 2, (int *)&app_config.audio_channels);

        // AAC-only advanced options (FAAC).
        // `aac_quantqual` enables quality/VBR mode when > 0 (range 10..5000 in our FAAC build).
        // In this mode `bitrate` is ignored by FAAC (bitRate=0).
        parse_int(&ini, "audio", "aac_quantqual", 0, 5000, (int *)&app_config.audio_aac_quantqual);
        // Encoder bandwidth in Hz; 0 lets FAAC choose defaults.
        parse_int(&ini, "audio", "aac_bandwidth", 0, 96000, (int *)&app_config.audio_aac_bandwidth);
        // Temporal Noise Shaping (TNS).
        parse_bool(&ini, "audio", "aac_tns", &app_config.audio_aac_tns);

        // SpeexDSP preprocess options (used only when codec=AAC).
        parse_bool(&ini, "audio", "speex_enable", &app_config.audio_speex_enable);
        parse_bool(&ini, "audio", "speex_denoise", &app_config.audio_speex_denoise);
        parse_bool(&ini, "audio", "speex_agc", &app_config.audio_speex_agc);
        parse_bool(&ini, "audio", "speex_vad", &app_config.audio_speex_vad);
        parse_bool(&ini, "audio", "speex_dereverb", &app_config.audio_speex_dereverb);
        {
            int fs = 0;
            // 0 or missing = auto (srate/50). Values < 80 are clamped at runtime.
            if (parse_int(&ini, "audio", "speex_frame_size", 0, 8192, &fs) == CONFIG_OK)
                app_config.audio_speex_frame_size = (unsigned int)fs;
        }
        parse_int(&ini, "audio", "speex_noise_suppress_db", -60, 0, &app_config.audio_speex_noise_suppress_db);
        parse_int(&ini, "audio", "speex_agc_level", 1, 32768, &app_config.audio_speex_agc_level);
        parse_int(&ini, "audio", "speex_agc_increment", 0, 100, &app_config.audio_speex_agc_increment);
        parse_int(&ini, "audio", "speex_agc_decrement", 0, 100, &app_config.audio_speex_agc_decrement);
        parse_int(&ini, "audio", "speex_agc_max_gain_db", 0, 60, &app_config.audio_speex_agc_max_gain_db);
        parse_int(&ini, "audio", "speex_vad_prob_start", 0, 100, &app_config.audio_speex_vad_prob_start);
        parse_int(&ini, "audio", "speex_vad_prob_continue", 0, 100, &app_config.audio_speex_vad_prob_continue);
    }

    parse_bool(&ini, "mp4", "enable", &app_config.mp4_enable);
    if (app_config.mp4_enable) {
        {
            // Accept: H.264/H264/AVC, H.265/H265/HEVC, and "+" variants: H.264+/H264+/AVC+.
            // Do NOT rely on enum index parity (e.g. "H264+" used to map to H.265 by accident).
            char codec_raw[32] = {0};
            if (parse_param_value(&ini, "mp4", "codec", codec_raw) == CONFIG_OK) {
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
            const char *possible_values[] = {"CBR", "VBR", "QP", "ABR", "AVBR"};
            const int count = sizeof(possible_values) / sizeof(const char *);
            int val = 0;
            parse_enum(&ini, "mp4", "mode", (void *)&val,
                possible_values, count, 0);
            app_config.mp4_mode = val;
        }
        err = parse_int(
            &ini, "mp4", "width", 160, INT_MAX, &app_config.mp4_width);
        if (err != CONFIG_OK)
            goto RET_ERR;
        err = parse_int(
            &ini, "mp4", "height", 120, INT_MAX, &app_config.mp4_height);
        if (err != CONFIG_OK)
            goto RET_ERR;
        err = parse_int(&ini, "mp4", "fps", 1, INT_MAX, &app_config.mp4_fps);
        if (err != CONFIG_OK)
            goto RET_ERR;
        app_config.mp4_gop = app_config.mp4_fps;
        parse_int(&ini, "mp4", "gop", 1, INT_MAX, &app_config.mp4_gop);
        {
            const char *possible_values[] = {"BP", "MP", "HP"};
            const int count = sizeof(possible_values) / sizeof(const char *);
            const char *possible_values2[] = {"BASELINE", "MAIN", "HIGH"};
            const int count2 = sizeof(possible_values2) / sizeof(const char *);
            int val = 0;
            if (parse_enum(&ini, "mp4", "profile", (void *)&val,
                    possible_values, count, 0) != CONFIG_OK)
                parse_enum( &ini, "mp4", "profile", (void *)&val,
                    possible_values2, count2, 0);
            app_config.mp4_profile = val;
        }
        parse_int(&ini, "mp4", "profile", 0, 2, &app_config.mp4_profile);

        err = parse_int(
            &ini, "mp4", "bitrate", 32, INT_MAX, &app_config.mp4_bitrate);
        if (err != CONFIG_OK)
            goto RET_ERR;
    }

    // JPEG section represents MJPEG (multipart JPEG) stream.
    // Accept legacy section name "mjpeg" as an alias.
    bool legacy_mjpeg_enable = false;
    bool jpeg_from_legacy = false;

    err = parse_bool(&ini, "jpeg", "enable", &app_config.jpeg_enable);
    if (err != CONFIG_OK)
        goto RET_ERR;
    // Legacy alias: older configs used [mjpeg]. This section is optional.
    // If it's missing, just treat it as disabled.
    err = parse_bool(&ini, "mjpeg", "enable", &legacy_mjpeg_enable);
    if (err != CONFIG_OK) {
        if (err == CONFIG_SECTION_NOT_FOUND || err == CONFIG_PARAM_NOT_FOUND) {
            legacy_mjpeg_enable = false;
        } else {
            goto RET_ERR;
        }
    }

    const char *jpeg_sec = "jpeg";
    if (!app_config.jpeg_enable && legacy_mjpeg_enable) {
        app_config.jpeg_enable = true;
        jpeg_from_legacy = true;
        jpeg_sec = "mjpeg";
    }

    // Per-stream OSD control (default: true).
    // Prefer the chosen section, but allow legacy field to populate defaults.
    parse_bool(&ini, jpeg_sec, "osd_enable", &app_config.jpeg_osd_enable);
    if (!jpeg_from_legacy)
        parse_bool(&ini, "mjpeg", "osd_enable", &app_config.jpeg_osd_enable);

    if (app_config.jpeg_enable) {
        {
            const char *possible_values[] = {"CBR", "VBR", "QP"};
            const int count = sizeof(possible_values) / sizeof(const char *);
            int val = 0;
            parse_enum(&ini, jpeg_sec, "mode", (void *)&val,
                possible_values, count, 0);
            // MJPEG is configured via JPEG quality factor (qfactor) now.
            // Keep parsing `mode` for compatibility, but force QP in runtime.
            app_config.jpeg_mode = val;
        }
        err = parse_int(
            &ini, jpeg_sec, "width", 160, INT_MAX, &app_config.jpeg_width);
        if (err != CONFIG_OK)
            goto RET_ERR;
        err = parse_int(
            &ini, jpeg_sec, "height", 120, INT_MAX, &app_config.jpeg_height);
        if (err != CONFIG_OK)
            goto RET_ERR;
        err =
            parse_int(&ini, jpeg_sec, "fps", 1, INT_MAX, &app_config.jpeg_fps);
        if (err != CONFIG_OK)
            goto RET_ERR;
        // Preferred MJPEG quality control.
        err = parse_int(&ini, jpeg_sec, "qfactor", 1, 99, &app_config.jpeg_qfactor);
        if (err != CONFIG_OK)
            goto RET_ERR;

        // Backwards compatibility: accept (but ignore) legacy `bitrate`.
        // This intentionally does not affect MJPEG encoding anymore.
        {
            int legacy_bitrate = 0;
            parse_int(&ini, jpeg_sec, "bitrate", 0, INT_MAX, &legacy_bitrate);
        }

        // MJPEG bitrate control is deprecated; always operate in QP mode.
        app_config.jpeg_mode = HAL_VIDMODE_QP;
    }

    parse_bool(&ini, "http_post", "enable", &app_config.http_post_enable);
    if (app_config.http_post_enable) {
        err = parse_param_value(
            &ini, "http_post", "host", app_config.http_post_host);
        if (err != CONFIG_OK)
            goto RET_ERR;
        err = parse_param_value(
            &ini, "http_post", "url", app_config.http_post_url);
        if (err != CONFIG_OK)
            goto RET_ERR;

        err = parse_param_value(
            &ini, "http_post", "login", app_config.http_post_login);
        if (err != CONFIG_OK)
            goto RET_ERR;
        err = parse_param_value(
            &ini, "http_post", "password", app_config.http_post_password);
        if (err != CONFIG_OK)
            goto RET_ERR;

        err = parse_int(
            &ini, "http_post", "width", 160, INT_MAX,
            &app_config.http_post_width);
        if (err != CONFIG_OK)
            goto RET_ERR;
        err = parse_int(
            &ini, "http_post", "height", 120, INT_MAX,
            &app_config.http_post_height);
        if (err != CONFIG_OK)
            goto RET_ERR;
        err = parse_int(
            &ini, "http_post", "interval", 1, INT_MAX,
            &app_config.http_post_interval);
        if (err != CONFIG_OK)
            goto RET_ERR;
        err = parse_int(
            &ini, "http_post", "qfactor", 1, 99, &app_config.http_post_qfactor);
        if (err != CONFIG_OK)
            goto RET_ERR;
    }

    free(ini.str);
    return CONFIG_OK;
RET_ERR:
    free(ini.str);
    return err;
}