#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hal/config.h"
#include "hal/support.h"
#include "region.h"

// Single, fixed config path (no fallback search).
// If you need a different location, change it here and rebuild.
#ifndef DIVINUS_CONFIG_PATH
#define DIVINUS_CONFIG_PATH "/etc/divinus.yaml"
#endif

struct AppConfig {
    // [system]
    char sensor_config[128];
    // Optional ISP/IQ profile INI (platform-specific; used by hisi/v4 goke as "scene_auto" IQ)
    char iq_config[256];
    unsigned short web_port;
    char web_bind[64];
    char web_whitelist[4][256];
    bool web_enable_auth;
    char web_auth_user[32];
    char web_auth_pass[32];
    bool web_auth_skiplocal;
    bool web_enable_static;
    unsigned int isp_thread_stack_size;
    unsigned int venc_stream_thread_stack_size;
    unsigned int web_server_thread_stack_size;
    // Stack size for night mode worker thread (auto day/night switching + IQ reload).
    // Some SDK/HAL paths (notably hisi/v4 IQ reload) can require more stack than 16KB.
    unsigned int night_thread_stack_size;
    unsigned int watchdog;

    // [night_mode]
    bool night_mode_enable;
    // Persisted manual mode (true disables automatic switching).
    bool night_mode_manual;
    // If true, enable encoder grayscale when switching to night (IR) mode.
    // If false, stay in color when switching to night (IR) mode.
    bool night_mode_grayscale;
    // GPIO pins can be configured as:
    // - 999: disabled
    // - N (0..PIN_MAX): GPIO number
    // - any negative value: treated as disabled (for legacy configs)
    int ir_cut_pin1;
    int ir_cut_pin2;
    int ir_led_pin;
    int white_led_pin;
    int ir_sensor_pin;
    unsigned int check_interval_s;
    unsigned int pin_switch_delay_us;
    char adc_device[128];
    int adc_threshold;
    // ISP-derived day/night trigger (hisi/v4 only). -1 means "unset/disabled".
    int isp_lum_low;
    int isp_lum_hi;
    // ISP-derived day/night trigger by ISO (hisi/v4 only). -1 means "unset/disabled".
    int isp_iso_low;
    int isp_iso_hi;
    // ISP-derived day/night trigger helper (hisi/v4 only). -1 means "unset/disabled".
    // Used to decide when to attempt leaving IR mode (probe day): if exptime <= this value.
    int isp_exptime_low;
    // Minimum time (seconds) between mode switches to avoid oscillations.
    unsigned int isp_switch_lockout_s;

    // [isp]
    // Fixed per-device orientation (factory), applied before user-facing mirror/flip.
    bool sensor_mirror;
    bool sensor_flip;
    bool mirror;
    bool flip;
    int antiflicker;

    // [osd]
    bool osd_enable;
    // If true, show ISP/AE/IQ debug text in bottom-left (1 Hz).
    bool osd_isp_debug;

    // [mdns]
    bool mdns_enable;

    // [onvif]
    bool onvif_enable;
    bool onvif_enable_auth;
    char onvif_auth_user[32];
    char onvif_auth_pass[32];

    // [rtsp]
    bool rtsp_enable;
    bool rtsp_enable_auth;
    char rtsp_auth_user[32];
    char rtsp_auth_pass[32];
    int rtsp_port;
    char rtsp_bind[64];

    // [record]
    bool record_enable;
    bool record_continuous;
    char record_filename[128];
    char record_path[128];
    int record_segment_duration;
    int record_segment_size;

    // [stream]
    bool stream_enable;
    unsigned short stream_udp_srcport;
    char stream_dests[4][256];

    // [audio]
    bool audio_enable;
    // Persisted mute state: keep audio/RTSP track alive, but send silence.
    bool audio_mute;
    hal_audcodec audio_codec;
    unsigned int audio_bitrate;
    int audio_gain;
    unsigned int audio_srate;
    unsigned char audio_channels;
    // AAC (FAAC) advanced tuning. These are ignored unless audio_codec == AAC.
    // - aac_quantqual: if > 0 enables quality/VBR mode (FAAC quantqual) and disables `audio_bitrate`.
    // - aac_bandwidth: encoder bandwidth in Hz (0 = auto).
    // - aac_tns: enable Temporal Noise Shaping.
    unsigned int audio_aac_quantqual;
    unsigned int audio_aac_bandwidth;
    bool audio_aac_tns;

    // SpeexDSP (preprocess) for AAC PCM path (denoise/AGC/VAD/dereverb).
    // NOTE: This is applied only when audio_codec == AAC and channels == 1.
    bool audio_speex_enable;              // master switch (bypass everything when false)
    bool audio_speex_denoise;
    bool audio_speex_agc;
    bool audio_speex_vad;
    bool audio_speex_dereverb;
    // Number of PCM samples per channel per preprocess call.
    // Typical values: 160 @ 8kHz, 320 @ 16kHz, 960 @ 48kHz (20 ms).
    // If 0, it's computed automatically as `audio_srate / 50` (20 ms).
    unsigned int audio_speex_frame_size;
    // Noise suppression (dB), typical -15..-30.
    int audio_speex_noise_suppress_db;
    // AGC target level (SpeexDSP expects float in floating-point builds).
    // SpeexDSP clamps it to [1..32768].
    int audio_speex_agc_level;
    // AGC speed and limits (dB units expected by SpeexDSP ctl).
    int audio_speex_agc_increment;
    int audio_speex_agc_decrement;
    int audio_speex_agc_max_gain_db;
    // VAD thresholds as probabilities in percent [0..100].
    int audio_speex_vad_prob_start;
    int audio_speex_vad_prob_continue;

    // [mp4]
    bool mp4_enable;
    bool mp4_codecH265;
    bool mp4_h264_plus;
    unsigned int mp4_mode;
    unsigned int mp4_fps;
    unsigned int mp4_gop;
    unsigned int mp4_width;
    unsigned int mp4_height;
    unsigned int mp4_profile;
    unsigned int mp4_bitrate;

    // [jpeg]
    bool jpeg_enable;
    bool jpeg_osd_enable;
    // If true, MJPEG/JPEG encoder channels will follow night_mode.grayscale toggles
    // (best-effort; platform/SDK dependent).
    bool jpeg_grayscale_night;
    unsigned int jpeg_mode;
    unsigned int jpeg_fps;
    unsigned int jpeg_width;
    unsigned int jpeg_height;
    unsigned int jpeg_qfactor;

    // [http_post]
    bool http_post_enable;
    char http_post_host[128];
    char http_post_url[128];
    char http_post_login[128];
    char http_post_password[128];
    unsigned int http_post_width;
    unsigned int http_post_height;
    unsigned int http_post_qfactor;
    unsigned int http_post_interval;
};

extern struct AppConfig app_config;
enum ConfigError parse_app_config(void);
int save_app_config(void);
// Sanitize and update the canonical/runtime time format strings.
bool timefmt_set(const char *src);
// Repair runtime timefmt if corrupted (uses canonical copy as source).
void timefmt_repair_runtime(void);
