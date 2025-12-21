#pragma once

#include "v4_common.h"
#include "v4_aud.h"
#include "v4_config.h"
#include "v4_isp.h"
#include "v4_rgn.h"
#include "v4_snr.h"
#include "v4_sys.h"
#include "v4_vb.h"
#include "v4_venc.h"
#include "v4_vi.h"
#include "v4_vpss.h"

#include "../support.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

extern char audioOn, keepRunning;

extern hal_chnstate v4_state[V4_VENC_CHN_NUM];
extern int (*v4_aud_cb)(hal_audframe*);
extern int (*v4_vid_cb)(char, hal_vidstream*);

void v4_hal_deinit(void);
int v4_hal_init(void);
void *v4_audio_thread(void);

void v4_audio_deinit(void);
int v4_audio_init(int samplerate, int gain_percent);

int v4_channel_bind(char index);
int v4_channel_create(char index, char mirror, char flip, char framerate);
int v4_channel_grayscale(char enable);
// Best-effort runtime orientation update for all enabled channels.
// Applies VPSS mirror/flip and may cause a short frame gap, but does not stop RTSP/VENC.
// Returns 0 on success, non-zero otherwise.
int v4_channel_set_orientation(char mirror, char flip, char h26x_fps, char mjpeg_fps);
int v4_channel_unbind(char index);

void *v4_image_thread(void);

// Returns 0 on success and writes 0..255 average luminance into *lum.
int v4_get_isp_avelum(unsigned char *lum);

// Returns 0 on success and writes ISP exposure info fields.
int v4_get_isp_exposure_info(unsigned int *iso, unsigned int *exp_time,
    unsigned int *again, unsigned int *dgain, unsigned int *ispdgain,
    int *exposure_is_max);

// Returns 0 on success and writes current DRC strength into *strength.
// Best-effort: may return non-zero if the platform/SDK doesn't expose it.
int v4_get_drc_strength(unsigned int *strength);

// Returns 0 on success and writes 0/1 into *active depending on whether
// the low-light auto-AE path is currently considered active (v4 dynamic IQ).
int v4_get_iq_lowlight_state(unsigned int iso, unsigned int exp_time, int *active);

// Re-apply IQ config at runtime (best-effort).
// Useful when switching DAY <-> IR so that [static_*] vs [ir_static_*] sections take effect.
// Returns 0 on success, non-zero otherwise.
int v4_iq_reload(void);

// Returns 0 on success and writes current AUTO AE knobs.
// Useful for OSD/debug: shows what profile is actually active.
int v4_get_ae_auto_params(unsigned int *comp, unsigned int *expmax, unsigned int *sysgainmax);

int v4_pipeline_create(const char *iqConfig);
void v4_pipeline_destroy(void);

int v4_region_create(char handle, hal_rect rect, short opacity);
// Extended: allow separate alpha for background (alpha-bit 0) and foreground (alpha-bit 1).
int v4_region_create_ex(char handle, hal_rect rect, short fg_opacity, short bg_opacity);
void v4_region_destroy(char handle);
int v4_region_setbitmap(int handle, hal_bitmap *bitmap);

void v4_sensor_deconfig(void);
int v4_sensor_config(void);
void v4_sensor_deinit(void);
int v4_sensor_init(char *name, char *obj);

int v4_video_create(char index, hal_vidconfig *config);
int v4_video_destroy(char index);
int v4_video_destroy_all(void);
void v4_video_request_idr(char index);
int v4_video_snapshot_grab(char index, hal_jpegdata *jpeg);
void *v4_video_thread(void);

int v4_system_calculate_block(short width, short height, v4_common_pixfmt pixFmt,
    unsigned int alignWidth);
void v4_system_deinit(void);
int v4_system_init(char *snrConfig);
float v4_system_readtemp(void);