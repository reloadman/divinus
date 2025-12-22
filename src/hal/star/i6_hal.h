#pragma once

#include "i6_common.h"
#include "i6_aud.h"
#include "i6_ipu.h"
#include "i6_isp.h"
#include "i6_rgn.h"
#include "i6_snr.h"
#include "i6_sys.h"
#include "i6_venc.h"
#include "i6_vif.h"
#include "i6_vpe.h"

#include "../support.h"

#include <sys/select.h>
#include <unistd.h>

extern char audioOn, keepRunning;

extern hal_chnstate i6_state[I6_VENC_CHN_NUM];
extern int (*i6_aud_cb)(hal_audframe*);
extern int (*i6_vid_cb)(char, hal_vidstream*);

void i6_hal_deinit(void);
int i6_hal_init(void);

void i6_audio_deinit(void);
int i6_audio_init(int samplerate, int gain);
void *i6_audio_thread(void);

int i6_channel_bind(char index, char framerate);
int i6_channel_create(char index, short width, short height, char jpeg);
int i6_channel_grayscale(char enable);
int i6_channel_unbind(char index);

int i6_config_load(char *path);

int i6_pipeline_create(char sensor, short width, short height, char mirror, char flip, char framerate);
void i6_pipeline_destroy(void);
// Best-effort runtime orientation update (platform support dependent).
// Returns 0 on success.
int i6_set_orientation(char mirror, char flip);

int i6_region_create(char handle, hal_rect rect, short opacity);
// Extended: allow separate alpha for background (alpha-bit 0) and foreground (alpha-bit 1).
int i6_region_create_ex(char handle, hal_rect rect, short fg_opacity, short bg_opacity);
void i6_region_deinit(void);
void i6_region_destroy(char handle);
void i6_region_init(void);
int i6_region_setbitmap(int handle, hal_bitmap *bitmap);

int i6_sensor_exposure(unsigned int micros);

// Best-effort exposure readback for ISP debug OSD.
// Values are derived from MI_SNR_GetPlaneInfo (shutter/sensor gain/comp gain).
// Units:
// - exp_time: microseconds
// - again/dgain/ispdgain: fixed-point (x1024) as reported by MI_SNR
int i6_get_isp_exposure_info(unsigned int *iso, unsigned int *exp_time,
    unsigned int *again, unsigned int *dgain, unsigned int *ispdgain,
    int *exposure_is_max);

int i6_video_create(char index, hal_vidconfig *config);
int i6_video_destroy(char index);
int i6_video_destroy_all(void);
void i6_video_request_idr(char index);
int i6_video_snapshot_grab(char index, char quality, hal_jpegdata *jpeg);
void *i6_video_thread(void);

void i6_system_deinit(void);
int i6_system_init(void);