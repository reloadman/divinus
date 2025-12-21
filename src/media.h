#pragma once

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "app_config.h"
#include "error.h"
#include "hal/types.h"
#include "http_post.h"
#include "jpeg.h"
#include "rtsp_smol.h"
#include "server.h"
#include "stream.h"

extern char audioOn, recordOn, udpOn;

int start_sdk(void);
int stop_sdk(void);

int start_streaming(void);
void stop_streaming(void);

void request_idr(void);
void set_grayscale(bool active);
// Best-effort runtime orientation update. Returns 0 on success, non-zero otherwise.
int media_set_isp_orientation(bool mirror, bool flip);
// Best-effort runtime IQ reload. Useful after toggling DAY <-> IR.
// Returns 0 on success, non-zero otherwise.
int media_reload_iq(void);
// Returns 0 on success and writes 0..255 average luma into *lum.
// Currently implemented only for hisi/v4; other platforms return non-zero.
int get_isp_avelum(unsigned char *lum);
// Returns 0 on success and writes ISP exposure info fields.
// Currently implemented only for hisi/v4; other platforms return non-zero.
int get_isp_exposure_info(unsigned int *iso, unsigned int *exp_time,
    unsigned int *again, unsigned int *dgain, unsigned int *ispdgain,
    int *exposure_is_max);

// Returns 0 on success and writes current DRC strength into *strength.
// Currently best-effort and implemented only for hisi/v4.
int get_isp_drc_strength(unsigned int *strength);

// Returns 0 on success and writes 0/1 into *active indicating whether
// low-light auto-AE is currently considered active (platform-dependent).
// Currently implemented only for hisi/v4.
int get_iq_lowlight_state(unsigned int iso, unsigned int exp_time, int *active);

// Returns 0 on success and writes current AUTO AE knobs.
// Useful to confirm which AE profile is actually active.
int get_isp_ae_auto_params(unsigned int *comp, unsigned int *expmax, unsigned int *sysgainmax);
int take_next_free_channel(bool mainLoop);

int create_channel(char index, short width, short height, char framerate, char jpeg);
int bind_channel(char index, char framerate, char jpeg);
int unbind_channel(char index, char jpeg);
int disable_video(char index, char jpeg);

void disable_audio(void);
int enable_audio(void);
// Runtime audio muting: keep RTSP/audio track alive, but feed silence to encoder.
// NOTE: This does NOT remove the RTSP audio track; it only affects PCM content.
void media_set_audio_mute(int muted);
int media_get_audio_mute(void);
// Best-effort runtime bitrate update (AAC bitrate mode only; ignored in VBR/quantqual mode).
void media_set_audio_bitrate_kbps(unsigned int kbps);
int disable_mjpeg(void);
int enable_mjpeg(void);
int disable_mp4(void);
int enable_mp4(void);

// Returns the last encoded MJPEG frame as a raw JPEG bitstream.
// - Returns 0 on success, non-zero if no frame is available within timeout.
// - Copies data into `jpeg->data` (reallocs as needed) and sets jpeg->jpegSize.
int media_get_last_mjpeg_frame(hal_jpegdata *jpeg, unsigned int timeout_ms);