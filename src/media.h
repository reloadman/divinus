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
// Returns 0 on success and writes 0..255 average luma into *lum.
// Currently implemented only for hisi/v4; other platforms return non-zero.
int get_isp_avelum(unsigned char *lum);
// Returns 0 on success and writes ISP exposure info fields.
// Currently implemented only for hisi/v4; other platforms return non-zero.
int get_isp_exposure_info(unsigned int *iso, unsigned int *exp_time,
    unsigned int *again, unsigned int *dgain, unsigned int *ispdgain,
    int *exposure_is_max);
int take_next_free_channel(bool mainLoop);

int create_channel(char index, short width, short height, char framerate, char jpeg);
int bind_channel(char index, char framerate, char jpeg);
int unbind_channel(char index, char jpeg);
int disable_video(char index, char jpeg);

void disable_audio(void);
int enable_audio(void);
int disable_mjpeg(void);
int enable_mjpeg(void);
int disable_mp4(void);
int enable_mp4(void);

// Returns the last encoded MJPEG frame as a raw JPEG bitstream.
// - Returns 0 on success, non-zero if no frame is available within timeout.
// - Copies data into `jpeg->data` (reallocs as needed) and sets jpeg->jpegSize.
int media_get_last_mjpeg_frame(hal_jpegdata *jpeg, unsigned int timeout_ms);