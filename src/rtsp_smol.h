// SmolRTSP integration layer for divinus.
#pragma once

#include <stdint.h>
#include <stddef.h>

int smolrtsp_server_start(void);
void smolrtsp_server_stop(void);

// Push encoded elementary streams into all active RTSP sessions.
// Video buffer should be a single NALU with start code (H.264/H.265).
int smolrtsp_push_video(const uint8_t *buf, size_t len, int is_h265, uint64_t ts_us);
// AAC-LC elementary stream; timestamp in microseconds (if unavailable pass 0).
int smolrtsp_push_aac(const uint8_t *buf, size_t len, uint64_t ts_us);
