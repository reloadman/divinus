#include "media.h"
#include "hal/config.h"
#include <faac.h>
#if defined(DIVINUS_WITH_SPEEXDSP)
#include <speex/speex_preprocess.h>
#endif

char audioOn = 0, udpOn = 0;
pthread_mutex_t aencMtx, chnMtx, mp4Mtx;
pthread_t aencPid = 0, audPid = 0, ispPid = 0, vidPid = 0;

static hal_audcodec active_audio_codec;

// Cache the last MJPEG (JPEG) frame so snapshots (/image.jpg, http_post, etc)
// can be served without creating a dedicated JPEG encoder channel.
static pthread_mutex_t mjpeg_last_mtx = PTHREAD_MUTEX_INITIALIZER;
static unsigned char *mjpeg_last_buf = NULL;
static size_t mjpeg_last_cap = 0;
static size_t mjpeg_last_len = 0;
static uint64_t mjpeg_last_ts = 0;
static int mjpeg_last_valid = 0;

static void mjpeg_last_clear(void) {
    pthread_mutex_lock(&mjpeg_last_mtx);
    mjpeg_last_valid = 0;
    mjpeg_last_len = 0;
    mjpeg_last_ts = 0;
    pthread_mutex_unlock(&mjpeg_last_mtx);
}

static void mjpeg_last_update(const unsigned char *buf, size_t len, uint64_t ts) {
    if (!buf || !len)
        return;
    pthread_mutex_lock(&mjpeg_last_mtx);
    if (len > mjpeg_last_cap) {
        unsigned char *tmp = realloc(mjpeg_last_buf, len);
        if (!tmp) {
            pthread_mutex_unlock(&mjpeg_last_mtx);
            return;
        }
        mjpeg_last_buf = tmp;
        mjpeg_last_cap = len;
    }
    memcpy(mjpeg_last_buf, buf, len);
    mjpeg_last_len = len;
    mjpeg_last_ts = ts;
    mjpeg_last_valid = 1;
    pthread_mutex_unlock(&mjpeg_last_mtx);
}

int media_get_last_mjpeg_frame(hal_jpegdata *jpeg, unsigned int timeout_ms) {
    if (!jpeg)
        return EXIT_FAILURE;

    // Wait a bit for the first frame after startup/reconfigure.
    const unsigned int step_ms = 10;
    unsigned int waited = 0;
    while (waited <= timeout_ms) {
        pthread_mutex_lock(&mjpeg_last_mtx);
        const int valid = mjpeg_last_valid;
        const size_t len = mjpeg_last_len;
        // Snapshot state quickly, then copy outside? We need the buffer stable.
        // Keep the mutex while copying to guarantee coherent data.
        if (valid && len > 0 && mjpeg_last_buf) {
            if (len > jpeg->length) {
                unsigned char *tmp = realloc(jpeg->data, len);
                if (!tmp) {
                    pthread_mutex_unlock(&mjpeg_last_mtx);
                    return EXIT_FAILURE;
                }
                jpeg->data = tmp;
                jpeg->length = (unsigned int)len;
            }
            memcpy(jpeg->data, mjpeg_last_buf, len);
            jpeg->jpegSize = (unsigned int)len;
            pthread_mutex_unlock(&mjpeg_last_mtx);
            return EXIT_SUCCESS;
        }
        pthread_mutex_unlock(&mjpeg_last_mtx);

        if (waited == timeout_ms)
            break;
        usleep(step_ms * 1000);
        waited += step_ms;
        if (waited > timeout_ms)
            waited = timeout_ms;
    }

    return EXIT_FAILURE;
}

// Prevent unbounded growth if downstream (network/client) stalls.
// When exceeded, we drop queued audio to keep capture threads healthy.
#define AUDIO_ENC_BUF_MAX (32 * 1024)

struct BitBuf mp3Buf;
shine_config_t mp3Cnf;
shine_t mp3Enc;
unsigned int pcmPos;
unsigned int pcmSamp;
short pcmSrc[SHINE_MAX_SAMPLES];

struct BitBuf aacBuf;
faacEncHandle aacEnc = NULL;
unsigned long aacInputSamples = 0;
unsigned long aacMaxOutputBytes = 0;
int16_t *aacPcm = NULL;      // encoder input buffer (16-bit LE PCM)
unsigned char *aacOut = NULL;
unsigned int aacPcmPos = 0;
unsigned int aacChannels = 1;
int16_t *aacStash = NULL;    // stash of incoming PCM (16-bit LE samples)
unsigned int aacStashLen = 0;
uint64_t aacStashTsUs = 0;

#if defined(DIVINUS_WITH_SPEEXDSP)
// SpeexDSP preprocess state for AAC PCM path (optional).
static struct {
    SpeexPreprocessState *st;
    unsigned int srate;
    unsigned int channels;
    unsigned int frame_size;      // samples per channel per preprocess run
    int active;
    int16_t *stash;               // input stash (interleaved, but we only support mono)
    unsigned int stash_len;       // in samples (int16_t)
    unsigned int stash_cap;       // in samples (int16_t)
} speex_aac = {0};
#endif

// Temporary buffers to send an extracted encoded frame without holding aencMtx.
static unsigned char *mp3_send_buf = NULL;
static size_t mp3_send_cap = 0;
static unsigned char *aac_send_buf = NULL;
static size_t aac_send_cap = 0;

static inline uint8_t aac_samplerate_index(uint32_t srate) {
    switch (srate) {
        case 96000: return 0;
        case 88200: return 1;
        case 64000: return 2;
        case 48000: return 3;
        case 44100: return 4;
        case 32000: return 5;
        case 24000: return 6;
        case 22050: return 7;
        case 16000: return 8;
        case 12000: return 9;
        case 11025: return 10;
        case 8000:  return 11;
        case 7350:  return 12;
        default:    return 3; // fallback to 48 kHz index
    }
}

static void *aenc_thread_mp3(void);
static void *aenc_thread_aac(void);
static int save_audio_stream_mp3(hal_audframe *frame);
static int save_audio_stream_aac(hal_audframe *frame);

static inline void aac_stash_append(const int16_t *pcm16, unsigned int total_samples) {
    if (!aacStash || total_samples == 0)
        return;
    // Keep enough headroom for jitter/bursts (matches existing allocation).
    unsigned int channels = aacChannels ? aacChannels : 1;
    unsigned int max_stash = (unsigned int)aacInputSamples * channels * 4U;
    if (aacStashLen + total_samples > max_stash) {
        // If encoder can't keep up, drop accumulated samples and keep newest.
        aacStashLen = 0;
    }
    if (aacStashLen + total_samples <= max_stash) {
        memcpy(aacStash + aacStashLen, pcm16, total_samples * sizeof(int16_t));
        aacStashLen += total_samples;
    }
}

#if defined(DIVINUS_WITH_SPEEXDSP)
static void speex_aac_reset(void) {
    if (speex_aac.st) {
        speex_preprocess_state_destroy(speex_aac.st);
        speex_aac.st = NULL;
    }
    free(speex_aac.stash);
    speex_aac.stash = NULL;
    speex_aac.stash_len = 0;
    speex_aac.stash_cap = 0;
    speex_aac.srate = 0;
    speex_aac.channels = 0;
    speex_aac.frame_size = 0;
    speex_aac.active = 0;
}

static void speex_aac_init_from_config(unsigned int srate, unsigned int channels) {
    speex_aac_reset();

    // Use SpeexDSP only for AAC and mono input (simple, predictable CPU).
    if (!app_config.audio_speex_enable)
        return;
    if (channels != 1) {
        HAL_WARNING("media", "SpeexDSP preprocess is enabled but channels=%u; only mono is supported, bypassing.\n",
            channels);
        return;
    }

    unsigned int frame_size = app_config.audio_speex_frame_size;
    if (frame_size == 0)
        frame_size = srate / 50U; // 20 ms fallback
    if (frame_size < 80U)
        frame_size = 80U;
    if (frame_size > 8192U)
        frame_size = 8192U;

    SpeexPreprocessState *st = speex_preprocess_state_init((int)frame_size, (int)srate);
    if (!st) {
        HAL_ERROR("media", "SpeexDSP preprocess init failed (frame_size=%u srate=%u); bypassing.\n",
            frame_size, srate);
        return;
    }

    // Apply config. Most ctl() take spx_int32_t* in our fixed-point build.
    spx_int32_t v = 0;
    v = app_config.audio_speex_denoise ? 1 : 0;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_DENOISE, &v);
    v = app_config.audio_speex_agc ? 1 : 0;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC, &v);
    v = app_config.audio_speex_vad ? 1 : 0;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_VAD, &v);
    v = app_config.audio_speex_dereverb ? 1 : 0;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_DEREVERB, &v);

    v = (spx_int32_t)app_config.audio_speex_noise_suppress_db;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, &v);
    {
        // SpeexDSP expects float* for AGC_LEVEL in floating-point builds.
        // Keep config type numeric and cast here.
        float agc_level = (float)app_config.audio_speex_agc_level;
        speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_LEVEL, &agc_level);
    }
    v = (spx_int32_t)app_config.audio_speex_agc_increment;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_INCREMENT, &v);
    v = (spx_int32_t)app_config.audio_speex_agc_decrement;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_DECREMENT, &v);
    v = (spx_int32_t)app_config.audio_speex_agc_max_gain_db;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_MAX_GAIN, &v);
    v = (spx_int32_t)app_config.audio_speex_vad_prob_start;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_START, &v);
    v = (spx_int32_t)app_config.audio_speex_vad_prob_continue;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_CONTINUE, &v);

    // Allocate stash for incoming PCM. Keep a small multiple of frame size.
    unsigned int cap = frame_size * 8U;
    int16_t *stash = calloc(cap, sizeof(int16_t));
    if (!stash) {
        HAL_ERROR("media", "SpeexDSP stash allocation failed (%u samples); bypassing.\n", cap);
        speex_preprocess_state_destroy(st);
        return;
    }

    speex_aac.st = st;
    speex_aac.srate = srate;
    speex_aac.channels = channels;
    speex_aac.frame_size = frame_size;
    speex_aac.stash = stash;
    speex_aac.stash_cap = cap;
    speex_aac.stash_len = 0;
    speex_aac.active = 1;

    HAL_INFO("media", "SpeexDSP preprocess enabled for AAC: frame_size=%u srate=%u (denoise=%d agc=%d vad=%d dereverb=%d)\n",
        frame_size, srate,
        app_config.audio_speex_denoise ? 1 : 0,
        app_config.audio_speex_agc ? 1 : 0,
        app_config.audio_speex_vad ? 1 : 0,
        app_config.audio_speex_dereverb ? 1 : 0);
}

static inline void speex_aac_push_pcm(const int16_t *pcm16, unsigned int total_samples) {
    if (!speex_aac.active || !speex_aac.st || !speex_aac.stash || total_samples == 0) {
        aac_stash_append(pcm16, total_samples);
        return;
    }

    // Mono only in this integration.
    const unsigned int frame_samples = speex_aac.frame_size;
    if (frame_samples == 0) {
        aac_stash_append(pcm16, total_samples);
        return;
    }

    // Ensure stash can hold the incoming block; if not, drop buffered data.
    if (total_samples > speex_aac.stash_cap) {
        // Worst case: input chunk larger than our stash. Process in-place by chunks.
        unsigned int pos = 0;
        while (pos + frame_samples <= total_samples) {
            // Copy into stash to run preprocess (needs mutable buffer).
            memcpy(speex_aac.stash, pcm16 + pos, frame_samples * sizeof(int16_t));
            speex_preprocess_run(speex_aac.st, (spx_int16_t *)speex_aac.stash);
            aac_stash_append(speex_aac.stash, frame_samples);
            pos += frame_samples;
        }
        if (pos < total_samples) {
            aac_stash_append(pcm16 + pos, total_samples - pos);
        }
        return;
    }

    if (speex_aac.stash_len + total_samples > speex_aac.stash_cap) {
        speex_aac.stash_len = 0;
    }
    if (speex_aac.stash_len + total_samples <= speex_aac.stash_cap) {
        memcpy(speex_aac.stash + speex_aac.stash_len, pcm16, total_samples * sizeof(int16_t));
        speex_aac.stash_len += total_samples;
    }

    while (speex_aac.stash_len >= frame_samples) {
        // Run preprocess on the first frame in stash.
        speex_preprocess_run(speex_aac.st, (spx_int16_t *)speex_aac.stash);
        aac_stash_append(speex_aac.stash, frame_samples);

        unsigned int remain = speex_aac.stash_len - frame_samples;
        if (remain)
            memmove(speex_aac.stash, speex_aac.stash + frame_samples, remain * sizeof(int16_t));
        speex_aac.stash_len = remain;
    }
}
#endif

void *aenc_thread(void) {
    if (active_audio_codec == HAL_AUDCODEC_AAC)
        return aenc_thread_aac();
    return aenc_thread_mp3();
}

static void *aenc_thread_mp3(void) {
    while (keepRunning && audioOn) {
        uint16_t frame_len = 0;

        // Extract one frame quickly under the mutex.
        pthread_mutex_lock(&aencMtx);
        if (mp3Buf.offset < sizeof(uint16_t)) {
            pthread_mutex_unlock(&aencMtx);
            usleep(10000);
            continue;
        }

        frame_len = (uint8_t)mp3Buf.buf[0] | ((uint8_t)mp3Buf.buf[1] << 8);
        if (!frame_len || frame_len > mp3Buf.size) {
            // Corrupted length; drop buffer to resync.
            HAL_WARNING("media", "MP3 frame_len invalid: %u offset=%u\n",
                frame_len, mp3Buf.offset);
            mp3Buf.offset = 0;
            pthread_mutex_unlock(&aencMtx);
            continue;
        }

        if (mp3Buf.offset < (uint32_t)frame_len + sizeof(uint16_t)) {
            pthread_mutex_unlock(&aencMtx);
            usleep(5000);
            continue;
        }

        char *payload = mp3Buf.buf + sizeof(uint16_t);

        // Basic MPEG audio sync check to avoid sending garbage.
        if (frame_len >= 2) {
            uint16_t hdr = (uint8_t)payload[0] << 8 | (uint8_t)payload[1];
            if ((hdr & 0xFFE0) != 0xFFE0) {
                HAL_WARNING("media", "MP3 desync: hdr=%04x offset=%u len=%u\n",
                    hdr, mp3Buf.offset, frame_len);
                // Drop one byte to try to resync.
                memmove(mp3Buf.buf, mp3Buf.buf + 1, mp3Buf.offset - 1);
                mp3Buf.offset -= 1;
                pthread_mutex_unlock(&aencMtx);
                continue;
            }
        }

        if (frame_len > mp3_send_cap) {
            unsigned char *nb = realloc(mp3_send_buf, frame_len);
            if (!nb) {
                // Out of memory: drop queued audio and continue.
                HAL_ERROR("media", "MP3 send buffer realloc failed (%u)\n", frame_len);
                mp3Buf.offset = 0;
                pthread_mutex_unlock(&aencMtx);
                continue;
            }
            mp3_send_buf = nb;
            mp3_send_cap = frame_len;
        }
        memcpy(mp3_send_buf, payload, frame_len);

        // Remove extracted frame from queue.
        mp3Buf.offset -= (uint32_t)frame_len + sizeof(uint16_t);
        if (mp3Buf.offset)
            memmove(mp3Buf.buf, mp3Buf.buf + frame_len + sizeof(uint16_t), mp3Buf.offset);
        pthread_mutex_unlock(&aencMtx);

        // Send/ingest WITHOUT holding aencMtx (avoid stalling MI_AI fetch thread).
        send_mp3_to_client((char *)mp3_send_buf, frame_len);
        pthread_mutex_lock(&mp4Mtx);
        mp4_ingest_audio((char *)mp3_send_buf, frame_len);
        pthread_mutex_unlock(&mp4Mtx);
        if (app_config.rtsp_enable)
            smolrtsp_push_mp3((uint8_t *)mp3_send_buf, frame_len, 0);
    }
    HAL_INFO("media", "Shutting down audio encoding thread...\n");
    return NULL;
}

static void *aenc_thread_aac(void) {
    HAL_INFO("media", "AAC encode thread loop start\n");
    while (keepRunning && audioOn) {
        uint16_t frame_len = 0;
        uint64_t ts_us = 0;

        pthread_mutex_lock(&aencMtx);
        if (aacBuf.offset < sizeof(uint16_t) + sizeof(uint64_t)) {
            pthread_mutex_unlock(&aencMtx);
            usleep(10000);
            continue;
        }

        frame_len = (uint8_t)aacBuf.buf[0] |
            ((uint8_t)aacBuf.buf[1] << 8);
        memcpy(&ts_us, aacBuf.buf + sizeof(uint16_t), sizeof(uint64_t));

        if (frame_len == 0 || frame_len > aacMaxOutputBytes) {
            HAL_WARNING("media", "AAC frame_len invalid: %u offset=%u\n",
                frame_len, aacBuf.offset);
            aacBuf.offset = 0;
            pthread_mutex_unlock(&aencMtx);
            continue;
        }

        if (aacBuf.offset < frame_len + sizeof(uint16_t) + sizeof(uint64_t)) {
            pthread_mutex_unlock(&aencMtx);
            usleep(5000);
            continue;
        }

        unsigned char *payload =
            (unsigned char *)(aacBuf.buf + sizeof(uint16_t) + sizeof(uint64_t));

        if (frame_len > aac_send_cap) {
            unsigned char *nb = realloc(aac_send_buf, frame_len);
            if (!nb) {
                HAL_ERROR("media", "AAC send buffer realloc failed (%u)\n", frame_len);
                aacBuf.offset = 0;
                pthread_mutex_unlock(&aencMtx);
                continue;
            }
            aac_send_buf = nb;
            aac_send_cap = frame_len;
        }
        memcpy(aac_send_buf, payload, frame_len);

        aacBuf.offset -= frame_len + sizeof(uint16_t) + sizeof(uint64_t);
        if (aacBuf.offset)
            memmove(aacBuf.buf,
                aacBuf.buf + frame_len + sizeof(uint16_t) + sizeof(uint64_t),
                aacBuf.offset);
        pthread_mutex_unlock(&aencMtx);

        pthread_mutex_lock(&mp4Mtx);
        mp4_ingest_audio((char *)aac_send_buf, frame_len);
        pthread_mutex_unlock(&mp4Mtx);

        if (app_config.rtsp_enable)
            smolrtsp_push_aac(aac_send_buf, frame_len, ts_us);
    }
    HAL_INFO("media", "Shutting down AAC encoding thread...\n");
    return NULL;
}

int save_audio_stream(hal_audframe *frame) {
    int ret = EXIT_SUCCESS;

#ifdef DEBUG_AUDIO
    printf("[audio] data:%p - %02x %02x %02x %02x %02x %02x %02x %02x\n", 
        frame->data, frame->data[0][0], frame->data[0][1], frame->data[0][2], frame->data[0][3],
        frame->data[0][4], frame->data[0][5], frame->data[0][6], frame->data[0][7]);
    printf("        len:%d\n", frame->length[0]);
    printf("        seq:%d\n", frame->seq);
    printf("        ts:%d\n", frame->timestamp);
#endif

    // Avoid taking server mutex on every frame when nobody is subscribed to raw PCM.
    if (server_pcm_clients > 0)
        send_pcm_to_client(frame);

    if (active_audio_codec == HAL_AUDCODEC_AAC)
        return save_audio_stream_aac(frame);
    return save_audio_stream_mp3(frame);
}

static int save_audio_stream_mp3(hal_audframe *frame) {
    int ret = EXIT_SUCCESS;

    unsigned int pcmLen = frame->length[0] / 2;
    unsigned int pcmOrig = pcmLen;
    short *pcmPack = (short *)frame->data[0];

    while (pcmPos + pcmLen >= pcmSamp) {
        memcpy(pcmSrc + pcmPos, pcmPack + pcmOrig - pcmLen,
            (pcmSamp - pcmPos) * 2);
        unsigned char *mp3Ptr =
            shine_encode_buffer_interleaved(mp3Enc, pcmSrc, &ret);
        pthread_mutex_lock(&aencMtx);
        if (ret > 0) {
            const uint32_t need = (uint32_t)ret + sizeof(uint16_t);
            if (!mp3Buf.buf && mp3Buf.size == 0) {
                mp3Buf.buf = malloc(AUDIO_ENC_BUF_MAX);
                mp3Buf.size = mp3Buf.buf ? AUDIO_ENC_BUF_MAX : 0;
                mp3Buf.offset = 0;
            }
            // Drop queued audio when buffer is full; keep capture thread real-time.
            if (mp3Buf.size && (mp3Buf.offset + need > mp3Buf.size))
                mp3Buf.offset = 0;
            if (mp3Buf.size && need <= mp3Buf.size) {
                enum BufError e1 = put_u16_le(&mp3Buf, (uint16_t)ret);
                enum BufError e2 = put(&mp3Buf, (char *)mp3Ptr, (uint32_t)ret);
                if (e1 != BUF_OK || e2 != BUF_OK) {
                    HAL_WARNING("media", "MP3 buffer put failed e1=%d e2=%d (drop)\n", e1, e2);
                    mp3Buf.offset = 0;
                }
            }
        }
        pthread_mutex_unlock(&aencMtx);

        static int log_mp3_enc = 0;
        if (log_mp3_enc < 3) {
            HAL_INFO("media", "MP3 enc frame bytes=%d pcmPos reset\n", ret);
            log_mp3_enc++;
        }

        pcmLen -= (pcmSamp - pcmPos);
        pcmPos = 0;
    }

    memcpy(pcmSrc + pcmPos, pcmPack + pcmOrig - pcmLen, pcmLen * 2);
    pcmPos += pcmLen;

    return ret;
}

static int save_audio_stream_aac(hal_audframe *frame) {
    if (!aacEnc || !aacPcm || !aacOut) {
        HAL_ERROR("media", "AAC path not initialized (enc=%p pcm=%p out=%p)\n",
            (void*)aacEnc, (void*)aacPcm, (void*)aacOut);
        return EXIT_FAILURE;
    }

    unsigned int channels = frame->channelCnt ? (unsigned int)frame->channelCnt : aacChannels;
    if (channels == 0) channels = 1;

    // Ignore HAL channel changes; force configured channel count (mono) to keep timing correct.
    channels = aacChannels;

    // HAL PCM is 16-bit interleaved
    unsigned int samples_per_ch = frame->length[0] / (2 * channels);
    short *pcm16 = (short *)frame->data[0];
    static uint32_t last_ts = 0;
    static int log_cnt = 0;
    uint32_t delta_ts = frame->timestamp - last_ts;
        if (log_cnt < 3) {
        HAL_INFO("media", "AAC in frame len=%u samples/ch=%u ch=%u ts=%u dt=%u\n",
            frame->length[0], samples_per_ch, channels, frame->timestamp, delta_ts);
            log_cnt++;
        }
    last_ts = frame->timestamp;

    // Append to stash (16-bit LE samples, interleaved).
    unsigned int total_samples = samples_per_ch * channels;
#if defined(DIVINUS_WITH_SPEEXDSP)
    speex_aac_push_pcm(pcm16, total_samples);
#else
    aac_stash_append(pcm16, total_samples);
#endif

    // Consume stash in blocks of aacInputSamples * channels (16-bit LE).
    while (aacStash && aacStashLen >= (unsigned int)aacInputSamples * channels) {
        memcpy(aacPcm, aacStash, (unsigned int)aacInputSamples * channels * sizeof(int16_t));

        unsigned int remain = aacStashLen - (unsigned int)aacInputSamples * channels;
        if (remain)
            memmove(aacStash, aacStash + (unsigned int)aacInputSamples * channels,
                remain * sizeof(int16_t));
        aacStashLen = remain;

        int bytes = faacEncEncode(aacEnc, (int32_t *)aacPcm, (unsigned int)aacInputSamples,
            aacOut, aacMaxOutputBytes);
        static int log_fail = 0;
        static int log_ok = 0;
        if (bytes <= 0) {
            if (log_fail < 5) {
                HAL_WARNING("media", "faacEncEncode returned %d (ts=%u)\n",
                    bytes, frame->timestamp);
                log_fail++;
            }
            continue;
        }
        if ((unsigned int)bytes > aacMaxOutputBytes) {
            HAL_ERROR("media", "AAC frame %d exceeds buffer %lu\n",
                bytes, aacMaxOutputBytes);
            bytes = (int)aacMaxOutputBytes;
        }
        if (bytes > UINT16_MAX)
            bytes = UINT16_MAX;

        // RTP timestamp from stash base
        // Let RTSP layer derive timestamps from monotonic clock to avoid drift.
        uint64_t ts_us = 0;

        pthread_mutex_lock(&aencMtx);
        if (!aacBuf.buf || aacBuf.size != AUDIO_ENC_BUF_MAX) {
            free(aacBuf.buf);
            aacBuf.buf = malloc(AUDIO_ENC_BUF_MAX);
            aacBuf.size = aacBuf.buf ? AUDIO_ENC_BUF_MAX : 0;
            aacBuf.offset = 0;
        }
        // Drop queued audio if buffer would overflow. Keep capture thread real-time.
        const uint32_t need = (uint32_t)bytes + sizeof(uint16_t) + sizeof(uint64_t);
        if (aacBuf.size && (aacBuf.offset + need > aacBuf.size))
            aacBuf.offset = 0;
        enum BufError e1 = put_u16_le(&aacBuf, (uint16_t)bytes);
        char ts_buf[8];
        for (int i = 0; i < 8; i++) ts_buf[i] = (ts_us >> (i * 8)) & 0xFF;
        enum BufError e2 = put(&aacBuf, ts_buf, 8);
        enum BufError e3 = put(&aacBuf, (char *)aacOut, (uint32_t)bytes);
        if (e1 != BUF_OK || e2 != BUF_OK || e3 != BUF_OK) {
            HAL_ERROR("media", "AAC buffer put failed e1=%d e2=%d e3=%d\n", e1, e2, e3);
            aacBuf.offset = 0;
        } else if (log_ok < 3) {
            HAL_INFO("media", "AAC encoded bytes=%d ts_calc=%llu\n",
                bytes, (unsigned long long)ts_us);
            log_ok++;
        }
        pthread_mutex_unlock(&aencMtx);
    }

    return EXIT_SUCCESS;
}

int save_video_stream(char index, hal_vidstream *stream) {
    int ret;

    switch (chnState[index].payload) {
        case HAL_VIDCODEC_H264:
        case HAL_VIDCODEC_H265:
        {
            char isH265 = chnState[index].payload == HAL_VIDCODEC_H265 ? 1 : 0;

            const int have_h26x_clients = (server_h26x_clients > 0);
            const int have_mp4_clients = (server_mp4_clients > 0);
            const int do_mp4 = (app_config.mp4_enable && (have_mp4_clients || recordOn));

            if (do_mp4) {
                pthread_mutex_lock(&mp4Mtx);
                if (have_mp4_clients)
                    send_mp4_to_client(index, stream, isH265);
                if (recordOn)
                    send_mp4_to_record(stream, isH265);
                pthread_mutex_unlock(&mp4Mtx);
                
                if (have_h26x_clients)
                    send_h26x_to_client(index, stream);
            } else if (app_config.mp4_enable && have_h26x_clients) {
                // Raw H.26x over HTTP (video.264/video.265) does not require MP4 muxing.
                send_h26x_to_client(index, stream);
            }
            if (app_config.rtsp_enable)
                for (int i = 0; i < stream->count; i++)
                    smolrtsp_push_video(
                        stream->pack[i].data + stream->pack[i].offset,
                        stream->pack[i].length - stream->pack[i].offset,
                        isH265,
                        stream->pack[i].timestamp);

            if (app_config.stream_enable && udp_stream_has_clients())
                for (int i = 0; i < stream->count; i++)
                    udp_stream_send_nal(stream->pack[i].data + stream->pack[i].offset,
                        stream->pack[i].length - stream->pack[i].offset,
                        stream->pack[i].nalu[0].type == NalUnitType_CodedSliceIdr, isH265);
            
            break;
        }
        case HAL_VIDCODEC_MJPG:
            if (app_config.jpeg_enable) {
                static char *mjpeg_buf;
                static ssize_t mjpeg_buf_size = 0;
                ssize_t buf_size = 0;
                const int want_mjpeg_stream = (server_mjpeg_clients > 0);
                for (unsigned int i = 0; i < stream->count; i++) {
                    hal_vidpack *data = &stream->pack[i];
                    // Keep a raw JPEG bitstream for snapshot cache; only reserve extra bytes
                    // for HTTP MJPEG streaming when there are active MJPEG clients.
                    ssize_t need_size = buf_size + data->length - data->offset;
                    if (need_size > mjpeg_buf_size)
                        mjpeg_buf = realloc(mjpeg_buf, mjpeg_buf_size = need_size);
                    memcpy(mjpeg_buf + buf_size, data->data + data->offset,
                        data->length - data->offset);
                    buf_size += data->length - data->offset;
                }
                // Keep a copy for snapshot users (jpeg_get when MJPEG is enabled).
                uint64_t ts = 0;
                if (stream->count)
                    ts = stream->pack[stream->count - 1].timestamp;
                mjpeg_last_update((unsigned char *)mjpeg_buf, (size_t)buf_size, ts);
                if (want_mjpeg_stream) {
                    // send_mjpeg_to_client appends "\r\n" in-place; ensure capacity for 2 bytes once.
                    if (buf_size + 2 > mjpeg_buf_size)
                        mjpeg_buf = realloc(mjpeg_buf, mjpeg_buf_size = buf_size + 2);
                    send_mjpeg_to_client(index, mjpeg_buf, buf_size);
                }
            }
            break;
        case HAL_VIDCODEC_JPG:
        {
            static char *jpeg_buf;
            static ssize_t jpeg_buf_size = 0;
            ssize_t buf_size = 0;
            for (unsigned int i = 0; i < stream->count; i++) {
                hal_vidpack *data = &stream->pack[i];
                // send_jpeg_to_client appends "\r\n" in-place; reserve extra bytes once below.
                ssize_t need_size = buf_size + data->length - data->offset;
                if (need_size > jpeg_buf_size)
                    jpeg_buf = realloc(jpeg_buf, jpeg_buf_size = need_size);
                memcpy(jpeg_buf + buf_size, data->data + data->offset,
                    data->length - data->offset);
                buf_size += data->length - data->offset;
            }
            if (app_config.jpeg_enable) {
                if (buf_size + 2 > jpeg_buf_size)
                    jpeg_buf = realloc(jpeg_buf, jpeg_buf_size = buf_size + 2);
                send_jpeg_to_client(index, jpeg_buf, buf_size);
            }
            break;
        }
        default:
            return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int start_streaming(void) {
    int ret = EXIT_SUCCESS;

    for (int i = 0; app_config.stream_dests[i] && *app_config.stream_dests[i]; i++) {
        if (STARTS_WITH(app_config.stream_dests[i], "udp://")) {
            char *endptr, *hostptr, *portptr, dst[16];
            unsigned short port = 0;
            long val;

            if (portptr = strrchr(app_config.stream_dests[i], ':')) {
                val = strtol(portptr + 1, &endptr, 10);
                if (endptr != portptr + 1)
                    port = (unsigned short)val;
                else {
                    if (portptr[2] != '/')
                        HAL_DANGER("media", "Invalid UDP port: %s, going with defaults!\n",
                            app_config.stream_dests[i]);
                }
            }

            hostptr = &app_config.stream_dests[i][6];
            if (portptr) {
                size_t hostlen = portptr - hostptr;
                if (hostlen > sizeof(dst) - 1) hostlen = sizeof(dst) - 1;
                strncpy(dst, hostptr, hostlen);
                dst[hostlen] = '\0';
            } else {
                strncpy(dst, hostptr, sizeof(dst) - 1);
                dst[sizeof(dst) - 1] = '\0';
            }

            if (!udpOn) {
                val = strtol(hostptr, &endptr, 10);
                if (endptr != hostptr && val >= 224 && val <= 239) {
                    if (udp_stream_init(app_config.stream_udp_srcport, dst))
                        udpOn = 1;
                    else return EXIT_FAILURE;
                } else {
                    if (udp_stream_init(app_config.stream_udp_srcport, NULL))
                        udpOn = 1;
                    else return EXIT_FAILURE;
                }
            }
            
            if (udp_stream_add_client(dst, port) != -1)
                HAL_INFO("media", "Starting streaming to %s...\n", app_config.stream_dests[i]);
        }
    }
}

void stop_streaming(void) {
    if (udpOn) {
        udp_stream_close();
        udpOn = 0;
    }
}

void request_idr(void) {
    signed char index = -1;
    pthread_mutex_lock(&chnMtx);
    for (int i = 0; i < chnCount; i++) {
        if (!chnState[i].enable) continue;
        if (chnState[i].payload != HAL_VIDCODEC_H264 &&
            chnState[i].payload != HAL_VIDCODEC_H265) continue;
        index = i;
        break;
    }
    if (index != -1) switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_video_request_idr(index); break;
        case HAL_PLATFORM_I6C: i6c_video_request_idr(index); break;
        case HAL_PLATFORM_M6:  m6_video_request_idr(index); break;
        case HAL_PLATFORM_RK:  rk_video_request_idr(index); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_video_request_idr(index); break;
        case HAL_PLATFORM_GM:  gm_video_request_idr(index); break;
        case HAL_PLATFORM_V1:  v1_video_request_idr(index); break;
        case HAL_PLATFORM_V2:  v2_video_request_idr(index); break;
        case HAL_PLATFORM_V3:  v3_video_request_idr(index); break;
        case HAL_PLATFORM_V4:  v4_video_request_idr(index); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_video_request_idr(index); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_video_request_idr(index); break;
#endif
    }  
    pthread_mutex_unlock(&chnMtx);
}

void set_grayscale(bool active) {
    pthread_mutex_lock(&chnMtx);
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_channel_grayscale(active); break;
        case HAL_PLATFORM_I6C: i6c_channel_grayscale(active); break;
        case HAL_PLATFORM_M6:  m6_channel_grayscale(active); break;
        case HAL_PLATFORM_RK:  rk_channel_grayscale(active); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_channel_grayscale(active); break;
        case HAL_PLATFORM_V1:  v1_channel_grayscale(active); break;
        case HAL_PLATFORM_V2:  v2_channel_grayscale(active); break;
        case HAL_PLATFORM_V3:  v3_channel_grayscale(active); break;
        case HAL_PLATFORM_V4:  v4_channel_grayscale(active); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_channel_grayscale(active); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_channel_grayscale(active); break;
#endif
    }
    pthread_mutex_unlock(&chnMtx);
}

int get_isp_avelum(unsigned char *lum) {
    if (!lum) return EXIT_FAILURE;
    switch (plat) {
#if defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V4:
            return v4_get_isp_avelum(lum);
#endif
        default:
            return EXIT_FAILURE;
    }
}

int get_isp_exposure_info(unsigned int *iso, unsigned int *exp_time,
    unsigned int *again, unsigned int *dgain, unsigned int *ispdgain,
    int *exposure_is_max) {
    if (!iso || !exp_time || !again || !dgain || !ispdgain || !exposure_is_max)
        return EXIT_FAILURE;
    switch (plat) {
#if defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V4:
            return v4_get_isp_exposure_info(iso, exp_time, again, dgain, ispdgain, exposure_is_max);
#endif
        default:
            return EXIT_FAILURE;
    }
}

int take_next_free_channel(bool mainLoop) {
    pthread_mutex_lock(&chnMtx);
    for (int i = 0; i < chnCount; i++) {
        if (chnState[i].enable) continue;
        chnState[i].enable = true;
        chnState[i].mainLoop = mainLoop;
        pthread_mutex_unlock(&chnMtx);
        return i;
    }
    pthread_mutex_unlock(&chnMtx);
    return -1;
}

int create_channel(char index, short width, short height, char framerate, char jpeg) {
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  return i6_channel_create(index, width, height, jpeg);
        case HAL_PLATFORM_I6C: return i6c_channel_create(index, width, height, jpeg);
        case HAL_PLATFORM_M6:  return m6_channel_create(index, width, height, jpeg);
        case HAL_PLATFORM_RK:  return rk_channel_create(index, width, height,
            app_config.mirror, app_config.flip);
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  return EXIT_SUCCESS;
        case HAL_PLATFORM_GM:  return EXIT_SUCCESS;
        case HAL_PLATFORM_V1:  return v1_channel_create(index, width, height,
            app_config.mirror, app_config.flip, framerate);
        case HAL_PLATFORM_V2:  return v2_channel_create(index, width, height,
            app_config.mirror, app_config.flip, framerate);
        case HAL_PLATFORM_V3:  return v3_channel_create(index, width, height,
            app_config.mirror, app_config.flip, framerate);
        case HAL_PLATFORM_V4:  return v4_channel_create(index, app_config.mirror,
            app_config.flip, framerate);
#elif defined(__mips__)
        case HAL_PLATFORM_T31: return t31_channel_create(index, width, height,
            framerate, jpeg);
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: return cvi_channel_create(index, width, height,
            app_config.mirror, app_config.flip);
#endif
    }
}

int bind_channel(char index, char framerate, char jpeg) {
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  return i6_channel_bind(index, framerate);
        case HAL_PLATFORM_I6C: return i6c_channel_bind(index, framerate);
        case HAL_PLATFORM_M6:  return m6_channel_bind(index, framerate);
        case HAL_PLATFORM_RK:  return rk_channel_bind(index);
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  return ak_channel_bind(index);
        case HAL_PLATFORM_GM:  return gm_channel_bind(index);
        case HAL_PLATFORM_V1:  return v1_channel_bind(index);
        case HAL_PLATFORM_V2:  return v2_channel_bind(index);
        case HAL_PLATFORM_V3:  return v3_channel_bind(index);
        case HAL_PLATFORM_V4:  return v4_channel_bind(index);
#elif defined(__mips__)
        case HAL_PLATFORM_T31: return t31_channel_bind(index);
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: return cvi_channel_bind(index);
#endif
    }
}

int unbind_channel(char index, char jpeg) {
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  return i6_channel_unbind(index);
        case HAL_PLATFORM_I6C: return i6c_channel_unbind(index);
        case HAL_PLATFORM_M6:  return m6_channel_unbind(index);
        case HAL_PLATFORM_RK:  return rk_channel_unbind(index);
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  return ak_channel_unbind(index);
        case HAL_PLATFORM_GM:  return gm_channel_unbind(index);
        case HAL_PLATFORM_V1:  return v1_channel_unbind(index);
        case HAL_PLATFORM_V2:  return v2_channel_unbind(index);
        case HAL_PLATFORM_V3:  return v3_channel_unbind(index);
        case HAL_PLATFORM_V4:  return v4_channel_unbind(index);
#elif defined(__mips__)
        case HAL_PLATFORM_T31: return t31_channel_unbind(index);
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: return cvi_channel_unbind(index);
#endif
    }
}

int disable_video(char index, char jpeg) {
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  return i6_video_destroy(index);
        case HAL_PLATFORM_I6C: return i6c_video_destroy(index);
        case HAL_PLATFORM_M6:  return m6_video_destroy(index);
        case HAL_PLATFORM_RK:  return rk_video_destroy(index);
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  return ak_video_destroy(index);
        case HAL_PLATFORM_GM:  return gm_video_destroy(index);
        case HAL_PLATFORM_V1:  return v1_video_destroy(index);
        case HAL_PLATFORM_V2:  return v2_video_destroy(index);
        case HAL_PLATFORM_V3:  return v3_video_destroy(index);
        case HAL_PLATFORM_V4:  return v4_video_destroy(index);
#elif defined(__mips__)
        case HAL_PLATFORM_T31: return t31_video_destroy(index);
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: return cvi_video_destroy(index);
#endif
    }    
    return 0;
}

void disable_audio(void) {
    if (!audioOn) return;

    audioOn = 0;

    pthread_join(aencPid, NULL);
    pthread_join(audPid, NULL);
    if (active_audio_codec == HAL_AUDCODEC_AAC) {
#if defined(DIVINUS_WITH_SPEEXDSP)
        speex_aac_reset();
#endif
        if (aacEnc)
            faacEncClose(aacEnc);
        free(aacPcm);
        free(aacOut);
        free(aacStash);
        free(aacBuf.buf);
        aacBuf.buf = NULL;
        aacBuf.size = 0;
        free(aac_send_buf);
        aac_send_buf = NULL;
        aac_send_cap = 0;
        aacEnc = NULL;
        aacPcm = NULL;
        aacOut = NULL;
        aacStash = NULL;
        aacInputSamples = 0;
        aacMaxOutputBytes = 0;
        aacPcmPos = 0;
        aacStashLen = 0;
        aacStashTsUs = 0;
        aacBuf.offset = 0;
    } else {
        shine_close(mp3Enc);
        free(mp3Buf.buf);
        mp3Buf.buf = NULL;
        mp3Buf.size = 0;
        free(mp3_send_buf);
        mp3_send_buf = NULL;
        mp3_send_cap = 0;
        mp3Buf.offset = 0;
        pcmPos = 0;
    }
    active_audio_codec = HAL_AUDCODEC_UNSPEC;

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_audio_deinit(); break;
        case HAL_PLATFORM_I6C: i6c_audio_deinit(); break;
        case HAL_PLATFORM_M6:  m6_audio_deinit(); break;
        case HAL_PLATFORM_RK:  rk_audio_deinit(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_GM:  gm_audio_deinit(); break;
        case HAL_PLATFORM_V1:  v1_audio_deinit(); break;
        case HAL_PLATFORM_V2:  v2_audio_deinit(); break;
        case HAL_PLATFORM_V3:  v3_audio_deinit(); break;
        case HAL_PLATFORM_V4:  v4_audio_deinit(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_audio_deinit(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_audio_deinit(); break;
#endif
    }
}

int enable_audio(void) {
    int ret = EXIT_SUCCESS;

    if (audioOn) return ret;

    active_audio_codec = app_config.audio_codec ? app_config.audio_codec : HAL_AUDCODEC_MP3;
    aacChannels = app_config.audio_channels ? app_config.audio_channels : 1;
    if (aacChannels > 2) aacChannels = 2;
    HAL_INFO("media", "Audio init: codec=%s srate=%u bitrate=%u channels=%u gain=%d\n",
        active_audio_codec == HAL_AUDCODEC_AAC ? "AAC" :
        (active_audio_codec == HAL_AUDCODEC_MP3 ? "MP3" : "UNSPEC"),
        app_config.audio_srate, app_config.audio_bitrate,
        aacChannels,
        app_config.audio_gain);

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  ret = i6_audio_init(app_config.audio_srate, app_config.audio_gain); break;
        case HAL_PLATFORM_I6C: ret = i6c_audio_init(app_config.audio_srate, app_config.audio_gain); break;
        case HAL_PLATFORM_M6:  ret = m6_audio_init(app_config.audio_srate, app_config.audio_gain); break;
        case HAL_PLATFORM_RK:  ret = rk_audio_init(app_config.audio_srate); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_GM:  ret = gm_audio_init(app_config.audio_srate); break;
        case HAL_PLATFORM_V1:  ret = v1_audio_init(app_config.audio_srate); break;
        case HAL_PLATFORM_V2:  ret = v2_audio_init(app_config.audio_srate); break;
        case HAL_PLATFORM_V3:  ret = v3_audio_init(app_config.audio_srate); break;
        case HAL_PLATFORM_V4:  ret = v4_audio_init(app_config.audio_srate, app_config.audio_gain); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: ret = t31_audio_init(app_config.audio_srate); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: ret = cvi_audio_init(app_config.audio_srate); break;
#endif
    }
    if (ret)
        HAL_ERROR("media", "Audio initialization failed with %#x!\n%s\n",
            ret, errstr(ret));

    if (active_audio_codec != HAL_AUDCODEC_AAC &&
        shine_check_config(app_config.audio_srate, app_config.audio_bitrate) < 0)
        HAL_ERROR("media", "MP3 samplerate/bitrate configuration is unsupported!\n");

    if (active_audio_codec == HAL_AUDCODEC_AAC) {
        aacBuf.offset = 0;
        if (!aacBuf.buf || aacBuf.size != AUDIO_ENC_BUF_MAX) {
            free(aacBuf.buf);
            aacBuf.buf = malloc(AUDIO_ENC_BUF_MAX);
            aacBuf.size = aacBuf.buf ? AUDIO_ENC_BUF_MAX : 0;
            aacBuf.offset = 0;
        }
        aacEnc = faacEncOpen(app_config.audio_srate, app_config.audio_channels,
            &aacInputSamples, &aacMaxOutputBytes);
        if (!aacEnc) {
            HAL_ERROR("media", "AAC encoder initialization failed!\n");
            return EXIT_FAILURE;
        }
        HAL_INFO("media", "faacEncOpen ok: inputSamples=%lu maxOut=%lu\n",
            aacInputSamples, aacMaxOutputBytes);

        faacEncConfigurationPtr cfg = faacEncGetCurrentConfiguration(aacEnc);
        cfg->aacObjectType = LOW;
        cfg->mpegVersion = MPEG4;
        cfg->useTns = app_config.audio_aac_tns ? 1 : 0;
        cfg->allowMidside = aacChannels > 1;
        cfg->outputFormat = 0; // raw AAC-LC frames
        // FAAC supports two modes:
        // - bitrate mode: bitRate > 0 (per-channel), quantqual == 0
        // - quality/VBR mode: quantqual > 0, bitRate == 0
        cfg->bandWidth = app_config.audio_aac_bandwidth; // 0 = auto
        if (app_config.audio_aac_quantqual > 0) {
            cfg->quantqual = app_config.audio_aac_quantqual;
            cfg->bitRate = 0;
        } else {
            cfg->quantqual = 0;
            cfg->bitRate = app_config.audio_bitrate * 1000 / aacChannels;
        }
        cfg->inputFormat = FAAC_INPUT_16BIT;
        if (!faacEncSetConfiguration(aacEnc, cfg)) {
            HAL_ERROR("media", "AAC encoder configuration failed!\n");
            return EXIT_FAILURE;
        }

        aacPcm = calloc(aacInputSamples * app_config.audio_channels, sizeof(int16_t));
        aacOut = malloc(aacMaxOutputBytes);
        aacStash = calloc(aacInputSamples * app_config.audio_channels * 4, sizeof(int16_t));
        if (!aacPcm || !aacOut || !aacStash) {
            HAL_ERROR("media", "AAC encoder buffer allocation failed!\n");
            return EXIT_FAILURE;
        }
        aacStashLen = 0;
        HAL_INFO("media", "AAC buffers allocated: pcm=%p out=%p\n", (void*)aacPcm, (void*)aacOut);

#if defined(DIVINUS_WITH_SPEEXDSP)
        // Optional SpeexDSP preprocess for AAC PCM path (denoise/AGC/VAD).
        speex_aac_init_from_config(app_config.audio_srate, aacChannels);
#endif
    } else {
        mp3Buf.offset = 0;
        if (!mp3Buf.buf || mp3Buf.size != AUDIO_ENC_BUF_MAX) {
            free(mp3Buf.buf);
            mp3Buf.buf = malloc(AUDIO_ENC_BUF_MAX);
            mp3Buf.size = mp3Buf.buf ? AUDIO_ENC_BUF_MAX : 0;
            mp3Buf.offset = 0;
        }
        mp3Cnf.mpeg.mode = MONO;
        mp3Cnf.mpeg.bitr = app_config.audio_bitrate;
        mp3Cnf.mpeg.emph = NONE;
        mp3Cnf.mpeg.copyright = 0;
        mp3Cnf.mpeg.original = 1;
        mp3Cnf.wave.channels = PCM_MONO;
        mp3Cnf.wave.samplerate = app_config.audio_srate;
        if (!(mp3Enc = shine_initialise(&mp3Cnf)))
            HAL_ERROR("media", "MP3 encoder initialization failed!\n");

        pcmSamp = shine_samples_per_pass(mp3Enc);
        pcmPos = 0;
    }

    // IMPORTANT: audio capture/encode threads spin on `while (keepRunning && audioOn)`.
    // Set audioOn BEFORE starting threads, otherwise they may exit immediately.
    audioOn = 1;

    {
        pthread_attr_t thread_attr;
        pthread_attr_init(&thread_attr);
        size_t stacksize;
        pthread_attr_getstacksize(&thread_attr, &stacksize);
        size_t new_stacksize = 64 * 1024;
        if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", new_stacksize);
        if (!aud_thread) {
            HAL_ERROR("media", "Audio capture thread pointer is NULL!\n");
        } else if (pthread_create(
                        &audPid, &thread_attr, (void *(*)(void *))aud_thread, NULL)) {
            HAL_ERROR("media", "Starting the audio capture thread failed!\n");
        } else {
            HAL_INFO("media", "Audio capture thread started (aud_thread=%p)\n",
                aud_thread);
        }
        if (pthread_attr_setstacksize(&thread_attr, stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", stacksize);
        pthread_attr_destroy(&thread_attr);
    }

    {
        pthread_attr_t thread_attr;
        pthread_attr_init(&thread_attr);
        size_t stacksize;
        pthread_attr_getstacksize(&thread_attr, &stacksize);
        size_t new_stacksize = 64 * 1024;
        if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", new_stacksize);
        if (pthread_create(
                        &aencPid, &thread_attr, (void *(*)(void *))aenc_thread, NULL))
            HAL_ERROR("media", "Starting the audio encoding thread failed!\n");
        else
            HAL_INFO("media", "Audio encoding thread started (codec=%s)\n",
                active_audio_codec == HAL_AUDCODEC_AAC ? "AAC" : "MP3");
        if (pthread_attr_setstacksize(&thread_attr, stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", stacksize);
        pthread_attr_destroy(&thread_attr);
    }

    return ret;
}

int disable_mjpeg(void) {
    int ret;

    // If MJPEG is being stopped/reconfigured, drop the cached snapshot.
    mjpeg_last_clear();

    for (char i = 0; i < chnCount; i++) {
        if (!chnState[i].enable) continue;
        if (chnState[i].payload != HAL_VIDCODEC_MJPG) continue;

        if (ret = unbind_channel(i, 1))
            HAL_ERROR("media", "Unbinding channel %d failed with %#x!\n%s\n", 
                i, ret, errstr(ret));

        if (ret = disable_video(i, 1))
            HAL_ERROR("media", "Disabling encoder %d failed with %#x!\n%s\n", 
                i, ret, errstr(ret));
    }

    return EXIT_SUCCESS;
}

int enable_mjpeg(void) {
    int ret;

    int index = take_next_free_channel(true);

    if (ret = create_channel(index, app_config.jpeg_width,
        app_config.jpeg_height, app_config.jpeg_fps, 1))
        HAL_ERROR("media", "Creating channel %d failed with %#x!\n%s\n", 
            index, ret, errstr(ret));

    {
        hal_vidconfig config = {0};
        config.width = app_config.jpeg_width;
        config.height = app_config.jpeg_height;
        config.codec = HAL_VIDCODEC_MJPG;
        // MJPEG is controlled via JPEG quality factor (qfactor) now.
        // We force QP mode, because bitrate-based modes are not exposed/used.
        config.mode = HAL_VIDMODE_QP;
        config.framerate = app_config.jpeg_fps;
        // Some vendor HALs still read bitrate fields even in QP; keep safe defaults.
        config.bitrate = 1024;
        config.maxBitrate = 1024 * 5 / 4;
        unsigned int q = app_config.jpeg_qfactor;
        if (q < 1) q = 1;
        if (q > 99) q = 99;
        config.minQual = (unsigned char)q;
        config.maxQual = (unsigned char)q;

        switch (plat) {
#if defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_I6:  ret = i6_video_create(index, &config); break;
            case HAL_PLATFORM_I6C: ret = i6c_video_create(index, &config); break;
            case HAL_PLATFORM_M6:  ret = m6_video_create(index, &config); break;
            case HAL_PLATFORM_RK:  ret = rk_video_create(index, &config); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_AK:  ret = ak_video_create(index, &config); break;
            case HAL_PLATFORM_GM:  ret = gm_video_create(index, &config); break;
            case HAL_PLATFORM_V1:  ret = v1_video_create(index, &config); break;
            case HAL_PLATFORM_V2:  ret = v2_video_create(index, &config); break;
            case HAL_PLATFORM_V3:  ret = v3_video_create(index, &config); break;
            case HAL_PLATFORM_V4:  ret = v4_video_create(index, &config); break;
#elif defined(__mips__)
            case HAL_PLATFORM_T31: ret = t31_video_create(index, &config); break;
#elif defined(__riscv) || defined(__riscv__)
            case HAL_PLATFORM_CVI: ret = cvi_video_create(index, &config); break;
#endif
        }

        if (ret)
            HAL_ERROR("media", "Creating encoder %d failed with %#x!\n%s\n", 
                index, ret, errstr(ret));
    }

    if (ret = bind_channel(index, app_config.jpeg_fps, 1))
        HAL_ERROR("media", "Binding channel %d failed with %#x!\n%s\n",
            index, ret, errstr(ret));

    return EXIT_SUCCESS;
}

int disable_mp4(void) {
    int ret;

    for (char i = 0; i < chnCount; i++) {
        if (!chnState[i].enable) continue;
        if (chnState[i].payload != HAL_VIDCODEC_H264 ||
            chnState[i].payload != HAL_VIDCODEC_H265) continue;

        if (ret = unbind_channel(i, 1))
            HAL_ERROR("media", "Unbinding channel %d failed with %#x!\n%s\n", 
                i, ret, errstr(ret));

        if (ret = disable_video(i, 1))
            HAL_ERROR("media", "Disabling encoder %d failed with %#x!\n%s\n", 
                i, ret, errstr(ret));
    }

    return EXIT_SUCCESS;
}

int enable_mp4(void) {
    int ret;

    int index = take_next_free_channel(true);

    if (ret = create_channel(index, app_config.mp4_width, 
        app_config.mp4_height, app_config.mp4_fps, 0))
        HAL_ERROR("media", "Creating channel %d failed with %#x!\n%s\n", 
            index, ret, errstr(ret));

    {
        hal_vidconfig config = {0};
        config.width = app_config.mp4_width;
        config.height = app_config.mp4_height;
        config.codec = app_config.mp4_codecH265 ?
            HAL_VIDCODEC_H265 : HAL_VIDCODEC_H264;
        config.mode = app_config.mp4_mode;
        config.profile = app_config.mp4_profile;
        config.gop = app_config.mp4_gop;
        config.framerate = app_config.mp4_fps;
        config.bitrate = app_config.mp4_bitrate;
        config.maxBitrate = app_config.mp4_bitrate * 5 / 4;
        // Deterministic defaults + baseline for H.264+ logic in HAL.
        config.minQual = 34;
        config.maxQual = 48;
        if (!app_config.mp4_codecH265 && app_config.mp4_h264_plus) {
            // GM/Goke (libgm.so) firmwares can reject extended H.264 settings with NOT_SUPPORT.
            // Keep "H.264+" as a best-effort feature: enable where supported, otherwise fall back.
            if (plat == HAL_PLATFORM_GM) {
                HAL_WARNING("media", "H.264+ requested, but GM platform may not support it; falling back to H.264.\n");
            } else {
                config.flags |= HAL_VIDOPT_H264_PLUS;
            }
        }

        switch (plat) {
#if defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_I6:  ret = i6_video_create(index, &config); break;
            case HAL_PLATFORM_I6C: ret = i6c_video_create(index, &config); break;
            case HAL_PLATFORM_M6:  ret = m6_video_create(index, &config); break;
            case HAL_PLATFORM_RK:  ret = rk_video_create(index, &config); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_AK:  ret = ak_video_create(index, &config); break;
            case HAL_PLATFORM_GM:  ret = gm_video_create(index, &config); break;
            case HAL_PLATFORM_V1:  ret = v1_video_create(index, &config); break;
            case HAL_PLATFORM_V2:  ret = v2_video_create(index, &config); break;
            case HAL_PLATFORM_V3:  ret = v3_video_create(index, &config); break;
            case HAL_PLATFORM_V4:  ret = v4_video_create(index, &config); break;
#elif defined(__mips__)
            case HAL_PLATFORM_T31: ret = t31_video_create(index, &config); break;
#elif defined(__riscv) || defined(__riscv__)
            case HAL_PLATFORM_CVI: ret = cvi_video_create(index, &config); break;
#endif
        }

        if (ret)
            HAL_ERROR("media", "Creating encoder %d failed with %#x!\n%s\n", 
                index, ret, errstr(ret));

        mp4_set_config(app_config.mp4_width, app_config.mp4_height, app_config.mp4_fps,
            app_config.audio_enable ? active_audio_codec : HAL_AUDCODEC_UNSPEC,
            app_config.audio_bitrate,
            app_config.audio_channels ? app_config.audio_channels : 1,
            app_config.audio_srate);
    }

    if (ret = bind_channel(index, app_config.mp4_fps, 0))
        HAL_ERROR("media", "Binding channel %d failed with %#x!\n%s\n",
            index, ret, errstr(ret));

    return EXIT_SUCCESS;
}

int start_sdk(void) {
    int ret;

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I3:  ret = i3_hal_init(); break;
        case HAL_PLATFORM_I6:  ret = i6_hal_init(); break;
        case HAL_PLATFORM_I6C: ret = i6c_hal_init(); break;
        case HAL_PLATFORM_M6:  ret = m6_hal_init(); break;
        case HAL_PLATFORM_RK:  ret = rk_hal_init(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ret = ak_hal_init(); break;
        case HAL_PLATFORM_GM:  ret = gm_hal_init(); break;
        case HAL_PLATFORM_V1:  ret = v1_hal_init(); break;
        case HAL_PLATFORM_V2:  ret = v2_hal_init(); break;
        case HAL_PLATFORM_V3:  ret = v3_hal_init(); break;
        case HAL_PLATFORM_V4:  ret = v4_hal_init(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: ret = t31_hal_init(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: ret = cvi_hal_init(); break;
#endif
    }
    if (ret)
        HAL_ERROR("media", "HAL initialization failed with %#x!\n%s\n",
            ret, errstr(ret));

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:
            i6_aud_cb = save_audio_stream;
            i6_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_I6C:
            i6c_aud_cb = save_audio_stream;
            i6c_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_M6:
            m6_aud_cb = save_audio_stream;
            m6_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_RK:
            rk_aud_cb = save_audio_stream;
            rk_vid_cb = save_video_stream;
            break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_GM:
            gm_aud_cb = save_audio_stream;
            gm_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_V1:
            v1_aud_cb = save_audio_stream;
            v1_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_V2:
            v2_aud_cb = save_audio_stream;
            v2_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_V3:
            v3_aud_cb = save_audio_stream;
            v3_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_V4:
            v4_aud_cb = save_audio_stream;
            v4_vid_cb = save_video_stream;
            break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31:
            t31_aud_cb = save_audio_stream;
            t31_vid_cb = save_video_stream;
            break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI:
            cvi_aud_cb = save_audio_stream;
            cvi_vid_cb = save_video_stream;
            break;
#endif
    }

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I3:  ret = i3_system_init(); break;
        case HAL_PLATFORM_I6:  ret = i6_system_init(); break;
        case HAL_PLATFORM_I6C: ret = i6c_system_init(); break;
        case HAL_PLATFORM_M6:  ret = m6_system_init(); break;
        case HAL_PLATFORM_RK:  ret = rk_system_init(0); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ret = ak_system_init(app_config.sensor_config); break;
        case HAL_PLATFORM_GM:  ret = gm_system_init(); break;
        case HAL_PLATFORM_V1:  ret = v1_system_init(app_config.sensor_config); break;
        case HAL_PLATFORM_V2:  ret = v2_system_init(app_config.sensor_config); break;
        case HAL_PLATFORM_V3:  ret = v3_system_init(app_config.sensor_config); break;
        case HAL_PLATFORM_V4:  ret = v4_system_init(app_config.sensor_config); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: ret = t31_system_init(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: ret = cvi_system_init(app_config.sensor_config); break;
#endif
    }
    if (ret)
        HAL_ERROR("media", "System initialization failed with %#x!\n%s\n",
            ret, errstr(ret));

    if (app_config.audio_enable) {
        ret = enable_audio();
        if (ret)
            HAL_ERROR("media", "Audio initialization failed with %#x!\n%s\n",
                ret, errstr(ret));
    }

    short width = MAX(app_config.mp4_width, app_config.jpeg_width);
    short height = MAX(app_config.mp4_height, app_config.jpeg_height);
    short framerate = MAX(app_config.mp4_fps, app_config.jpeg_fps);

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  ret = i6_pipeline_create(0, width,
            height, app_config.mirror, app_config.flip, framerate); break;
        case HAL_PLATFORM_I6C: ret = i6c_pipeline_create(0, width,
            height, app_config.mirror, app_config.flip, framerate); break;
        case HAL_PLATFORM_M6:  ret = m6_pipeline_create(0, width,
            height, app_config.mirror, app_config.flip, framerate); break;
        case HAL_PLATFORM_RK:  ret = rk_pipeline_create(width, height); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ret = ak_pipeline_create(app_config.mirror,
            app_config.flip); break;
        case HAL_PLATFORM_GM:  ret = gm_pipeline_create(app_config.mirror,
            app_config.flip); break;
        case HAL_PLATFORM_V1:  ret = v1_pipeline_create(); break;
        case HAL_PLATFORM_V2:  ret = v2_pipeline_create(); break;
        case HAL_PLATFORM_V3:  ret = v3_pipeline_create(); break;
        case HAL_PLATFORM_V4:  ret = v4_pipeline_create(app_config.iq_config); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: ret = t31_pipeline_create(app_config.mirror,
            app_config.flip, app_config.antiflicker, framerate); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: ret = cvi_pipeline_create(); break;
#endif
    }
    if (ret)
        HAL_ERROR("media", "Pipeline creation failed with %#x!\n%s\n",
            ret, errstr(ret));

    if (isp_thread) {
        pthread_attr_t thread_attr;
        pthread_attr_init(&thread_attr);
        size_t stacksize;
        pthread_attr_getstacksize(&thread_attr, &stacksize);
        size_t new_stacksize = app_config.isp_thread_stack_size;
        if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
            HAL_DANGER("media", "Can't set stack size %zu!\n", new_stacksize);
        if (pthread_create(
                     &ispPid, &thread_attr, (void *(*)(void *))isp_thread, NULL))
            HAL_ERROR("media", "Starting the imaging thread failed!\n");
        if (pthread_attr_setstacksize(&thread_attr, stacksize))
            HAL_DANGER("media", "Can't set stack size %zu!\n", stacksize);
        pthread_attr_destroy(&thread_attr);
    }

    if (app_config.mp4_enable && (ret = enable_mp4()))
        HAL_ERROR("media", "MP4 initialization failed with %#x!\n", ret);

    if (app_config.jpeg_enable && (ret = enable_mjpeg()))
        HAL_ERROR("media", "MJPEG initialization failed with %#x!\n", ret);

    if (app_config.jpeg_enable && (ret = jpeg_init()))
        HAL_ERROR("media", "JPEG initialization failed with %#x!\n", ret);

    {
        pthread_attr_t thread_attr;
        pthread_attr_init(&thread_attr);
        size_t stacksize;
        pthread_attr_getstacksize(&thread_attr, &stacksize);
        size_t new_stacksize = app_config.venc_stream_thread_stack_size;
        if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", new_stacksize);
        if (pthread_create(
                     &vidPid, &thread_attr, (void *(*)(void *))vid_thread, NULL))
            HAL_ERROR("media", "Starting the video encoding thread failed!\n");
        if (pthread_attr_setstacksize(&thread_attr, stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", stacksize);
        pthread_attr_destroy(&thread_attr);
    }

    if (!access(app_config.sensor_config, F_OK) && !sleep(1))
        switch (plat) {
#if defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_I3:  i3_config_load(app_config.sensor_config); break;
            case HAL_PLATFORM_I6:  i6_config_load(app_config.sensor_config); break;
            case HAL_PLATFORM_I6C: i6c_config_load(app_config.sensor_config); break;
            case HAL_PLATFORM_M6:  m6_config_load(app_config.sensor_config); break;
#elif defined(__mips__)
            case HAL_PLATFORM_T31: t31_config_load(app_config.sensor_config); break;
#endif
        }

    HAL_INFO("media", "SDK has started successfully!\n");

    return EXIT_SUCCESS;
}

int stop_sdk(void) {
    pthread_join(vidPid, NULL);

    if (app_config.jpeg_enable)
        jpeg_deinit();

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_video_destroy_all(); break;
        case HAL_PLATFORM_I6C: i6c_video_destroy_all(); break;
        case HAL_PLATFORM_M6:  m6_video_destroy_all(); break;
        case HAL_PLATFORM_RK:  rk_video_destroy_all(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_video_destroy_all(); break;
        case HAL_PLATFORM_GM:  gm_video_destroy_all(); break;
        case HAL_PLATFORM_V1:  v1_video_destroy_all(); break;
        case HAL_PLATFORM_V2:  v2_video_destroy_all(); break;
        case HAL_PLATFORM_V3:  v3_video_destroy_all(); break;
        case HAL_PLATFORM_V4:  v4_video_destroy_all(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_video_destroy_all(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_video_destroy_all(); break;
#endif
    }

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_pipeline_destroy(); break;
        case HAL_PLATFORM_I6C: i6c_pipeline_destroy(); break;
        case HAL_PLATFORM_M6:  m6_pipeline_destroy(); break;
        case HAL_PLATFORM_RK:  rk_pipeline_destroy(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_pipeline_destroy(); break;
        case HAL_PLATFORM_GM:  gm_pipeline_destroy(); break;
        case HAL_PLATFORM_V1:  v1_pipeline_destroy(); break;
        case HAL_PLATFORM_V2:  v2_pipeline_destroy(); break;
        case HAL_PLATFORM_V3:  v3_pipeline_destroy(); break;
        case HAL_PLATFORM_V4:  v4_pipeline_destroy(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_pipeline_destroy(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_pipeline_destroy(); break;
#endif
    }

    if (app_config.audio_enable)
        disable_audio();

    if (isp_thread)
        pthread_join(ispPid, NULL);

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I3:  i3_system_deinit(); break;
        case HAL_PLATFORM_I6:  i6_system_deinit(); break;
        case HAL_PLATFORM_I6C: i6c_system_deinit(); break;
        case HAL_PLATFORM_M6:  m6_system_deinit(); break;
        case HAL_PLATFORM_RK:  rk_system_deinit(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_system_deinit(); break;
        case HAL_PLATFORM_GM:  gm_system_deinit(); break;
        case HAL_PLATFORM_V1:  v1_system_deinit(); break;
        case HAL_PLATFORM_V2:  v2_system_deinit(); break;
        case HAL_PLATFORM_V3:  v3_system_deinit(); break;
        case HAL_PLATFORM_V4:  v4_system_deinit(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_system_deinit(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_system_deinit(); break;
#endif
    }

    switch (plat) {
#if defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V1: v1_sensor_deinit(); break;
        case HAL_PLATFORM_V2: v2_sensor_deinit(); break;
        case HAL_PLATFORM_V3: v3_sensor_deinit(); break;
        case HAL_PLATFORM_V4: v4_sensor_deinit(); break;
#endif
    }

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I3:  i3_hal_deinit(); break;
        case HAL_PLATFORM_I6:  i6_hal_deinit(); break;
        case HAL_PLATFORM_I6C: i6c_hal_deinit(); break;
        case HAL_PLATFORM_M6:  m6_hal_deinit(); break;
        case HAL_PLATFORM_RK:  rk_hal_deinit(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_hal_deinit(); break;
        case HAL_PLATFORM_GM:  gm_hal_deinit(); break;
        case HAL_PLATFORM_V1:  v1_hal_deinit(); break;
        case HAL_PLATFORM_V2:  v2_hal_deinit(); break;
        case HAL_PLATFORM_V3:  v3_hal_deinit(); break;
        case HAL_PLATFORM_V4:  v4_hal_deinit(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_hal_deinit(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_hal_deinit(); break;
#endif
    }

    HAL_INFO("media", "SDK had stopped successfully!\n");
    return EXIT_SUCCESS;
}
