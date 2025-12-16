#include "media.h"
#include <faac.h>

char audioOn = 0, udpOn = 0;
pthread_mutex_t aencMtx, chnMtx, mp4Mtx;
pthread_t aencPid = 0, audPid = 0, ispPid = 0, vidPid = 0;

static hal_audcodec active_audio_codec;

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
int32_t *aacPcm = NULL;
unsigned char *aacOut = NULL;
unsigned int aacPcmPos = 0;

static void *aenc_thread_mp3(void);
static void *aenc_thread_aac(void);
static int save_audio_stream_mp3(hal_audframe *frame);
static int save_audio_stream_aac(hal_audframe *frame);

void *aenc_thread(void) {
    if (active_audio_codec == HAL_AUDCODEC_AAC)
        return aenc_thread_aac();
    return aenc_thread_mp3();
}

static void *aenc_thread_mp3(void) {
    const uint32_t mp3FrmSize =
        (app_config.audio_srate >= 32000 ? 144 : 72) *
        (app_config.audio_bitrate * 1000) /
        app_config.audio_srate;

    while (keepRunning && audioOn) {
        pthread_mutex_lock(&aencMtx);
        if (mp3Buf.offset < mp3FrmSize) {
            pthread_mutex_unlock(&aencMtx);
            usleep(10000);
            continue;
        }

        send_mp3_to_client(mp3Buf.buf, mp3FrmSize);

        pthread_mutex_lock(&mp4Mtx);
        mp4_ingest_audio(mp3Buf.buf, mp3FrmSize);
        pthread_mutex_unlock(&mp4Mtx);

        if (app_config.rtsp_enable)
            smolrtsp_push_mp3((uint8_t *)mp3Buf.buf, mp3FrmSize, 0);

        mp3Buf.offset -= mp3FrmSize;
        if (mp3Buf.offset)
            memcpy(mp3Buf.buf, mp3Buf.buf + mp3FrmSize, mp3Buf.offset);
        pthread_mutex_unlock(&aencMtx);
    }
    HAL_INFO("media", "Shutting down audio encoding thread...\n");
    return NULL;
}

static void *aenc_thread_aac(void) {
    HAL_INFO("media", "AAC encode thread loop start\n");
    while (keepRunning && audioOn) {
        pthread_mutex_lock(&aencMtx);
        if (aacBuf.offset < sizeof(uint16_t)) {
            pthread_mutex_unlock(&aencMtx);
            usleep(10000);
            continue;
        }

        uint16_t frame_len = (uint8_t)aacBuf.buf[0] |
            ((uint8_t)aacBuf.buf[1] << 8);

        if (frame_len == 0 || frame_len > aacMaxOutputBytes) {
            HAL_WARNING("media", "AAC frame_len invalid: %u offset=%u\n",
                frame_len, aacBuf.offset);
            aacBuf.offset = 0;
            pthread_mutex_unlock(&aencMtx);
            continue;
        }

        if (aacBuf.offset < frame_len + sizeof(uint16_t)) {
            pthread_mutex_unlock(&aencMtx);
            usleep(5000);
            continue;
        }

        unsigned char *payload =
            (unsigned char *)(aacBuf.buf + sizeof(uint16_t));

        HAL_INFO("media", "AAC send frame_len=%u offset_before=%u\n",
            frame_len, aacBuf.offset);

        pthread_mutex_lock(&mp4Mtx);
        mp4_ingest_audio((char *)payload, frame_len);
        pthread_mutex_unlock(&mp4Mtx);

        if (app_config.rtsp_enable)
            smolrtsp_push_aac(payload, frame_len, 0);

        aacBuf.offset -= frame_len + sizeof(uint16_t);
        if (aacBuf.offset)
            memmove(aacBuf.buf, aacBuf.buf + frame_len + sizeof(uint16_t),
                aacBuf.offset);
        pthread_mutex_unlock(&aencMtx);
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
        put(&mp3Buf, mp3Ptr, ret);
        pthread_mutex_unlock(&aencMtx);
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

    unsigned int samples = frame->length[0] / 2;
    short *pcm = (short *)frame->data[0];
    unsigned int consumed = 0;
    HAL_INFO("media", "AAC in frame len=%u samples=%u pcmPos=%u offset=%u\n",
        frame->length[0], samples, aacPcmPos, aacBuf.offset);

    while (consumed < samples) {
        unsigned int chunk = MIN(aacInputSamples - aacPcmPos, samples - consumed);
        for (unsigned int i = 0; i < chunk; i++)
            aacPcm[aacPcmPos + i] = (int32_t)pcm[consumed + i];
        aacPcmPos += chunk;
        consumed += chunk;

        if (aacPcmPos == aacInputSamples) {
            HAL_INFO("media", "AAC encode call: samples=%lu outMax=%lu\n",
                aacInputSamples, aacMaxOutputBytes);
            int bytes = faacEncEncode(aacEnc, aacPcm, aacInputSamples,
                aacOut, aacMaxOutputBytes);
            aacPcmPos = 0;
            if (bytes <= 0) {
                HAL_ERROR("media", "faacEncEncode returned %d\n", bytes);
                continue;
            }
            if ((unsigned int)bytes > aacMaxOutputBytes) {
                HAL_ERROR("media", "AAC frame %d exceeds buffer %lu\n",
                    bytes, aacMaxOutputBytes);
                bytes = (int)aacMaxOutputBytes;
            }
            if (bytes > UINT16_MAX)
                bytes = UINT16_MAX;

            pthread_mutex_lock(&aencMtx);
            enum BufError e1 = put_u16_le(&aacBuf, (uint16_t)bytes);
            enum BufError e2 = put(&aacBuf, (char *)aacOut, (uint32_t)bytes);
            if (e1 != BUF_OK || e2 != BUF_OK) {
                HAL_ERROR("media", "AAC buffer put failed e1=%d e2=%d\n", e1, e2);
                aacBuf.offset = 0;
            } else {
                HAL_INFO("media", "AAC encoded bytes=%d queued_offset=%u\n",
                    bytes, aacBuf.offset);
            }
            pthread_mutex_unlock(&aencMtx);
        }
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

            if (app_config.mp4_enable) {
                pthread_mutex_lock(&mp4Mtx);
                send_mp4_to_client(index, stream, isH265);
                if (recordOn) send_mp4_to_record(stream, isH265);
                pthread_mutex_unlock(&mp4Mtx);
                
                send_h26x_to_client(index, stream);
            }
            if (app_config.rtsp_enable)
                for (int i = 0; i < stream->count; i++)
                    smolrtsp_push_video(
                        stream->pack[i].data + stream->pack[i].offset,
                        stream->pack[i].length - stream->pack[i].offset,
                        isH265,
                        stream->pack[i].timestamp);

            if (app_config.stream_enable)
                for (int i = 0; i < stream->count; i++)
                    udp_stream_send_nal(stream->pack[i].data + stream->pack[i].offset, 
                        stream->pack[i].length - stream->pack[i].offset, 
                        stream->pack[i].nalu[0].type == NalUnitType_CodedSliceIdr, isH265);
            
            break;
        }
        case HAL_VIDCODEC_MJPG:
            if (app_config.mjpeg_enable) {
                static char *mjpeg_buf;
                static ssize_t mjpeg_buf_size = 0;
                ssize_t buf_size = 0;
                for (unsigned int i = 0; i < stream->count; i++) {
                    hal_vidpack *data = &stream->pack[i];
                    ssize_t need_size = buf_size + data->length - data->offset + 2;
                    if (need_size > mjpeg_buf_size)
                        mjpeg_buf = realloc(mjpeg_buf, mjpeg_buf_size = need_size);
                    memcpy(mjpeg_buf + buf_size, data->data + data->offset,
                        data->length - data->offset);
                    buf_size += data->length - data->offset;
                }
                send_mjpeg_to_client(index, mjpeg_buf, buf_size);
            }
            break;
        case HAL_VIDCODEC_JPG:
        {
            static char *jpeg_buf;
            static ssize_t jpeg_buf_size = 0;
            ssize_t buf_size = 0;
            for (unsigned int i = 0; i < stream->count; i++) {
                hal_vidpack *data = &stream->pack[i];
                ssize_t need_size = buf_size + data->length - data->offset + 2;
                if (need_size > jpeg_buf_size)
                    jpeg_buf = realloc(jpeg_buf, jpeg_buf_size = need_size);
                memcpy(jpeg_buf + buf_size, data->data + data->offset,
                    data->length - data->offset);
                buf_size += data->length - data->offset;
            }
            if (app_config.jpeg_enable)
                send_jpeg_to_client(index, jpeg_buf, buf_size);
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
        if (aacEnc)
            faacEncClose(aacEnc);
        free(aacPcm);
        free(aacOut);
        aacEnc = NULL;
        aacPcm = NULL;
        aacOut = NULL;
        aacInputSamples = 0;
        aacMaxOutputBytes = 0;
        aacPcmPos = 0;
        aacBuf.offset = 0;
    } else {
        shine_close(mp3Enc);
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
    HAL_INFO("media", "Audio init: codec=%s srate=%u bitrate=%u channels=%u gain=%d\n",
        active_audio_codec == HAL_AUDCODEC_AAC ? "AAC" :
        (active_audio_codec == HAL_AUDCODEC_MP3 ? "MP3" : "UNSPEC"),
        app_config.audio_srate, app_config.audio_bitrate,
        app_config.audio_channels ? app_config.audio_channels : 1,
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
        case HAL_PLATFORM_V4:  ret = v4_audio_init(app_config.audio_srate); break;
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
        cfg->useTns = 0;
        cfg->allowMidside = app_config.audio_channels > 1;
        cfg->outputFormat = 0; // raw AAC-LC frames
        cfg->bitRate = app_config.audio_bitrate * 1000 / (app_config.audio_channels ? app_config.audio_channels : 1);
        cfg->inputFormat = FAAC_INPUT_16BIT;
        if (!faacEncSetConfiguration(aacEnc, cfg)) {
            HAL_ERROR("media", "AAC encoder configuration failed!\n");
            return EXIT_FAILURE;
        }

        aacPcm = calloc(aacInputSamples, sizeof(int32_t));
        aacOut = malloc(aacMaxOutputBytes);
        if (!aacPcm || !aacOut) {
            HAL_ERROR("media", "AAC encoder buffer allocation failed!\n");
            return EXIT_FAILURE;
        }
        HAL_INFO("media", "AAC buffers allocated: pcm=%p out=%p\n", (void*)aacPcm, (void*)aacOut);
    } else {
        mp3Buf.offset = 0;
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

    {
        pthread_attr_t thread_attr;
        pthread_attr_init(&thread_attr);
        size_t stacksize;
        pthread_attr_getstacksize(&thread_attr, &stacksize);
        size_t new_stacksize = 16384;
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
        size_t new_stacksize = 16384;
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

    audioOn = 1;

    return ret;
}

int disable_mjpeg(void) {
    int ret;

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

    if (ret = create_channel(index, app_config.mjpeg_width,
        app_config.mjpeg_height, app_config.mjpeg_fps, 1))
        HAL_ERROR("media", "Creating channel %d failed with %#x!\n%s\n", 
            index, ret, errstr(ret));

    {
        hal_vidconfig config;
        config.width = app_config.mjpeg_width;
        config.height = app_config.mjpeg_height;
        config.codec = HAL_VIDCODEC_MJPG;
        config.mode = app_config.mjpeg_mode;
        config.framerate = app_config.mjpeg_fps;
        config.bitrate = app_config.mjpeg_bitrate;
        config.maxBitrate = app_config.mjpeg_bitrate * 5 / 4;

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

    if (ret = bind_channel(index, app_config.mjpeg_fps, 1))
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
        hal_vidconfig config;
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

    short width = MAX(app_config.mp4_width, app_config.mjpeg_width);
    short height = MAX(app_config.mp4_height, app_config.mjpeg_height);
    short framerate = MAX(app_config.mp4_fps, app_config.mjpeg_fps);

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
        case HAL_PLATFORM_V4:  ret = v4_pipeline_create(); break;
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

    if (app_config.mjpeg_enable && (ret = enable_mjpeg()))
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
