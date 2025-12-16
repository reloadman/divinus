#include "rtsp_smol.h"

#include "app_config.h"
#include "hal/types.h"
#include "media.h"

#include <smolrtsp.h>
#include <smolrtsp-libevent.h>
#include <datatype99.h>

#include <event2/buffer.h>
#include <event2/bufferevent.h>
#include <event2/event.h>
#include <event2/listener.h>
#include <event2/thread.h>

#include <arpa/inet.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define MAX_CLIENTS 16
#define VIDEO_PAYLOAD_TYPE 96
#define VIDEO_CLOCK 90000
#define MP3_PAYLOAD_TYPE 14
#define DEFAULT_TCP_CHANNEL_RTP 0
#define DEFAULT_TCP_CHANNEL_RTCP 1

typedef enum {
    TRACK_VIDEO,
    TRACK_AUDIO
} track_kind;

// Forward declarations for helpers used before their definitions.
static inline uint32_t audio_clock_hz(void);
static inline uint64_t audio_frame_duration_us(void);
static inline void reset_audio_ts(void);

typedef struct SmolRtspClient SmolRtspClient;

typedef struct Controller {
    SmolRtspClient *client;
} Controller;

static int g_client_count = 0;
static uint64_t g_audio_ts_us = 0;

typedef struct SmolRtspClient {
    uint64_t session_id;
    struct bufferevent *bev;
    Controller controller_state;
    SmolRTSP_Controller controller_iface;
    void *dispatch_ctx;
    SmolRTSP_RtpTransport *video_rtp;
    SmolRTSP_NalTransport *video_nal;
    SmolRTSP_RtpTransport *audio_rtp;
    SmolRTSP_ChannelPair channels;
    int playing;
    int alive;
} SmolRtspClient;

typedef struct {
    struct event_base *base;
    struct evconnlistener *listener;
    pthread_t loop_thread;
    pthread_mutex_t mtx;
    SmolRtspClient clients[MAX_CLIENTS];
    int running;
} SmolRtspServer;

static SmolRtspServer g_srv;

static uint64_t gen_session_id(void) {
    uint64_t hi = (uint64_t)rand();
    uint64_t lo = (uint64_t)rand();
    return (hi << 32) | lo;
}

static SmolRtspClient *alloc_client(void) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!g_srv.clients[i].alive) {
            memset(&g_srv.clients[i], 0, sizeof(SmolRtspClient));
            g_srv.clients[i].alive = 1;
            g_client_count++;
            return &g_srv.clients[i];
        }
    }
    return NULL;
}

static SmolRtspClient *find_client(uint64_t session_id) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (g_srv.clients[i].alive && g_srv.clients[i].session_id == session_id)
            return &g_srv.clients[i];
    }
    return NULL;
}

static void drop_client(SmolRtspClient *c) {
    if (!c) return;
    fprintf(stderr, "[rtsp] drop_client session=%llu alive=%d bev=%p\n",
            (unsigned long long)c->session_id, c->alive, (void *)c->bev);
    if (c->dispatch_ctx) {
        smolrtsp_libevent_ctx_free(c->dispatch_ctx);
        c->dispatch_ctx = NULL;
    }
    if (c->video_nal) {
        VTABLE(SmolRTSP_NalTransport, SmolRTSP_Droppable).drop(c->video_nal);
        c->video_nal = NULL;
    }
    if (c->video_rtp) {
        VTABLE(SmolRTSP_RtpTransport, SmolRTSP_Droppable).drop(c->video_rtp);
        c->video_rtp = NULL;
    }
    if (c->audio_rtp) {
        VTABLE(SmolRTSP_RtpTransport, SmolRTSP_Droppable).drop(c->audio_rtp);
        c->audio_rtp = NULL;
    }
    if (c->bev) {
        bufferevent_free(c->bev);
        c->bev = NULL;
    }
    memset(c, 0, sizeof(*c));
    if (g_client_count > 0)
        g_client_count--;
    if (g_client_count == 0)
        reset_audio_ts();
}

static void Controller_drop(VSelf) {
    (void)iface99_self;
}

declImpl(SmolRTSP_Controller, Controller);
declImpl(SmolRTSP_Droppable, Controller);

static SmolRTSP_RtpTransport *setup_rtp_transport(
    SmolRtspClient *c, SmolRTSP_Context *ctx, SmolRTSP_TransportConfig cfg,
    track_kind kind) {
    SmolRTSP_ChannelPair pair = {.rtp_channel = DEFAULT_TCP_CHANNEL_RTP,
        .rtcp_channel = DEFAULT_TCP_CHANNEL_RTCP};

    match(cfg.interleaved) {
        of(SmolRTSP_ChannelPair_Some, v) {
            pair = *v;
        }
        otherwise {
            if (kind == TRACK_AUDIO) {
                pair.rtp_channel = 2;
                pair.rtcp_channel = 3;
            }
        }
    }

    SmolRTSP_Writer writer = SmolRTSP_Context_get_writer(ctx);
    SmolRTSP_Transport t =
        smolrtsp_transport_tcp(writer, pair.rtp_channel, 256 * 1024 /* max buffer */);

    uint8_t payload = (kind == TRACK_VIDEO) ? VIDEO_PAYLOAD_TYPE : MP3_PAYLOAD_TYPE;
    uint32_t clock = (kind == TRACK_VIDEO) ? VIDEO_CLOCK : audio_clock_hz();

    SmolRTSP_RtpTransport *rtp = SmolRTSP_RtpTransport_new(t, payload, clock);

    if (kind == TRACK_VIDEO) {
        c->video_rtp = rtp;
        c->video_nal = SmolRTSP_NalTransport_new(rtp);
    } else {
        c->audio_rtp = rtp;
    }

    c->channels = pair;
    fprintf(stderr,
        "[rtsp] setup track=%s session=%llu ch=%d/%d payload=%u clock=%u\n",
        (kind == TRACK_VIDEO) ? "video" : "audio",
        (unsigned long long)c->session_id,
        pair.rtp_channel, pair.rtcp_channel, payload, clock);
    return rtp;
}

static void Controller_options(VSelf, SmolRTSP_Context *ctx, const SmolRTSP_Request *req) {
    (void)req;
    smolrtsp_header(ctx, SMOLRTSP_HEADER_PUBLIC, "DESCRIBE, SETUP, PLAY, TEARDOWN");
    smolrtsp_respond_ok(ctx);
}

static void Controller_describe(VSelf, SmolRTSP_Context *ctx, const SmolRTSP_Request *req) {
    (void)req;

    char sdp[512] = {0};
    SmolRTSP_Writer w = smolrtsp_string_writer(sdp);
    ssize_t ret = 0;

    SMOLRTSP_SDP_DESCRIBE(
        ret, w,
        (SMOLRTSP_SDP_VERSION, "0"),
        (SMOLRTSP_SDP_ORIGIN, "divinus-smolrtsp 0 0 IN IP4 0.0.0.0"),
        (SMOLRTSP_SDP_SESSION_NAME, "divinus"),
        (SMOLRTSP_SDP_CONNECTION, "IN IP4 0.0.0.0"),
        (SMOLRTSP_SDP_TIME, "0 0"),
        (SMOLRTSP_SDP_ATTR, "tool:smolrtsp"));

    SMOLRTSP_SDP_DESCRIBE(
        ret, w,
        (SMOLRTSP_SDP_MEDIA, "video 0 RTP/AVP %d", VIDEO_PAYLOAD_TYPE),
        (SMOLRTSP_SDP_ATTR, "control:video"),
        (SMOLRTSP_SDP_ATTR, "rtpmap:%d H264/%d", VIDEO_PAYLOAD_TYPE, VIDEO_CLOCK),
        (SMOLRTSP_SDP_ATTR, "fmtp:%d packetization-mode=1", VIDEO_PAYLOAD_TYPE));

    if (app_config.audio_enable && app_config.audio_bitrate) {
        SMOLRTSP_SDP_DESCRIBE(
            ret, w,
            (SMOLRTSP_SDP_MEDIA, "audio 0 RTP/AVP %d", MP3_PAYLOAD_TYPE),
            (SMOLRTSP_SDP_ATTR, "control:audio"),
            // Payload type 14 is static MPA (MP1/MP2/MP3); use actual sample rate
            (SMOLRTSP_SDP_ATTR, "rtpmap:%d MPA/%d", MP3_PAYLOAD_TYPE, audio_clock_hz()),
            (SMOLRTSP_SDP_ATTR, "fmtp:%d layer=3", MP3_PAYLOAD_TYPE));
    }

    smolrtsp_header(ctx, SMOLRTSP_HEADER_CONTENT_TYPE, "application/sdp");
    smolrtsp_body(ctx, CharSlice99_from_str(sdp));
    smolrtsp_respond_ok(ctx);
}

static void Controller_setup(VSelf, SmolRTSP_Context *ctx, const SmolRTSP_Request *req) {
    VSELF(Controller);
    SmolRtspClient *client = self->client;

    CharSlice99 transport_value;
    if (!SmolRTSP_HeaderMap_find(&req->header_map, SMOLRTSP_HEADER_TRANSPORT, &transport_value)) {
        smolrtsp_respond(ctx, SMOLRTSP_STATUS_BAD_REQUEST, "Transport required");
        return;
    }

    SmolRTSP_TransportConfig cfg = {0};
    if (smolrtsp_parse_transport(&cfg, transport_value) < 0 ||
        cfg.lower != SmolRTSP_LowerTransport_TCP) {
        smolrtsp_respond(ctx, SMOLRTSP_STATUS_UNSUPPORTED_TRANSPORT, "TCP interleaved only");
        return;
    }

    const bool is_audio =
        CharSlice99_primitive_ends_with(req->start_line.uri, CharSlice99_from_str("/audio"));
    const track_kind kind = is_audio ? TRACK_AUDIO : TRACK_VIDEO;

    pthread_mutex_lock(&g_srv.mtx);
    if (!client->session_id)
        client->session_id = gen_session_id();
    setup_rtp_transport(client, ctx, cfg, kind);
    pthread_mutex_unlock(&g_srv.mtx);

    uint8_t rtp_ch = client->channels.rtp_channel;
    uint8_t rtcp_ch = client->channels.rtcp_channel;
    smolrtsp_header(
        ctx, SMOLRTSP_HEADER_TRANSPORT, "RTP/AVP/TCP;unicast;interleaved=%d-%d",
        rtp_ch, rtcp_ch);
    smolrtsp_header(ctx, SMOLRTSP_HEADER_SESSION, "%llu;timeout=60", client->session_id);
    smolrtsp_respond_ok(ctx);
}

static void Controller_play(VSelf, SmolRTSP_Context *ctx, const SmolRTSP_Request *req) {
    (void)req;
    VSELF(Controller);
    pthread_mutex_lock(&g_srv.mtx);
    self->client->playing = 1;
    pthread_mutex_unlock(&g_srv.mtx);
    fprintf(stderr, "[rtsp] PLAY session=%llu\n", (unsigned long long)self->client->session_id);
    // Nudge encoder to send fresh IDR/SPS/PPS so new client can decode right away.
    request_idr();
    smolrtsp_header(ctx, SMOLRTSP_HEADER_SESSION, "%llu", self->client->session_id);
    smolrtsp_respond_ok(ctx);
}

static void Controller_teardown(VSelf, SmolRTSP_Context *ctx, const SmolRTSP_Request *req) {
    (void)req;
    VSELF(Controller);
    pthread_mutex_lock(&g_srv.mtx);
    drop_client(self->client);
    pthread_mutex_unlock(&g_srv.mtx);
    smolrtsp_respond_ok(ctx);
}

static void Controller_unknown(VSelf, SmolRTSP_Context *ctx, const SmolRTSP_Request *req) {
    (void)req;
    smolrtsp_respond(ctx, SMOLRTSP_STATUS_NOT_IMPLEMENTED, "Not implemented");
}

static SmolRTSP_ControlFlow Controller_before(
    VSelf, SmolRTSP_Context *ctx, const SmolRTSP_Request *req) {
    (void)ctx;
    (void)req;
    return SmolRTSP_ControlFlow_Continue;
}

static void Controller_after(
    VSelf, ssize_t ret, SmolRTSP_Context *ctx, const SmolRTSP_Request *req) {
    (void)ctx;
    (void)req;
    if (ret < 0)
        fprintf(stderr, "RTSP respond failed: %zd\n", ret);
}

impl(SmolRTSP_Controller, Controller);
impl(SmolRTSP_Droppable, Controller);

static inline uint32_t audio_clock_hz(void) {
    return app_config.audio_srate ? (uint32_t)app_config.audio_srate : 90000;
}

static inline uint64_t audio_frame_duration_us(void) {
    const uint32_t sr = audio_clock_hz();
    if (!sr) return 0;
    // MP3 (MPEG-1 Layer III) frame holds 1152 samples at 32/44.1/48 kHz.
    return (uint64_t)1152 * 1000000ULL / sr;
}

static inline uint64_t monotonic_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static inline void reset_audio_ts(void) { g_audio_ts_us = 0; }

static void on_event_cb(struct bufferevent *bev, short events, void *ctx) {
    (void)ctx;
    if (events & (BEV_EVENT_EOF | BEV_EVENT_ERROR)) {
        fprintf(stderr, "[rtsp] client event %s bev=%p\n",
                (events & BEV_EVENT_ERROR) ? "ERROR" : "EOF", (void *)bev);
        pthread_mutex_lock(&g_srv.mtx);
        for (int i = 0; i < MAX_CLIENTS; i++) {
            SmolRtspClient *c = &g_srv.clients[i];
            if (!c->alive || c->bev != bev)
                continue;
            fprintf(stderr, "[rtsp] dropping client session=%llu\n",
                    (unsigned long long)c->session_id);
            drop_client(c);
            c->alive = 0;
            break;
        }
        pthread_mutex_unlock(&g_srv.mtx);
        // Do not propagate further; just return.
        return;
    }
}

static void listener_cb(
    struct evconnlistener *listener, evutil_socket_t fd, struct sockaddr *sa,
    int socklen, void *arg) {
    (void)listener;
    (void)sa;
    (void)socklen;
    struct event_base *base = arg;

    struct bufferevent *bev =
        bufferevent_socket_new(base, fd, BEV_OPT_CLOSE_ON_FREE | BEV_OPT_THREADSAFE);
    if (!bev) {
        close(fd);
        return;
    }

    pthread_mutex_lock(&g_srv.mtx);
    SmolRtspClient *slot = alloc_client();
    pthread_mutex_unlock(&g_srv.mtx);
    if (!slot) {
        bufferevent_free(bev);
        return;
    }

    slot->bev = bev;
    slot->controller_state.client = slot;
    slot->controller_iface =
        DYN(Controller, SmolRTSP_Controller, &slot->controller_state);
    slot->dispatch_ctx = smolrtsp_libevent_ctx(slot->controller_iface);

    bufferevent_setcb(bev, smolrtsp_libevent_cb, NULL, on_event_cb, slot->dispatch_ctx);
    bufferevent_enable(bev, EV_READ | EV_WRITE);
}

static void *loop_fn(void *arg) {
    SmolRtspServer *s = arg;
    event_base_dispatch(s->base);
    return NULL;
}

int smolrtsp_server_start(void) {
    srand(time(NULL));
    memset(&g_srv, 0, sizeof(g_srv));
    pthread_mutex_init(&g_srv.mtx, NULL);
    evthread_use_pthreads();

    g_srv.base = event_base_new();
    if (!g_srv.base)
        return -1;

    struct sockaddr_in sin = {
        .sin_family = AF_INET,
        .sin_port = htons(app_config.rtsp_port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    g_srv.listener = evconnlistener_new_bind(
        g_srv.base, listener_cb, g_srv.base,
        LEV_OPT_CLOSE_ON_FREE | LEV_OPT_REUSEABLE | LEV_OPT_THREADSAFE, -1,
        (struct sockaddr *)&sin, sizeof(sin));
    if (!g_srv.listener)
        return -1;

    g_srv.running = 1;
    if (pthread_create(&g_srv.loop_thread, NULL, loop_fn, &g_srv))
        return -1;

    return 0;
}

void smolrtsp_server_stop(void) {
    if (!g_srv.running)
        return;

    g_srv.running = 0;
    if (g_srv.base) {
        event_base_loopbreak(g_srv.base);
    }
    if (g_srv.loop_thread)
        pthread_join(g_srv.loop_thread, NULL);

    pthread_mutex_lock(&g_srv.mtx);
    for (int i = 0; i < MAX_CLIENTS; i++)
        drop_client(&g_srv.clients[i]);
    pthread_mutex_unlock(&g_srv.mtx);

    if (g_srv.listener)
        evconnlistener_free(g_srv.listener);
    if (g_srv.base)
        event_base_free(g_srv.base);
    pthread_mutex_destroy(&g_srv.mtx);
    memset(&g_srv, 0, sizeof(g_srv));
}

static size_t skip_start_code(const uint8_t *buf, size_t len) {
    if (len >= 4 && buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 1)
        return 4;
    if (len >= 3 && buf[0] == 0 && buf[1] == 0 && buf[2] == 1)
        return 3;
    return 0;
}

int smolrtsp_push_video(const uint8_t *buf, size_t len, int is_h265, uint64_t ts_us) {
    if (!g_srv.running || !buf || len < 2)
        return -1;

    size_t offset = skip_start_code(buf, len);
    if (offset >= len)
        return -1;

    SmolRTSP_NalUnit nalu;
    if (is_h265) {
        if (len - offset < 2) {
            return -1;
        }
        uint8_t hdr_bytes[2] = {buf[offset], buf[offset + 1]};
        SmolRTSP_H265NalHeader hdr = SmolRTSP_H265NalHeader_parse(hdr_bytes);
        nalu.header = SmolRTSP_NalHeader_H265(hdr);
        nalu.payload = U8Slice99_from_ptrdiff(
            (uint8_t *)(buf + offset + 2), (uint8_t *)(buf + len));
    } else {
        SmolRTSP_H264NalHeader hdr = SmolRTSP_H264NalHeader_parse(buf[offset]);
        nalu.header = SmolRTSP_NalHeader_H264(hdr);
        nalu.payload = U8Slice99_from_ptrdiff(
            (uint8_t *)(buf + offset + 1), (uint8_t *)(buf + len));
    }

    SmolRTSP_RtpTimestamp ts = ts_us ? SmolRTSP_RtpTimestamp_SysClockUs(ts_us)
                                      : SmolRTSP_RtpTimestamp_SysClockUs(0);

    pthread_mutex_lock(&g_srv.mtx);
    int sent = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        SmolRtspClient *c = &g_srv.clients[i];
        if (!c->alive || !c->video_nal || !c->playing)
            continue;
        int ret = SmolRTSP_NalTransport_send_packet(c->video_nal, ts, nalu);
        if (ret < 0) {
            // Best-effort send; skip on error.
            continue;
        }
        sent++;
    }
    pthread_mutex_unlock(&g_srv.mtx);
    if (sent == 0) {
        static uint64_t last_log = 0;
        uint64_t now = monotonic_us();
        if (now - last_log > 2 * 1000000ULL) { // log once per 2s to avoid spam
            fprintf(stderr, "[rtsp] video drop: no active clients\n");
            last_log = now;
        }
    }
    return 0;
}

int smolrtsp_push_mp3(const uint8_t *buf, size_t len, uint64_t ts_us) {
    if (!g_srv.running || !buf || !len)
        return -1;
    if (!ts_us) {
        uint64_t frame_us = audio_frame_duration_us();
        if (!g_audio_ts_us)
            g_audio_ts_us = monotonic_us();
        else
            g_audio_ts_us += frame_us ? frame_us : 0;
        ts_us = g_audio_ts_us;
    }
    SmolRTSP_RtpTimestamp ts = SmolRTSP_RtpTimestamp_SysClockUs(ts_us);
    U8Slice99 payload = U8Slice99_from_ptrdiff((uint8_t *)buf, (uint8_t *)(buf + len));
    pthread_mutex_lock(&g_srv.mtx);
    int sent = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        SmolRtspClient *c = &g_srv.clients[i];
        if (!c->alive || !c->audio_rtp || !c->playing)
            continue;
        int ret = SmolRTSP_RtpTransport_send_packet(
            c->audio_rtp, ts, true, U8Slice99_empty(), payload);
        if (ret < 0) {
            continue;
        }
        sent++;
    }
    pthread_mutex_unlock(&g_srv.mtx);
    if (sent == 0) {
        static uint64_t last_log = 0;
        uint64_t now = monotonic_us();
        if (now - last_log > 2 * 1000000ULL) {
            fprintf(stderr, "[rtsp] audio drop: no active clients\n");
            last_log = now;
        }
    }
    return 0;
}
