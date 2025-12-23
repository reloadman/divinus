#include "rtsp_smol.h"

#include "app_config.h"
#include "hal/tools.h"
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
#include <inttypes.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#define MAX_CLIENTS 4
#define VIDEO_PAYLOAD_TYPE 96
#define VIDEO_CLOCK 90000
#define AAC_PAYLOAD_TYPE 97
#define DEFAULT_TCP_CHANNEL_RTP 0
#define DEFAULT_TCP_CHANNEL_RTCP 1
// Must match max_buffer passed to smolrtsp_transport_tcp(...) in setup_rtp_transport().
#define RTSP_TCP_MAX_BUFFER (512 * 1024)
// When TCP buffer is full, drain oldest interleaved frames down to this size.
#define RTSP_TCP_TRIM_TARGET (RTSP_TCP_MAX_BUFFER / 2)

// NAL unit types (H.264 / H.265) used for SDP parameter collection.
#define H264_NAL_TYPE_SPS 7
#define H264_NAL_TYPE_PPS 8
#define H265_NAL_TYPE_VPS 32
#define H265_NAL_TYPE_SPS 33
#define H265_NAL_TYPE_PPS 34

typedef enum {
    TRACK_VIDEO,
    TRACK_AUDIO
} track_kind;

// Forward declarations for helpers used before their definitions.
static inline uint32_t audio_clock_hz(void);
static inline uint32_t audio_frame_samples(void);
static inline uint32_t audio_ts_step(void);
static inline uint64_t audio_frame_duration_us(void);
static inline void reset_audio_ts(void);
static inline uint8_t aac_samplerate_index(uint32_t srate);
static inline uint16_t aac_audio_specific_config(void);
static inline void aac_config_hex(char *dst, size_t dst_sz);
static void on_event_cb(struct bufferevent *bev, short events, void *ctx);
static int trim_tcp_interleaved_oldest(struct bufferevent *bev, size_t target_len);
static size_t bev_output_len(struct bufferevent *bev);

typedef struct SmolRtspClient SmolRtspClient;

typedef struct Controller {
    SmolRtspClient *client;
} Controller;

static int g_client_count = 0;
static uint64_t g_audio_ts_us = 0;
static uint32_t g_audio_ts_raw = 0;

typedef struct SmolRtspClient {
    uint64_t session_id;
    struct bufferevent *bev;
    struct sockaddr_storage peer_addr;
    socklen_t peer_addr_len;
    Controller controller_state;
    SmolRTSP_Controller controller_iface;
    void *dispatch_ctx;
    SmolRTSP_RtpTransport *video_rtp;
    SmolRTSP_NalTransport *video_nal;
    SmolRTSP_RtpTransport *audio_rtp;
    SmolRTSP_ChannelPair channels;
    int playing;
    int alive;
    // Slot is reserved for cleanup; avoid reuse until resources are freed.
    int closing;
    // Prevent scheduling multiple deferred drops for same client.
    int drop_scheduled;
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

// Latest codec parameter sets, collected from the live bitstream.
// Used to populate SDP (sprop-parameter-sets) so ffplay can decode immediately.
static char g_h264_sps_b64[2048];
static char g_h264_pps_b64[2048];
static int g_h264_have_sps = 0;
static int g_h264_have_pps = 0;
static uint8_t g_h264_profile_level_id[3] = {0}; // bytes 1..3 of SPS NAL (after NAL header)
static int g_h264_have_profile = 0;

static char g_h265_vps_b64[2048];
static char g_h265_sps_b64[2048];
static char g_h265_pps_b64[2048];
static int g_h265_have_vps = 0;
static int g_h265_have_sps = 0;
static int g_h265_have_pps = 0;

static uint64_t gen_session_id(void) {
    uint64_t hi = (uint64_t)rand();
    uint64_t lo = (uint64_t)rand();
    return (hi << 32) | lo;
}

static SmolRtspClient *alloc_client(void) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!g_srv.clients[i].alive && !g_srv.clients[i].closing) {
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
    const int was_alive = c->alive;
    fprintf(stderr, "[rtsp] drop_client session=%llu alive=%d bev=%p\n",
            (unsigned long long)c->session_id, c->alive, (void *)c->bev);
    if (c->video_nal) {
        VTABLE(SmolRTSP_NalTransport, SmolRTSP_Droppable).drop(c->video_nal);
        c->video_nal = NULL;
        // NalTransport owns the underlying RTP transport and drops it as well.
        c->video_rtp = NULL;
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
    // Keep dispatch_ctx allocated to avoid use-after-free in libevent callbacks.
    c->playing = 0;
    c->alive = 0;
    c->closing = 0;
    c->drop_scheduled = 0;
    if (was_alive) {
        if (g_client_count > 0)
            g_client_count--;
        if (g_client_count == 0)
            reset_audio_ts();
    }
}

static void drop_client_deferred(evutil_socket_t fd, short what, void *arg) {
    (void)fd;
    (void)what;
    SmolRtspClient *c = arg;
    pthread_mutex_lock(&g_srv.mtx);
    drop_client(c);
    pthread_mutex_unlock(&g_srv.mtx);
}

static void on_write_close_cb(struct bufferevent *bev, void *ctx) {
    (void)ctx;
    if (!bev)
        return;
    // When output buffer is drained, we can safely drop the client.
    struct evbuffer *out = bufferevent_get_output(bev);
    if (!out || evbuffer_get_length(out) != 0)
        return;

    SmolRtspClient *target = NULL;
    pthread_mutex_lock(&g_srv.mtx);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        SmolRtspClient *c = &g_srv.clients[i];
        if (c->bev == bev) {
            if (c->closing && !c->drop_scheduled)
                target = c;
            break;
        }
    }
    if (target)
        target->drop_scheduled = 1;
    pthread_mutex_unlock(&g_srv.mtx);

    if (target) {
        struct timeval tv = {.tv_sec = 0, .tv_usec = 0};
        event_base_once(g_srv.base, -1, EV_TIMEOUT, drop_client_deferred, target, &tv);
    }
}

static void Controller_drop(VSelf) {
    (void)iface99_self;
}

declImpl(SmolRTSP_Controller, Controller);
declImpl(SmolRTSP_Droppable, Controller);

static int setup_rtp_transport(
    SmolRtspClient *c, SmolRTSP_Context *ctx, SmolRTSP_TransportConfig cfg,
    track_kind kind) {
    // Drop previous transport for this track (best-effort).
    if (kind == TRACK_VIDEO) {
        if (c->video_nal) {
            VTABLE(SmolRTSP_NalTransport, SmolRTSP_Droppable).drop(c->video_nal);
            c->video_nal = NULL;
            // NalTransport owns the underlying RTP transport and drops it too.
            c->video_rtp = NULL;
        }
        if (c->video_rtp) {
            VTABLE(SmolRTSP_RtpTransport, SmolRTSP_Droppable).drop(c->video_rtp);
            c->video_rtp = NULL;
        }
    } else {
        if (c->audio_rtp) {
            VTABLE(SmolRTSP_RtpTransport, SmolRTSP_Droppable).drop(c->audio_rtp);
            c->audio_rtp = NULL;
        }
    }

    uint8_t payload = (kind == TRACK_VIDEO) ? VIDEO_PAYLOAD_TYPE : AAC_PAYLOAD_TYPE;
    uint32_t clock = (kind == TRACK_VIDEO) ? VIDEO_CLOCK : audio_clock_hz();

    if (cfg.lower == SmolRTSP_LowerTransport_TCP) {
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
            smolrtsp_transport_tcp(writer, pair.rtp_channel, RTSP_TCP_MAX_BUFFER /* max buffer */);

        SmolRTSP_RtpTransport *rtp = SmolRTSP_RtpTransport_new(t, payload, clock);

        if (kind == TRACK_VIDEO) {
            c->video_rtp = rtp;
            c->video_nal = SmolRTSP_NalTransport_new(rtp);
        } else {
            c->audio_rtp = rtp;
        }

        c->channels = pair;
        smolrtsp_header(
            ctx, SMOLRTSP_HEADER_TRANSPORT, "RTP/AVP/TCP;unicast;interleaved=%d-%d",
            pair.rtp_channel, pair.rtcp_channel);

        fprintf(stderr,
            "[rtsp] setup track=%s session=%llu tcp ch=%d/%d payload=%u clock=%u\n",
            (kind == TRACK_VIDEO) ? "video" : "audio",
            (unsigned long long)c->session_id,
            pair.rtp_channel, pair.rtcp_channel, payload, clock);
        return 0;
    }

    // Default RTSP transport is UDP; support it for ffmpeg/ffplay defaults.
    if (cfg.lower == SmolRTSP_LowerTransport_UDP) {
        ifLet(cfg.client_port, SmolRTSP_PortPair_Some, client_port) {
            const struct sockaddr *addr = (const struct sockaddr *)&c->peer_addr;
            const int af = addr->sa_family;
            const void *ip = smolrtsp_sockaddr_ip(addr);

            int rtp_fd = smolrtsp_dgram_socket(af, ip, client_port->rtp_port);
            if (rtp_fd == -1) {
                smolrtsp_respond_internal_error(ctx);
                return -1;
            }

            // Best-effort RTCP socket (we don't send RTCP, but some clients expect a pair).
            int rtcp_fd = smolrtsp_dgram_socket(af, ip, client_port->rtcp_port);
            if (rtcp_fd != -1) {
                close(rtcp_fd);
            }

            SmolRTSP_Transport t = smolrtsp_transport_udp(rtp_fd);
            SmolRTSP_RtpTransport *rtp = SmolRTSP_RtpTransport_new(t, payload, clock);

            if (kind == TRACK_VIDEO) {
                c->video_rtp = rtp;
                c->video_nal = SmolRTSP_NalTransport_new(rtp);
            } else {
                c->audio_rtp = rtp;
            }

            // Include server_port using the ephemeral local port chosen for this socket.
            struct sockaddr_storage local;
            socklen_t local_len = sizeof local;
            uint16_t server_rtp_port = 0;
            if (getsockname(rtp_fd, (struct sockaddr *)&local, &local_len) == 0) {
                if (local.ss_family == AF_INET)
                    server_rtp_port = ntohs(((struct sockaddr_in *)&local)->sin_port);
                else if (local.ss_family == AF_INET6)
                    server_rtp_port = ntohs(((struct sockaddr_in6 *)&local)->sin6_port);
            }

            if (server_rtp_port) {
                smolrtsp_header(
                    ctx, SMOLRTSP_HEADER_TRANSPORT,
                    "RTP/AVP/UDP;unicast;client_port=%" PRIu16 "-%" PRIu16 ";server_port=%" PRIu16 "-%" PRIu16,
                    client_port->rtp_port, client_port->rtcp_port,
                    server_rtp_port, (uint16_t)(server_rtp_port + 1));
            } else {
                smolrtsp_header(
                    ctx, SMOLRTSP_HEADER_TRANSPORT,
                    "RTP/AVP/UDP;unicast;client_port=%" PRIu16 "-%" PRIu16,
                    client_port->rtp_port, client_port->rtcp_port);
            }

            fprintf(stderr,
                "[rtsp] setup track=%s session=%llu udp dst=%" PRIu16 "/%" PRIu16 " payload=%u clock=%u\n",
                (kind == TRACK_VIDEO) ? "video" : "audio",
                (unsigned long long)c->session_id,
                client_port->rtp_port, client_port->rtcp_port, payload, clock);
            return 0;
        }

        smolrtsp_respond(ctx, SMOLRTSP_STATUS_BAD_REQUEST, "`client_port' required for UDP");
        return -1;
    }

    smolrtsp_respond(ctx, SMOLRTSP_STATUS_UNSUPPORTED_TRANSPORT, "Unsupported transport");
    return -1;
}

static void Controller_options(VSelf, SmolRTSP_Context *ctx, const SmolRTSP_Request *req) {
    (void)req;
    smolrtsp_header(ctx, SMOLRTSP_HEADER_PUBLIC, "DESCRIBE, SETUP, PLAY, TEARDOWN");
    smolrtsp_respond_ok(ctx);
}

static void Controller_describe(VSelf, SmolRTSP_Context *ctx, const SmolRTSP_Request *req) {
    (void)req;

    char sdp[2048] = {0};
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

    const char *video_codec = app_config.mp4_codecH265 ? "H265" : "H264";
    SMOLRTSP_SDP_DESCRIBE(
        ret, w,
        (SMOLRTSP_SDP_MEDIA, "video 0 RTP/AVP %d", VIDEO_PAYLOAD_TYPE),
        (SMOLRTSP_SDP_ATTR, "control:video"),
        (SMOLRTSP_SDP_ATTR, "rtpmap:%d %s/%d", VIDEO_PAYLOAD_TYPE, video_codec, VIDEO_CLOCK));
    // H.264 uses RFC6184 packetization-mode. For H.265 (RFC7798) this parameter
    // is not defined; omitting fmtp keeps clients happy.
    if (!app_config.mp4_codecH265) {
        // If we have SPS/PPS from the live stream, include them in SDP so new clients
        // can decode immediately without waiting for in-band parameter sets.
        int have_sprop = 0;
        int have_plid = 0;
        char plid[7] = {0};
        char sps[sizeof(g_h264_sps_b64)];
        char pps[sizeof(g_h264_pps_b64)];
        sps[0] = '\0';
        pps[0] = '\0';
        pthread_mutex_lock(&g_srv.mtx);
        have_sprop = g_h264_have_sps && g_h264_have_pps;
        have_plid = g_h264_have_profile;
        if (have_plid) {
            snprintf(plid, sizeof(plid), "%02X%02X%02X",
                g_h264_profile_level_id[0],
                g_h264_profile_level_id[1],
                g_h264_profile_level_id[2]);
        }
        // Copy under lock to avoid races while formatting SDP.
        strncpy(sps, g_h264_sps_b64, sizeof(sps) - 1);
        sps[sizeof(sps) - 1] = '\0';
        strncpy(pps, g_h264_pps_b64, sizeof(pps) - 1);
        pps[sizeof(pps) - 1] = '\0';
        pthread_mutex_unlock(&g_srv.mtx);

        if (have_sprop) {
            if (have_plid) {
                SMOLRTSP_SDP_DESCRIBE(
                    ret, w,
                    (SMOLRTSP_SDP_ATTR,
                        "fmtp:%d packetization-mode=1;profile-level-id=%s;sprop-parameter-sets=%s,%s",
                        VIDEO_PAYLOAD_TYPE, plid, sps, pps));
            } else {
                SMOLRTSP_SDP_DESCRIBE(
                    ret, w,
                    (SMOLRTSP_SDP_ATTR,
                        "fmtp:%d packetization-mode=1;sprop-parameter-sets=%s,%s",
                        VIDEO_PAYLOAD_TYPE, sps, pps));
            }
        } else {
            SMOLRTSP_SDP_DESCRIBE(
                ret, w,
                (SMOLRTSP_SDP_ATTR, "fmtp:%d packetization-mode=1", VIDEO_PAYLOAD_TYPE));
        }
    } else {
        // Best-effort: if we have VPS/SPS/PPS for H.265, include them in SDP.
        int have_sprop = 0;
        char vps[sizeof(g_h265_vps_b64)];
        char sps[sizeof(g_h265_sps_b64)];
        char pps[sizeof(g_h265_pps_b64)];
        vps[0] = '\0';
        sps[0] = '\0';
        pps[0] = '\0';
        pthread_mutex_lock(&g_srv.mtx);
        have_sprop = g_h265_have_vps && g_h265_have_sps && g_h265_have_pps;
        strncpy(vps, g_h265_vps_b64, sizeof(vps) - 1);
        vps[sizeof(vps) - 1] = '\0';
        strncpy(sps, g_h265_sps_b64, sizeof(sps) - 1);
        sps[sizeof(sps) - 1] = '\0';
        strncpy(pps, g_h265_pps_b64, sizeof(pps) - 1);
        pps[sizeof(pps) - 1] = '\0';
        pthread_mutex_unlock(&g_srv.mtx);
        if (have_sprop) {
            SMOLRTSP_SDP_DESCRIBE(
                ret, w,
                (SMOLRTSP_SDP_ATTR,
                    "fmtp:%d sprop-vps=%s;sprop-sps=%s;sprop-pps=%s",
                    VIDEO_PAYLOAD_TYPE, vps, sps, pps));
        }
    }

    if (app_config.audio_enable && app_config.audio_bitrate) {
        // AAC-only audio track.
        const uint8_t pt = AAC_PAYLOAD_TYPE;
        char config_hex[8] = {0};
        aac_config_hex(config_hex, sizeof(config_hex));
        SMOLRTSP_SDP_DESCRIBE(
            ret, w,
            (SMOLRTSP_SDP_MEDIA, "audio 0 RTP/AVP %d", pt),
            (SMOLRTSP_SDP_ATTR, "control:audio"),
            (SMOLRTSP_SDP_ATTR, "rtpmap:%d MPEG4-GENERIC/%d/%d", pt,
                audio_clock_hz(), app_config.audio_channels ? app_config.audio_channels : 1),
            (SMOLRTSP_SDP_ATTR,
                "fmtp:%d streamtype=5;profile-level-id=1;mode=AAC-hbr;"
                "sizelength=13;indexlength=3;indexdeltalength=3;config=%s",
                pt, config_hex));
    }

    // Helpful when debugging client-side codec detection (ffmpeg/ffplay).
    fprintf(stderr, "[rtsp] SDP (len=%zu):\n%s\n", strlen(sdp), sdp);

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
    if (smolrtsp_parse_transport(&cfg, transport_value) < 0) {
        smolrtsp_respond(ctx, SMOLRTSP_STATUS_BAD_REQUEST, "Malformed Transport");
        return;
    }

    const bool is_audio =
        CharSlice99_primitive_ends_with(req->start_line.uri, CharSlice99_from_str("/audio"));
    const track_kind kind = is_audio ? TRACK_AUDIO : TRACK_VIDEO;

    pthread_mutex_lock(&g_srv.mtx);
    if (!client->session_id)
        client->session_id = gen_session_id();
    if (setup_rtp_transport(client, ctx, cfg, kind) < 0) {
        pthread_mutex_unlock(&g_srv.mtx);
        return;
    }
    pthread_mutex_unlock(&g_srv.mtx);

    smolrtsp_header(ctx, SMOLRTSP_HEADER_SESSION, "%llu;timeout=30", client->session_id);
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
    // IMPORTANT: do not free the bufferevent inside the read callback
    // (smolrtsp_libevent_cb), because it will access the evbuffer after dispatch.
    smolrtsp_respond_ok(ctx);

    pthread_mutex_lock(&g_srv.mtx);
    if (self->client) {
        SmolRtspClient *c = self->client;
        c->playing = 0;
        if (c->alive) {
            c->alive = 0;
            if (g_client_count > 0)
                g_client_count--;
            if (g_client_count == 0)
                reset_audio_ts();
        }
        c->closing = 1;
        if (c->bev) {
            // Stop parsing new requests; keep writes enabled to flush response.
            bufferevent_disable(c->bev, EV_READ);
            bufferevent_setcb(c->bev, NULL, on_write_close_cb, on_event_cb, c->dispatch_ctx);
        }
    }
    pthread_mutex_unlock(&g_srv.mtx);
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

// Drain oldest TCP interleaved ($ <ch> <len>) frames from the start of the
// bufferevent output buffer until it shrinks to <= target_len.
// Returns 0 if buffer is <= target_len or trimming succeeded; -1 if we cannot
// safely trim (e.g. buffer doesn't begin with interleaved framing).
static int trim_tcp_interleaved_oldest(struct bufferevent *bev, size_t target_len) {
    if (!bev)
        return -1;

    bufferevent_lock(bev);
    struct evbuffer *out = bufferevent_get_output(bev);
    if (!out) {
        bufferevent_unlock(bev);
        return -1;
    }

    size_t len = evbuffer_get_length(out);
    const size_t len_before = len;
    if (len <= target_len) {
        bufferevent_unlock(bev);
        return 0;
    }

    unsigned char hdr[4];
    if (evbuffer_copyout(out, hdr, sizeof hdr) != (ssize_t)sizeof hdr || hdr[0] != '$') {
        bufferevent_unlock(bev);
        return -1;
    }

    int drained_any = 0;
    size_t drained_bytes = 0;
    while (len > target_len) {
        if (len < 4)
            break;
        if (evbuffer_copyout(out, hdr, sizeof hdr) != (ssize_t)sizeof hdr)
            break;
        if (hdr[0] != '$')
            break;
        const uint16_t payload_len = (uint16_t)(((uint16_t)hdr[2] << 8) | (uint16_t)hdr[3]);
        const size_t frame_len = 4u + (size_t)payload_len;
        if (frame_len <= 4 || frame_len > len) {
            // Corrupt/partial framing: stop trimming to avoid desync.
            break;
        }
        evbuffer_drain(out, frame_len);
        drained_any = 1;
        drained_bytes += frame_len;
        len = evbuffer_get_length(out);
    }

    const size_t len_after = len;
    bufferevent_unlock(bev);
    if (drained_any) {
        fprintf(stderr, "[rtsp] trim tcp output drained=%zu before=%zu after=%zu target=%zu\n",
                drained_bytes, len_before, len_after, target_len);
    }
    return drained_any ? 0 : -1;
}

static size_t bev_output_len(struct bufferevent *bev) {
    if (!bev)
        return 0;
    bufferevent_lock(bev);
    struct evbuffer *out = bufferevent_get_output(bev);
    size_t len = out ? evbuffer_get_length(out) : 0;
    bufferevent_unlock(bev);
    return len;
}

impl(SmolRTSP_Controller, Controller);
impl(SmolRTSP_Droppable, Controller);

static inline uint32_t audio_clock_hz(void) {
    uint32_t srate = app_config.audio_srate ? (uint32_t)app_config.audio_srate : 48000;
    if (!srate) srate = 48000;
    // Use RTP clock equal to the sampling rate for both AAC and MPEG audio.
    // This matches ffmpeg/ffplay behavior for RTSP sessions and avoids oddities
    // like treating the RTP clock (e.g. 90 kHz) as the decoded sample rate.
    return srate;
}

static inline uint32_t audio_frame_samples(void) {
    return 1024;
}

static inline uint32_t audio_ts_step(void) {
    // Convert frame samples into RTP clock ticks.
    const uint32_t clock = audio_clock_hz();
    const uint32_t sr = app_config.audio_srate ? (uint32_t)app_config.audio_srate : 48000;
    if (!clock || !sr) return audio_frame_samples();
    return (uint32_t)((uint64_t)audio_frame_samples() * clock / sr);
}

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

static inline uint16_t aac_audio_specific_config(void) {
    const uint8_t obj_type = 2; // AAC-LC
    uint8_t freq_idx = aac_samplerate_index(audio_clock_hz());
    uint8_t channels = app_config.audio_channels ? app_config.audio_channels : 1;
    if (channels > 2) channels = 2;
    return (uint16_t)((obj_type & 0x1F) << 11) |
           (uint16_t)((freq_idx & 0x0F) << 7) |
           (uint16_t)((channels & 0x0F) << 3);
}

static inline void aac_config_hex(char *dst, size_t dst_sz) {
    uint16_t cfg = aac_audio_specific_config();
    snprintf(dst, dst_sz, "%02X%02X", (cfg >> 8) & 0xFF, cfg & 0xFF);
}

static inline uint64_t audio_frame_duration_us(void) {
    const uint32_t sr = app_config.audio_srate ? (uint32_t)app_config.audio_srate : 48000;
    if (!sr) return 0;
    return (uint64_t)audio_frame_samples() * 1000000ULL / sr;
}

static inline uint64_t monotonic_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static void update_sprop_h264_locked(const uint8_t *nal, size_t nal_len, uint8_t nal_type) {
    if (!nal || nal_len < 2)
        return;

    if (nal_type == H264_NAL_TYPE_SPS) {
        // Base64 encode full NAL unit (NAL header + payload), per common RTSP practice.
        const int out_len = base64_encode_length((int)nal_len);
        if (out_len <= 0 || (size_t)out_len >= sizeof(g_h264_sps_b64))
            return;
        const int enc = base64_encode(g_h264_sps_b64, (const char *)nal, (int)nal_len);
        if (enc <= 0 || (size_t)enc >= sizeof(g_h264_sps_b64))
            return;
        g_h264_sps_b64[enc] = '\0';
        g_h264_have_sps = 1;

        // profile-level-id: 3 bytes from SPS after NAL header byte (if present).
        if (nal_len >= 4) {
            g_h264_profile_level_id[0] = nal[1];
            g_h264_profile_level_id[1] = nal[2];
            g_h264_profile_level_id[2] = nal[3];
            g_h264_have_profile = 1;
        }
    } else if (nal_type == H264_NAL_TYPE_PPS) {
        const int out_len = base64_encode_length((int)nal_len);
        if (out_len <= 0 || (size_t)out_len >= sizeof(g_h264_pps_b64))
            return;
        const int enc = base64_encode(g_h264_pps_b64, (const char *)nal, (int)nal_len);
        if (enc <= 0 || (size_t)enc >= sizeof(g_h264_pps_b64))
            return;
        g_h264_pps_b64[enc] = '\0';
        g_h264_have_pps = 1;
    }
}

static void update_sprop_h265_locked(const uint8_t *nal, size_t nal_len, uint8_t nal_type) {
    if (!nal || nal_len < 3)
        return;

    if (nal_type == H265_NAL_TYPE_VPS) {
        const int out_len = base64_encode_length((int)nal_len);
        if (out_len <= 0 || (size_t)out_len >= sizeof(g_h265_vps_b64))
            return;
        const int enc = base64_encode(g_h265_vps_b64, (const char *)nal, (int)nal_len);
        if (enc <= 0 || (size_t)enc >= sizeof(g_h265_vps_b64))
            return;
        g_h265_vps_b64[enc] = '\0';
        g_h265_have_vps = 1;
    } else if (nal_type == H265_NAL_TYPE_SPS) {
        const int out_len = base64_encode_length((int)nal_len);
        if (out_len <= 0 || (size_t)out_len >= sizeof(g_h265_sps_b64))
            return;
        const int enc = base64_encode(g_h265_sps_b64, (const char *)nal, (int)nal_len);
        if (enc <= 0 || (size_t)enc >= sizeof(g_h265_sps_b64))
            return;
        g_h265_sps_b64[enc] = '\0';
        g_h265_have_sps = 1;
    } else if (nal_type == H265_NAL_TYPE_PPS) {
        const int out_len = base64_encode_length((int)nal_len);
        if (out_len <= 0 || (size_t)out_len >= sizeof(g_h265_pps_b64))
            return;
        const int enc = base64_encode(g_h265_pps_b64, (const char *)nal, (int)nal_len);
        if (enc <= 0 || (size_t)enc >= sizeof(g_h265_pps_b64))
            return;
        g_h265_pps_b64[enc] = '\0';
        g_h265_have_pps = 1;
    }
}

static inline void reset_audio_ts(void) {
    g_audio_ts_us = 0;
    g_audio_ts_raw = 0;
}

static void on_event_cb(struct bufferevent *bev, short events, void *ctx) {
    (void)ctx;
    if (events & (BEV_EVENT_EOF | BEV_EVENT_ERROR | BEV_EVENT_TIMEOUT)) {
        const char *ev =
            (events & BEV_EVENT_ERROR)   ? "ERROR" :
            (events & BEV_EVENT_TIMEOUT) ? "TIMEOUT" : "EOF";
        fprintf(stderr, "[rtsp] client event %s bev=%p\n", ev, (void *)bev);
        SmolRtspClient *target = NULL;
        pthread_mutex_lock(&g_srv.mtx);
        for (int i = 0; i < MAX_CLIENTS; i++) {
            SmolRtspClient *c = &g_srv.clients[i];
            if (!c->bev || c->bev != bev)
                continue;
            if (!c->alive && !c->closing)
                continue;
            fprintf(stderr, "[rtsp] marking client dead (event=%s) session=%llu\n",
                    ev, (unsigned long long)c->session_id);
            // Disable further callbacks to avoid use-after-free during this callback.
            bufferevent_disable(bev, EV_READ | EV_WRITE);
            bufferevent_setcb(bev, NULL, NULL, NULL, NULL);
            // Do NOT free transports here; just mark the slot dead.
            c->playing = 0;
            const int was_alive = c->alive;
            c->alive = 0;
            c->closing = 1;
            if (was_alive) {
                if (g_client_count > 0)
                    g_client_count--;
                if (g_client_count == 0)
                    reset_audio_ts();
            }
            if (!c->drop_scheduled)
                target = c;
            break;
        }
        if (target)
            target->drop_scheduled = 1;
        pthread_mutex_unlock(&g_srv.mtx);
        if (target) {
            struct timeval tv = {.tv_sec = 0, .tv_usec = 0};
            event_base_once(g_srv.base, -1, EV_TIMEOUT, drop_client_deferred, target, &tv);
        }
        // Do not propagate further; just return.
        return;
    }
}

static void listener_cb(
    struct evconnlistener *listener, evutil_socket_t fd, struct sockaddr *sa,
    int socklen, void *arg) {
    (void)listener;
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
    // Save peer address for UDP RTP setup.
    memset(&slot->peer_addr, 0, sizeof slot->peer_addr);
    slot->peer_addr_len = 0;
    if (sa && socklen > 0) {
        size_t n = (size_t)socklen;
        if (n > sizeof slot->peer_addr)
            n = sizeof slot->peer_addr;
        memcpy(&slot->peer_addr, sa, n);
        slot->peer_addr_len = (socklen_t)n;
    }
    slot->controller_state.client = slot;
    slot->controller_iface =
        DYN(Controller, SmolRTSP_Controller, &slot->controller_state);
    slot->dispatch_ctx = smolrtsp_libevent_ctx(slot->controller_iface);

    struct timeval tv = {.tv_sec = 30, .tv_usec = 0};
    bufferevent_set_timeouts(bev, &tv, &tv);
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
    if (!EMPTY(app_config.rtsp_bind)) {
        struct in_addr bind_addr = {0};
        if (inet_aton(app_config.rtsp_bind, &bind_addr)) {
            sin.sin_addr = bind_addr;
        } else {
            fprintf(stderr, "[rtsp] invalid bind address '%s', falling back to 0.0.0.0\n",
                app_config.rtsp_bind);
        }
    }

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

    // Collect codec parameter sets for SDP (best-effort).
    // This helps clients like ffplay decode immediately (avoids "non-existing PPS").
    if (is_h265) {
        if (len - offset >= 2) {
            const uint8_t nal_type = (uint8_t)((buf[offset] >> 1) & 0x3F);
            if (nal_type == H265_NAL_TYPE_VPS || nal_type == H265_NAL_TYPE_SPS || nal_type == H265_NAL_TYPE_PPS) {
                pthread_mutex_lock(&g_srv.mtx);
                update_sprop_h265_locked(buf + offset, len - offset, nal_type);
                pthread_mutex_unlock(&g_srv.mtx);
            }
        }
    } else {
        const uint8_t nal_type = (uint8_t)(buf[offset] & 0x1F);
        if (nal_type == H264_NAL_TYPE_SPS || nal_type == H264_NAL_TYPE_PPS) {
            pthread_mutex_lock(&g_srv.mtx);
            update_sprop_h264_locked(buf + offset, len - offset, nal_type);
            pthread_mutex_unlock(&g_srv.mtx);
        }
    }

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

    // If upstream timestamp is missing/zero, use monotonic time (us) so RTP
    // timestamps advance and clients don't report excessive reordering/drops.
    if (!ts_us)
        ts_us = monotonic_us();
    SmolRTSP_RtpTimestamp ts = SmolRTSP_RtpTimestamp_SysClockUs(ts_us);

    pthread_mutex_lock(&g_srv.mtx);
    int sent = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        SmolRtspClient *c = &g_srv.clients[i];
        if (!c->alive || !c->video_nal || !c->playing)
            continue;
        // IMPORTANT: In RTSP/TCP interleaved mode, audio/video share one output buffer.
        // Under poor TCP conditions video can starve audio. Prefer keeping audio alive:
        // if output is congested, drop VIDEO packets (do not enqueue them).
        const size_t out_len = bev_output_len(c->bev);
        if (out_len > RTSP_TCP_MAX_BUFFER) {
            // TCP output is congested; drop oldest interleaved frames to recover.
            // If we can't trim safely, fall back to dropping newest video.
            if (trim_tcp_interleaved_oldest(c->bev, RTSP_TCP_TRIM_TARGET) < 0) {
                static uint64_t last_log = 0;
                uint64_t now = monotonic_us();
                if (now - last_log > 1000 * 1000ULL) {
                    fprintf(stderr,
                        "[rtsp] video drop: tcp buffer full (no trim) session=%llu len=%zu max=%u target=%u\n",
                        (unsigned long long)c->session_id, out_len,
                        (unsigned)RTSP_TCP_MAX_BUFFER, (unsigned)RTSP_TCP_TRIM_TARGET);
                    last_log = now;
                }
                continue;
            }
        }

        if (SmolRTSP_NalTransport_is_full(c->video_nal)) {
            static uint64_t last_log = 0;
            uint64_t now = monotonic_us();
            if (now - last_log > 1000 * 1000ULL) {
                const size_t cur = bev_output_len(c->bev);
                fprintf(stderr,
                    "[rtsp] video drop: transport full session=%llu len=%zu max=%u\n",
                    (unsigned long long)c->session_id, cur, (unsigned)RTSP_TCP_MAX_BUFFER);
                last_log = now;
            }
            continue;
        }
        int ret = SmolRTSP_NalTransport_send_packet(c->video_nal, ts, nalu);
        if (ret < 0) {
            // Best-effort send; skip on error.
            continue;
        }
        sent++;
    }
    pthread_mutex_unlock(&g_srv.mtx);
    return 0;
}

int smolrtsp_push_aac(const uint8_t *buf, size_t len, uint64_t ts_us) {
    if (!g_srv.running || !buf || !len)
        return -1;
    SmolRTSP_RtpTimestamp ts;
    if (!ts_us) {
        uint32_t raw = g_audio_ts_raw;
        g_audio_ts_raw += audio_ts_step();
        ts = SmolRTSP_RtpTimestamp_Raw(raw);
    } else {
        ts = SmolRTSP_RtpTimestamp_SysClockUs(ts_us);
    }

    // RFC 3640 AU headers: 16-bit AU-headers-length, then one AU header (size/offset).
    uint8_t au_header_section[4] = {0};
    // AU-headers-length in bits = 16 (one AU header of 2 bytes)
    au_header_section[0] = 0x00;
    au_header_section[1] = 0x10;
    // AU-size 13 bits, AU-Index 3 bits (set to 0)
    uint16_t au = (uint16_t)((len & 0x1FFF) << 3);
    au_header_section[2] = (uint8_t)((au >> 8) & 0xFF);
    au_header_section[3] = (uint8_t)(au & 0xFF);

    U8Slice99 hdr = U8Slice99_new(au_header_section, sizeof au_header_section);
    U8Slice99 payload = U8Slice99_from_ptrdiff((uint8_t *)buf, (uint8_t *)(buf + len));

    pthread_mutex_lock(&g_srv.mtx);
    int sent = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        SmolRtspClient *c = &g_srv.clients[i];
        if (!c->alive || !c->audio_rtp || !c->playing)
            continue;
        if (SmolRTSP_RtpTransport_is_full(c->audio_rtp)) {
            // Prefer freshest: drop oldest queued interleaved frames, then send new.
            if (trim_tcp_interleaved_oldest(c->bev, RTSP_TCP_TRIM_TARGET) < 0) {
                fprintf(stderr,
                    "[rtsp] audio drop: buffer full (no trim) session=%llu len=%zu\n",
                    (unsigned long long)c->session_id, bev_output_len(c->bev));
                continue; // can't trim safely -> drop newest
            }
        }
        int ret = SmolRTSP_RtpTransport_send_packet(
            c->audio_rtp, ts, true, hdr, payload);
        if (ret < 0)
            continue;
        sent++;
    }
    pthread_mutex_unlock(&g_srv.mtx);

    return 0;
}
