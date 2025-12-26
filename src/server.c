#include "server.h"

#define MAX_CLIENTS 50
#define REQSIZE (32 * 1024)

IMPORT_STR(.rodata, "../res/index.html", indexhtml);
extern const char indexhtml[];
IMPORT_STR(.rodata, "../res/onvif/badauth.xml", badauthxml);
extern const char badauthxml[];

enum StreamType {
    STREAM_H26X,
    STREAM_JPEG,
    STREAM_MJPEG,
    STREAM_MP4,
    STREAM_PCM
};

typedef struct {
    int clntFd;
    char *input, *method, *payload, *prot, *query, *uri;
    int paysize, total;
} http_request_t;

struct {
    int sockFd;
    enum StreamType type;
    struct Mp4State mp4;
    unsigned int nalCnt;
} client_fds[MAX_CLIENTS];

typedef struct {
    char *name, *value;
} http_header_t;

typedef struct {
    int code;
    const char *msg, *desc;
} http_error_t;

const http_error_t http_errors[] = {
    {400, "Bad Request", "The server has no handler to the request."},
    {401, "Unauthorized", "You are not authorized to access this resource."},
    {403, "Forbidden", "You have been denied access to this resource."},
    {404, "Not Found", "The requested resource was not found."},
    {405, "Method Not Allowed", "This method is not handled on this endpoint."},
    {500, "Internal Server Error", "An invalid operation was caught on this request."},
    {501, "Not Implemented", "The server does not support the functionality."}
};
http_header_t http_headers[17] = {{"\0", "\0"}};

int server_fd = -1;
pthread_t server_thread_id;
pthread_mutex_t client_fds_mutex;

// Count active HTTP streaming clients by type (best-effort).
// Used to avoid blocking audio/video pipelines when nobody is subscribed.
volatile int server_pcm_clients = 0;
volatile int server_h26x_clients = 0;
volatile int server_mp4_clients = 0;
volatile int server_mjpeg_clients = 0;

static bool is_local_address(const char *client_ip) {
    if (!client_ip) return false;
    
    if (!strcmp(client_ip, "127.0.0.1") ||
        !strncmp(client_ip, "127.", 4))
        return true;
    
    if (!strcmp(client_ip, "::1"))
        return true;
    
    if (!strncmp(client_ip, "::ffff:127.", 11))
        return true;
    
    return false;
}

// Decode a configured GPIO pin value for logging. Mirrors night.c decoding:
// - 999: disabled
// - N (0..95): GPIO N
// - any negative value: treated as disabled
static bool decode_cfg_pin_for_log(int cfg, int *pin_out) {
    if (pin_out) *pin_out = 0;
    if (cfg == 999)
        return false;
    if (cfg < 0)
        return false;
    if (cfg > 95)
        return false;
    if (pin_out) *pin_out = cfg;
    return true;
}

static void close_socket_fd(int sockFd) {
    shutdown(sockFd, SHUT_RDWR);
    close(sockFd);
}

void free_client(int i) {
    if (client_fds[i].sockFd < 0) return;

    close_socket_fd(client_fds[i].sockFd);
    client_fds[i].sockFd = -1;
    if (client_fds[i].type == STREAM_PCM) {
        if (server_pcm_clients > 0) server_pcm_clients--;
    } else if (client_fds[i].type == STREAM_H26X) {
        if (server_h26x_clients > 0) server_h26x_clients--;
    } else if (client_fds[i].type == STREAM_MP4) {
        if (server_mp4_clients > 0) server_mp4_clients--;
    } else if (client_fds[i].type == STREAM_MJPEG) {
        if (server_mjpeg_clients > 0) server_mjpeg_clients--;
    }
    client_fds[i].type = -1;
}

int send_to_fd(int fd, char *buf, ssize_t size) {
    ssize_t sent = 0, len = 0;
    if (fd < 0) return -1;

    while (sent < size) {
        len = send(fd, buf + sent, size - sent, MSG_NOSIGNAL);
        if (len < 0) return -1;
        sent += len;
    }

    return EXIT_SUCCESS;
}

int send_to_fd_nonblock(int fd, char *buf, ssize_t size) {
    if (fd < 0) return -1;

    send(fd, buf, size, MSG_DONTWAIT | MSG_NOSIGNAL);

    return EXIT_SUCCESS;
}

int send_to_client(int i, char *buf, ssize_t size) {
    if (send_to_fd(client_fds[i].sockFd, buf, size) < 0) {
        free_client(i);
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}

static void send_and_close(int client_fd, char *buf, ssize_t size) {
    send_to_fd(client_fd, buf, size);
    close_socket_fd(client_fd);
}

void send_http_error(int fd, int code) {
    const char *desc = "\0", *msg = "Unspecified";
    char buffer[256];
    int len;
    
    for (int i = 0; i < sizeof(http_errors) / sizeof(*http_errors); i++) {
        if (http_errors[i].code == code) {
            desc = http_errors[i].desc;
            msg = http_errors[i].msg;
            break;
        }
    }
    
    len = snprintf(buffer, sizeof(buffer),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: text/plain\r\n"
        "Connection: close\r\n"
        "\r\n%s\r\n",
        code, msg, desc);
    
    send_and_close(fd, buffer, len);
}

void send_h26x_to_client(char index, hal_vidstream *stream) {
    if (server_h26x_clients <= 0)
        return;
    for (unsigned int i = 0; i < stream->count; ++i) {
        hal_vidpack *pack = &stream->pack[i];
        unsigned int pack_len = pack->length - pack->offset;
        unsigned char *pack_data = pack->data + pack->offset;

        pthread_mutex_lock(&client_fds_mutex);
        for (unsigned int i = 0; i < MAX_CLIENTS; ++i) {
            if (client_fds[i].sockFd < 0) continue;
            if (client_fds[i].type != STREAM_H26X) continue;

            for (char j = 0; j < pack->naluCnt; j++) {
                if (client_fds[i].nalCnt == 0 &&
                    pack->nalu[j].type != NalUnitType_SPS &&
                    pack->nalu[j].type != NalUnitType_SPS_HEVC)
                    continue;

#ifdef DEBUG_VIDEO
                printf("NAL: %s send to %d\n", nal_type_to_str(pack->nalu[j].type), i);
#endif

                static char len_buf[50];
                ssize_t len_size = sprintf(len_buf, "%zX\r\n", pack->nalu[j].length);
                if (send_to_client(i, len_buf, len_size) < 0)
                    continue; // send <SIZE>\r\n
                if (send_to_client(i, pack_data + pack->nalu[j].offset, pack->nalu[j].length) < 0)
                    continue; // send <DATA>
                if (send_to_client(i, "\r\n", 2) < 0)
                    continue; // send \r\n

                client_fds[i].nalCnt++;
                if (client_fds[i].nalCnt == 300) {
                    char end[] = "0\r\n\r\n";
                    if (send_to_client(i, end, sizeof(end)) < 0)
                        continue;
                    free_client(i);
                }
            }
        }
        pthread_mutex_unlock(&client_fds_mutex);
    }
}

void send_mp4_to_client(char index, hal_vidstream *stream, char isH265) {
    if (server_mp4_clients <= 0)
        return;

    for (unsigned int i = 0; i < stream->count; ++i) {
        hal_vidpack *pack = &stream->pack[i];
        unsigned int pack_len = pack->length - pack->offset;
        unsigned char *pack_data = pack->data + pack->offset;

        for (char j = 0; j < pack->naluCnt; j++) {
#ifdef DEBUG_VIDEO
            printf("NAL: %s received in packet %d\n", nal_type_to_str(pack->nalu[j].type), i);
            printf("     starts at %p, ends at %p\n", pack_data + pack->nalu[j].offset, pack_data + pack->nalu[j].offset + pack->nalu[j].length);
#endif
            if ((pack->nalu[j].type == NalUnitType_SPS || pack->nalu[j].type == NalUnitType_SPS_HEVC) 
                && pack->nalu[j].length >= 4 && pack->nalu[j].length <= UINT16_MAX)
                mp4_set_sps(pack_data + pack->nalu[j].offset + 4, pack->nalu[j].length - 4, isH265);
            else if ((pack->nalu[j].type == NalUnitType_PPS || pack->nalu[j].type == NalUnitType_PPS_HEVC)
                && pack->nalu[j].length <= UINT16_MAX)
                mp4_set_pps(pack_data + pack->nalu[j].offset + 4, pack->nalu[j].length - 4, isH265);
            else if (pack->nalu[j].type == NalUnitType_VPS_HEVC && pack->nalu[j].length <= UINT16_MAX)
                mp4_set_vps(pack_data + pack->nalu[j].offset + 4, pack->nalu[j].length - 4);
            else if (pack->nalu[j].type == NalUnitType_CodedSliceIdr || pack->nalu[j].type == NalUnitType_CodedSliceAux)
                mp4_set_slice(pack_data + pack->nalu[j].offset + 4, pack->nalu[j].length - 4, 1);
            else if (pack->nalu[j].type == NalUnitType_CodedSliceNonIdr)
                mp4_set_slice(pack_data + pack->nalu[j].offset + 4, pack->nalu[j].length - 4, 0);
        }

        static enum BufError err;
        static char len_buf[50];
        pthread_mutex_lock(&client_fds_mutex);
        for (unsigned int i = 0; i < MAX_CLIENTS; ++i) {
            if (client_fds[i].sockFd < 0) continue;
            if (client_fds[i].type != STREAM_MP4) continue;

            if (!client_fds[i].mp4.header_sent) {
                struct BitBuf header_buf;
                err = mp4_get_header(&header_buf);
                chk_err_continue ssize_t len_size =
                    sprintf(len_buf, "%zX\r\n", header_buf.offset);
                if (send_to_client(i, len_buf, len_size) < 0)
                    continue; // send <SIZE>\r\n
                if (send_to_client(i, header_buf.buf, header_buf.offset) < 0)
                    continue; // send <DATA>
                if (send_to_client(i, "\r\n", 2) < 0)
                    continue; // send \r\n

                client_fds[i].mp4.sequence_number = 0;
                client_fds[i].mp4.base_data_offset = header_buf.offset;
                client_fds[i].mp4.base_media_decode_time = 0;
                client_fds[i].mp4.header_sent = true;
                client_fds[i].mp4.nals_count = 0;
                client_fds[i].mp4.default_sample_duration =
                    default_sample_size;
            }

            err = mp4_set_state(&client_fds[i].mp4);
            chk_err_continue {
                struct BitBuf moof_buf;
                err = mp4_get_moof(&moof_buf);
                chk_err_continue ssize_t len_size =
                    sprintf(len_buf, "%zX\r\n", (ssize_t)moof_buf.offset);
                if (send_to_client(i, len_buf, len_size) < 0)
                    continue; // send <SIZE>\r\n
                if (send_to_client(i, moof_buf.buf, moof_buf.offset) < 0)
                    continue; // send <DATA>
                if (send_to_client(i, "\r\n", 2) < 0)
                    continue; // send \r\n
            }
            {
                struct BitBuf mdat_buf;
                err = mp4_get_mdat(&mdat_buf);
                chk_err_continue ssize_t len_size =
                    sprintf(len_buf, "%zX\r\n", (ssize_t)mdat_buf.offset);
                if (send_to_client(i, len_buf, len_size) < 0)
                    continue; // send <SIZE>\r\n
                if (send_to_client(i, mdat_buf.buf, mdat_buf.offset) < 0)
                    continue; // send <DATA>
                if (send_to_client(i, "\r\n", 2) < 0)
                    continue; // send \r\n
            }
        }
        pthread_mutex_unlock(&client_fds_mutex);
    }
}

void send_pcm_to_client(hal_audframe *frame) {
    if (server_pcm_clients <= 0)
        return;
    pthread_mutex_lock(&client_fds_mutex);
    for (unsigned int i = 0; i < MAX_CLIENTS; ++i) {
        if (client_fds[i].sockFd < 0) continue;
        if (client_fds[i].type != STREAM_PCM) continue;

        static char len_buf[50];
        ssize_t len_size = sprintf(len_buf, "%zX\r\n", frame->length[0]);
        if (send_to_client(i, len_buf, len_size) < 0)
            continue; // send <SIZE>\r\n
        if (send_to_client(i, frame->data[0], frame->length[0]) < 0)
            continue; // send <DATA>
        if (send_to_client(i, "\r\n", 2) < 0)
            continue; // send \r\n
    }
    pthread_mutex_unlock(&client_fds_mutex);
}

void send_mjpeg_to_client(char index, char *buf, ssize_t size) {
    if (server_mjpeg_clients <= 0)
        return;
    static char prefix_buf[128];
    ssize_t prefix_size = sprintf(prefix_buf,
        "--boundarydonotcross\r\n"
        "Content-Type:image/jpeg\r\n"
        "Content-Length: %lu\r\n\r\n", size);
    buf[size++] = '\r';
    buf[size++] = '\n';

    pthread_mutex_lock(&client_fds_mutex);
    for (unsigned int i = 0; i < MAX_CLIENTS; ++i) {
        if (client_fds[i].sockFd < 0) continue;
        if (client_fds[i].type != STREAM_MJPEG) continue;

        if (send_to_client(i, prefix_buf, prefix_size) < 0)
            continue; // send <SIZE>\r\n
        if (send_to_client(i, buf, size) < 0)
            continue; // send <DATA>\r\n
    }
    pthread_mutex_unlock(&client_fds_mutex);
}

void send_jpeg_to_client(char index, char *buf, ssize_t size) {
    static char prefix_buf[128];
    ssize_t prefix_size = sprintf(
        prefix_buf,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %lu\r\n"
        "Connection: close\r\n\r\n", size);
    buf[size++] = '\r';
    buf[size++] = '\n';

    pthread_mutex_lock(&client_fds_mutex);
    for (unsigned int i = 0; i < MAX_CLIENTS; ++i) {
        if (client_fds[i].sockFd < 0) continue;
        if (client_fds[i].type != STREAM_JPEG) continue;

        if (send_to_client(i, prefix_buf, prefix_size) < 0)
            continue; // send <SIZE>\r\n
        if (send_to_client(i, buf, size) < 0)
            continue; // send <DATA>\r\n
        free_client(i);
    }
    pthread_mutex_unlock(&client_fds_mutex);
}

struct jpegtask {
    int client_fd;
    uint16_t width;
    uint16_t height;
    uint8_t qfactor;
    uint8_t color2Gray;
};

void *send_jpeg_thread(void *vargp) {
    struct jpegtask task = *((struct jpegtask *)vargp);
    hal_jpegdata jpeg = {0};
    HAL_INFO("server", "Requesting a JPEG snapshot (%ux%u, qfactor %u, color2Gray %d)...\n",
        task.width, task.height, task.qfactor, task.color2Gray);
    int ret =
        jpeg_get(task.width, task.height, task.qfactor, task.color2Gray, &jpeg);
    if (ret) {
        HAL_DANGER("server", "Failed to receive a JPEG snapshot...\n");
        static char response[] =
            "HTTP/1.1 503 Internal Error\r\n"
            "Connection: close\r\n\r\n";
        send_and_close(task.client_fd, response, sizeof(response) - 1); // zero ending string!
        return NULL;
    }
    HAL_INFO("server", "JPEG snapshot has been received!\n");
    char buf[1024];
    int buf_len = sprintf(
        buf, "HTTP/1.1 200 OK\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %lu\r\n"
        "Connection: close\r\n\r\n",
        jpeg.jpegSize);
    send_to_fd(task.client_fd, buf, buf_len);
    send_to_fd(task.client_fd, jpeg.data, jpeg.jpegSize);
    send_to_fd(task.client_fd, "\r\n", 2);
    close_socket_fd(task.client_fd);
    free(jpeg.data);
    HAL_INFO("server", "JPEG snapshot has been sent!\n");
    return NULL;
}

int send_file(const int client_fd, const char *path) {
    if (!access(path, F_OK)) {
        const char *mime = (path);
        FILE *file = fopen(path, "r");
        if (file == NULL) {
            close_socket_fd(client_fd);
            return EXIT_SUCCESS;
        }
        char header[1024];
        int header_len = sprintf(header,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: %s\r\n"
            "Transfer-Encoding: chunked\r\n"
            "Connection: keep-alive\r\n\r\n", mime);
        send_to_fd(client_fd, header, header_len); // zero ending string!
        const int buf_size = 1024;
        char buf[buf_size + 2];
        char len_buf[50];
        ssize_t len_size;
        while (1) {
            ssize_t size = fread(buf, sizeof(char), buf_size, file);
            if (size <= 0) break;
            len_size = sprintf(len_buf, "%zX\r\n", size);
            buf[size++] = '\r';
            buf[size++] = '\n';
            send_to_fd(client_fd, len_buf, len_size); // send <SIZE>\r\n
            send_to_fd(client_fd, buf, size);         // send <DATA>\r\n
        }
        char end[] = "0\r\n\r\n";
        send_to_fd(client_fd, end, sizeof(end));
        fclose(file);
        close_socket_fd(client_fd);
        return EXIT_FAILURE;
    }

    send_http_error(client_fd, 404);
    return EXIT_FAILURE;
}

void send_binary(const int fd, const char *data, const long size) {
    char *buf;
    int buf_len = asprintf(&buf,
        "HTTP/1.1 200 OK\r\n" \
        "Content-Type: application/octet-stream\r\n" \
        "Content-Length: %zu\r\n" \
        "Connection: close\r\n\r\n", size);
    send_to_fd(fd, buf, buf_len);
    send_to_fd(fd, (char*)data, size);
    send_to_fd(fd, "\r\n", 2);
    close_socket_fd(fd);
    free(buf);
}

void send_html(const int fd, const char *data) {
    char *buf;
    int buf_len = asprintf(&buf,
        "HTTP/1.1 200 OK\r\n" \
        "Content-Type: text/html\r\n" \
        "Content-Length: %zu\r\n" \
        "Connection: close\r\n" \
        "\r\n%s", strlen(data), data);
    buf[buf_len++] = 0;
    send_and_close(fd, buf, buf_len);
    free(buf);
}

char *request_header(const char *name) {
    http_header_t *h = http_headers;
    for (; h->name; h++)
        if (!strcasecmp(h->name, name))
            return h->value;
    return NULL;
}

http_header_t *request_headers(void) { return http_headers; }

void parse_request(http_request_t *req) {
    struct sockaddr_in client_sock;
    socklen_t client_sock_len = sizeof(client_sock);
    memset(&client_sock, 0, client_sock_len);

    getpeername(req->clntFd,
        (struct sockaddr *)&client_sock, &client_sock_len);
    char *client_ip = inet_ntoa(client_sock.sin_addr);

    req->total = 0;
    // Clear header pointers from previous requests (they point into req->input).
    memset(http_headers, 0, sizeof(http_headers));

    // recv() does NOT NUL-terminate. Keep 1 byte for '\0' because we parse with strtok/strlen/strchr.
    int received = recv(req->clntFd, req->input, REQSIZE - 1, 0);
    if (received < 0)
        HAL_WARNING("server", "Reading from client failed!\n");
    else if (!received)
        HAL_WARNING("server", "Client disconnected unexpectedly!\n");
    req->total += received;

    if (req->total <= 0) return;
    req->input[req->total] = '\0';

    char *state = NULL;
    req->method = strtok_r(req->input, " \t\r\n", &state);
    req->uri = strtok_r(NULL, " \t", &state);
    req->prot = strtok_r(NULL, " \t\r\n", &state);

    if (!req->method || !req->uri || !req->prot) {
        HAL_WARNING("server", "Malformed request line, closing.\n");
        close_socket_fd(req->clntFd);
        req->clntFd = -1;
        req->total = 0;
        return;
    }

    // Apply whitelist AFTER reading the request. Closing a TCP socket without
    // reading can trigger an RST on some stacks, which looks like
    // "Connection reset by peer" to clients (curl).
    if (!EMPTY(*app_config.web_whitelist)) {
        bool allowed = false;
        bool any_valid = false;

        // web_whitelist is a fixed array; entries may contain whitespace.
        // Treat whitespace-only entries as empty and do not enforce the whitelist
        // unless at least one valid entry is present.
        for (int i = 0; i < (int)(sizeof(app_config.web_whitelist) / sizeof(app_config.web_whitelist[0])); i++) {
            const char *cidr = app_config.web_whitelist[i];
            if (!cidr) continue;
            while (*cidr == ' ' || *cidr == '\t') cidr++;
            if (*cidr == '\0') continue;

            any_valid = true;
            if (ip_in_cidr(client_ip, (char *)cidr)) {
                allowed = true;
                break;
            }
        }

        if (any_valid && !allowed) {
            send_http_error(req->clntFd, 403);
            req->clntFd = -1;
            req->total = 0;
            return;
        }
    }

    HAL_INFO("server", "\x1b[32mNew request: (%s) %s\n"
        "         Received from: %s\x1b[0m\n",
        req->method, req->uri, client_ip);

    if (req->query = strchr(req->uri, '?'))
        *req->query++ = '\0';
    else
        // No query string: point to the terminating '\0' so EMPTY(req->query) is safe.
        req->query = req->uri + strlen(req->uri);

    http_header_t *h = http_headers;
    char *l;
    while (h < http_headers + 16) {
        char *k, *v, *e;
        if (!(k = strtok_r(NULL, "\r\n: \t", &state)))
            break;
        v = strtok_r(NULL, "\r\n", &state);
        if (!v)
            break;
        while (*v && *v == ' ' && v++);
        h->name = k;
        h++->value = v;
#ifdef DEBUG_HTTP
        fprintf(stderr, "         (H) %s: %s\n", k, v);
#endif
        e = v + 1 + strlen(v);
        if (e[1] == '\r' && e[2] == '\n')
            break;
    }

    l = request_header("Content-Length");
    req->paysize = l ? atol(l) : 0;

    while (l && req->total < req->paysize) {
        if (req->total >= REQSIZE - 1)
            break;
        received = recv(req->clntFd, req->input + req->total, (REQSIZE - 1) - req->total, 0);
        if (received < 0) {
            HAL_WARNING("server", "Reading from client failed!\n");
            break;
        } else if (!received) {
            HAL_WARNING("server", "Client disconnected unexpectedly!\n");
            break;
        }
        req->total += received;
        req->input[req->total] = '\0';
    }

    req->payload = strtok_r(NULL, "\r\n", &state);
}

void respond_request(http_request_t *req) {
    char response[8192] = {0};
    int respLen = 0;

    if (req->clntFd < 0) return;

    if (!EQUALS(req->method, "GET") && !EQUALS(req->method, "POST")) {
        send_http_error(req->clntFd, 405);
        return;
    }

    if (app_config.onvif_enable && STARTS_WITH(req->uri, "/onvif")) {
        char *path = req->uri + 6;
        if (*path == '/') path++;

        if (!EQUALS(req->method, "POST")) {
            send_http_error(req->clntFd, 405);
            return;
        }

        char *action = onvif_extract_soap_action(req->payload);
        HAL_INFO("onvif", "\x1b[32mAction: %s\x1b[0m\n", action);
        respLen = sizeof(response);

        if (app_config.onvif_enable_auth && !onvif_validate_soap_auth(req->payload)) {
            respLen = sprintf(response,
                "HTTP/1.1 401 Unauthorized\r\n"
                "Content-Type: text/plain\r\n"
                "WWW-Authenticate: Digest realm=\"Access the camera services\"\r\n"
                "Connection: close\r\n\r\n%s",
                badauthxml);
            send_and_close(req->clntFd, response, respLen);
            return;
        }

        if (EQUALS(path, "device_service")) {
            if (EQUALS(action, "GetCapabilities")) {
                onvif_respond_capabilities((char*)response, &respLen);
                send_and_close(req->clntFd, response, respLen);
                return;
            } else if (EQUALS(action, "GetDeviceInformation")) {
                onvif_respond_deviceinfo((char*)response, &respLen);
                send_and_close(req->clntFd, response, respLen);
                return;
            } else if (EQUALS(action, "GetSystemDateAndTime")) {
                onvif_respond_systemtime((char*)response, &respLen);
                send_and_close(req->clntFd, response, respLen);
                return;
            }
        } else if (EQUALS(path, "media_service")) {
            if (EQUALS(action, "GetProfiles")) {
                onvif_respond_mediaprofiles((char*)response, &respLen);
                send_and_close(req->clntFd, response, respLen);
                return;
            } else if (EQUALS(action, "GetSnapshotUri")) {
                onvif_respond_snapshot((char*)response, &respLen);
                send_and_close(req->clntFd, response, respLen);
                return;
            } else if (EQUALS(action, "GetStreamUri")) {
                onvif_respond_stream((char*)response, &respLen);
                send_and_close(req->clntFd, response, respLen);
                return;
            } else if (EQUALS(action, "GetVideoSources")) {
                onvif_respond_videosources((char*)response, &respLen);
                send_and_close(req->clntFd, response, respLen);
                return;
            }
        }

        if (!EMPTY(action))
            HAL_WARNING("server", "Unknown ONVIF request: %s->%s\n", path, action);
        send_http_error(req->clntFd, 501);
        return;
    }

    if (app_config.web_enable_auth) {
        bool should_skip_auth = false;

        if (app_config.web_auth_skiplocal) {
            struct sockaddr_in client_sock;
            socklen_t client_sock_len = sizeof(client_sock);
            memset(&client_sock, 0, client_sock_len);

            if (getpeername(req->clntFd, (struct sockaddr *)&client_sock, &client_sock_len) == 0) {
                char *client_ip = inet_ntoa(client_sock.sin_addr);
                should_skip_auth = is_local_address(client_ip);
            }
        }

        if (!should_skip_auth) {
            char *auth = request_header("Authorization");
            char cred[66], valid[256];

            strcpy(cred, app_config.web_auth_user);
            strcpy(cred + strlen(app_config.web_auth_user), ":");
            strcpy(cred + strlen(app_config.web_auth_user) + 1, app_config.web_auth_pass);
            strcpy(valid, "Basic ");
            base64_encode(valid + 6, cred, strlen(cred));

            if (!auth || !EQUALS(auth, valid)) {
                respLen = sprintf(response,
                    "HTTP/1.1 401 Unauthorized\r\n"
                    "Content-Type: text/plain\r\n"
                    "WWW-Authenticate: Basic realm=\"Access the camera services\"\r\n"
                    "Connection: close\r\n\r\n");
                send_and_close(req->clntFd, response, respLen);
                return;
            }
        }
    }

    if (EQUALS(req->uri, "/exit")) {
        respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Connection: close\r\n\r\n"
            "Closing...");
        send_and_close(req->clntFd, response, respLen);
        keepRunning = 0;
        graceful = 1;
        return;
    }

    if (EQUALS(req->uri, "/") || EQUALS(req->uri, "/index.htm") || EQUALS(req->uri, "/index.html")) {
        send_html(req->clntFd, indexhtml);
        return;
    }

    if (app_config.audio_enable && EQUALS(req->uri, "/audio.pcm")) {
        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: audio/pcm\r\n"
            "Transfer-Encoding: chunked\r\n"
            "Connection: keep-alive\r\n\r\n");
        send_to_fd(req->clntFd, response, respLen);
        pthread_mutex_lock(&client_fds_mutex);
        for (uint32_t i = 0; i < MAX_CLIENTS; ++i)
            if (client_fds[i].sockFd < 0) {
                client_fds[i].sockFd = req->clntFd;
                client_fds[i].type = STREAM_PCM;
                server_pcm_clients++;
                break;
            }
        pthread_mutex_unlock(&client_fds_mutex);
        return;
    }

    if ((!app_config.mp4_codecH265 && EQUALS(req->uri, "/video.264")) ||
        (app_config.mp4_codecH265 && EQUALS(req->uri, "/video.265"))) {
        request_idr();
        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/octet-stream\r\n"
            "Transfer-Encoding: chunked\r\n"
            "Connection: keep-alive\r\n\r\n");
        send_to_fd(req->clntFd, response, respLen);
        pthread_mutex_lock(&client_fds_mutex);
        for (uint32_t i = 0; i < MAX_CLIENTS; ++i)
            if (client_fds[i].sockFd < 0) {
                client_fds[i].sockFd = req->clntFd;
                client_fds[i].type = STREAM_H26X;
                client_fds[i].nalCnt = 0;
                server_h26x_clients++;
                break;
            }
        pthread_mutex_unlock(&client_fds_mutex);
        return;
    }

    if (app_config.mp4_enable && EQUALS(req->uri, "/video.mp4")) {
        request_idr();
        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: video/mp4\r\n"
            "Transfer-Encoding: chunked\r\n"
            "Connection: keep-alive\r\n\r\n");
        send_to_fd(req->clntFd, response, respLen);
        pthread_mutex_lock(&client_fds_mutex);
        for (uint32_t i = 0; i < MAX_CLIENTS; ++i)
            if (client_fds[i].sockFd < 0) {
                client_fds[i].sockFd = req->clntFd;
                client_fds[i].type = STREAM_MP4;
                client_fds[i].mp4.header_sent = false;
                server_mp4_clients++;
                break;
            }
        pthread_mutex_unlock(&client_fds_mutex);
        return;
    }

    if (app_config.jpeg_enable && EQUALS(req->uri, "/mjpeg")) {
        int respLen = sprintf(response,
            "HTTP/1.0 200 OK\r\n"
            "Cache-Control: no-cache\r\n"
            "Pragma: no-cache\r\n"
            "Connection: close\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=boundarydonotcross\r\n\r\n");
        send_to_fd(req->clntFd, response, respLen);
        pthread_mutex_lock(&client_fds_mutex);
        for (uint32_t i = 0; i < MAX_CLIENTS; ++i)
            if (client_fds[i].sockFd < 0) {
                client_fds[i].sockFd = req->clntFd;
                client_fds[i].type = STREAM_MJPEG;
                server_mjpeg_clients++;
                break;
            }
        pthread_mutex_unlock(&client_fds_mutex);
        return;
    }

    if (app_config.jpeg_enable && STARTS_WITH(req->uri, "/image.jpg")) {
        {
            struct jpegtask task;
            task.client_fd = req->clntFd;
            task.width = app_config.jpeg_width;
            task.height = app_config.jpeg_height;
            task.qfactor = app_config.jpeg_qfactor;
            task.color2Gray = 0;

            if (!EMPTY(req->query)) {
                char *remain;
                while (req->query) {
                    char *value = split(&req->query, "&");
                    if (!value || !*value) continue;
                    char *key = split(&value, "=");
                    if (!key || !*key || !value || !*value) continue;
                    if (EQUALS(key, "width")) {
                        short result = strtol(value, &remain, 10);
                        if (remain != value)
                            task.width = result;
                    }
                    else if (EQUALS(key, "height")) {
                        short result = strtol(value, &remain, 10);
                        if (remain != value)
                            task.height = result;
                    }
                    else if (EQUALS(key, "qfactor")) {
                        short result = strtol(value, &remain, 10);
                        if (remain != value)
                            task.qfactor = result;
                    }
                    else if (EQUALS(key, "color2gray") || EQUALS(key, "gray")) {
                        if (EQUALS_CASE(value, "true") || EQUALS(value, "1"))
                            task.color2Gray = 1;
                        else if (EQUALS_CASE(value, "false") || EQUALS(value, "0"))
                            task.color2Gray = 0;
                    }
                }
            }

            pthread_t thread_id;
            pthread_attr_t thread_attr;
            pthread_attr_init(&thread_attr);
            size_t stacksize;
            pthread_attr_getstacksize(&thread_attr, &stacksize);
            size_t new_stacksize = 16 * 1024;
            if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
                HAL_DANGER("jpeg", "Can't set stack size %zu\n", new_stacksize);
            pthread_create(
                &thread_id, &thread_attr, send_jpeg_thread, (void *)&task);
            if (pthread_attr_setstacksize(&thread_attr, stacksize))
                HAL_DANGER("jpeg", "Can't set stack size %zu\n", stacksize);
            pthread_attr_destroy(&thread_attr);
        }
        return;
    }

    if (EQUALS(req->uri, "/api/audio")) {
        const int prev_bitrate = (int)app_config.audio_bitrate;
        const int prev_gain = (int)app_config.audio_gain;
        const int prev_srate = (int)app_config.audio_srate;
        const int prev_mute = media_get_audio_mute();
        int mute_req = -1; // -1 = unchanged, 0 = unmute, 1 = mute

        if (!EMPTY(req->query)) {
            char *remain;
            while (req->query) {
                char *value = split(&req->query, "&");
                if (!value || !*value) continue;
                unescape_uri(value);
                char *key = split(&value, "=");
                if (!key || !*key || !value || !*value) continue;
                if (EQUALS(key, "bitrate")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value) {
                        if (result < 8) result = 8;
                        if (result > 320) result = 320;
                        app_config.audio_bitrate = result;
                    }
                } else if (EQUALS(key, "enable")) {
                    // Backwards compatible: "enable=false" means "mute" (keep RTSP audio track alive).
                    if (EQUALS_CASE(value, "true") || EQUALS(value, "1")) {
                        app_config.audio_enable = 1;
                        mute_req = 0;
                    } else if (EQUALS_CASE(value, "false") || EQUALS(value, "0")) {
                        app_config.audio_enable = 1; // do NOT disable audio pipeline; keep it alive
                        mute_req = 1;
                    }
                } else if (EQUALS(key, "mute")) {
                    if (EQUALS_CASE(value, "true") || EQUALS(value, "1"))
                        mute_req = 1;
                    else if (EQUALS_CASE(value, "false") || EQUALS(value, "0"))
                        mute_req = 0;
                } else if (EQUALS(key, "gain")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.audio_gain = result;
                } else if (EQUALS(key, "srate")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.audio_srate = result;
                }
            }
        }

        // Muting requires the audio pipeline to be active (we generate silence by zeroing PCM).
        if (mute_req == 1)
            app_config.audio_enable = 1;

        // Ensure audio pipeline is running when requested (mute requires encoder to keep producing frames).
        if (app_config.audio_enable && !audioOn) {
            enable_audio();
        }

        // Apply runtime mute toggle if provided and persist immediately.
        if (mute_req != -1) {
            app_config.audio_mute = (mute_req == 1);
            int sr = save_app_config();
            if (sr != 0)
                HAL_WARNING("server", "Failed to save config after audio mute change (ret=%d)\n", sr);
            media_set_audio_mute(mute_req);
        }

        // Apply bitrate change without restarting audio (best-effort).
        if ((int)app_config.audio_bitrate != prev_bitrate) {
            media_set_audio_bitrate_kbps(app_config.audio_bitrate);
        }

        // Some changes still require a full audio restart (HAL + encoder).
        // NOTE: this can cause a short gap in RTP audio.
        if ((int)app_config.audio_srate != prev_srate || (int)app_config.audio_gain != prev_gain) {
            const int want_mute = (mute_req != -1) ? mute_req : prev_mute;
            disable_audio();
            if (app_config.audio_enable) {
                enable_audio();
                if (want_mute)
                    media_set_audio_mute(1);
            }
        }

        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json;charset=UTF-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{\"enable\":%s,\"mute\":%s,\"bitrate\":%d,\"gain\":%d,\"srate\":%d}",
            app_config.audio_enable ? "true" : "false",
            media_get_audio_mute() ? "true" : "false",
            app_config.audio_bitrate, app_config.audio_gain, app_config.audio_srate);
        send_and_close(req->clntFd, response, respLen);
        return;
    }

    if (EQUALS(req->uri, "/api/cmd")) {
        int result = -1;
        if (!EMPTY(req->query)) {
            char *remain;
            while (req->query) {
                char *value = split(&req->query, "&");
                if (!value || !*value) continue;
                unescape_uri(value);
                char *key = split(&value, "=");
                if (!key || !*key) continue;
                if (EQUALS(key, "save")) {
                    result = save_app_config();
                    if (!result)
                        HAL_INFO("server", "Configuration saved!\n");
                    else
                        HAL_WARNING("server", "Failed to save configuration!\n");
                    break;
                }
            }
        }

        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json;charset=UTF-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{\"code\":%d}", result);
        send_and_close(req->clntFd, response, respLen);
        return;
    }

    // NOTE: /api/jpeg configures the MJPEG stream (not a separate snapshot encoder).
    // /api/mjpeg is kept as a compatibility alias.
    if (EQUALS(req->uri, "/api/jpeg") || EQUALS(req->uri, "/api/mjpeg")) {
        if (!EMPTY(req->query)) {
            char *remain;
            bool osd_jpeg_changed = false;
            while (req->query) {
                char *value = split(&req->query, "&");
                if (!value || !*value) continue;
                unescape_uri(value);
                char *key = split(&value, "=");
                if (!key || !*key || !value || !*value) continue;
                if (EQUALS(key, "enable")) {
                    if (EQUALS_CASE(value, "true") || EQUALS(value, "1"))
                        app_config.jpeg_enable = 1;
                    else if (EQUALS_CASE(value, "false") || EQUALS(value, "0"))
                        app_config.jpeg_enable = 0;
                } else if (EQUALS(key, "osd_enable")) {
                    bool prev = app_config.jpeg_osd_enable;
                    if (EQUALS_CASE(value, "true") || EQUALS(value, "1"))
                        app_config.jpeg_osd_enable = 1;
                    else if (EQUALS_CASE(value, "false") || EQUALS(value, "0"))
                        app_config.jpeg_osd_enable = 0;
                    if (prev != app_config.jpeg_osd_enable)
                        osd_jpeg_changed = true;
                } else if (EQUALS(key, "width")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.jpeg_width = result;
                } else if (EQUALS(key, "height")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.jpeg_height = result;
                } else if (EQUALS(key, "fps")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.jpeg_fps = result;
                } else if (EQUALS(key, "qfactor")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value) {
                        if (result < 1) result = 1;
                        if (result > 99) result = 99;
                        app_config.jpeg_qfactor = (unsigned int)result;
                    }
                } else if (EQUALS(key, "mode")) {
                    if (EQUALS_CASE(value, "CBR"))
                        app_config.jpeg_mode = HAL_VIDMODE_CBR;
                    else if (EQUALS_CASE(value, "VBR"))
                        app_config.jpeg_mode = HAL_VIDMODE_VBR;
                    else if (EQUALS_CASE(value, "QP"))
                        app_config.jpeg_mode = HAL_VIDMODE_QP;
                } else if (EQUALS(key, "bitrate")) {
                    // Legacy parameter: MJPEG bitrate is no longer configurable/used.
                    // Intentionally ignored for backwards compatibility.
                }
            }

            // MJPEG is quality (qfactor) driven now; force QP mode.
            app_config.jpeg_mode = HAL_VIDMODE_QP;

            disable_mjpeg();
            if (app_config.jpeg_enable) enable_mjpeg();

            // (Re)enable snapshot module state.
            jpeg_deinit();
            if (app_config.jpeg_enable) jpeg_init();

            if (osd_jpeg_changed && app_config.osd_enable) {
                for (char i = 0; i < MAX_OSD; i++)
                    osds[i].updt = 1;
            }
        }

        // Runtime forces QP for MJPEG.
        char mode[5] = "QP";
        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json;charset=UTF-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{\"enable\":%s,\"osd_enable\":%s,\"width\":%d,\"height\":%d,\"fps\":%d,\"mode\":\"%s\",\"qfactor\":%d}",
            app_config.jpeg_enable ? "true" : "false",
            app_config.jpeg_osd_enable ? "true" : "false",
            app_config.jpeg_width, app_config.jpeg_height, app_config.jpeg_fps, mode,
            app_config.jpeg_qfactor);
        send_and_close(req->clntFd, response, respLen);
        return;
    }

    if (EQUALS(req->uri, "/api/mp4")) {
        if (!EMPTY(req->query)) {
            char *remain;
            while (req->query) {
                char *value = split(&req->query, "&");
                if (!value || !*value) continue;
                unescape_uri(value);
                char *key = split(&value, "=");
                if (!key || !*key || !value || !*value) continue;
                if (EQUALS(key, "enable")) {
                    if (EQUALS_CASE(value, "true") || EQUALS(value, "1"))
                        app_config.mp4_enable = 1;
                    else if (EQUALS_CASE(value, "false") || EQUALS(value, "0"))
                        app_config.mp4_enable = 0;
                } else if (EQUALS(key, "width")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.mp4_width = result;
                } else if (EQUALS(key, "height")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.mp4_height = result;
                } else if (EQUALS(key, "fps")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.mp4_fps = result;
                } else if (EQUALS(key, "bitrate")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.mp4_bitrate = result;
                } else if (EQUALS(key, "h265")) {
                    if (EQUALS_CASE(value, "true") || EQUALS(value, "1"))
                        app_config.mp4_codecH265 = 1;
                    else if (EQUALS_CASE(value, "false") || EQUALS(value, "0"))
                        app_config.mp4_codecH265 = 0;
                } else if (EQUALS(key, "mode")) {
                    if (EQUALS_CASE(value, "CBR"))
                        app_config.mp4_mode = HAL_VIDMODE_CBR;
                    else if (EQUALS_CASE(value, "VBR"))
                        app_config.mp4_mode = HAL_VIDMODE_VBR;
                    else if (EQUALS_CASE(value, "QP"))
                        app_config.mp4_mode = HAL_VIDMODE_QP;
                    else if (EQUALS_CASE(value, "ABR"))
                        app_config.mp4_mode = HAL_VIDMODE_ABR;
                    else if (EQUALS_CASE(value, "AVBR"))
                        app_config.mp4_mode = HAL_VIDMODE_AVBR;
                } else if (EQUALS(key, "profile")) {
                    if (EQUALS_CASE(value, "BP") || EQUALS_CASE(value, "BASELINE"))
                        app_config.mp4_profile = HAL_VIDPROFILE_BASELINE;
                    else if (EQUALS_CASE(value, "MP") || EQUALS_CASE(value, "MAIN"))
                        app_config.mp4_profile = HAL_VIDPROFILE_MAIN;
                    else if (EQUALS_CASE(value, "HP") || EQUALS_CASE(value, "HIGH"))
                        app_config.mp4_profile = HAL_VIDPROFILE_HIGH;
                }
            }

            disable_mp4();
            if (app_config.mp4_enable) enable_mp4();
        }

        char h265[6] = "false";
        char mode[5] = "\0";
        char profile[3] = "\0";
        if (app_config.mp4_codecH265)
            strcpy(h265, "true");
        switch (app_config.mp4_mode) {
            case HAL_VIDMODE_CBR: strcpy(mode, "CBR"); break;
            case HAL_VIDMODE_VBR: strcpy(mode, "VBR"); break;
            case HAL_VIDMODE_QP: strcpy(mode, "QP"); break;
            case HAL_VIDMODE_ABR: strcpy(mode, "ABR"); break;
            case HAL_VIDMODE_AVBR: strcpy(mode, "AVBR"); break;
        }
        switch (app_config.mp4_profile) {
            case HAL_VIDPROFILE_BASELINE: strcpy(profile, "BP"); break;
            case HAL_VIDPROFILE_MAIN: strcpy(profile, "MP"); break;
            case HAL_VIDPROFILE_HIGH: strcpy(profile, "HP"); break;
        }
        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json;charset=UTF-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{\"enable\":%s,\"width\":%d,\"height\":%d,\"fps\":%d,"
            "\"h265\":%s,\"mode\":\"%s\",\"profile\":\"%s\",\"bitrate\":%d}",
            app_config.mp4_enable ? "true" : "false",
            app_config.mp4_width, app_config.mp4_height, app_config.mp4_fps, h265, mode,
            profile, app_config.mp4_bitrate);
        send_and_close(req->clntFd, response, respLen);
        return;
    }

    if (EQUALS(req->uri, "/api/night")) {
        if (!EMPTY(req->query)) {
            // Track prior state to decide whether a night thread restart is needed.
            const bool old_enable = app_config.night_mode_enable;
            const int old_isp_lum_low = app_config.isp_lum_low;
            const int old_isp_lum_hi = app_config.isp_lum_hi;
            const int old_isp_iso_low = app_config.isp_iso_low;
            const int old_isp_iso_hi = app_config.isp_iso_hi;
            const int old_isp_exptime_low = app_config.isp_exptime_low;
            const int old_ir_sensor_pin = app_config.ir_sensor_pin;
            char old_adc_device[sizeof(app_config.adc_device)];
            strncpy(old_adc_device, app_config.adc_device, sizeof(old_adc_device) - 1);
            old_adc_device[sizeof(old_adc_device) - 1] = '\0';

            bool enable_seen = false;
            bool enable_value = app_config.night_mode_enable;

            bool manual_seen = false;
            bool manual_value = night_manual_on();

            bool set_grayscale = false, set_ircut = false, set_irled = false, set_whiteled = false;
            bool grayscale_value = false, ircut_value = false, irled_value = false, whiteled_value = false;

            char *remain;
            while (req->query) {
                char *value = split(&req->query, "&");
                if (!value || !*value) continue;
                unescape_uri(value);
                char *key = split(&value, "=");
                if (!key || !*key || !value || !*value) continue;
                if (EQUALS(key, "enable")) {
                    enable_seen = true;
                    if (EQUALS_CASE(value, "true") || EQUALS(value, "1"))
                        app_config.night_mode_enable = enable_value = 1;
                    else if (EQUALS_CASE(value, "false") || EQUALS(value, "0"))
                        app_config.night_mode_enable = enable_value = 0;
                } else if (EQUALS(key, "adc_device")) {
                    strncpy(app_config.adc_device, value, sizeof(app_config.adc_device) - 1);
                    app_config.adc_device[sizeof(app_config.adc_device) - 1] = '\0';
                } else if (EQUALS(key, "adc_threshold")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.adc_threshold = result;
                } else if (EQUALS(key, "isp_lum_low")) {
                    long result = strtol(value, &remain, 10);
                    if (remain != value) {
                        if (result < 0) app_config.isp_lum_low = -1;
                        else if (result <= 255) app_config.isp_lum_low = (int)result;
                    }
                } else if (EQUALS(key, "isp_lum_hi")) {
                    long result = strtol(value, &remain, 10);
                    if (remain != value) {
                        if (result < 0) app_config.isp_lum_hi = -1;
                        else if (result <= 255) app_config.isp_lum_hi = (int)result;
                    }
                } else if (EQUALS(key, "isp_iso_low")) {
                    long result = strtol(value, &remain, 10);
                    if (remain != value) {
                        if (result < 0) app_config.isp_iso_low = -1;
                        else app_config.isp_iso_low = (int)result;
                    }
                } else if (EQUALS(key, "isp_iso_hi")) {
                    long result = strtol(value, &remain, 10);
                    if (remain != value) {
                        if (result < 0) app_config.isp_iso_hi = -1;
                        else app_config.isp_iso_hi = (int)result;
                    }
                } else if (EQUALS(key, "isp_exptime_low")) {
                    long result = strtol(value, &remain, 10);
                    if (remain != value) {
                        if (result < 0) app_config.isp_exptime_low = -1;
                        else app_config.isp_exptime_low = (int)result;
                    }
                } else if (EQUALS(key, "isp_switch_lockout_s")) {
                    long result = strtol(value, &remain, 10);
                    if (remain != value) {
                        if (result < 0) result = 0;
                        if (result > 3600) result = 3600;
                        app_config.isp_switch_lockout_s = (unsigned int)result;
                    }
                } else if (EQUALS(key, "grayscale")) {
                    set_grayscale = true;
                    grayscale_value = (EQUALS_CASE(value, "true") || EQUALS(value, "1"));
                } else if (EQUALS(key, "ircut")) {
                    set_ircut = true;
                    ircut_value = (EQUALS_CASE(value, "true") || EQUALS(value, "1"));
                } else if (EQUALS(key, "ircut_pin1")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.ir_cut_pin1 = result;
                } else if (EQUALS(key, "ircut_pin2")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.ir_cut_pin2 = result;
                } else if (EQUALS(key, "irled")) {
                    set_irled = true;
                    irled_value = (EQUALS_CASE(value, "true") || EQUALS(value, "1"));
                } else if (EQUALS(key, "whiteled")) {
                    set_whiteled = true;
                    // Prefer on/off (new API), keep backward compatibility with true/false/1/0.
                    if (EQUALS_CASE(value, "on"))
                        whiteled_value = true;
                    else if (EQUALS_CASE(value, "off"))
                        whiteled_value = false;
                    else
                        whiteled_value = (EQUALS_CASE(value, "true") || EQUALS(value, "1"));
                    HAL_INFO("server", "Night API: whiteled=%s (parsed=%d)\n", value, whiteled_value ? 1 : 0);
                } else if (EQUALS(key, "irled_pin")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.ir_led_pin = result;
                } else if (EQUALS(key, "whiteled_pin") || EQUALS(key, "white_led_pin")) {
                    // Intentionally ignored: `whiteled` is a manual action and must not
                    // mutate config (pins are configured via divinus.yaml).
                } else if (EQUALS(key, "irsense_pin")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.ir_sensor_pin = result;
                } else if (EQUALS(key, "manual")) {
                    manual_seen = true;
                    manual_value = (EQUALS_CASE(value, "true") || EQUALS(value, "1"));
                    night_manual(manual_value ? 1 : 0);
                    app_config.night_mode_manual = manual_value;
                }
            }

            const bool old_iso_mode = (old_isp_iso_low >= 0 && old_isp_iso_hi >= 0);
            const bool new_iso_mode = (app_config.isp_iso_low >= 0 && app_config.isp_iso_hi >= 0);
            const bool old_lum_mode = (old_isp_lum_low >= 0 && old_isp_lum_hi >= 0);
            const bool new_lum_mode = (app_config.isp_lum_low >= 0 && app_config.isp_lum_hi >= 0);

            bool need_restart =
                (old_enable != app_config.night_mode_enable) ||
                (strcmp(old_adc_device, app_config.adc_device) != 0) ||
                (old_ir_sensor_pin != app_config.ir_sensor_pin) ||
                (old_iso_mode != new_iso_mode) ||
                (old_lum_mode != new_lum_mode) ||
                (old_isp_exptime_low != app_config.isp_exptime_low);

            if (need_restart) {
                disable_night();
                if (app_config.night_mode_enable) enable_night();
            } else {
                // Ensure thread is running if enable was explicitly requested.
                if (enable_seen && enable_value && app_config.night_mode_enable)
                    enable_night();
                if (enable_seen && !enable_value)
                    disable_night();
            }

            // If user explicitly disabled night mode support, don't apply direct controls.
            // If user explicitly set manual=false, force DAY first (like on startup),
            // then allow auto logic to re-enter night if needed.
            if (manual_seen && !manual_value) {
                night_mode(false);
                if (app_config.night_mode_enable)
                    enable_night();
            } else if (!(enable_seen && !enable_value)) {
                // If user enables manual mode without explicit hardware params, treat it as
                // "manual night": switch to IR mode immediately and keep automatics disabled.
                if (manual_seen && manual_value && !set_ircut && !set_irled && !set_grayscale)
                    night_mode(true);
                if (set_ircut) night_ircut(ircut_value);
                if (set_irled) night_irled(irled_value);
                if (set_whiteled) {
                    int pin = 0;
                    bool ok = decode_cfg_pin_for_log(app_config.white_led_pin, &pin);
                    if (ok) {
                        HAL_INFO("server", "Night API: apply whiteled=%d (pin_cfg=%d -> pin=%d)\n",
                            whiteled_value ? 1 : 0, app_config.white_led_pin, pin);
                    } else {
                        HAL_INFO("server", "Night API: apply whiteled=%d (pin_cfg=%d -> disabled/invalid)\n",
                            whiteled_value ? 1 : 0, app_config.white_led_pin);
                    }
                    night_whiteled(whiteled_value);
                }
                if (set_grayscale) night_grayscale(grayscale_value);
            }

            // Persist manual toggle (and any other changed fields in app_config).
            // Best-effort: ignore failures to keep API responsive.
            if (manual_seen) {
                int sr = save_app_config();
                if (sr != 0)
                    HAL_WARNING("server", "Failed to save config after night manual change (ret=%d)\n", sr);
            }
        }

        int isp_lum = -1;
        unsigned char lum;
        if (get_isp_avelum(&lum) == EXIT_SUCCESS)
            isp_lum = (int)lum;

        int isp_exposure_is_max = -1;
        int isp_iso = -1, isp_exptime = -1, isp_again = -1, isp_dgain = -1, isp_ispdgain = -1;
        {
            unsigned int iso = 0, exptime = 0, again = 0, dgain = 0, ispdgain = 0;
            int ismax = 0;
            if (get_isp_exposure_info(&iso, &exptime, &again, &dgain, &ispdgain, &ismax) == EXIT_SUCCESS) {
                isp_iso = (int)iso;
                isp_exptime = (int)exptime;
                isp_again = (int)again;
                isp_dgain = (int)dgain;
                isp_ispdgain = (int)ispdgain;
                isp_exposure_is_max = ismax;
            }
        }

        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json;charset=UTF-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{\"active\":%s,\"manual\":%s,\"grayscale\":%s,\"ircut\":%s,\"ircut_pin1\":%d,\"ircut_pin2\":%d,"
            "\"irled\":%s,\"irled_pin\":%d,\"whiteled\":%s,\"whiteled_pin\":%d,\"irsense_pin\":%d,"
            "\"adc_device\":\"%s\",\"adc_threshold\":%d,"
            "\"isp_lum\":%d,\"isp_lum_low\":%d,\"isp_lum_hi\":%d,"
            "\"isp_iso_low\":%d,\"isp_iso_hi\":%d,\"isp_exptime_low\":%d,\"isp_switch_lockout_s\":%u,"
            "\"isp_iso\":%d,\"isp_exptime\":%d,\"isp_again\":%d,\"isp_dgain\":%d,\"isp_ispdgain\":%d,"
            "\"isp_exposure_is_max\":%d}",
            app_config.night_mode_enable ? "true" : "false", night_manual_on() ? "true" : "false", 
            night_grayscale_on() ? "true" : "false",
            night_ircut_on() ? "true" : "false", app_config.ir_cut_pin1, app_config.ir_cut_pin2,
            night_irled_on() ? "true" : "false", app_config.ir_led_pin,
            night_whiteled_on() ? "true" : "false", app_config.white_led_pin,
            app_config.ir_sensor_pin,
            app_config.adc_device, app_config.adc_threshold, isp_lum,
            app_config.isp_lum_low, app_config.isp_lum_hi,
            app_config.isp_iso_low, app_config.isp_iso_hi, app_config.isp_exptime_low, app_config.isp_switch_lockout_s,
            isp_iso, isp_exptime, isp_again, isp_dgain, isp_ispdgain, isp_exposure_is_max);
        send_and_close(req->clntFd, response, respLen);
        return;
    }

    // ISP orientation controls (persisted): mirror / flip (+ optional antiflicker).
    // NOTE: These settings are applied at SDK/pipeline creation time on most platforms.
    // This endpoint persists changes to divinus.yaml; runtime application may require a process restart.
    if (EQUALS(req->uri, "/api/isp")) {
        bool changed = false;
        bool changed_orient = false;
        bool changed_antiflicker = false;
        int save_rc = 0;
        bool saved = false;
        int apply_rc = 0;
        bool applied = true;

        if (!EMPTY(req->query)) {
            bool mirror_seen = false, flip_seen = false, antiflicker_seen = false;
            bool sensor_mirror_seen = false, sensor_flip_seen = false;
            bool mirror_val = app_config.mirror;
            bool flip_val = app_config.flip;
            bool sensor_mirror_val = app_config.sensor_mirror;
            bool sensor_flip_val = app_config.sensor_flip;
            int antiflicker_val = app_config.antiflicker;

            char *remain;
            while (req->query) {
                char *value = split(&req->query, "&");
                if (!value || !*value) continue;
                unescape_uri(value);
                char *key = split(&value, "=");
                if (!key || !*key || !value || !*value) continue;

                if (EQUALS(key, "mirror")) {
                    mirror_seen = true;
                    mirror_val = (EQUALS_CASE(value, "true") || EQUALS(value, "1"));
                } else if (EQUALS(key, "flip")) {
                    flip_seen = true;
                    flip_val = (EQUALS_CASE(value, "true") || EQUALS(value, "1"));
                } else if (EQUALS(key, "sensor_mirror")) {
                    sensor_mirror_seen = true;
                    sensor_mirror_val = (EQUALS_CASE(value, "true") || EQUALS(value, "1"));
                } else if (EQUALS(key, "sensor_flip")) {
                    sensor_flip_seen = true;
                    sensor_flip_val = (EQUALS_CASE(value, "true") || EQUALS(value, "1"));
                } else if (EQUALS(key, "antiflicker")) {
                    long result = strtol(value, &remain, 10);
                    if (remain != value) {
                        antiflicker_seen = true;
                        antiflicker_val = (int)result;
                    }
                }
            }

            if (mirror_seen && (app_config.mirror != mirror_val)) {
                app_config.mirror = mirror_val;
                changed = true;
                changed_orient = true;
            }
            if (flip_seen && (app_config.flip != flip_val)) {
                app_config.flip = flip_val;
                changed = true;
                changed_orient = true;
            }
            if (sensor_mirror_seen && (app_config.sensor_mirror != sensor_mirror_val)) {
                app_config.sensor_mirror = sensor_mirror_val;
                changed = true;
                changed_orient = true;
            }
            if (sensor_flip_seen && (app_config.sensor_flip != sensor_flip_val)) {
                app_config.sensor_flip = sensor_flip_val;
                changed = true;
                changed_orient = true;
            }
            if (antiflicker_seen && (app_config.antiflicker != antiflicker_val)) {
                app_config.antiflicker = antiflicker_val;
                changed = true;
                changed_antiflicker = true;
            }

            if (changed) {
                save_rc = save_app_config();
                saved = (save_rc == 0);
                if (!saved)
                    HAL_WARNING("server", "Failed to save config after isp change (ret=%d)\n", save_rc);

                // Best-effort runtime apply (platform-dependent).
                // Antiflicker is typically applied at pipeline creation time; we don't attempt runtime update here.
                if (changed_orient) {
                    apply_rc = media_set_isp_orientation(app_config.mirror, app_config.flip);
                    applied = (apply_rc == 0);
                }
            }
        }

        bool needs_restart = changed_antiflicker || (changed_orient && !applied);
        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json;charset=UTF-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{\"sensor_mirror\":%s,\"sensor_flip\":%s,"
            "\"mirror\":%s,\"flip\":%s,\"antiflicker\":%d,"
            "\"changed\":%s,\"saved\":%s,\"save_code\":%d,"
            "\"applied\":%s,\"apply_code\":%d,"
            "\"needs_restart\":%s}",
            app_config.sensor_mirror ? "true" : "false",
            app_config.sensor_flip ? "true" : "false",
            app_config.mirror ? "true" : "false",
            app_config.flip ? "true" : "false",
            app_config.antiflicker,
            changed ? "true" : "false",
            saved ? "true" : "false",
            save_rc,
            applied ? "true" : "false",
            apply_rc,
            needs_restart ? "true" : "false");
        send_and_close(req->clntFd, response, respLen);
        return;
    }

    if (app_config.osd_enable && STARTS_WITH(req->uri, "/api/osd/")) {
        char *remain;
        int respLen;
        short id = strtol(req->uri + 9, &remain, 10);
        if (remain == req->uri + 9 || id < 0 || id >= MAX_OSD) {
            send_http_error(req->clntFd, 404);
            return;
        }
        if (EQUALS(req->method, "POST")) {
            char *type = request_header("Content-Type");
            if (STARTS_WITH(type, "multipart/form-data")) {
                char *bound = strstr(type, "boundary=") + strlen("boundary=");

                char *payloadb = strstr(req->payload, bound);
                payloadb = memstr(payloadb, "\r\n\r\n", req->total - (payloadb - req->input), 4);
                if (payloadb) payloadb += 4;

                char *payloade = memstr(payloadb, bound,
                    req->total - (payloadb - req->input), strlen(bound));
                if (payloade) payloade -= 4;

                char path[32];

                if (!memcmp(payloadb, "\x89\x50\x4E\x47\xD\xA\x1A\xA", 8)) 
                    sprintf(path, "/tmp/osd%d.png", id);
                else
                    sprintf(path, "/tmp/osd%d.bmp", id);

                FILE *img = fopen(path, "wb");
                fwrite(payloadb, sizeof(char), payloade - payloadb, img);
                fclose(img);

                strcpy(osds[id].text, "");
                osds[id].persist = 1;
                osds[id].updt = 1;
            } else {
                respLen = sprintf(response,
                    "HTTP/1.1 415 Unsupported Media Type\r\n"
                    "Content-Type: text/plain\r\n"
                    "Connection: close\r\n"
                    "\r\n"
                    "The payload must be presented as multipart/form-data.\r\n"
                );
                send_and_close(req->clntFd, response, respLen);
                return;
            }
        }
        if (!EMPTY(req->query))
        {
            char *remain;
            while (req->query) {
                char *value = split(&req->query, "&");
                if (!value || !*value) continue;
                unescape_uri(value);
                char *key = split(&value, "=");
                if (!key || !*key || !value || !*value) continue;
                if (EQUALS(key, "img"))
                    strncpy(osds[id].img, value,
                        sizeof(osds[id].img) - 1);
                else if (EQUALS(key, "font"))
                    strncpy(osds[id].font, !EMPTY(value) ? value : DEF_FONT,
                        sizeof(osds[id].font) - 1);
                else if (EQUALS(key, "text")) {
                    strncpy(osds[id].text, value,
                        sizeof(osds[id].text) - 1);
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "size")) {
                    double result = strtod(value, &remain);
                    if (remain == value) continue;
                    osds[id].size = (result != 0 ? result : DEF_SIZE);
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "color")) {
                    int result = color_parse(value);
                    osds[id].color = result;
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "opal")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        osds[id].opal = result & 0xFF;
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "posx")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        osds[id].posx = result;
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "posy")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        osds[id].posy = result;
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "pos")) {
                    int x, y;
                    if (sscanf(value, "%d,%d", &x, &y) == 2) {
                        osds[id].posx = x;
                        osds[id].posy = y;
                    }
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "outl")) {
                    int result = color_parse(value);
                    osds[id].outl = result;
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "thick")) {
                    double result = strtod(value, &remain);
                    if (remain == value) continue;
                        osds[id].thick = result;
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "bg")) {
                    int result = color_parse(value);
                    // bg is RGB555 (alpha ignored). Use bgopal to enable/disable.
                    osds[id].bg = result & 0x7FFF;
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "bgopal")) {
                    long result = strtol(value, &remain, 10);
                    if (remain != value) {
                        if (result < 0) result = 0;
                        if (result > 255) result = 255;
                        osds[id].bgopal = (short)result;
                    }
                    osds[id].persist = 1;
                }
                else if (EQUALS(key, "pad")) {
                    long result = strtol(value, &remain, 10);
                    if (remain != value) {
                        if (result < 0) result = 0;
                        if (result > 64) result = 64;
                        osds[id].pad = (short)result;
                    }
                    osds[id].persist = 1;
                }
            }
            osds[id].updt = 1;
        }
        int color = (((osds[id].color >> 10) & 0x1F) * 255 / 31) << 16 |
                    (((osds[id].color >> 5) & 0x1F) * 255 / 31) << 8 |
                    ((osds[id].color & 0x1F) * 255 / 31);
        respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json;charset=UTF-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{\"id\":%d,\"color\":\"#%x\",\"opal\":%d,\"pos\":[%d,%d],"
            "\"font\":\"%s\",\"size\":%.1f,\"text\":\"%s\",\"img\":\"%s\","
            "\"outl\":\"#%x\",\"thick\":%.1f,\"bg\":\"#%x\",\"bgopal\":%d,\"pad\":%d}",
            id, color, osds[id].opal, osds[id].posx, osds[id].posy,
            osds[id].font, osds[id].size, osds[id].text, osds[id].img,
            osds[id].outl, osds[id].thick, osds[id].bg, osds[id].bgopal, osds[id].pad);
        send_and_close(req->clntFd, response, respLen);
        return;
    }

    if (EQUALS(req->uri, "/api/record")) {
        if (!EMPTY(req->query)) {
            char *remain;
            while (req->query) {
                char *value = split(&req->query, "&");
                if (!value || !*value) continue;
                unescape_uri(value);
                char *key = split(&value, "=");
                if (!key || !*key || !value || !*value) continue;
                if (EQUALS(key, "enable")) {
                    if (EQUALS_CASE(value, "true") || EQUALS(value, "1"))
                        app_config.record_enable = 1;
                    else if (EQUALS_CASE(value, "false") || EQUALS(value, "0"))
                        app_config.record_enable = 0;
                }
                else if (EQUALS(key, "continuous")) {
                    if (EQUALS_CASE(value, "true") || EQUALS(value, "1"))
                        app_config.record_continuous = 1;
                    else if (EQUALS_CASE(value, "false") || EQUALS(value, "0"))
                        app_config.record_continuous = 0;
                }
                else if (EQUALS(key, "path"))
                    strncpy(app_config.record_path, value, sizeof(app_config.record_path) - 1);
                else if (EQUALS(key, "filename"))
                    strncpy(app_config.record_filename, value, sizeof(app_config.record_filename) - 1);
                else if (EQUALS(key, "segment_duration")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.record_segment_duration = result;
                }
                else if (EQUALS(key, "segment_size")) {
                    short result = strtol(value, &remain, 10);
                    if (remain != value)
                        app_config.record_segment_size = result;
                }

                if (!app_config.record_enable) continue;
                if (app_config.record_continuous) continue;
                if (EQUALS(key, "start"))
                    record_start();
                else if (EQUALS(key, "stop"))
                    record_stop();
            }
        }
        struct tm tm_buf, *tm_info = localtime_r(&recordStartTime, &tm_buf);
        char start_time[64];
        strftime(start_time, sizeof(start_time), "%Y-%m-%dT%H:%M:%SZ", tm_info);

        respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json;charset=UTF-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{\"recording\":%s,\"start_time\":\"%s\",\"continuous\":%s,\"path\":\"%s\","
            "\"filename\":\"%s\",\"segment_duration\":%d,\"segment_size\":%d}",
                recordOn ? "true" : "false", start_time, app_config.record_continuous ? "true" : "false",
                app_config.record_path, app_config.record_filename, 
                app_config.record_segment_duration, app_config.record_segment_size);
        send_and_close(req->clntFd, response, respLen);
        return;
    }

    if (EQUALS(req->uri, "/api/status")) {
        struct sysinfo si;
        sysinfo(&si);
        char memory[16], uptime[48];
        short free = (si.freeram + si.bufferram) / 1024 / 1024;
        short total = si.totalram / 1024 / 1024;
        sprintf(memory, "%d/%dMB", total - free, total);
        if (si.uptime > 86400)
            sprintf(uptime, "%ld days, %ld:%02ld:%02ld", si.uptime / 86400, (si.uptime % 86400) / 3600, (si.uptime % 3600) / 60, si.uptime % 60);
        else if (si.uptime > 3600)
            sprintf(uptime, "%ld:%02ld:%02ld", si.uptime / 3600, (si.uptime % 3600) / 60, si.uptime % 60);
        else
            sprintf(uptime, "%ld:%02ld", si.uptime / 60, si.uptime % 60);
        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json;charset=UTF-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{\"chip\":\"%s\",\"loadavg\":[%.2f,%.2f,%.2f],\"memory\":\"%s\","
            "\"sensor\":\"%s\",\"temp\":\"%.1f\u00B0C\",\"uptime\":\"%s\"}",
            chip, si.loads[0] / 65536.0, si.loads[1] / 65536.0, si.loads[2] / 65536.0, 
            memory, sensor, hal_temperature_read(), uptime);
        send_and_close(req->clntFd, response, respLen);
        return;
    }

    if (EQUALS(req->uri, "/api/time")) {
        struct timespec t;
        if (!EMPTY(req->query)) {
            char *remain;
            while (req->query) {
                char *value = split(&req->query, "&");
                if (!value || !*value) continue;
                unescape_uri(value);
                char *key = split(&value, "=");
                if (!key || !*key || !value || !*value) continue;
                if (EQUALS(key, "fmt")) {
                    // Sanitize and update both runtime and canonical copies.
                    timefmt_set(value);
                } else if (EQUALS(key, "ts")) {
                    short result = strtol(value, &remain, 10);
                    if (remain == value) continue;
                    t.tv_sec = result;
                    clock_settime(CLOCK_REALTIME, &t);
                }
            }
        }
        clock_gettime(CLOCK_REALTIME, &t);
        int respLen = sprintf(response,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json;charset=UTF-8\r\n"
            "Connection: close\r\n"
            "\r\n"
            "{\"fmt\":\"%s\",\"ts\":%zu}", timefmt, t.tv_sec);
        send_and_close(req->clntFd, response, respLen);
        return;
    }

    if (app_config.web_enable_static && send_file(req->clntFd, req->uri))
        return;

    send_http_error(req->clntFd, 400);
}

void *server_thread(void *vargp) {
    http_request_t req = {0};
    int ret, server_fd = *((int *)vargp);
    int enable = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
        HAL_WARNING("server", "setsockopt(SO_REUSEADDR) failed");
        fflush(stdout);
    }
    struct sockaddr_in client, server = {
        .sin_family = AF_INET,
        .sin_port = htons(app_config.web_port),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };
    if (!EMPTY(app_config.web_bind)) {
        struct in_addr bind_addr = {0};
        if (inet_aton(app_config.web_bind, &bind_addr)) {
            server.sin_addr = bind_addr;
        } else {
            HAL_WARNING("server", "Invalid web_bind '%s', falling back to 0.0.0.0\n", app_config.web_bind);
        }
    }
    if (ret = bind(server_fd, (struct sockaddr *)&server, sizeof(server))) {
        HAL_DANGER("server", "%s (%d)\n", strerror(errno), errno);
        keepRunning = 0;
        close_socket_fd(server_fd);
        return NULL;
    }
    listen(server_fd, 128);

    req.input = malloc(REQSIZE);

    while (keepRunning) {
        if ((req.clntFd = accept(server_fd, NULL, NULL)) == -1)
            break;

        parse_request(&req);

        respond_request(&req);
    }

    if (req.input)
        free(req.input);

    close_socket_fd(server_fd);
    HAL_INFO("server", "Thread has exited\n");
    return NULL;
}

int start_server() {
    server_pcm_clients = 0;
    server_h26x_clients = 0;
    server_mp4_clients = 0;
    server_mjpeg_clients = 0;
    for (unsigned int i = 0; i < MAX_CLIENTS; i++) {
        client_fds[i].sockFd = -1;
        client_fds[i].type = -1;
    }
    pthread_mutex_init(&client_fds_mutex, NULL);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);

    {
        pthread_attr_t thread_attr;
        pthread_attr_init(&thread_attr);
        size_t stacksize;
        pthread_attr_getstacksize(&thread_attr, &stacksize);
        size_t new_stacksize = app_config.web_server_thread_stack_size + REQSIZE;
        if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
            HAL_WARNING("server", "Can't set stack size %zu\n", new_stacksize);
        if (pthread_create(
            &server_thread_id, &thread_attr, server_thread, (void *)&server_fd))
            HAL_ERROR("server", "Starting the server thread failed!\n");
        if (pthread_attr_setstacksize(&thread_attr, stacksize))
            HAL_DANGER("server", "Can't set stack size %zu\n", stacksize);
        pthread_attr_destroy(&thread_attr);
    }

    return EXIT_SUCCESS;
}

int stop_server() {
    keepRunning = 0;

    close_socket_fd(server_fd);
    pthread_join(server_thread_id, NULL);

    pthread_mutex_destroy(&client_fds_mutex);
    HAL_INFO("server", "Shutting down server...\n");

    return EXIT_SUCCESS;
}