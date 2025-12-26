#include "app_config.h"
#include "hal/macros.h"
#include "http_post.h"
#include "media.h"
#include "network.h"
#include "night.h"
#include "rtsp_smol.h"
#include "server.h"
#include "single_instance.h"
#include "watchdog.h"

#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

char graceful = 0, keepRunning = 1;
static volatile sig_atomic_t iq_reload_pending = 0;

// Startup phase marker for crash diagnostics.
// Not strictly async-signal-safe, but used only for printing a best-effort hint.
static volatile const char *g_phase = "init";

// Optional backtrace support (best-effort).
// We use weak symbols so builds still work on toolchains/libc that don't provide execinfo.
extern int backtrace(void **buffer, int size) __attribute__((weak));

static const char *signal_name(int signo) {
    switch (signo) {
    case SIGABRT: return "SIGABRT";
    case SIGBUS:  return "SIGBUS";
    case SIGFPE:  return "SIGFPE";
    case SIGILL:  return "SIGILL";
    case SIGSEGV: return "SIGSEGV";
    case SIGINT:  return "SIGINT";
    case SIGQUIT: return "SIGQUIT";
    case SIGTERM: return "SIGTERM";
    case SIGPIPE: return "SIGPIPE";
    default:      return "SIGUNKNOWN";
    }
}

void handle_error(int signo) {
    char msg[128];
    const int saved_errno = errno;

    // SIGPIPE is expected when a client disconnects while we're writing.
    // Never "ignore" crash signals like SIGSEGV/SIGILL based on errno: errno may
    // refer to an unrelated previous syscall, and returning from SIGSEGV will
    // just re-trigger the fault and spam logs in an infinite loop.
    if (signo == SIGPIPE) {
        int len = snprintf(
            msg, sizeof(msg),
            "Non-fatal network signal (%d:%s) errno=%d (%s); ignore\n",
            signo, signal_name(signo), saved_errno, strerror(saved_errno));
        if (len > 0)
            write(STDERR_FILENO, msg, (size_t)len);
        return;
    }

    int len = snprintf(
        msg, sizeof(msg),
        "Error occured (%d:%s) errno=%d (%s) ! Quitting...\n",
        signo, signal_name(signo), saved_errno, strerror(saved_errno));
    if (len > 0)
        write(STDERR_FILENO, msg, (size_t)len);
    if (g_phase) {
        int l2 = snprintf(msg, sizeof(msg), "Last phase: %s\n", (const char *)g_phase);
        if (l2 > 0)
            write(STDERR_FILENO, msg, (size_t)l2);
    }

    // Best-effort backtrace: helps pinpoint startup crashes on embedded targets.
    // (Uses snprintf/write like the rest of this handler; not strictly async-signal-safe,
    // but it's already used here and is valuable for diagnostics.)
    if (backtrace) {
        void *bt[32];
        int n = backtrace(bt, (int)(sizeof(bt) / sizeof(bt[0])));
        if (n > 0) {
            write(STDERR_FILENO, "Backtrace addresses:\n", 21);
            for (int i = 0; i < n; i++) {
                char line[64];
                int l = snprintf(line, sizeof(line), "  #%d %p\n", i, bt[i]);
                if (l > 0)
                    write(STDERR_FILENO, line, (size_t)l);
            }
        }
    }
    keepRunning = 0;
    exit(EXIT_FAILURE);
}

void handle_exit(int signo) {
    char msg[128];
    int len = snprintf(
        msg, sizeof(msg),
        "Graceful shutdown... (%d:%s)\n",
        signo, signal_name(signo));
    if (len > 0)
        write(STDERR_FILENO, msg, (size_t)len);
    keepRunning = 0;
    graceful = 1;
    // Wake main loop quickly if sleeping.
    alarm(1);
}

static void handle_reload(int signo) {
    (void)signo;
    iq_reload_pending = 1;
    // Nudge the main loop so it reloads promptly even if sleeping.
    alarm(1);
}

int main(int argc, char *argv[]) {
    {
        char signal_error[] = {SIGABRT, SIGBUS, SIGFPE, SIGSEGV};
        char signal_exit[] = {SIGINT, SIGQUIT, SIGTERM};
        char signal_reload[] = {SIGHUP};
        // SIGPIPE can occur on client disconnect/teardown; ignore it.
        char signal_null[] = {SIGPIPE};

        for (char *s = signal_error; s < (&signal_error)[1]; s++)
            signal(*s, handle_error);
        for (char *s = signal_exit; s < (&signal_exit)[1]; s++)
            signal(*s, handle_exit);
        for (char *s = signal_reload; s < (&signal_reload)[1]; s++)
            signal(*s, handle_reload);
        for (char *s = signal_null; s < (&signal_null)[1]; s++)
            signal(*s, SIG_IGN);
    }

    // Protect against launching a second copy (BusyBox-friendly: pidfile + flock()).
    // Acquire before touching hardware/network to avoid side effects from a duplicate start.
    if (single_instance_acquire("divinus") != 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            const char *p = single_instance_pidfile_path();
            if (p)
                fprintf(stderr, "divinus: already running (pidfile locked: %s)\n", p);
            else
                fprintf(stderr, "divinus: already running (pidfile locked)\n");
            return EXIT_FAILURE;
        }
        fprintf(stderr, "divinus: failed to acquire pidfile lock (%d:%s)\n", errno, strerror(errno));
        return EXIT_FAILURE;
    }

    g_phase = "hal_identify";
    hal_identify();

    if (!*family)
        HAL_ERROR("hal", "Unsupported chip family! Quitting...\n");

    fprintf(stderr, "\033[7m Divinus for %s \033[0m\n", family);
    fprintf(stderr, "Chip ID: %s\n", chip);

    g_phase = "parse_app_config";
    if (parse_app_config() != CONFIG_OK)
        HAL_ERROR("hal", "Can't load app config 'divinus.yaml'\n");

    // Apply persisted night_mode.manual as early as possible.
    g_phase = "night_manual";
    night_manual(app_config.night_mode_manual);

    // Always "exercise" IR-cut on startup to reduce chance of a stuck filter.
    // This runs regardless of night_mode.enable.
    g_phase = "night_ircut_exercise_startup";
    night_ircut_exercise_startup();

    // Always force white LED OFF on boot (safety / avoid blinding on restart).
    g_phase = "night_whiteled_off";
    night_whiteled(false);

    g_phase = "watchdog_start";
    if (app_config.watchdog)
        watchdog_start(app_config.watchdog);

    g_phase = "start_network";
    HAL_INFO("main", "Starting network...\n");
    start_network();

    g_phase = "start_server";
    HAL_INFO("main", "Starting HTTP server...\n");
    start_server();

    if (app_config.rtsp_enable) {
        g_phase = "start_rtsp";
        if (smolrtsp_server_start() == 0)
            HAL_INFO("rtsp", "Started smolrtsp server on port %d\n", app_config.rtsp_port);
        else
            HAL_ERROR("rtsp", "Failed to start smolrtsp server\n");
    }

    g_phase = "start_streaming";
    if (app_config.stream_enable)
        start_streaming();

    g_phase = "start_sdk";
    HAL_INFO("main", "Starting SDK...\n");
    if (start_sdk())
        HAL_ERROR("hal", "Failed to start SDK!\n");

    if (app_config.night_mode_enable) {
        g_phase = "enable_night";
        enable_night();
        // If persisted manual mode is enabled, user expects "manual night" on boot:
        // force IR mode immediately and keep automatics disabled.
        if (app_config.night_mode_manual)
            night_mode(true);
    }

    g_phase = "start_http_post";
    if (app_config.http_post_enable)
        start_http_post_send();

    g_phase = "start_region_handler";
    if (app_config.osd_enable)
        start_region_handler();

    g_phase = "record_start";
    if (app_config.record_enable && app_config.record_continuous)
        record_start();

    g_phase = "main_loop";
    while (keepRunning) {
        if (iq_reload_pending) {
            iq_reload_pending = 0;
            int rc = media_reload_iq();
            if (rc == EXIT_SUCCESS)
                HAL_INFO("main", "Reloaded IQ config (SIGHUP)\n");
            else
                HAL_WARNING("main", "IQ reload (SIGHUP) failed with %#x\n", rc);
        }
        watchdog_reset();
        sleep(1);
    }

    if (app_config.record_enable && app_config.record_continuous)
        record_stop();

    if (app_config.rtsp_enable) {
        smolrtsp_server_stop();
        HAL_INFO("rtsp", "SmolRTSP server has closed!\n");
    }

    if (app_config.osd_enable)
        stop_region_handler();

    if (app_config.night_mode_enable)
        disable_night();

    stop_sdk();

    if (app_config.stream_enable)
        stop_streaming();

    stop_server();

    stop_network();

    if (app_config.watchdog)
        watchdog_stop();

    fprintf(stderr, "Main thread is shutting down...\n");
    return EXIT_SUCCESS;
}