#include "app_config.h"
#include "hal/macros.h"
#include "http_post.h"
#include "media.h"
#include "network.h"
#include "night.h"
#include "rtsp_smol.h"
#include "server.h"
#include "watchdog.h"

#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

char graceful = 0, keepRunning = 1;

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

    // SIGPIPE is expected when a client disconnects while we're writing.
    // Never "ignore" crash signals like SIGSEGV/SIGILL based on errno: errno may
    // refer to an unrelated previous syscall, and returning from SIGSEGV will
    // just re-trigger the fault and spam logs in an infinite loop.
    if (signo == SIGPIPE) {
        int len = snprintf(
            msg, sizeof(msg),
            "Non-fatal network signal (%d:%s) errno=%d (%s); ignore\n",
            signo, signal_name(signo), errno, strerror(errno));
        if (len > 0)
            write(STDERR_FILENO, msg, (size_t)len);
        return;
    }

    int len = snprintf(
        msg, sizeof(msg),
        "Error occured (%d:%s) errno=%d (%s) ! Quitting...\n",
        signo, signal_name(signo), errno, strerror(errno));
    if (len > 0)
        write(STDERR_FILENO, msg, (size_t)len);
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

int main(int argc, char *argv[]) {
    {
        char signal_error[] = {SIGABRT, SIGBUS, SIGFPE, SIGSEGV};
        char signal_exit[] = {SIGINT, SIGQUIT, SIGTERM};
        // SIGPIPE can occur on client disconnect/teardown; ignore it.
        char signal_null[] = {SIGPIPE};

        for (char *s = signal_error; s < (&signal_error)[1]; s++)
            signal(*s, handle_error);
        for (char *s = signal_exit; s < (&signal_exit)[1]; s++)
            signal(*s, handle_exit);
        for (char *s = signal_null; s < (&signal_null)[1]; s++)
            signal(*s, SIG_IGN);
    }

    hal_identify();

    if (!*family)
        HAL_ERROR("hal", "Unsupported chip family! Quitting...\n");

    fprintf(stderr, "\033[7m Divinus for %s \033[0m\n", family);
    fprintf(stderr, "Chip ID: %s\n", chip);

    if (parse_app_config() != CONFIG_OK)
        HAL_ERROR("hal", "Can't load app config 'divinus.yaml'\n");

    // Always "exercise" IR-cut on startup to reduce chance of a stuck filter.
    // This runs regardless of night_mode.enable.
    night_ircut_exercise_startup();

    if (app_config.watchdog)
        watchdog_start(app_config.watchdog);

    start_network();

    start_server();

    if (app_config.rtsp_enable) {
        if (smolrtsp_server_start() == 0)
            HAL_INFO("rtsp", "Started smolrtsp server on port %d\n", app_config.rtsp_port);
        else
            HAL_ERROR("rtsp", "Failed to start smolrtsp server\n");
    }

    if (app_config.stream_enable)
        start_streaming();

    if (start_sdk())
        HAL_ERROR("hal", "Failed to start SDK!\n");

    if (app_config.night_mode_enable)
        enable_night();

    if (app_config.http_post_enable)
        start_http_post_send();

    if (app_config.osd_enable)
        start_region_handler();

    if (app_config.record_enable && app_config.record_continuous)
        record_start();

    while (keepRunning) {
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

    if (!graceful)
        restore_app_config();

    fprintf(stderr, "Main thread is shutting down...\n");
    return EXIT_SUCCESS;
}