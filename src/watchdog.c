#include "watchdog.h"

int fd = -1;

void watchdog_reset(void) {
    if (fd < 0) return;
    write(fd, "", 1);
}

int watchdog_start(int timeout) {
    if (fd >= 0) return EXIT_SUCCESS;
    const char *paths[] = {"/dev/watchdog0", "/dev/watchdog"};

    // Try known watchdog device nodes. Do NOT treat `paths` as NULL-terminated:
    // if neither exists, walking past the array can crash on embedded systems.
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
        const char *p = paths[i];
        if (access(p, F_OK) != 0)
            continue;
        int tmp = open(p, O_WRONLY);
        if (tmp == -1)
            HAL_ERROR("watchdog", "%s could not be opened!\n", p);
        fd = tmp;
        break;
    }
    if (fd < 0)
        HAL_ERROR("watchdog", "No matching device has been found!\n");

    ioctl(fd, WDIOC_SETTIMEOUT, &timeout);

    HAL_INFO("watchdog", "Watchdog started!\n");
    return EXIT_SUCCESS;
}

void watchdog_stop(void) {
    if (fd < 0) return;
    write(fd, "V", 1);
    close(fd);
    fd = -1;

    HAL_INFO("watchdog", "Watchdog stopped!\n");
}