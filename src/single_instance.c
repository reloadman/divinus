#include "single_instance.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/file.h>

static int g_pidfd = -1;
static char g_pidfile_path[128] = {0};

const char *single_instance_pidfile_path(void) {
    return g_pidfile_path[0] ? g_pidfile_path : NULL;
}

static int try_lock_pidfile(const char *path) {
    int fd = open(path, O_RDWR | O_CREAT, 0644);
    if (fd < 0)
        return -1;

    // Avoid leaking fd across exec (best-effort).
    (void)fcntl(fd, F_SETFD, FD_CLOEXEC);

    if (flock(fd, LOCK_EX | LOCK_NB) < 0) {
        const int saved = errno;
        close(fd);
        errno = saved;
        return -1;
    }

    // Write our pid (best-effort). Keep fd open to hold the lock.
    (void)ftruncate(fd, 0);
    (void)lseek(fd, 0, SEEK_SET);
    {
        char buf[64];
        int n = snprintf(buf, sizeof(buf), "%ld\n", (long)getpid());
        if (n > 0)
            (void)write(fd, buf, (size_t)n);
    }

    g_pidfd = fd;
    strncpy(g_pidfile_path, path, sizeof(g_pidfile_path) - 1);
    g_pidfile_path[sizeof(g_pidfile_path) - 1] = '\0';
    return 0;
}

int single_instance_acquire(const char *name) {
    if (!name || !*name) {
        errno = EINVAL;
        return -1;
    }
    if (g_pidfd >= 0) {
        errno = EALREADY;
        return -1;
    }

    // Prefer modern runtime dirs, fallback to /tmp for BusyBox/minimal systems.
    // Note: /var/run is often a symlink to /run.
    const char *dirs[] = {"/run", "/var/run", "/tmp"};
    char path[128];

    for (size_t i = 0; i < sizeof(dirs) / sizeof(dirs[0]); i++) {
        int n = snprintf(path, sizeof(path), "%s/%s.pid", dirs[i], name);
        if (n <= 0 || (size_t)n >= sizeof(path)) {
            errno = ENAMETOOLONG;
            return -1;
        }

        // Keep last-attempted path for error reporting.
        strncpy(g_pidfile_path, path, sizeof(g_pidfile_path) - 1);
        g_pidfile_path[sizeof(g_pidfile_path) - 1] = '\0';

        if (try_lock_pidfile(path) == 0)
            return 0;

        // If directory doesn't exist, try the next one.
        if (errno == ENOENT)
            continue;

        // If another instance holds the lock, keep errno as-is (EWOULDBLOCK/EAGAIN).
        if (errno == EWOULDBLOCK || errno == EAGAIN)
            return -1;

        // Other errors (e.g., permission issues): try next directory anyway.
        // Keep going so systems without /run write permission can still use /tmp.
    }

    // errno is from the last attempt.
    return -1;
}


