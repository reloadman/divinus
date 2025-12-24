#include "gpio.h"

#include <dirent.h>
#include <string.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
#include <linux/gpio.h>
#include <sys/ioctl.h>
#endif

typedef enum {
    GPIO_BACKEND_UNSET = 0,
    GPIO_BACKEND_CDEV,
    GPIO_BACKEND_SYSFS,
} gpio_backend_t;

static gpio_backend_t gpio_backend = GPIO_BACKEND_UNSET;

static bool path_exists(const char *path) {
    return access(path, F_OK) == 0;
}

static int read_int_file(const char *path, int *out) {
    if (!out) {
        errno = EINVAL;
        return -1;
    }

    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;

    char buf[64];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    int saved = errno;
    close(fd);
    errno = saved;
    if (n <= 0) return -1;

    buf[n] = '\0';
    char *end = NULL;
    long v = strtol(buf, &end, 10);
    if (end == buf) {
        errno = EINVAL;
        return -1;
    }
    *out = (int)v;
    return 0;
}

static int gpio_sysfs_export(int pin) {
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return -1;

    char val[16];
    int len = snprintf(val, sizeof(val), "%d", pin);
    ssize_t w = write(fd, val, (size_t)len);
    int saved = errno;
    close(fd);
    errno = saved;

    if (w < 0) {
        // already exported
        if (errno == EBUSY) return 0;
        return -1;
    }
    return 0;
}

static int gpio_sysfs_unexport(int pin) {
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) return -1;

    char val[16];
    int len = snprintf(val, sizeof(val), "%d", pin);
    ssize_t w = write(fd, val, (size_t)len);
    int saved = errno;
    close(fd);
    errno = saved;

    if (w < 0) {
        // ignore "not exported" style errors
        if (errno == EINVAL || errno == ENOENT) return 0;
        return -1;
    }
    return 0;
}

static int gpio_sysfs_direction(int pin, const char *mode) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;

    ssize_t w = write(fd, mode, strlen(mode));
    int saved = errno;
    close(fd);
    errno = saved;
    return (w < 0) ? -1 : 0;
}

static int gpio_sysfs_write_value(int pin, bool value) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;

    char val = value ? '1' : '0';
    ssize_t w = write(fd, &val, 1);
    int saved = errno;
    close(fd);
    errno = saved;
    return (w < 0) ? -1 : 0;
}

static int gpio_sysfs_read_value(int pin, bool *out) {
    if (!out) {
        errno = EINVAL;
        return -1;
    }

    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;

    char val = 0;
    lseek(fd, 0, SEEK_SET);
    ssize_t r = read(fd, &val, 1);
    int saved = errno;
    close(fd);
    errno = saved;
    if (r <= 0) return -1;

    *out = (val != '0');
    return 0;
}

static int gpio_sysfs_read(int pin, bool *value) {
    if (gpio_sysfs_export(pin) < 0) return -1;
    if (gpio_sysfs_direction(pin, "in") < 0) return -1;
    int ret = gpio_sysfs_read_value(pin, value);
    // return pin back to kernel space (matches /usr/sbin/gpio behavior)
    (void)gpio_sysfs_direction(pin, "in");
    (void)gpio_sysfs_unexport(pin);
    return ret;
}

static int gpio_sysfs_write(int pin, bool value) {
    if (gpio_sysfs_export(pin) < 0) return -1;
    if (gpio_sysfs_direction(pin, "out") < 0) return -1;
    // Keep the line exported so the driven level is held (needed for steady outputs
    // such as IR/white LEDs). Export is idempotent; callers can rewrite freely.
    return gpio_sysfs_write_value(pin, value);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
static int gpio_map_global_to_chip(int pin, int *chip_index, int *line_offset) {
    if (!chip_index || !line_offset) {
        errno = EINVAL;
        return -1;
    }

    DIR *d = opendir("/sys/class/gpio");
    if (!d) {
        // If sysfs gpiochip base mapping is unavailable, fall back to legacy
        // behavior: treat "pin" as a line offset on gpiochip0.
        *chip_index = 0;
        *line_offset = pin;
        return 0;
    }

    struct dirent *de = NULL;
    int found = 0;

    while ((de = readdir(d))) {
        if (strncmp(de->d_name, "gpiochip", 7) != 0) continue;

        int idx = atoi(de->d_name + 7);
        char base_path[128], ngpio_path[128];
        snprintf(base_path, sizeof(base_path), "/sys/class/gpio/%s/base", de->d_name);
        snprintf(ngpio_path, sizeof(ngpio_path), "/sys/class/gpio/%s/ngpio", de->d_name);

        int base = 0, ngpio = 0;
        if (read_int_file(base_path, &base) < 0) continue;
        if (read_int_file(ngpio_path, &ngpio) < 0) continue;
        if (ngpio <= 0) continue;

        if (pin >= base && pin < (base + ngpio)) {
            *chip_index = idx;
            *line_offset = pin - base;
            found = 1;
            break;
        }
    }

    closedir(d);
    if (!found) {
        // Fall back to legacy behavior: treat "pin" as a line offset on gpiochip0.
        *chip_index = 0;
        *line_offset = pin;
        return 0;
    }
    return 0;
}

static int gpio_cdev_probe(void) {
    // If we can query chipinfo on gpiochip0, assume cdev backend is usable.
    int fd = open("/dev/gpiochip0", O_RDONLY);
    if (fd < 0) return -1;
    struct gpiochip_info info;
    int ret = ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info);
    int saved = errno;
    close(fd);
    errno = saved;
    return (ret < 0) ? -1 : 0;
}

static int gpio_cdev_read(int pin, bool *value) {
    if (!value) {
        errno = EINVAL;
        return -1;
    }

    int chip = 0, offset = 0;
    if (gpio_map_global_to_chip(pin, &chip, &offset) < 0) return -1;

    char dev[32];
    snprintf(dev, sizeof(dev), "/dev/gpiochip%d", chip);
    int chip_fd = open(dev, O_RDONLY);
    if (chip_fd < 0) return -1;

    struct gpiohandle_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffsets[0] = (unsigned int)offset;
    req.lines = 1;
    req.flags = GPIOHANDLE_REQUEST_INPUT;

    if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        int saved = errno;
        close(chip_fd);
        errno = saved;
        return -1;
    }
    close(chip_fd);

    struct gpiohandle_data data;
    memset(&data, 0, sizeof(data));
    if (ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data) < 0) {
        int saved = errno;
        close(req.fd);
        errno = saved;
        return -1;
    }

    *value = data.values[0];
    close(req.fd);
    return 0;
}

static int gpio_cdev_write(int pin, bool value) {
    int chip = 0, offset = 0;
    if (gpio_map_global_to_chip(pin, &chip, &offset) < 0) return -1;

    char dev[32];
    snprintf(dev, sizeof(dev), "/dev/gpiochip%d", chip);
    int chip_fd = open(dev, O_RDONLY);
    if (chip_fd < 0) return -1;

    struct gpiohandle_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffsets[0] = (unsigned int)offset;
    req.lines = 1;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.default_values[0] = value ? 1 : 0;

    if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        int saved = errno;
        close(chip_fd);
        errno = saved;
        return -1;
    }
    close(chip_fd);

    close(req.fd);
    return 0;
}

static int gpio_cdev_request_output_hold(int pin, bool value, int *out_fd) {
    if (!out_fd) {
        errno = EINVAL;
        return -1;
    }
    *out_fd = -1;

    int chip = 0, offset = 0;
    if (gpio_map_global_to_chip(pin, &chip, &offset) < 0) return -1;

    char dev[32];
    snprintf(dev, sizeof(dev), "/dev/gpiochip%d", chip);
    int chip_fd = open(dev, O_RDONLY);
    if (chip_fd < 0) return -1;

    struct gpiohandle_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffsets[0] = (unsigned int)offset;
    req.lines = 1;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.default_values[0] = value ? 1 : 0;

    if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        int saved = errno;
        close(chip_fd);
        errno = saved;
        return -1;
    }
    close(chip_fd);

    *out_fd = req.fd; // caller must close
    return 0;
}
#endif

void gpio_deinit(void) {
    // currently stateless; backend selection is runtime
    gpio_backend = GPIO_BACKEND_UNSET;
}

int gpio_init(void) {
    // Prefer cdev backend if it works; otherwise fall back to sysfs.
    if (gpio_backend != GPIO_BACKEND_UNSET) return EXIT_SUCCESS;

    if (path_exists("/sys/class/gpio/export")) {
        gpio_backend = GPIO_BACKEND_SYSFS;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
    if (path_exists("/dev/gpiochip0") && gpio_cdev_probe() == 0) {
        gpio_backend = GPIO_BACKEND_CDEV;
    }
#endif

    if (gpio_backend == GPIO_BACKEND_UNSET) {
        HAL_ERROR("gpio", "No usable GPIO backend found!\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int gpio_read(int pin, bool *value) {
    if (gpio_init() != EXIT_SUCCESS) return EXIT_FAILURE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
    if (gpio_backend == GPIO_BACKEND_CDEV) {
        if (gpio_cdev_read(pin, value) == 0) return EXIT_SUCCESS;
        // If cdev fails (e.g. EINVAL on GK7205), try sysfs fallback.
        if (path_exists("/sys/class/gpio/export") && gpio_sysfs_read(pin, value) == 0)
            return EXIT_SUCCESS;
        HAL_ERROR("gpio", "Unable to request a read on GPIO pin %d (error #%d)!\n", pin, errno);
        return EXIT_FAILURE;
    }
#endif

    if (gpio_sysfs_read(pin, value) < 0) {
        HAL_ERROR("gpio", "Unable to read from GPIO pin %d (error #%d)!\n", pin, errno);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int gpio_write(int pin, bool value) {
    if (gpio_init() != EXIT_SUCCESS) return EXIT_FAILURE;

    // Prefer sysfs for global pin numbering and to keep outputs latched (line stays
    // exported and driven). Fall back to cdev if sysfs is unavailable.
    if (path_exists("/sys/class/gpio/export")) {
        if (gpio_sysfs_write(pin, value) == 0) return EXIT_SUCCESS;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
    if (gpio_backend == GPIO_BACKEND_CDEV) {
        if (gpio_cdev_write(pin, value) == 0) return EXIT_SUCCESS;
    }
#endif

    if (gpio_sysfs_write(pin, value) < 0) {
        HAL_ERROR("gpio", "Unable to write to GPIO pin %d (error #%d)!\n", pin, errno);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int gpio_pulse_pair(int pin1, bool val1, int pin2, bool val2, unsigned int pulse_us) {
    if (gpio_init() != EXIT_SUCCESS) return EXIT_FAILURE;

    // Prefer sysfs for "global pin number" semantics when available.
    // On some platforms (e.g. GK7205), cdev base mapping may not match the
    // numbering used by other tools (majestic), while sysfs does.
    if (path_exists("/sys/class/gpio/export")) {
        // SYSFS: export both pins and keep them exported during the pulse.
        if (gpio_sysfs_export(pin1) < 0) return EXIT_FAILURE;
        if (gpio_sysfs_export(pin2) < 0) { (void)gpio_sysfs_unexport(pin1); return EXIT_FAILURE; }

        if (gpio_sysfs_direction(pin1, "out") < 0) goto SYSFS_FAIL;
        if (gpio_sysfs_direction(pin2, "out") < 0) goto SYSFS_FAIL;

        if (gpio_sysfs_write_value(pin1, val1) < 0) goto SYSFS_FAIL;
        if (gpio_sysfs_write_value(pin2, val2) < 0) goto SYSFS_FAIL;
        usleep(pulse_us);
        (void)gpio_sysfs_write_value(pin1, false);
        (void)gpio_sysfs_write_value(pin2, false);

        (void)gpio_sysfs_unexport(pin1);
        (void)gpio_sysfs_unexport(pin2);
        return EXIT_SUCCESS;

SYSFS_FAIL:
        (void)gpio_sysfs_write_value(pin1, false);
        (void)gpio_sysfs_write_value(pin2, false);
        (void)gpio_sysfs_unexport(pin1);
        (void)gpio_sysfs_unexport(pin2);
        return EXIT_FAILURE;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
    if (gpio_backend == GPIO_BACKEND_CDEV) {
        // Hold both lines asserted for the duration of the pulse by keeping
        // line handles open.
        int fd1 = -1, fd2 = -1;
        if (gpio_cdev_request_output_hold(pin1, val1, &fd1) != 0) return EXIT_FAILURE;
        if (gpio_cdev_request_output_hold(pin2, val2, &fd2) != 0) { close(fd1); return EXIT_FAILURE; }

        usleep(pulse_us);

        struct gpiohandle_data data;
        memset(&data, 0, sizeof(data));
        data.values[0] = 0;
        (void)ioctl(fd1, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        (void)ioctl(fd2, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        close(fd1);
        close(fd2);
        return EXIT_SUCCESS;
    }
#endif

    return EXIT_FAILURE;
}
