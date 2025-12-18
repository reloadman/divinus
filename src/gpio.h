#include <errno.h>
#include <fcntl.h>
#include <linux/version.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
#include <linux/gpio.h>
#include <sys/ioctl.h>
#else
#include <string.h>
#endif

#include "hal/macros.h"

void gpio_deinit(void);
int gpio_init(void);
int gpio_read(int pin, bool *value);
int gpio_write(int pin, bool value);

// Applies a level to two GPIOs, holds it for `pulse_us`, then drives both low.
// Returns EXIT_SUCCESS on success.
int gpio_pulse_pair(int pin1, bool val1, int pin2, bool val2, unsigned int pulse_us);
