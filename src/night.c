#include "night.h"

#include <errno.h>
#include <string.h>

char nightOn = 0;
static bool grayscale = false, ircut = true, irled = false, manual = false;
pthread_t nightPid = 0;

bool night_grayscale_on(void) { return grayscale; }

bool night_ircut_on(void) { return ircut; }

bool night_irled_on(void) { return irled; }

bool night_manual_on(void) { return manual; }

// "Night mode" for other subsystems (e.g. ISP IQ) should mean "IR mode",
// not "grayscale enabled". IR mode is active when IR-cut is removed and IR LED is on.
bool night_mode_on(void) { return !ircut && irled; }

static inline bool night_should_grayscale(void) { return app_config.night_mode_grayscale; }
static bool night_applied(void) {
    return (!ircut && irled) && (grayscale == night_should_grayscale());
}
static bool day_applied(void) { return (ircut && !irled) && !grayscale; }

static void night_reset_state(void) {
    // Reset our cached state so that a subsequent enable/restart won't suppress
    // a needed transition due to stale booleans.
    grayscale = false;
    ircut = true;
    irled = false;
}

void night_grayscale(bool enable) {
    set_grayscale(enable);
    grayscale = enable;
}

void night_ircut(bool enable) {
    if (app_config.ir_cut_pin1 == 999 || app_config.ir_cut_pin2 == 999) {
        HAL_WARNING("night", "IR-cut pins not configured, skipping\n");
        ircut = enable;
        return;
    }
    if (app_config.ir_cut_pin1 == app_config.ir_cut_pin2) {
        HAL_WARNING("night", "IR-cut pins invalid (pin1==pin2), skipping\n");
        ircut = enable;
        return;
    }

    HAL_INFO("night", "IR-cut -> %s (pin1=%u pin2=%u pulse=%uus)\n",
        enable ? "ON(DAY)" : "OFF(IR)",
        app_config.ir_cut_pin1, app_config.ir_cut_pin2,
        app_config.pin_switch_delay_us * 100);

    unsigned int pulse_us = app_config.pin_switch_delay_us * 100;
    int r = gpio_pulse_pair(app_config.ir_cut_pin1, !enable, app_config.ir_cut_pin2, enable, pulse_us);
    if (r != EXIT_SUCCESS) {
        HAL_WARNING("night", "GPIO pulse failed: (pin1=%u val1=%d) (pin2=%u val2=%d) errno=%d (%s)\n",
            app_config.ir_cut_pin1, !enable, app_config.ir_cut_pin2, enable, errno, strerror(errno));
    } else {
        HAL_INFO("night", "GPIO pulse ok\n");
    }

    ircut = enable;
}

void night_irled(bool enable) {
    if (app_config.ir_led_pin == 999) {
        HAL_WARNING("night", "IR LED pin not configured, skipping\n");
        irled = enable;
        return;
    }
    int r = gpio_write(app_config.ir_led_pin, enable);
    if (r != EXIT_SUCCESS)
        HAL_WARNING("night", "GPIO write failed: pin=%u val=%d errno=%d (%s)\n",
            app_config.ir_led_pin, enable, errno, strerror(errno));
    else
        HAL_INFO("night", "GPIO write ok: pin=%u val=%d\n", app_config.ir_led_pin, enable);
    irled = enable;
}

void night_manual(bool enable) { manual = enable; }

void night_mode(bool enable) {
    // Avoid log spam + avoid re-pulsing IR-cut coil / toggling encoder params
    // when the requested mode is already applied.
    if (enable) {
        if (night_applied()) return;
    } else {
        if (day_applied()) return;
    }

    HAL_INFO("night", "Changing mode to %s\n", enable ? "NIGHT" : "DAY");
    // Always switch IR hardware when entering/leaving night.
    // Grayscale is applied only if configured at the moment of transition.
    night_ircut(!enable);
    night_irled(enable);
    if (enable) night_grayscale(night_should_grayscale());
    else night_grayscale(false);
}

void night_ircut_exercise_startup(void) {
    // Skip if pins are not configured.
    if (app_config.ir_cut_pin1 == 999 || app_config.ir_cut_pin2 == 999) {
        HAL_INFO("night", "IR-cut exercise skipped (pins not configured)\n");
        return;
    }
    if (app_config.ir_cut_pin1 == app_config.ir_cut_pin2) {
        HAL_WARNING("night", "IR-cut exercise skipped (ir_cut_pin1 == ir_cut_pin2)\n");
        return;
    }

    HAL_INFO("night", "Exercising IR-cut (unstick): remove -> restore\n");
    gpio_init();
    usleep(10 * 1000);

    // Remove IR-cut filter (IR mode)
    night_ircut(false);
    usleep(200 * 1000);
    // Restore IR-cut filter (day mode)
    night_ircut(true);
    usleep(200 * 1000);

    // Ensure we don't leave grayscale enabled from any previous state.
    night_grayscale(false);
    // Ensure IR LED is off if configured.
    if (app_config.ir_led_pin != 999) night_irled(false);

    gpio_deinit();

    // Ensure our cached state is consistent: DAY.
    night_reset_state();
}

void *night_thread(void) {
    gpio_init();
    usleep(10000);

    // Sync state on startup based on the selected source (ISP/ADC/GPIO) when possible.

    if (app_config.isp_lum_low >= 0 && app_config.isp_lum_hi >= 0) {
        if (app_config.isp_lum_hi <= app_config.isp_lum_low) {
            HAL_WARNING("night",
                "ISP luminance thresholds invalid (isp_lum_low=%d isp_lum_hi=%d), ignoring\n",
                app_config.isp_lum_low, app_config.isp_lum_hi);
            while (keepRunning && nightOn) sleep(1);
        } else {
            HAL_INFO("night",
                "Using ISP luminance source (u8AveLum): low=%d hi=%d interval=%us\n",
                app_config.isp_lum_low, app_config.isp_lum_hi, app_config.check_interval_s);

            // Apply immediately once at start (do not wait check_interval_s).
            {
                unsigned char lum = 0;
                if (get_isp_avelum(&lum) == EXIT_SUCCESS && !manual) {
                    if ((int)lum <= app_config.isp_lum_low) night_mode(true);
                    else if ((int)lum >= app_config.isp_lum_hi) night_mode(false);
                }
            }

            while (keepRunning && nightOn) {
                unsigned char lum = 0;
                if (get_isp_avelum(&lum) == EXIT_SUCCESS) {
                    if (!manual) {
                        // Hysteresis:
                        // lum <= low  => NIGHT
                        // lum >= hi   => DAY
                        // otherwise   => keep current
                        if ((int)lum <= app_config.isp_lum_low) night_mode(true);
                        else if ((int)lum >= app_config.isp_lum_hi) night_mode(false);
                    }
                }
                sleep(app_config.check_interval_s);
            }
        }
    } else if (app_config.adc_device[0]) {
        int adc_fd = -1;
        fd_set adc_fds;
        int cnt = 0, tmp = 0, val;

        if ((adc_fd = open(app_config.adc_device, O_RDONLY | O_NONBLOCK)) <= 0) {
            HAL_DANGER("night", "Could not open the ADC virtual device!\n");
            return NULL;
        }
        while (keepRunning && nightOn) {
            if (read(adc_fd, &val, sizeof(val)) > 0) {
                usleep(10000);
                tmp += val;
                cnt++;
            }
            if (cnt == 12) {
                tmp /= cnt;
                if (!manual) night_mode(tmp >= app_config.adc_threshold);
                cnt = tmp = 0;
            }
            usleep(app_config.check_interval_s * 1000000 / 12);
        }
        if (adc_fd) close(adc_fd);
    } else if (app_config.ir_sensor_pin == 999) {
        while (keepRunning && nightOn) sleep(1);
    } else {
        // Apply immediately once at start (do not wait check_interval_s).
        {
            bool state = false;
            if (gpio_read(app_config.ir_sensor_pin, &state) == EXIT_SUCCESS && !manual)
                night_mode(state);
        }
        while (keepRunning && nightOn) {
            bool state = false;
            if (gpio_read(app_config.ir_sensor_pin, &state) != EXIT_SUCCESS) {
                sleep(app_config.check_interval_s);
                continue;
            }
            if (!manual) night_mode(state);
            sleep(app_config.check_interval_s);
        }
    }

    usleep(10000);
    gpio_deinit();
    HAL_INFO("night", "Night mode thread is closing...\n");
    nightOn = 0;
}

int enable_night(void) {
    int ret = EXIT_SUCCESS;

    if (nightOn) return ret;

    // Assume DAY state on (re)start to avoid suppressing transitions due to stale cached state.
    // Hardware may already be in a different state; the first evaluation will reconcile it.
    night_reset_state();

    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    size_t stacksize;
    pthread_attr_getstacksize(&thread_attr, &stacksize);
    size_t new_stacksize = 16 * 1024;
    if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
        HAL_DANGER("night", "Error:  Can't set stack size %zu\n", new_stacksize);
    // Set the flag before starting the thread to avoid a race where the thread
    // checks `while (keepRunning && nightOn)` before `nightOn` is set.
    nightOn = 1;
    if (pthread_create(&nightPid, &thread_attr, (void *(*)(void *))night_thread, NULL) != 0) {
        nightOn = 0;
        pthread_attr_destroy(&thread_attr);
        return EXIT_FAILURE;
    }
    if (pthread_attr_setstacksize(&thread_attr, stacksize))
        HAL_DANGER("night", "Error:  Can't set stack size %zu\n", stacksize);
    pthread_attr_destroy(&thread_attr);

    return ret;
}

void disable_night(void) {
    // Always restore non-grayscale output when disabling night mode support,
    // even if the thread is already stopped.
    if (!nightOn) {
        night_grayscale(false);
        // Ensure we return to DAY hardware state when disabling night support.
        night_irled(false);
        night_ircut(true);
        night_reset_state();
        return;
    }

    nightOn = 0;
    pthread_join(nightPid, NULL);
    night_grayscale(false);
    night_irled(false);
    night_ircut(true);
    night_reset_state();
}