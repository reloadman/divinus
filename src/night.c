#include "night.h"

#include <errno.h>
#include <time.h>
#include <string.h>

char nightOn = 0;
static bool grayscale = false, ircut = true, irled = false, whiteled = false, manual = false;
pthread_t nightPid = 0;
static pthread_mutex_t night_mode_mtx = PTHREAD_MUTEX_INITIALIZER;

// Mirror decoding rules from app_config.c.
// Supported encodings:
// - 999: disabled
// - N (0..95): GPIO number
// - any negative value: treated as disabled
static bool decode_cfg_pin(int cfg, int *pin_out) {
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

bool night_grayscale_on(void) { return grayscale; }

bool night_ircut_on(void) { return ircut; }

bool night_irled_on(void) { return irled; }

bool night_whiteled_on(void) { return whiteled; }

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

static unsigned long long night_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000ull + (unsigned long long)ts.tv_nsec / 1000000ull;
}

// Sleep in small chunks so disable_night() can join quickly.
// Avoids blocking HTTP handlers (e.g. /api/night) on long check_interval_s sleeps.
static void night_sleep_ms_interruptible(unsigned int ms) {
    while (keepRunning && nightOn && ms) {
        unsigned int step = (ms > 50) ? 50 : ms;
        usleep(step * 1000);
        ms -= step;
    }
}

void night_grayscale(bool enable) {
    set_grayscale(enable);
    grayscale = enable;
}

void night_ircut(bool enable) {
    int pin1 = 0, pin2 = 0;
    if (!decode_cfg_pin(app_config.ir_cut_pin1, &pin1) ||
        !decode_cfg_pin(app_config.ir_cut_pin2, &pin2)) {
        HAL_WARNING("night", "IR-cut pins not configured, skipping\n");
        ircut = enable;
        return;
    }
    if (pin1 == pin2) {
        HAL_WARNING("night", "IR-cut pins invalid (pin1==pin2), skipping\n");
        ircut = enable;
        return;
    }

    HAL_INFO("night", "IR-cut -> %s (pin1=%u pin2=%u pulse=%uus)\n",
        enable ? "ON(DAY)" : "OFF(IR)",
        (unsigned int)pin1, (unsigned int)pin2,
        app_config.pin_switch_delay_us * 100);

    unsigned int pulse_us = app_config.pin_switch_delay_us * 100;
    int r = gpio_pulse_pair(pin1, !enable, pin2, enable, pulse_us);
    if (r != EXIT_SUCCESS) {
        HAL_WARNING("night", "GPIO pulse failed: (pin1=%u val1=%d) (pin2=%u val2=%d) errno=%d (%s)\n",
            (unsigned int)pin1, !enable, (unsigned int)pin2, enable, errno, strerror(errno));
    } else {
        HAL_INFO("night", "GPIO pulse ok\n");
    }

    ircut = enable;
}

void night_irled(bool enable) {
    int pin = 0;
    if (!decode_cfg_pin(app_config.ir_led_pin, &pin)) {
        HAL_WARNING("night", "IR LED pin not configured, skipping\n");
        irled = enable;
        return;
    }
    int r = gpio_write(pin, enable);
    if (r != EXIT_SUCCESS)
        HAL_WARNING("night", "GPIO write failed: pin=%u val=%d errno=%d (%s)\n",
            (unsigned int)pin, enable, errno, strerror(errno));
    else
        HAL_INFO("night", "GPIO write ok: pin=%u val=%d\n", (unsigned int)pin, enable);
    irled = enable;
}

void night_whiteled(bool enable) {
    int pin = 0;
    if (!decode_cfg_pin(app_config.white_led_pin, &pin)) {
        HAL_WARNING("night", "White LED pin not configured, skipping\n");
        whiteled = enable;
        return;
    }
    int r = gpio_write(pin, enable);
    if (r != EXIT_SUCCESS)
        HAL_WARNING("night", "GPIO write failed: pin=%u val=%d errno=%d (%s)\n",
            (unsigned int)pin, enable, errno, strerror(errno));
    else
        HAL_INFO("night", "GPIO write ok: pin=%u val=%d\n", (unsigned int)pin, enable);
    whiteled = enable;
}

void night_manual(bool enable) { manual = enable; }

void night_mode(bool enable) {
    pthread_mutex_lock(&night_mode_mtx);
    // Avoid log spam + avoid re-pulsing IR-cut coil / toggling encoder params
    // when the requested mode is already applied.
    if (enable) {
        if (night_applied()) { pthread_mutex_unlock(&night_mode_mtx); return; }
    } else {
        if (day_applied()) { pthread_mutex_unlock(&night_mode_mtx); return; }
    }

    HAL_INFO("night", "Changing mode to %s\n", enable ? "NIGHT" : "DAY");
    // Always switch IR hardware when entering/leaving night.
    // Grayscale is applied only if configured at the moment of transition.
    night_ircut(!enable);
    night_irled(enable);
    if (enable) night_grayscale(night_should_grayscale());
    else night_grayscale(false);

    // Re-apply IQ so that [static_*] vs [ir_static_*] sections take effect immediately.
    // Without this, AE/DRC/etc may remain from the previous mode until the next restart.
    int r = media_reload_iq();
    if (r != 0)
        HAL_WARNING("night", "IQ reload failed with %#x (continuing)\n", r);
    pthread_mutex_unlock(&night_mode_mtx);
}

void night_ircut_exercise_startup(void) {
    // Skip if pins are not configured.
    int pin1 = 0, pin2 = 0;
    if (!decode_cfg_pin(app_config.ir_cut_pin1, &pin1) ||
        !decode_cfg_pin(app_config.ir_cut_pin2, &pin2)) {
        HAL_INFO("night", "IR-cut exercise skipped (pins not configured)\n");
        return;
    }
    if (pin1 == pin2) {
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

    // NOTE: Do NOT call night_grayscale()/set_grayscale() here.
    // This startup "unstick" routine runs before start_sdk(), and on some platforms
    // grayscale toggling touches ISP/encoder state that isn't initialized yet,
    // causing a crash. Grayscale will be applied later by night mode logic after SDK init.
    // Ensure IR LED is off (function is safe when not configured).
    night_irled(false);

    gpio_deinit();

    // Ensure our cached state is consistent: DAY.
    night_reset_state();
}

void *night_thread(void) {
    gpio_init();
    usleep(10000);

    // Sync state on startup based on the selected source (ISP/ADC/GPIO) when possible.

    // ISO-based ISP switching (preferred): uses hysteresis + "probe" to avoid IR LED brightness
    // falsely triggering a switch back to DAY.
    if (app_config.isp_iso_low >= 0 && app_config.isp_iso_hi >= 0) {
        if (app_config.isp_iso_hi <= app_config.isp_iso_low) {
            HAL_WARNING("night",
                "ISP ISO thresholds invalid (isp_iso_low=%d isp_iso_hi=%d), ignoring\n",
                app_config.isp_iso_low, app_config.isp_iso_hi);
            while (keepRunning && nightOn) night_sleep_ms_interruptible(1000);
        } else {
            const unsigned int lockout_s = app_config.isp_switch_lockout_s;
            const unsigned int probe_settle_ms = 1200; // allow AE to re-converge after toggling IR
            const int exptime_low = app_config.isp_exptime_low;
            HAL_INFO("night",
                "Using ISP ISO source: low=%d hi=%d exptime_low=%d lockout=%us interval=%us\n",
                app_config.isp_iso_low, app_config.isp_iso_hi, exptime_low,
                lockout_s, app_config.check_interval_s);

            unsigned long long last_switch_ms = 0;

            // Apply immediately once at start.
            {
                unsigned int iso=0, exptime=0, again=0, dgain=0, ispdgain=0; int ismax=0;
                if (get_isp_exposure_info(&iso, &exptime, &again, &dgain, &ispdgain, &ismax) == EXIT_SUCCESS && !manual) {
                    if ((int)iso >= app_config.isp_iso_hi) { night_mode(true); last_switch_ms = night_now_ms(); }
                    else if ((int)iso <= app_config.isp_iso_low) { night_mode(false); last_switch_ms = night_now_ms(); }
                }
            }

            while (keepRunning && nightOn) {
                unsigned int iso=0, exptime=0, again=0, dgain=0, ispdgain=0; int ismax=0;
                if (get_isp_exposure_info(&iso, &exptime, &again, &dgain, &ispdgain, &ismax) == EXIT_SUCCESS) {
                    if (!manual) {
                        unsigned long long now = night_now_ms();
                        bool in_ir = night_mode_on();

                        // Enter IR when in DAY and ISO rises above high threshold.
                        if (!in_ir && (int)iso >= app_config.isp_iso_hi) {
                            if (lockout_s == 0 || now - last_switch_ms >= (unsigned long long)lockout_s * 1000ull) {
                                night_mode(true);
                                last_switch_ms = now;
                            }
                        }

                        // Exit IR: do a "probe" to measure ambient without IR.
                        // Gate probe by exposure time (AE settled and truly bright), because ISO
                        // will often be very low in IR due to bright IR LEDs.
                        if (in_ir && exptime_low >= 0 && (int)exptime <= exptime_low) {
                            if (lockout_s == 0 || now - last_switch_ms >= (unsigned long long)lockout_s * 1000ull) {
                                // Probe DAY
                                night_mode(false);
                                night_sleep_ms_interruptible(probe_settle_ms);
                                // Sample a few times without IR; if ISO stays above iso_low,
                                // ambient is still dark -> revert to IR.
                                int max_iso2 = -1;
                                for (int i = 0; i < 3; i++) {
                                    unsigned int iso2=0, exptime2=0, again2=0, dgain2=0, ispdgain2=0; int ismax2=0;
                                    if (get_isp_exposure_info(&iso2, &exptime2, &again2, &dgain2, &ispdgain2, &ismax2) == EXIT_SUCCESS) {
                                        if ((int)iso2 > max_iso2) max_iso2 = (int)iso2;
                                    }
                                    night_sleep_ms_interruptible(200);
                                }
                                if (max_iso2 >= 0 && max_iso2 > app_config.isp_iso_low) {
                                    night_mode(true);
                                }
                                last_switch_ms = night_now_ms();
                            }
                        }
                    }
                }
                night_sleep_ms_interruptible(app_config.check_interval_s * 1000u);
            }
        }
    } else if (app_config.isp_lum_low >= 0 && app_config.isp_lum_hi >= 0) {
        if (app_config.isp_lum_hi <= app_config.isp_lum_low) {
            HAL_WARNING("night",
                "ISP luminance thresholds invalid (isp_lum_low=%d isp_lum_hi=%d), ignoring\n",
                app_config.isp_lum_low, app_config.isp_lum_hi);
            while (keepRunning && nightOn) night_sleep_ms_interruptible(1000);
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
                night_sleep_ms_interruptible(app_config.check_interval_s * 1000u);
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
            night_sleep_ms_interruptible((app_config.check_interval_s * 1000u) / 12u);
        }
        if (adc_fd) close(adc_fd);
    } else {
        int pin = 0;
        if (!decode_cfg_pin(app_config.ir_sensor_pin, &pin)) {
            while (keepRunning && nightOn) night_sleep_ms_interruptible(1000);
        } else {
            // Apply immediately once at start (do not wait check_interval_s).
            {
                bool state = false;
                if (gpio_read(pin, &state) == EXIT_SUCCESS && !manual) {
                    night_mode(state);
                }
            }
            while (keepRunning && nightOn) {
                bool state = false;
                if (gpio_read(pin, &state) != EXIT_SUCCESS) {
                    night_sleep_ms_interruptible(app_config.check_interval_s * 1000u);
                    continue;
                }
                if (!manual) night_mode(state);
                night_sleep_ms_interruptible(app_config.check_interval_s * 1000u);
            }
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

    // Sync manual mode from config on (re)start so it is effective immediately after boot
    // and also after any thread restarts triggered via /api/night.
    night_manual(app_config.night_mode_manual);

    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    size_t stacksize;
    pthread_attr_getstacksize(&thread_attr, &stacksize);
    size_t new_stacksize = (size_t)app_config.night_thread_stack_size;
#ifdef PTHREAD_STACK_MIN
    if (new_stacksize < (size_t)PTHREAD_STACK_MIN)
        new_stacksize = (size_t)PTHREAD_STACK_MIN;
#endif
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