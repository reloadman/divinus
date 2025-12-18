#include "night.h"

char nightOn = 0;
static bool grayscale = false, ircut = true, irled = false, manual = false;
pthread_t nightPid = 0;

bool night_grayscale_on(void) { return grayscale; }

bool night_ircut_on(void) { return ircut; }

bool night_irled_on(void) { return irled; }

bool night_manual_on(void) { return manual; }

bool night_mode_on(void) { return grayscale && !ircut && irled; }

void night_grayscale(bool enable) {
    set_grayscale(enable);
    grayscale = enable;
}

void night_ircut(bool enable) {
    gpio_write(app_config.ir_cut_pin1, !enable);
    gpio_write(app_config.ir_cut_pin2, enable);
    usleep(app_config.pin_switch_delay_us * 100);
    gpio_write(app_config.ir_cut_pin1, false);
    gpio_write(app_config.ir_cut_pin2, false);
    ircut = enable;
}

void night_irled(bool enable) {
    gpio_write(app_config.ir_led_pin, enable);
    irled = enable;
}

void night_manual(bool enable) { manual = enable; }

void night_mode(bool enable) {
    HAL_INFO("night", "Changing mode to %s\n", enable ? "NIGHT" : "DAY");
    night_grayscale(enable);
    night_ircut(!enable);
    night_irled(enable);
}

void *night_thread(void) {
    gpio_init();
    usleep(10000);

    night_mode(night_mode_on());

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

    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    size_t stacksize;
    pthread_attr_getstacksize(&thread_attr, &stacksize);
    size_t new_stacksize = 16 * 1024;
    if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
        HAL_DANGER("night", "Error:  Can't set stack size %zu\n", new_stacksize);
    pthread_create(&nightPid, &thread_attr, (void *(*)(void *))night_thread, NULL);
    if (pthread_attr_setstacksize(&thread_attr, stacksize))
        HAL_DANGER("night", "Error:  Can't set stack size %zu\n", stacksize);
    pthread_attr_destroy(&thread_attr);

    nightOn = 1;

    return ret;
}

void disable_night(void) {
    if (!nightOn) return;

    nightOn = 0;
    pthread_join(nightPid, NULL);
}