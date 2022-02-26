//
// Created by rts on 19.01.2022.
//

#include "indication.h"
#include <zephyr.h>
#include <device.h>

const struct device *strip_dev = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels_rgb[STRIP_NUM_PIXELS] = {0};
static struct led_rgb empty_pixels_rgb[STRIP_NUM_PIXELS] = {0};

atomic_t led_strip_busy = ATOMIC_INIT(0);
K_MUTEX_DEFINE(mut_led_strip_busy); // Mutex for led_strip
K_MUTEX_DEFINE(mut_blink_param);

struct led_strip_state_s led_strip_state = {0};
struct blink_param_s blink_param = {0};

//// Struct with using colors begin ////
const static struct led_hsv blue_hsv = {
        .h = 240,
        .s = 255,
        .v = 30
};

const static struct led_hsv green_hsv = {
        .h = 120,
        .s = 255,
        .v = 30
};

const static struct led_hsv red_hsv = {
        .h = 0,
        .s = 255,
        .v = 30
};

const static struct led_hsv purple_hsv = {
        .h = 300,
        .s = 255,
        .v = 50
};

const static struct led_hsv yellow_hsv = {
        .h = 60,
        .s = 255,
        .v = 100
};

const static struct led_hsv empty_hsv = {
        .h = 0,
        .s = 0,
        .v = 0,
};
//// Struct with using colors end ////


//// Function definition begin ////
void set_blink_param(enum COMMON_STRIP_COLOR_e blink_color, k_timeout_t msec_timeout, uint8_t blink_cnt)
{
    k_mutex_lock(&mut_blink_param, K_FOREVER);
    blink_param.blink_color = blink_color;
    blink_param.msec_timeout = msec_timeout;
    blink_param.blink_cnt = blink_cnt;
    k_mutex_unlock(&mut_blink_param);
}


void blink(struct k_work *item)
{
    uint8_t i = 0;
    struct led_hsv color_hsv = {0};
    static struct led_rgb blink_pixels_rgb[STRIP_NUM_PIXELS] = {0};

    k_mutex_lock(&mut_blink_param, K_FOREVER);
    enum COMMON_STRIP_COLOR_e blink_color = blink_param.blink_color;
    k_timeout_t msec_timeout = blink_param.msec_timeout;
    uint8_t blink_cnt = blink_param.blink_cnt;
    k_mutex_unlock(&mut_blink_param);

    switch (blink_color) {
        case COMMON_STRIP_COLOR_RED:
            color_hsv = red_hsv;
            break;
        case COMMON_STRIP_COLOR_GREEN:
            color_hsv = green_hsv;
            break;
        case COMMON_STRIP_COLOR_BLUE:
            color_hsv = blue_hsv;
            break;
        case COMMON_STRIP_COLOR_PURPLE:
            color_hsv = purple_hsv;
            break;
        default:
            break;
    }

    while (i < STRIP_NUM_PIXELS) {
        led_hsv2rgb(&color_hsv, &blink_pixels_rgb[i++]);
    }

    i = 0;
    k_mutex_lock(&mut_led_strip_busy, K_FOREVER);
    while (i < blink_cnt) {
//        printk(" Blinked\n");
        led_strip_update_rgb(strip_dev, blink_pixels_rgb, STRIP_NUM_PIXELS);
        k_sleep(msec_timeout);
        led_strip_update_rgb(strip_dev, empty_pixels_rgb, STRIP_NUM_PIXELS);
        k_sleep(msec_timeout);
        i++;
    }

    led_strip_update_rgb(strip_dev, pixels_rgb, STRIP_NUM_PIXELS);
    k_mutex_unlock(&mut_led_strip_busy);
//    printk(" Stop blinked\n");
}


void stop_blink(void)
{
//    printk(" stop_blink begin\n");
    atomic_set(&led_strip_busy, 0);
//    printk(" stop_blink end\n");
}


void set_color(enum COMMON_STRIP_COLOR_e color)
{
    uint8_t cnt = 0;
    struct led_rgb color_rgb[STRIP_NUM_PIXELS] = {0};
    struct led_hsv color_hsv = {0};

    switch (color) {
        case COMMON_STRIP_COLOR_RED:
            color_hsv = red_hsv;
            break;
        case COMMON_STRIP_COLOR_GREEN:
            color_hsv = green_hsv;
            break;
        case COMMON_STRIP_COLOR_BLUE:
            color_hsv = blue_hsv;
            break;
        case COMMON_STRIP_COLOR_PURPLE:
            color_hsv = purple_hsv;
            break;
        case COMMON_STRIP_COLOR_YELLOW:
            color_hsv = yellow_hsv;
            break;
        default:
            break;
    }

    while (cnt < STRIP_NUM_PIXELS) {
        led_hsv2rgb(&color_hsv, &color_rgb[cnt++]);
    }

    led_strip_update_rgb(strip_dev, color_rgb, STRIP_NUM_PIXELS);
}


void set_con_status_pixels(uint8_t con_status, uint8_t *pos)
{
    uint8_t start_pos = (*pos);
    while (*pos < start_pos + RSSI_LED_LEN) {
        if (*pos < start_pos + con_status) {
            if (*pos < start_pos + NUM_OF_RED_LEDS)
                led_hsv2rgb(&red_hsv, &pixels_rgb[(*pos)]);
            else
                led_hsv2rgb(&green_hsv, &pixels_rgb[(*pos)]);
        } else {
            led_hsv2rgb(&empty_hsv, &pixels_rgb[(*pos)]);
        }
        (*pos)++;
    }
}


void set_people_num_pixels(uint8_t people_num, uint8_t *pos)
{
    uint8_t start_pos = (*pos);
    while (*pos < start_pos + WORKERS_LED_LEN) {
        (people_num & BIT((*pos)-start_pos)) ? led_hsv2rgb(&blue_hsv, &pixels_rgb[ (*pos)])
            : led_hsv2rgb(&empty_hsv, &pixels_rgb[ (*pos)]);
        (*pos)++;
    }
}


void update_indication(struct led_strip_state_s *strip_state, bool set_con_status, bool set_people_num)
{
    if ((!set_people_num) && (!set_con_status))
        return;

    uint8_t pos = 0;

    if (set_con_status)
        set_con_status_pixels(strip_state->con_status, &pos);

    if (set_people_num)
        set_people_num_pixels(strip_state->people_num, &pos);

    k_mutex_lock(&mut_led_strip_busy, K_FOREVER);
    led_strip_update_rgb(strip_dev, pixels_rgb, STRIP_NUM_PIXELS);
    k_mutex_unlock(&mut_led_strip_busy);
}
//// Function definition end ////
