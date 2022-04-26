//
// Created by rts on 19.01.2022.
//

#include "indication_rr/indication.h"
#include <zephyr.h>
#include <device.h>

const struct device *strip_dev = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels_rgb[STRIP_NUM_PIXELS] = {0};
static struct led_rgb empty_rgb[STRIP_NUM_PIXELS] = {0};


K_MSGQ_DEFINE(msgq_led_strip, sizeof(struct led_strip_indicate_s *), 10, 1);
struct k_poll_signal signal_indicate = {0};
struct k_poll_event event_indicate = {0};

/**
 * Struct with using colors begin
 * */
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
/**
 * Struct with using colors end
 * */


/**
 * Function declaration begin
 * */
static inline void set_con_status_pixels(uint8_t con_status, uint8_t *pos);
static inline void set_people_num_pixels(uint8_t people_num, uint8_t *pos);
/**
 * Function declaration end
 * */


/**
 * Function definition begin
 * */
static inline void set_con_status_pixels(uint8_t con_status, uint8_t *pos)
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


static inline void set_people_num_pixels(uint8_t people_num, uint8_t *pos)
{
    uint8_t start_pos = (*pos);
    while (*pos < start_pos + WORKERS_LED_LEN) {
        if (((*pos) - start_pos) < people_num)
            led_hsv2rgb(&blue_hsv, &pixels_rgb[*pos]);
        else
            led_hsv2rgb(&empty_hsv, &pixels_rgb[*pos]);
        (*pos)++;
    }
}


_Noreturn void update_indication_task(void)
{
    uint8_t cnt = 0;
    uint8_t start_led_pos = 0;
    uint8_t end_led_pos = 0;
    uint8_t indication_type = 0;
    struct led_hsv color_hsv = {0};
    struct led_rgb color_rgb[STRIP_NUM_PIXELS] = {0};
    struct led_strip_indicate_s *strip_indicate = {0};
    union led_strip_state_u led_strip_state = {0};

    k_poll_signal_init(&signal_indicate);
    k_poll_event_init(&event_indicate,_POLL_TYPE_SIGNAL,
                      K_POLL_MODE_NOTIFY_ONLY,
                      &signal_indicate);

    if (!device_is_ready(strip_dev)) {
        k_sleep(K_FOREVER);
    }

    while(1) {
        if (k_msgq_num_used_get(&msgq_led_strip)) {
            k_msgq_get(&msgq_led_strip, &strip_indicate, K_NO_WAIT);
            led_strip_state = strip_indicate->led_strip_state;
            start_led_pos = strip_indicate->start_led_pos;
            end_led_pos = strip_indicate->end_led_pos;
            indication_type = strip_indicate->indication_type;

            /* If "blink" set -> we blinked
             * Esle -> set on strip connection quality and people number */
            switch (indication_type) {
                case INDICATION_TYPE_STATUS_INFO:
                    cnt = start_led_pos;
                    if (led_strip_state.status.con_status >= 0)
                        set_con_status_pixels(led_strip_state.status.con_status, &cnt);
                    else
                        cnt += STRIP_NUM_PIXELS/2;

                    if (led_strip_state.status.people_num >= 0)
                        set_people_num_pixels(led_strip_state.status.people_num, &cnt);
                    else
                        cnt += STRIP_NUM_PIXELS/2;

                    led_strip_update_rgb(strip_dev, pixels_rgb, end_led_pos);
                    break;
                case INDICATION_TYPE_BLINK:
                    switch (led_strip_state.strip_param.color) {
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

                    cnt = 0;
                    while (cnt < end_led_pos) {
                        led_hsv2rgb(&empty_hsv, &color_rgb[cnt]);
                        if ((cnt >= start_led_pos) && (cnt <= end_led_pos))
                            led_hsv2rgb(&color_hsv, &color_rgb[cnt]);
                        cnt++;
                    }

                    cnt = 0;
                    while (cnt < led_strip_state.strip_param.blink_cnt) {
                        led_strip_update_rgb(strip_dev, color_rgb, end_led_pos);
                        k_sleep(K_MSEC(BLINK_PERIOD_MS));
                        led_strip_update_rgb(strip_dev, empty_rgb, end_led_pos);
                        k_sleep(K_MSEC(BLINK_PERIOD_MS));
                        cnt++;
                    }
                    led_strip_update_rgb(strip_dev, pixels_rgb, end_led_pos);
                    break;
                case INDICATION_TYPE_STATIC_COLOR:
                    switch (led_strip_state.strip_param.color) {
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

                    cnt = 0;
                    while (cnt < end_led_pos) {
                        led_hsv2rgb(&empty_hsv, &color_rgb[cnt]);
                        if ((cnt >= start_led_pos) && (cnt <= end_led_pos))
                            led_hsv2rgb(&color_hsv, &color_rgb[cnt]);
                        cnt++;
                    }

                    led_strip_update_rgb(strip_dev, color_rgb, end_led_pos);
                    break;
            }

        } else {
            k_sleep(K_MSEC(10));
        }
    }
}
/**
 * Function definition end
 * */
