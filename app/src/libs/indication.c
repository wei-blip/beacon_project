//
// Created by rts on 19.01.2022.
//

#include "indication.h"
#include <zephyr.h>
#include <device.h>

const struct device *strip_dev = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels_rgb[STRIP_NUM_PIXELS] = {0};
static struct led_rgb empty_strip[STRIP_NUM_PIXELS] = {0};

//struct led_strip_state_s led_strip_state = {0};

//// Struct with using colors begin ////
const static struct led_hsv blue_hsv = {
        .h = 240,
        .s = 255,
        .v = 30
};

const static struct led_hsv green_hsv = {
        .h = 70,
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

const static struct led_hsv empty_hsv = {
        .h = 0,
        .s = 0,
        .v = 0,
};
//// Struct with using colors end ////


//// Function definition begin ////
void blink_strip(enum BLINKED_COLOR_e color, timeout_t msec_timeout, uint8_t cnt)
{
    uint8_t i = 0;
    struct led_hsv color_hsv = {0};
    static struct pixels_rgb blink_pixels_rgb[STRIP_NUM_PIXELS] = {0};

    switch (color) {
        case BLINK_COLOR_RED:
            color_hsv = red_hsv;
            break;
        case BLINK_COLOR_GREEN:
            color_hsv = green_hsv;
            break;
        case BLINK_COLOR_BLUE:
            color_hsv = blue_hsv;
            break;
        case BLINK_COLOR_PURPLE:
            color_hsv = purple_hsv;
            break;
    }
    while (i < STRIP_NUM_PIXELS) {
        led_hsv2rgb(&color_hsv, &blink_pixels_rgb[i++]);
    }

    i = 0;
    while (i < cnt) {
        led_strip_update_rgb(strip_dev, blink_pixels_rgb, STRIP_NUM_PIXELS);
        k_sleep(msec_timeout);
        led_strip_update_rgb(strip_dev, empty_rgb, STRIP_NUM_PIXELS);
        k_sleep(msec_timeout);
        i++;
    }
    led_strip_update_rgb(strip_dev, pixels_rgb, STRIP_NUM_PIXELS);
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
        (people_num & BIT((*pos)-start_pos)) ? led_hsv2rgb(&blud_hsv, &pixels_rgb[ (*pos)])
            : led_hsv2rgb(&empty_hsv, &pixels_rgb[ (*pos)]);
        (*pos)++;
    }
}


void update_indication(struct led_strip_state_s *strip_state, bool set_con_status, bool set_people_num,
                       bool homeward, bool disable_alarm)
{
    if ( (!set_people_num) && (!set_con_status) && (!homeward) && (!disable_alarm) )
        return;

    uint8_t pos = 0;
/// TODO: homeward and disable alarm should light off over time
    if (homeward)
        strip_state->homeward ? led_hsv2rgb(&purple_hsv, &pixels_rgb[pos++]) :
        led_hsv2rgb(&empty_hsv, &pixels_rgb[pos++]);

    if (set_con_status)
        set_con_status_pixels(strip_state->con_status, &pos);

    if (disable_alarm)
        strip_state->disable_alarm ? led_hsv2rgb(&purple_hsv, &pixels_rgb[pos++]) :
        led_hsv2rgb(&empty_hsv, &pixels_rgb[pos++]);

    if (set_people_num)
        set_people_num_pixels(strip_state->people_num, &pos);

    led_strip_update_rgb(strip_dev, pixels_rgb, STRIP_NUM_PIXELS);
}
//// Function definition end ////
