//
// Created by rts on 19.01.2022.
//

#include "indication.h"
#include <zephyr.h>
#include <device.h>

#define STRIP_NODE		DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)

const struct device *strip_dev = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels_rgb[(STRIP_NUM_PIXELS)] = {0};

//// Struct with using colors begin ////
struct led_hsv blud_hsv = {
        .h = 240,
        .s = 255,
        .v = 20
};

struct led_hsv green_hsv = {
        .h = 70,
        .s = 255,
        .v = 20
};

struct led_hsv red_hsv = {
        .h = 0,
        .s = 255,
        .v = 20
};

struct led_hsv empty_hsv = {
        .h = 0,
        .s = 0,
        .v = 0,
};
//// Struct with using colors end ////

//// Function definition begin ////
void connection_quality_pixels(uint8_t con_status) {
    int i = 0;
    while (i < con_status) {
        if (i < NUM_OF_RED_LEDS) {
            led_hsv2rgb(&red_hsv, &pixels_rgb[i++]);
            continue;
        }
        led_hsv2rgb(&green_hsv, &pixels_rgb[i++]);
    }

    while (i < STRIP_NUM_PIXELS/2) {
        led_hsv2rgb(&empty_hsv, &pixels_rgb[i++]);
    }

}

void number_of_people_in_zone_pixels(uint8_t people_num) {
    for (int i = STRIP_NUM_PIXELS/2; i < STRIP_NUM_PIXELS; ++i) {
        if ( (people_num & BIT(i-STRIP_NUM_PIXELS/2)) ) {
            led_hsv2rgb(&blud_hsv, &pixels_rgb[i]);
            continue;
        }
        led_hsv2rgb(&empty_hsv, &pixels_rgb[i]);
    }
}

void update_indication(uint8_t people_num, bool set_people_num, uint8_t con_status, bool set_con_status) {
    unsigned int key;
    if ( (!set_people_num) && (!set_con_status) )
        return;

    if (set_con_status)
        connection_quality_pixels(con_status);

    if (set_people_num)
        number_of_people_in_zone_pixels(people_num);

    led_strip_update_rgb(strip_dev, pixels_rgb, STRIP_NUM_PIXELS);
}
//// Function definition end ////
