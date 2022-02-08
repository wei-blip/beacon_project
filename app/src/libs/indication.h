//
// Created by rts on 19.01.2022.
//

#ifndef WS2812_PWM_INDICATION_H
#define WS2812_PWM_INDICATION_H

#include <errno.h>
#include <string.h>

#include <drivers/led_strip.h>
#include <led_utils/led_utils.h>
#include <sys/util.h>

#define NUM_OF_GREEN_LEDS 5
#define NUM_OF_RED_LEDS 3

//// Typedef area begin ////
typedef struct connection_strip_hsv {
    struct led_hsv* green_hsv;
//    struct led_hsv* yellow_hsv;
//    struct led_hsv* maroon_hsv;
    struct led_hsv* red_hsv;

}connection_strip_hsv_t;
//// Typedef area end ////

//// Function declaration begin ////
void connection_quality_pixels(uint8_t con_status);

void number_of_people_in_zone_pixels(uint8_t people_num);

void update_indication(uint8_t people_num, bool set_people_num,uint8_t con_status, bool set_con_status);
//// Function declaration end ////

#endif //WS2812_PWM_INDICATION_H
