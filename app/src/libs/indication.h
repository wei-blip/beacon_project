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

#define STRIP_NODE		DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)

#define HOMEWARD_LED_LEN 1
#define RSSI_LED_LEN 7
#define DISABLE_ALARM_LED_LEN 1
#define WORKERS_LED_LEN (STRIP_NUM_PIXELS - RSSI_LED_LEN - DISABLE_ALARM_LED_LEN - HOMEWARD_LED_LEN)

#define NUM_OF_RED_LEDS 2
#define NUM_OF_GREEN_LEDS (RSSI_LED_LEN-NUM_OF_RED_LEDS)


//// Structs area begin ////
struct led_strip_state_s {
    bool homeward;
    bool disable_alarm;
    uint8_t con_status;
    uint8_t people_num;
};
//// Structs area end ////

extern struct led_strip_state_s led_strip_state;

//// Function declaration begin ////
void set_con_status_pixels(uint8_t con_status, uint8_t *pos);

void set_people_num_pixels(uint8_t people_num, uint8_t *pos);

void update_indication(struct led_strip_state_s *strip_state, bool set_con_status, bool set_people_num,
        bool homeward, bool disable_alarm);
//// Function declaration end ////

#endif //WS2812_PWM_INDICATION_H
