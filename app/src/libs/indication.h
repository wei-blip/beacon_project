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

#define BLINKED_COUNT 5

#define HOMEWARD_LED_LEN 1
#define RSSI_LED_LEN 7
#define DISABLE_ALARM_LED_LEN 1
#define WORKERS_LED_LEN (STRIP_NUM_PIXELS - RSSI_LED_LEN - DISABLE_ALARM_LED_LEN - HOMEWARD_LED_LEN)

#define NUM_OF_RED_LEDS 2
#define NUM_OF_GREEN_LEDS (RSSI_LED_LEN-NUM_OF_RED_LEDS)


/// Enum area begin
enum COMMON_STRIP_COLOR_e {
  COMMON_STRIP_COLOR_RED,
  COMMON_STRIP_COLOR_GREEN,
  COMMON_STRIP_COLOR_BLUE,
  COMMON_STRIP_COLOR_PURPLE,
  COMMON_STRIP_COLOR_YELLOW
};
/// Enum area end


////// Structs area begin ////
struct led_strip_state_s {
//    bool homeward;
//    bool disable_alarm;
    uint8_t con_status;
    uint8_t people_num;
};

struct blink_param_s {
    k_timeout_t msec_timeout;
    enum COMMON_STRIP_COLOR_e blink_color;
    uint8_t blink_cnt;
};
////// Structs area end ////

extern struct led_strip_state_s led_strip_state;
extern struct blink_param_s blink_param;

//// Function declaration begin ////
void blink(struct k_work *item);

void set_color(enum COMMON_STRIP_COLOR_e color);

void set_blink_param(enum COMMON_STRIP_COLOR_e blink_color, k_timeout_t msec_timeout, uint8_t blink_cnt);

void set_con_status_pixels(uint8_t con_status, uint8_t *pos);

void set_people_num_pixels(uint8_t people_num, uint8_t *pos);

void update_indication(struct led_strip_state_s *strip_state, bool set_con_status, bool set_people_num);
//// Function declaration end ////

#endif //WS2812_PWM_INDICATION_H
