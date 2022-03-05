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
struct status_info_s {
    uint8_t con_status;
    uint8_t people_num;
    bool set_con_status;
    bool set_people_num;
};

struct blink_param_s {
    k_timeout_t msec_timeout;
    enum COMMON_STRIP_COLOR_e blink_color;
    uint8_t blink_cnt;
};

union led_strip_state_u {
  struct status_info_s status;
  struct blink_param_s blink_param;
};

struct led_strip_indicate_s {
  union led_strip_state_u led_strip_state;
  bool blink;
};
////// Structs area end ////

//extern struct status_info_s led_strip_state_global;
//extern struct blink_param_s blink_param;
extern struct k_msgq msgq_led_strip;
extern const k_tid_t update_indication_task_id;

//// Function declaration begin ////
//void blink(struct k_work *item);
//
//void set_color(enum COMMON_STRIP_COLOR_e color);
//
//void set_blink_param(enum COMMON_STRIP_COLOR_e blink_color, k_timeout_t msec_timeout, uint8_t blink_cnt);

//void update_indication(void);
//// Function declaration end ////

#endif //WS2812_PWM_INDICATION_H
