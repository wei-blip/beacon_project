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

#ifdef __cplusplus
extern "C" {
#endif

#define STRIP_NODE		DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)

#define STRIP_INDICATION_TIMEOUT_MIN 1

#define STANDARD_MODE_INDICATION 1
#define ALARM_MODE_INDICATION 2

#define HOMEWARD_LED_LEN 1
#define RSSI_LED_LEN 8
#define DISABLE_ALARM_LED_LEN 1
#define WORKERS_LED_LEN (STRIP_NUM_PIXELS - RSSI_LED_LEN - DISABLE_ALARM_LED_LEN - HOMEWARD_LED_LEN)

#define NUM_OF_RED_LEDS 2
#define NUM_OF_GREEN_LEDS (RSSI_LED_LEN-NUM_OF_RED_LEDS)

#define BLINK_PERIOD_MS 100

/**
 * Enum area begin
 * */
enum COMMON_STRIP_COLOR_e {
  COMMON_STRIP_COLOR_RED,
  COMMON_STRIP_COLOR_GREEN,
  COMMON_STRIP_COLOR_BLUE,
  COMMON_STRIP_COLOR_PURPLE,
  COMMON_STRIP_COLOR_YELLOW
};

enum INDICATION_TYPE_e {
  INDICATION_TYPE_STATIC_COLOR,
  INDICATION_TYPE_STATUS_INFO,
  INDICATION_TYPE_BLINK
};
/**
 * Enum area end
 * */


/**
 * Structs area begin
 * */
struct status_info_s {
  atomic_t con_status;
  atomic_t people_num;
};

struct strip_param_s {
  uint8_t color;
  uint8_t blink_cnt;
};

union led_strip_state_u {
  struct status_info_s status;
  struct strip_param_s strip_param;
};

struct led_strip_indicate_s {
  uint8_t indication_type;
  uint8_t start_led_pos;
  uint8_t end_led_pos;
  union led_strip_state_u led_strip_state;
};
/**
 * Structs area end
 * */

/**
 * Kernel services begin
 * */
extern struct k_msgq msgq_led_strip;
extern struct k_poll_signal signal_indicate;
extern struct k_poll_event event_indicate;
extern const k_tid_t update_indication_task_id;

void update_indication_task(void);

//static inline void set_ind(struct led_strip_indicate_s **strip_ind)
//{
//    k_msgq_put(&msgq_led_strip, strip_ind, K_NO_WAIT);
//}


/* Return true if enabled */
static inline unsigned int indicate_is_enabled()
{
    static unsigned int signaled = 0;
    static int result = 0;
    k_poll_signal_check(&signal_indicate, &signaled, &result);
    return signaled;
}

static inline void enable_ind()
{
    k_poll_signal_raise(&signal_indicate, STANDARD_MODE_INDICATION);
}

static inline void disable_ind()
{
    event_indicate.signal->signaled = 0;
    event_indicate.state = K_POLL_STATE_NOT_READY;
    k_msgq_purge(&msgq_led_strip);
}

#ifdef __cplusplus
}
#endif
/**
 * Kernel services end
 * */
#endif //WS2812_PWM_INDICATION_H
