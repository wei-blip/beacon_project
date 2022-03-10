//
// Created by rts on 05.02.2022.
//
#ifndef RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H
#define RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H

#include <devicetree.h>
#include <device.h>
#include <errno.h>
#include <drivers/lora.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>

#include "message_format.h"
#include "indication.h"

/**
 * Available devices begin
 * */
#define SIGNALMAN 0
#define BASE_STATION 1
#define BRIGADE_CHIEF 2

#define CUR_DEVICE BASE_STATION
/**
 * Available devices end
 * */

#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

#define PWM_SOUND	DT_ALIAS(pwm_sound)

#if DT_NODE_HAS_STATUS(PWM_SOUND, okay)
#define PWM_CTLR	DT_PWMS_CTLR(PWM_SOUND)
#define PWM_CHANNEL	DT_PWMS_CHANNEL(PWM_SOUND)
#define PWM_FLAGS	DT_PWMS_FLAGS(PWM_SOUND)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR	DT_INVALID_NODE
#define PWM_CHANNEL	0
#define PWM_FLAGS	0
#endif

#if CUR_DEVICE == BASE_STATION
#elif CUR_DEVICE == SIGNALMAN

#define CURRENT_DEVICE_NUM 1

#define ALARM_NODE	DT_ALIAS(alarm_sw)
#if !DT_NODE_HAS_STATUS(ALARM_NODE, okay)
#error "Unsupported board: alarm_sw devicetree alias is not defined"
#endif

/*TODO: Uncomment this*/
//#define ANTI_DREAM_NODE	DT_ALIAS(anti_dream_sw)
//#if !DT_NODE_HAS_STATUS(ANTI_DREAM_NODE, okay)
//#error "Unsupported board: anti_dream_sw devicetree alias is not defined"
//#endif
//
//#define TRAIN_PASSED_NODE DT_ALIAS(train_passed_sw)
//#if !DT_NODE_HAS_STATUS(TRAIN_PASSED_NODE, okay)
//#error "Unsupported board: train_passed_sw alias is not defined"
//#endif

#elif CUR_DEVICE == BRIGADE_CHIEF

#define CURRENT_DEVICE_NUM 2

#define DISABLE_ALARM_NODE	DT_ALIAS(disable_alarm_sw)
#if !DT_NODE_HAS_STATUS(DISABLE_ALARM_NODE, okay)
#error "Unsupported board: disable_alarm_sw devicetree alias is not defined"
#endif

//#define LEFT_TRAIN_PASSED_NODE	DT_ALIAS(left_train_passed_sw)
//#if !DT_NODE_HAS_STATUS(LEFT_TRAIN_PASSED_NODE, okay)
//#error "Unsupported board: left_train_passed_sw devicetree alias is not defined"
//#endif
//
//#define RIGHT_TRAIN_PASSED_NODE DT_ALIAS(right_train_passed_sw)
//#if !DT_NODE_HAS_STATUS(RIGHT_TRAIN_PASSED_NODE, okay)
//#error "Unsupported board: right_train_passed_sw alias is not defined"
//#endif

#endif

#define QUEUE_LEN_IN_ELEMENTS 10
#define WAITING_PERIOD_NUM 2

#define SLOT_TIME_MSEC 764UL /* Time on receive(664ms) plus DELAY_TIME_MSEC */
#define PERIOD_TIME_MSEC (4*SLOT_TIME_MSEC)
#define DELAY_TIME_MSEC 100U
#define STOCK_TIME_MSEC 10
#define DURATION_TIME_MSEC (SLOT_TIME_MSEC*(CURRENT_DEVICE_NUM-1))

#define BUTTON_PRESSED_PERIOD_TIME_USEC 40000UL

#define IS_SYNC_MSG (rx_buf[0] == 13) /* SENDER_ADDR = BASE_STATION, RECV_ADDR = BROADCAST, MESSAGE_TYPE = SYNC  */
#define DISABLE_INDICATE (indicate_cnt == 5)

/*
 * This macro using in processing thread
 * rx_buf_proc - local variable in processing thread
 * Array indexes have range from 0 to MESSAGE_LEN_IN_BYTES-1
 * */
#define IS_EMPTY_MSG ((rx_buf_proc[0] == 0) && (rx_buf_proc[MESSAGE_LEN_IN_BYTES-1] == 0))

/*
 * atomic_interval_count value for detected short pressed
 * If fun_call_count value will be greater than this macros then it long pressed
 * Interval time = 100 ms
 * */
#define INTERVAL_TIME_MS 100
#define SHORT_PRESSED_MIN_VAL 2  /* 200 ms */
#define SHORT_PRESSED_MAX_VAL 10 /* 1000 ms */
#define MIDDLE_PRESSED_MIN_VAL (SHORT_PRESSED_MAX_VAL+1)  /* 1100 ms */
#define MIDDLE_PRESSED_MAX_VAL 20   /* 2000 ms */
#define LONG_PRESSED_MIN_VAL (MIDDLE_PRESSED_MAX_VAL+1) /* 2100 ms */

extern atomic_t atomic_interval_count; /* Counted number of function button pressed call */

/**
 * Enum, typedefs and structs area begin
 * */
enum CONNECTION_QUALITY_RSSI {
    CONNECTION_QUALITY_RSSI_1 = -70,
    CONNECTION_QUALITY_RSSI_2 = -80,
    CONNECTION_QUALITY_RSSI_3 = -90,
    CONNECTION_QUALITY_RSSI_4 = -100,
    CONNECTION_QUALITY_RSSI_5 = -105,
    CONNECTION_QUALITY_RSSI_6 = -125,
    CONNECTION_QUALITY_RSSI_7 = -127,
    CONNECTION_QUALITY_RSSI_8 = -130
};


enum LIGHT_UP_LEDS {
    LIGHT_UP_ZERO = 0,
    LIGHT_UP_ONE = 1,
    LIGHT_UP_TWO = 2,
    LIGHT_UP_THREE = 3,
    LIGHT_UP_FOUR = 4,
    LIGHT_UP_FIVE = 5,
    LIGHT_UP_SIX = 6,
    LIGHT_UP_SEVEN = 7,
    LIGHT_UP_EIGHT = 8
};


enum MODEM_STATES {
    RECEIVE = 0,
    TRANSMIT = 1
};


typedef struct modem_state_s {
    struct modem_state_s* next;
    enum MODEM_STATES state;
} modem_state_t;


struct buzzer_mode_s {
    bool continuous;
    bool single;
    bool ding_dong;
};

struct msg_info_s {
    atomic_t req_is_send;
    atomic_t resp_is_recv;
    struct k_msgq *msg_buf;
    struct message_s *msg;

};
/**
 * Enum, typedefs and structs area end
 * */


/**
 * Extern variable declaration area begin
 * */

/*
 * Buttons begin
 * */
extern const struct gpio_dt_spec button_disable_alarm;
extern const struct gpio_dt_spec button_right_train_passed;
extern const struct gpio_dt_spec button_left_train_passed;

extern const struct gpio_dt_spec button_alarm;
extern const struct gpio_dt_spec button_anti_dream;
extern const struct gpio_dt_spec button_train_passed;
/*
 * Buttons end
 * */

extern struct lora_modem_config lora_cfg;

extern const struct device *lora_dev_ptr;
extern const struct device *buzzer_dev_ptr;

extern struct gpio_dt_spec *cur_irq_gpio_ptr;

extern struct k_timer periodic_timer; /* For switching in tx mode */

extern struct k_work work_buzzer; /* For signalisation */
extern struct k_work work_button_pressed;

extern struct k_mutex mut_buzzer_mode; /* Block buzzer_mode */

extern struct buzzer_mode_s buzzer_mode;

extern struct message_s tx_msg;

extern struct k_msgq msgq_tx_msg_prio;
extern struct k_msgq msgq_tx_msg;
extern struct k_msgq msgq_rx_msg;
extern struct k_msgq msgq_rssi;

extern const modem_state_t recv_state;
extern const modem_state_t transmit_state;

extern uint8_t tx_buf[MESSAGE_LEN_IN_BYTES];
extern uint8_t rx_buf[MESSAGE_LEN_IN_BYTES];

/* Indicate structure */
extern struct led_strip_indicate_s status_ind;

extern const struct led_strip_indicate_s disable_indication;
extern const struct led_strip_indicate_s middle_pressed_button_ind;
extern const struct led_strip_indicate_s msg_send_good_ind;
extern const struct led_strip_indicate_s msg_send_bad_ind;
extern const struct led_strip_indicate_s msg_recv_ind;
/**
 * Extern variable declaration area end
 * */


/**
 * Function declaration area begin
 * */
void work_button_pressed_handler(struct k_work *item);
/**
 * Function declaration area end
 * */


/**
 * Function definition area begin
 * */
static inline uint8_t reverse(uint8_t input)
{
    uint8_t output;
    uint8_t bit = 0;
    uint8_t pos = 0;
    while( pos < 7 ) {
        bit = input & BIT(0);
        output |= bit;
        output = output << 1;
        input = input >> 1;
        pos++;
    }
    bit = input & BIT(0);
    output |= bit;
    return output;
}


static inline uint8_t check_rssi(int16_t rssi)
{
    if ( rssi >= CONNECTION_QUALITY_RSSI_1 ) {
        return LIGHT_UP_EIGHT;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_1) && (rssi >= CONNECTION_QUALITY_RSSI_2) ) {
        return LIGHT_UP_SEVEN;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_2) && (rssi >= CONNECTION_QUALITY_RSSI_3) ) {
        return LIGHT_UP_SIX;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_3) && (rssi >= CONNECTION_QUALITY_RSSI_4) ) {
        return LIGHT_UP_FIVE;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_4) && (rssi >= CONNECTION_QUALITY_RSSI_5) ) {
        return LIGHT_UP_FOUR;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_5) && (rssi >= CONNECTION_QUALITY_RSSI_6) ) {
        return LIGHT_UP_THREE;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_6) && (rssi >= CONNECTION_QUALITY_RSSI_7) ) {
        return LIGHT_UP_TWO;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_7) && (rssi >= CONNECTION_QUALITY_RSSI_8) ) {
        return LIGHT_UP_ONE;
    }
    else if ( rssi < CONNECTION_QUALITY_RSSI_8 ) {
        return LIGHT_UP_ZERO;
    }
}


static inline void fill_msg_bit_field(uint32_t *msg_ptr, const uint8_t field_val, uint8_t field_len, uint8_t *pos)
{
    uint8_t start_pos = *pos;
    while ( *pos < start_pos + field_len ) {
        *msg_ptr &= ( ~BIT(*pos) ); // clear bit
        *msg_ptr |= ( field_val & BIT((*pos) - start_pos) ) << start_pos;
        (*pos)++;
    }
}


static inline void extract_msg_bit_field(const uint32_t *msg_ptr, uint8_t *field_val, uint8_t field_len, uint8_t *pos)
{
    uint8_t start_pos = *pos;
    while ( *pos < start_pos + field_len ) {
        *field_val &= ( ~BIT((*pos) - start_pos) ); // clear bit
        (*field_val) |= ( (*msg_ptr) & BIT((*pos) ) ) >> start_pos;
        (*pos)++;
    }
}


static inline void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write)
{
    uint8_t pos = 0;
    for (int cur_field = 0; cur_field < MESSAGE_FIELD_NUMBER; ++cur_field) {
        switch (cur_field) {
            case SENDER_ADDR:
                write ? fill_msg_bit_field(new_msg, msg_ptr->sender_addr, SENDER_ADDR_FIELD_LEN, &pos) :
                extract_msg_bit_field(new_msg, &msg_ptr->sender_addr, SENDER_ADDR_FIELD_LEN, &pos);
                break;
            case RECEIVER_ADDR:
                write ? fill_msg_bit_field(new_msg, msg_ptr->receiver_addr, RECEIVER_ADDR_FIELD_LEN, &pos) :
                extract_msg_bit_field(new_msg, &msg_ptr->receiver_addr, RECEIVER_ADDR_FIELD_LEN, &pos);
                break;
            case MESSAGE_TYPE:
                write ? fill_msg_bit_field(new_msg, msg_ptr->message_type, MESSAGE_TYPE_FIELD_LEN, &pos) :
                extract_msg_bit_field(new_msg, &msg_ptr->message_type, MESSAGE_TYPE_FIELD_LEN, &pos);
                break;
            case DIRECTION:
                write ? fill_msg_bit_field(new_msg, msg_ptr->direction, DIRECTION_FIELD_LEN, &pos) :
                extract_msg_bit_field(new_msg, &msg_ptr->direction, DIRECTION_FIELD_LEN, &pos);
                break;
            case BATTERY:
                write ? fill_msg_bit_field(new_msg, msg_ptr->battery_level, BATTERY_FIELD_LEN, &pos) :
                extract_msg_bit_field(new_msg, &msg_ptr->battery_level, BATTERY_FIELD_LEN, &pos);
                break;
            case PEOPLE_IN_SAFE_ZONE:
                write ? fill_msg_bit_field(new_msg, msg_ptr->workers_in_safe_zone, PEOPLE_IN_SAFE_ZONE_FIELD_LEN, &pos) :
                extract_msg_bit_field(new_msg, &msg_ptr->workers_in_safe_zone, PEOPLE_IN_SAFE_ZONE_FIELD_LEN, &pos);
                break;
            default:
                break;
        }
    }
}
/**
 * Function definition area end
 * */

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H
