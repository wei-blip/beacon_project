//
// Created by rts on 05.02.2022.
//

#ifndef RADIO_SIGNALMAN_LORA_RR_COMMON_H
#define RADIO_SIGNALMAN_LORA_RR_COMMON_H

#include <devicetree.h>
#include <device.h>
#include <errno.h>


#include <drivers/lora.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>

#include "message_format.h"
#include "indication_rr/indication.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Enum value equal "active_events" array indexes into lora_rr_common.cc file.
 * */
#define APPLICATION_EVENTS_NUM 3

enum APP_EVENTS_s {
  EVENT_TX_MODE = 0,
  EVENT_PROC_RX_DATA,
  EVENT_RX_MODE
};

#define SLOT_TIME_MSEC 266UL /* Time on receive(166ms) + DELAY_TIME_MSEC */
#define PERIOD_TIME_MSEC (4*SLOT_TIME_MSEC)  /* Timer period (super frame time) */
#define DELAY_TIME_MSEC 100U
#define DURATION_TIME_MSEC (SLOT_TIME_MSEC*(CONFIG_CURRENT_DEVICE_NUM-1)) /* Started delay for each device */

/**
 * Common peripheral settings area begin
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
/**
 * Common peripheral settings area end
 * */

/**
 * My thread ids begin
 * */
extern const k_tid_t proc_task_id;
extern const k_tid_t modem_task_id;
extern const k_tid_t app_task_id;
/**
 * My thread ids end
 * */

/**
 * Devices settings area begin
 * */


/**
 * Thread functions begin
 * */
[[noreturn]] void app_task(void);
/**
 * Thread functions end
 * */

/**
 * Devices settings area end
 * */


/**
 * Buzzer modes begin
 * */
#define BUZZER_MODE_SINGLE 1
#define BUZZER_MODE_CONTINUOUS 2
#define BUZZER_MODE_DING_DONG 3
#define BUZZER_MODE_IDLE 4
/**
* Buzzer modes end
* */

#define QUEUE_LEN_IN_ELEMENTS 10

#define BUTTON_PRESSED_PERIOD_TIME_USEC 40000UL /* PWM period time */

#define INTERVAL_TIME_MS 100
/* Counts interval time for detected pressed */
#define SHORT_PRESSED_MIN_VAL 4  /* 400 ms */
#define SHORT_PRESSED_MAX_VAL 10 /* 1000 ms */
#define MIDDLE_PRESSED_MIN_VAL (SHORT_PRESSED_MAX_VAL+1)  /* 1100 ms */
#define MIDDLE_PRESSED_MAX_VAL 20   /* 2000 ms */
#define LONG_PRESSED_MIN_VAL (MIDDLE_PRESSED_MAX_VAL+1) /* 2100 ms */

extern atomic_t alarm_is_active;

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

enum MODEM_STATES {
    RECEIVE = 0,
    TRANSMIT = 1
};

typedef struct modem_state_s {
    struct modem_state_s* next;
    enum MODEM_STATES state;
} modem_state_t;
/**
 * Enum, typedefs and structs area end
 * */


/**
 * Extern variable declaration area begin
 * */

extern atomic_t reconfig_modem;

extern const modem_state_t recv_state;
extern const modem_state_t transmit_state;
extern modem_state_t current_state;

/* Indicate structure */
extern struct led_strip_indicate_s status_ind;
extern struct led_strip_indicate_s disable_indication;
extern struct led_strip_indicate_s middle_pressed_button_ind;
extern struct led_strip_indicate_s msg_send_good_ind;
extern struct led_strip_indicate_s msg_send_bad_ind;
extern struct led_strip_indicate_s msg_recv_ind;
extern struct led_strip_indicate_s alarm_ind;
/**
 * Extern variable declaration area end
 * */


/**
 * Function declaration area begin
 * */
void work_buzzer_handler(struct k_work *item);
void work_button_pressed_handler(struct k_work *item);
void work_button_pressed_handler_dev(struct gpio_dt_spec *irq_gpio);
void dwork_disable_ind_handler(struct k_work *item);
void periodic_timer_handler(struct k_timer *tim);

void common_kernel_services_init();
void lora_receive_error_timeout(const struct device *dev);
void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr);

bool proc_rx_data(uint8_t *recv_data, size_t len, struct message_s *rx_msg, uint8_t cur_dev_addr);
bool proc_tx_data(k_msgq *msgq, uint8_t *tx_data, size_t len, struct message_s *tx_msg);

bool radio_rx_queue_is_empty();

void set_rssi(int16_t *rssi);
void get_rssi(int16_t *rssi);

void set_msg(struct message_s *msg, bool prio);

void tim_start(k_timeout_t duration, k_timeout_t period);
void tim_restart(k_timeout_t duration, k_timeout_t period);

void set_buzzer_mode(uint8_t buzzer_mode);

void irq_routine(struct gpio_dt_spec *dev);

void set_ind(led_strip_indicate_s **ind, k_timeout_t duration_min);

int8_t wait_app_event();
int8_t get_app_event();

int32_t modem_fun(const struct device *lora_dev, struct lora_modem_config* lora_cfg);
void start_rx(const struct device *lora_dev, struct lora_modem_config* lora_cfg);
void proc_fun(void *dev_data);
void request_analysis(const struct message_s *rx_msg, struct message_s *tx_msg, struct led_strip_indicate_s *strip_ind);
void response_analysis(const struct message_s *rx_msg, struct message_s *tx_msg, struct led_strip_indicate_s *strip_ind);
/**
 * Function declaration area end
 * */


/**
 * Function definition area begin
 * */
static inline bool is_empty_msg(const uint8_t *buf, size_t len)
{
    uint8_t i = 0;
    uint8_t cnt = 0;
    while(i < len) {
        if (!(*(buf + i))) {
            cnt++;
        }
        i++;
    }
    return (cnt==len);
}


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
    uint8_t leds_num = 0;
    if ( rssi >= CONNECTION_QUALITY_RSSI_1 ) {
        leds_num = 8;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_1) && (rssi >= CONNECTION_QUALITY_RSSI_2) ) {
        leds_num = 7;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_2) && (rssi >= CONNECTION_QUALITY_RSSI_3) ) {
        leds_num = 6;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_3) && (rssi >= CONNECTION_QUALITY_RSSI_4) ) {
        leds_num = 5;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_4) && (rssi >= CONNECTION_QUALITY_RSSI_5) ) {
        leds_num = 4;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_5) && (rssi >= CONNECTION_QUALITY_RSSI_6) ) {
        leds_num = 3;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_6) && (rssi >= CONNECTION_QUALITY_RSSI_7) ) {
        leds_num = 2;
        return leds_num;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_7) && (rssi >= CONNECTION_QUALITY_RSSI_8) ) {
        leds_num = 1;
        return leds_num;
    }
    else if ( rssi < CONNECTION_QUALITY_RSSI_8 ) {
        return leds_num;
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


#ifdef __cplusplus
}
#endif

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H
