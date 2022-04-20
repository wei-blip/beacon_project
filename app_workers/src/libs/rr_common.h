//
// Created by rts on 05.02.2022.
//

#ifndef RADIO_SIGNALMAN_LORA_RR_COMMON_H
#define RADIO_SIGNALMAN_LORA_RR_COMMON_H

#include <devicetree.h>
#include <device.h>
#include <errno.h>



/**
 * Available devices begin
 * */
#define SIGNALMAN 0
#define BASE_STATION 1
#define BRIGADE_CHIEF 2
#define WORKERS 3

#define CUR_DEVICE SIGNALMAN
/**
 * Available devices end
 * */

#if CUR_DEVICE != WORKERS
#include <drivers/lora.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>

#include "message_format.h"
#include "indication.h"
#endif

#if (CUR_DEVICE == BASE_STATION) || (CUR_DEVICE == WORKERS)
#include "dwm_rr_common.h"
#endif




#ifdef __cplusplus
extern "C" {
#endif

#define SLOT_TIME_MSEC 765UL /* Time on receive(664ms) + DELAY_TIME_MSEC */
#define PERIOD_TIME_MSEC (4*SLOT_TIME_MSEC)  /* Timer period (super frame time) */
#define DELAY_TIME_MSEC 100U
#define DURATION_TIME_MSEC (SLOT_TIME_MSEC*(CURRENT_DEVICE_NUM-1)) /* Started delay for each device */

#if CUR_DEVICE != WORKERS

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
/**
 * My thread ids end
 * */
#endif

/**
 * Devices settings area begin
 * */
#if CUR_DEVICE == BASE_STATION

extern const k_tid_t dwm_task_id;

/**
 * Base station thread functions begin
 * */
[[noreturn]] void base_station_proc_task(void);
[[noreturn]] void base_station_modem_task(void);
/**
 * Base station thread functions end
 * */

#define CURRENT_DEVICE_NUM 0

#elif CUR_DEVICE == SIGNALMAN
/**
 * Signalman thread functions begin
 * */
[[noreturn]] void signalman_proc_task(void);
[[noreturn]] void signalman_modem_task(void);
/**
 * Signalman thread functions end
 * */

#define CURRENT_DEVICE_NUM 1

/*
 * SW0 - alarm button
 * SW1 - anti-dream button
 * SW2 - train passed button
 * */
#define ALARM_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(ALARM_NODE, okay)
#error "Unsupported board: alarm_sw devicetree alias is not defined"
#endif

#define ANTI_DREAM_NODE	DT_ALIAS(sw1)
#if !DT_NODE_HAS_STATUS(ANTI_DREAM_NODE, okay)
#error "Unsupported board: anti_dream_sw devicetree alias is not defined"
#endif

#define TRAIN_PASSED_NODE DT_ALIAS(sw2)
#if !DT_NODE_HAS_STATUS(TRAIN_PASSED_NODE, okay)
#error "Unsupported board: train_passed_sw alias is not defined"
#endif

extern struct gpio_dt_spec button_alarm;
extern struct gpio_dt_spec button_anti_dream;
extern struct gpio_dt_spec button_train_passed;

#define ANTI_DREAM_PERIOD (2*PERIOD_TIME_MSEC)

extern atomic_t anti_dream_active;

#elif CUR_DEVICE == BRIGADE_CHIEF
/**
 * Brigade chief thread functions begin
 * */
[[noreturn]] void brigade_chief_proc_task(void);
[[noreturn]] void brigade_chief_modem_task(void);
/**
 * Brigade chief thread functions end
 * */


#define CURRENT_DEVICE_NUM 2

/*
 * SW0 - disable alarm button
 * SW1 - left train passed button
 * SW2 - right train passed button
 * */
#define DISABLE_ALARM_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(DISABLE_ALARM_NODE, okay)
#error "Unsupported board: disable_alarm_sw devicetree alias is not defined"
#endif

//#define LEFT_TRAIN_PASSED_NODE	DT_ALIAS(sw1)
//#if !DT_NODE_HAS_STATUS(LEFT_TRAIN_PASSED_NODE, okay)
//#error "Unsupported board: left_train_passed_sw devicetree alias is not defined"
//#endif
//
//#define RIGHT_TRAIN_PASSED_NODE DT_ALIAS(sw2)
//#if !DT_NODE_HAS_STATUS(RIGHT_TRAIN_PASSED_NODE, okay)
//#error "Unsupported board: right_train_passed_sw alias is not defined"
//#endif

extern struct gpio_dt_spec button_disable_alarm;
extern struct gpio_dt_spec button_right_train_passed;
extern struct gpio_dt_spec button_left_train_passed;
#elif CUR_DEVICE == WORKERS

#define CURRENT_DEVICE_NUM 0

extern const k_tid_t dwm_task_id;

#endif
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
#define SHORT_PRESSED_MIN_VAL 2  /* 200 ms */
#define SHORT_PRESSED_MAX_VAL 10 /* 1000 ms */
#define MIDDLE_PRESSED_MIN_VAL (SHORT_PRESSED_MAX_VAL+1)  /* 1100 ms */
#define MIDDLE_PRESSED_MAX_VAL 20   /* 2000 ms */
#define LONG_PRESSED_MIN_VAL (MIDDLE_PRESSED_MAX_VAL+1) /* 2100 ms */

#if CUR_DEVICE != WORKERS
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

extern const struct device *buzzer_dev;

extern struct gpio_dt_spec *irq_gpio_dev;

extern struct k_timer periodic_timer; /* For switching in tx mode */

extern struct k_work work_buzzer; /* For signalisation */
extern struct k_work work_button_pressed;
extern struct k_work_delayable dwork_enable_ind;

extern struct k_poll_event event_buzzer;
extern struct k_poll_signal signal_buzzer;

//extern struct message_s tx_msg;

extern struct k_msgq msgq_tx_msg_prio;
extern struct k_msgq msgq_tx_msg;
extern struct k_msgq msgq_rx_msg;
extern struct k_msgq msgq_rssi;

extern const modem_state_t recv_state;
extern const modem_state_t transmit_state;
extern modem_state_t current_state;

//extern uint8_t tx_buf[MESSAGE_LEN_IN_BYTES];
//extern uint8_t rx_buf[MESSAGE_LEN_IN_BYTES];

/* Indicate structure */
extern struct led_strip_indicate_s status_ind;

extern struct led_strip_indicate_s disable_indication;
extern struct led_strip_indicate_s middle_pressed_button_ind;
extern struct led_strip_indicate_s msg_send_good_ind;
extern struct led_strip_indicate_s msg_send_bad_ind;
extern struct led_strip_indicate_s msg_recv_ind;
/**
 * Extern variable declaration area end
 * */


/**
 * Function declaration area begin
 * */
void work_buzzer_handler(struct k_work *item);
void work_button_pressed_handler(struct k_work *item);
void dwork_enable_ind_handler(struct k_work *item);

void lora_receive_error_timeout(const struct device *dev);
void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr);

bool proc_rx_data(struct k_msgq *msgq, uint8_t *recv_data, size_t len, struct message_s *rx_msg,
                  uint8_t cur_dev_addr);
bool proc_tx_data(struct k_msgq *msgq, uint8_t *tx_data, size_t len, struct message_s *tx_msg);
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


static inline int32_t modem_fun(const struct device *lora_dev, struct lora_modem_config* lora_cfg)
{
    int32_t rc = 1;
    struct k_msgq *cur_queue = nullptr;
    static struct k_spinlock spin;
    static k_spinlock_key_t key;
    static message_s tx_msg = {0};
    static uint8_t tx_buf[MESSAGE_LEN_IN_BYTES] = {0};
    if (current_state.state == TRANSMIT) {
        if (!proc_tx_data(cur_queue, tx_buf, sizeof(tx_buf), &tx_msg)) {
            return 1;
        }
        key = k_spin_lock(&spin);

        /* Stop receiving */
        lora_recv_async(lora_dev, nullptr, nullptr);

        lora_cfg->tx = true;
        rc = lora_config(lora_dev, lora_cfg);
        if (rc < 0) {
            k_msgq_put(cur_queue, &tx_msg, K_NO_WAIT);
            current_state = *current_state.next;
            k_spin_unlock(&spin, key);
            return rc;
        }

        rc = lora_send(lora_dev, tx_buf, MESSAGE_LEN_IN_BYTES);
        /*
         * If message not transmit then putting back him into queue and reconfig modem
         * */
        if (rc < 0) {
            k_msgq_put(cur_queue, &tx_msg, K_NO_WAIT);
        }
        atomic_set(&reconfig_modem, 1);

        if (tx_msg.message_type == MESSAGE_TYPE_SYNC)
            rc = 1;

        current_state = *current_state.next;
        k_spin_unlock(&spin, key);
        return rc;

    } else {
        if (atomic_cas(&reconfig_modem, 1, 0)) {
            lora_recv_async(lora_dev, nullptr, nullptr);
            lora_cfg->tx = false;
            rc = lora_config(lora_dev, lora_cfg);
            if (!rc) {
                rc = lora_recv_async(lora_dev, lora_receive_cb, lora_receive_error_timeout);
                if (rc) {
                    atomic_set(&reconfig_modem, 1);
                }
            } else {
                atomic_set(&reconfig_modem, 1);
            }
        }
    }
    /*
     * Return 1 if current_state.state != TRANSMIT_STATE
     * */
    return rc;
}
/**
 * Function definition area end
 * */

#endif

#ifdef __cplusplus
}
#endif

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H
