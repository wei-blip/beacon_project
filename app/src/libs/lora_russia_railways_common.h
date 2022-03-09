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

#define QUEUE_LEN_IN_ELEMENTS 10
#define WAITING_PERIOD_NUM 2

#define SLOT_TIME_MSEC 764UL /* Time on receive(664ms) plus DELAY_TIME_MSEC */
#define PERIOD_TIME_MSEC (4*SLOT_TIME_MSEC)
#define DELAY_TIME_MSEC 100U
#define STOCK_TIME_MSEC 10

#define BUTTON_PRESSED_PERIOD_TIME_USEC 40000UL

#define IS_SYNC_MSG (rx_buf[0] == 13) /* SENDER_ADDR = BASE_STATION, RECV_ADDR = BROADCAST, MESSAGE_TYPE = SYNC  */
#define DISABLE_INDICATE (indicate_cnt == 5)

/*
 * This macro using in processing thread
 * rx_buf_proc - local variable processing thread
 * Array indexes have range from 0 to MESSAGE_LEN_IN_BYTES-1
 * */
#define IS_EMPTY_MSG ((rx_buf_proc[0] == 0) && (rx_buf_proc[MESSAGE_LEN_IN_BYTES-1] == 0))

/*
 * fun_call_count value for detected short pressed
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
extern struct lora_modem_config lora_cfg;

extern const struct device *lora_dev_ptr;
extern const struct device *buzzer_dev_ptr;

extern struct gpio_dt_spec cur_irq_gpio;

extern struct k_timer periodic_timer; /* For switch in tx mode */

extern struct k_work work_buzzer; /* For signalisation */
extern struct k_work work_msg_mngr; /* For putting messages into queues */
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
uint8_t reverse(uint8_t input);
uint8_t check_rssi(int16_t rssi);
void check_msg_status(struct msg_info_s *msg_info);
void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write);
/**
 * Function declaration area end
 * */

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H
