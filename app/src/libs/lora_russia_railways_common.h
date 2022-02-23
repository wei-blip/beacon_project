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

#define SLOT_TIME_MSEC 980UL
#define PERIOD_TIME_MSEC (4*SLOT_TIME_MSEC)
#define DELAY_TIME_MSEC 150U
#define STOCK_TIME_MSEC 10
#define CORRECT_VALUE_MSEC 10
#define RECV_TIME_MSEC 900

//#define BUZZER_GPIO_PORT "GPIOA"
//#define BUZZER_GPIO_PIN 1

#define BUTTON_PRESSED_PERIOD_TIME_USEC 40000UL


#define IS_SYNC_MSG ( (rx_buf[0] == 13) && (rx_buf[1] == 64) && (rx_buf[2] == 0) )


enum CONNECTION_QUALITY_RSSI {
    CONNECTION_QUALITY_RSSI_1 = -70,
    CONNECTION_QUALITY_RSSI_2 = -80,
    CONNECTION_QUALITY_RSSI_3 = -90,
    CONNECTION_QUALITY_RSSI_4 = -100,
    CONNECTION_QUALITY_RSSI_5 = -105,
    CONNECTION_QUALITY_RSSI_6 = -110,
    CONNECTION_QUALITY_RSSI_7 = -115,
    CONNECTION_QUALITY_RSSI_8 = -120
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
};

struct msg_info_s {
    atomic_t req_is_send;
    atomic_t resp_is_recv;
    struct k_msgq *msg_buf;
    struct message_s *msg;

};


/// Extern variable declaration begin
extern struct lora_modem_config lora_cfg;

extern const struct device *lora_dev_ptr;
extern const struct device *buzzer_dev_ptr;

extern struct k_timer periodic_timer; // For switch in tx mode
extern struct k_work work_buzzer; // For signalisation
extern struct k_work work_msg_mngr; // For putting messages into queues
extern struct k_work work_led_strip_blink; // For blinking led strip
extern struct k_mutex mut_msg_info; // Block msgq_tx_msg and msgq_tx_msg_prio
extern struct k_mutex mut_buzzer_mode; // Block buzzer_mode
extern struct k_mutex mut_led_strip_busy; // Block led strip

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
/// Extern variable declaration end


uint8_t reverse(uint8_t input);
uint8_t check_rssi(int16_t rssi);
void check_msg_status(struct msg_info_s *msg_info);
void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write);
void fill_msg_bit_field(uint32_t* msg_ptr, uint8_t field_val, uint8_t field_len, uint8_t* pos);
void extract_msg_bit_field(const uint32_t* msg_ptr, uint8_t *field_val, uint8_t field_len, uint8_t* pos);


#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H
