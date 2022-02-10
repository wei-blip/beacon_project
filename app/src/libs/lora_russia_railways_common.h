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

#include "message_format.h"
#include "indication.h"

#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

#define QUEUE_LEN_IN_ELEMENTS 10
#define WAITING_PERIOD_NUM 4

#define SLOT_TIME_MSEC 980UL
#define PERIOD_TIME_MSEC (4*SLOT_TIME_MSEC)
#define DELAY_TIME_MSEC 150U
#define STOCK_TIME_MSEC 10
#define CORRECT_VALUE_MSEC 10
#define RECV_TIME_MSEC 900

#define BUZZER_GPIO_PORT "GPIOA"
#define BUZZER_GPIO_PIN 1

#define IS_SYNC_MSG ( (rx_buf[0] == 9) && (rx_buf[1] == 64) && (rx_buf[2] == 0) )

uint8_t reverse(uint8_t input);
uint8_t check_rssi(int16_t rssi);
void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write);
void fill_msg_bit_field(uint32_t* msg_ptr, uint8_t field_val, uint8_t field_len, uint8_t* pos);
void extract_msg_bit_field(const uint32_t* msg_ptr, uint8_t *field_val, uint8_t field_len, uint8_t* pos);


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


struct msg_info_s {
    bool req_is_send;
    bool resp_is_recv;
    struct k_msgq *msg_buf;
    uint8_t cnt;
    struct message_s *msg;

};


/// Extern variable declaration begin
extern struct lora_modem_config lora_cfg;

extern const struct device *lora_dev_ptr;
extern const struct device *buzzer_dev_ptr;

extern struct k_timer periodic_timer;
extern struct k_work work_buzzer;
extern struct k_work work_msg_mngr;

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

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H
