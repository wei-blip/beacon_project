//
// Created by rts on 05.02.2022.
//

#include "message_format.h"

#ifndef RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H
#define RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H

#define QUEUE_LEN_IN_ELEMENTS 10


#define SLOT_TIME_MSEC 980
#define DELAY_TIME_MSEC 150
#define STOCK_TIME_MSEC 10
#define CORRECT_VALUE_MSEC 10
#define RECV_TIME_MSEC 900


void fill_msg_bit_field(uint32_t* msg_ptr, uint8_t field_val, uint8_t field_len, uint8_t* pos);
uint8_t reverse(uint8_t input);
void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write);
void extract_msg_bit_field(const uint32_t* msg_ptr, uint8_t *field_val, uint8_t field_len, uint8_t* pos);
uint8_t check_rssi(const int16_t rssi);


enum CONNECTION_QUALITY_RSSI {
    CONNECTION_QUALITY_RSSI_1 = -50,
    CONNECTION_QUALITY_RSSI_2 = -60,
    CONNECTION_QUALITY_RSSI_3 = -70,
    CONNECTION_QUALITY_RSSI_4 = -80,
    CONNECTION_QUALITY_RSSI_5 = -90,
    CONNECTION_QUALITY_RSSI_6 = -100,
    CONNECTION_QUALITY_RSSI_7 = -110,
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

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_COMMON_H
