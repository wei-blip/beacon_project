//
// Created by rts on 21.01.2022.
//

#ifndef RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_H
#define RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_H

#include "message_format.h"
#include <devicetree.h>
#include <device.h>
#include <drivers/lora.h>
#include <errno.h>


#define PERIPHERAL
#define QUEUE_LEN_IN_ELEMENTS 10
#define SEM_LORA_BUSY_INIT_VAL 1
#define SEM_LORA_BUSY_LIM 1
#define SLOT_TIME_MSEC 500

extern struct message_s sync_msg;

#ifdef PERIPHERAL
#define DEVICE_NUM 3
#define CURRENT_DEVICE_NUM 0  // counter start value equal zero
#define DEVICE_SESSION_TIMEOUT_MSEC SLOT_TIME_MSEC*CURRENT_DEVICE_NUM
#define BUTTON_ALARM_GPIO_PORT "GPIOC"
#define BUTTON_ANTI_DREAM_GPIO_PORT "GPIOC"
#define BUTTON_ALARM_GPIO_PIN 13
#define BUTTON_ANTI_DREAM_GPIO_PIN 2
#endif

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

//extern struct device* lora_dev_ptr;

//extern uint8_t tx_buf[MESSAGE_LEN_IN_BYTES];
//extern uint8_t rx_buf[MESSAGE_LEN_IN_BYTES];

//extern struct message_s rx_msg;
//extern struct message_s tx_msg;


extern struct device* lora_dev_ptr;
extern struct lora_modem_config lora_cfg;


#ifdef BASE_STATION
void system_init();

#else
void system_init();
#endif

void send_task();
void recv_task();
void proc_task();
//void transcieve_task();

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_H
