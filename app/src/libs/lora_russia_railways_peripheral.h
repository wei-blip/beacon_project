//
// Created by rts on 21.01.2022.
//

#ifndef RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_PERIPHERAL_H
#define RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_PERIPHERAL_H

#include "lora_russia_railways_common.h"

#include <devicetree.h>
#include <device.h>
#include <drivers/lora.h>
#include <errno.h>


#define IS_SYNC_MSG ( (peripheral_rx_buf[0] == 193) && (peripheral_rx_buf[1] == 64) && (peripheral_rx_buf[2] == 0) )


#define QUEUE_LEN_IN_ELEMENTS 10

#define SEM_LORA_BUSY_INIT_VAL 1
#define SEM_LORA_BUSY_LIM 1
#define SEM_COUNT_INIT_VAL DEVICE_NUM

#define DEVICE_NUM 1
#define CURRENT_DEVICE_NUM 0  // counter start value equal zero
#define DEVICE_SESSION_TIMEOUT_MSEC SLOT_TIME_MSEC*CURRENT_DEVICE_NUM
#define BUTTON_ALARM_GPIO_PORT "GPIOC"
#define BUTTON_ANTI_DREAM_GPIO_PORT "GPIOC"
#define BUTTON_ALARM_GPIO_PIN 13
#define BUTTON_ANTI_DREAM_GPIO_PIN 2


enum SLOTS {
    TRANSMIT_SLOT  = 0,
    RECEIVE_3_SLOT = 1,
    RECEIVE_2_SLOT = 2,
    RECEIVE_1_SLOT = 3
};

extern struct device* lora_dev_ptr;
extern struct lora_modem_config lora_cfg;


void peripheral_start_system(void);
void peripheral_proc_task(void);
void peripheral_modem_task(void);

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_PERIPHERAL_H
