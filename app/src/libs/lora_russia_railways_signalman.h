//
// Created by rts on 21.01.2022.
//

#ifndef RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_SIGNALMAN_H
#define RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_SIGNALMAN_H

#include "lora_russia_railways_common.h"

#include <devicetree.h>
#include <device.h>
#include <drivers/lora.h>
#include <errno.h>



#define QUEUE_LEN_IN_ELEMENTS 10


#define IS_SYNC_MSG ( (signalman_rx_buf[0] == 193) && (signalman_rx_buf[1] == 64) && (signalman_rx_buf[2] == 0) )

#define BUTTON_ALARM_GPIO_PORT "GPIOC"
#define BUTTON_ALARM_GPIO_PIN 13
#define BUTTON_ANTI_DREAM_GPIO_PORT "GPIOC"
#define BUTTON_ANTI_DREAM_GPIO_PIN 2
#define BUTTON_TRAIN_PASSED_GPIO_PORT "GPIOC"
#define BUTTON_TRAIN_PASSED_GPIO_PIN 13


extern struct device* signalman_lora_dev_ptr;
extern struct lora_modem_config signalman_lora_cfg;


void signalman_start_system(void);
void signalman_proc_task(void);
void signalman_modem_task(void);

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_SIGNALMAN_H
