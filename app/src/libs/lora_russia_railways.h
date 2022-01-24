//
// Created by rts on 21.01.2022.
//

#ifndef RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_H
#define RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_H

#include "message_format.h"
#include <drivers/lora.h>

struct device* lora_dev_ptr;

uint8_t tx_buf[MESSAGE_LEN_IN_BYTES] = {0};
uint8_t rx_buf[MESSAGE_LEN_IN_BYTES] = {0};

struct message_s rx_msg = {0};
struct message_s tx_msg = {0};

int system_init(uint8_t tim_duration_min, unsigned int sem_anti_dream_init_val, unsigned int sem_anti_dream_lim,
                unsigned int sem_proc_data_init_val, unsigned int sem_proc_data_lim);
int send_msg(struct message_s* msg_ptr);
void recv_msg();

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_H
