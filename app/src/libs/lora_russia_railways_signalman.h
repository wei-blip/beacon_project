//
// Created by rts on 21.01.2022.
//

#ifndef RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_SIGNALMAN_H
#define RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_SIGNALMAN_H

#include "lora_russia_railways_common.h"

#define ANTI_DREAM_TIME_MIN 1
#define ANTI_DREAM_START (anti_dream_cnt == 20)

void signalman_proc_task(void);
void signalman_modem_task(void);

#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_SIGNALMAN_H
