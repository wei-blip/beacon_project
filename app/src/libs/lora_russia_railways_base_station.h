//
// Created by rts on 05.02.2022.
//

#ifndef RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_BASE_STATION_H
#define RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_BASE_STATION_H

#include "lora_russia_railways_common.h"

void base_station_proc_task(void);
void base_station_modem_task(void);

#define BUTTON_HOMEWARD_GPIO_PORT "GPIOC"
#define BUTTON_HOMEWARD_GPIO_PIN 13


#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_BASE_STATION_H
