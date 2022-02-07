//
// Created by rts on 05.02.2022.
//

#ifndef RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_BASE_STATION_H
#define RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_BASE_STATION_H

#include "lora_russia_railways_common.h"

#include <devicetree.h>
#include <device.h>
#include <drivers/lora.h>
#include <errno.h>


#define SEM_LORA_BUSY_INIT_VAL 1
#define SEM_LORA_BUSY_LIM 1

extern struct device* base_station_lora_dev_ptr;
extern struct lora_modem_config base_station_lora_cfg;


void base_station_start_system(void);
void base_station_proc_task(void);
void base_station_modem_task(void);


#endif //RADIO_SIGNALMAN_LORA_RUSSIA_RAILWAYS_BASE_STATION_H
