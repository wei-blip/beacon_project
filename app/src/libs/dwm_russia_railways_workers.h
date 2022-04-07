//
// Created by rts on 14.03.2022.
//
#ifndef LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RUSSIA_RAILWAYS_WORKERS_H_
#define LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RUSSIA_RAILWAYS_WORKERS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TX_DONE 1
#define RX_DONE 2
#define RX_TO 3
#define RX_ERR 4

[[noreturn]] void workers_dwm_task();

#ifdef __cplusplus
}
#endif

#endif //LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RUSSIA_RAILWAYS_WORKERS_H_
