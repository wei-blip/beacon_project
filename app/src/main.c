//#define BASE_STATION
//#define SIGNALMAN
#define BRIGADE_CHIEF

#ifdef BASE_STATION
#include "lora_russia_railways_base_station.h"
#endif
#ifdef SIGNALMAN
#include "lora_russia_railways_signalman.h"
#endif
#ifdef BRIGADE_CHIEF
#include "lora_russia_railways_brigade_chief.h"
#endif

void SecureElementRandomNumber(uint32_t* rand_num) {
    return;
}

#define STACK_SIZE 1024
#define PRIORITY_PROC 2
#define PRIORITY_MODEM_TASK (-1)

#ifdef BASE_STATION
K_THREAD_DEFINE(proc_task_id, STACK_SIZE, base_station_proc_task, NULL, NULL, NULL,
                PRIORITY_PROC, 0, 0);
K_THREAD_DEFINE(modem_task_id, STACK_SIZE, base_station_modem_task, NULL, NULL, NULL,
                PRIORITY_MODEM_TASK, 0, 0);
#endif
#ifdef SIGNALMAN
K_THREAD_DEFINE(proc_task_id, STACK_SIZE, signalman_proc_task, NULL, NULL, NULL,
                PRIORITY_PROC, 0, 0);
K_THREAD_DEFINE(modem_task_id, STACK_SIZE, signalman_modem_task, NULL, NULL, NULL,
                PRIORITY_MODEM_TASK, 0, 0);
#endif
#ifdef BRIGADE_CHIEF
K_THREAD_DEFINE(proc_task_id, STACK_SIZE, brigade_chief_proc_task, NULL, NULL, NULL,
                PRIORITY_PROC, 0, 0);
K_THREAD_DEFINE(modem_task_id, STACK_SIZE, brigade_chief_modem_task, NULL, NULL, NULL,
                PRIORITY_MODEM_TASK, 0, 0);
#endif