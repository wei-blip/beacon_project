//#define BASE_STATION
#define PERIPHERAL

#ifdef BASE_STATION
#include "lora_russia_railways_base_station.h"
#else
#include "lora_russia_railways_peripheral.h"
#endif

void SecureElementRandomNumber(uint32_t* rand_num) {
    return;
}

#define STACK_SIZE 1024
#define PRIORITY_PROC 2
#define PRIORITY_START_SYSTEM 0
#define PRIORITY_MODEM_TASK (-1)

#ifdef BASE_STATION
K_THREAD_DEFINE(proc_task_id, STACK_SIZE, base_station_proc_task, NULL, NULL, NULL,
                PRIORITY_PROC, 0, 0);
K_THREAD_DEFINE(start_system_id, STACK_SIZE, base_station_start_system, NULL, NULL, NULL,
                PRIORITY_START_SYSTEM, 0, 0);
K_THREAD_DEFINE(modem_task_id, STACK_SIZE, base_station_modem_task, NULL, NULL, NULL,
                PRIORITY_MODEM_TASK, 0, 0);
#else
K_THREAD_DEFINE(proc_task_id, STACK_SIZE, peripheral_proc_task, NULL, NULL, NULL,
                PRIORITY_PROC, 0, 0);
K_THREAD_DEFINE(start_system_id, STACK_SIZE, peripheral_start_system, NULL, NULL, NULL,
                PRIORITY_START_SYSTEM, 0, 0);
K_THREAD_DEFINE(modem_task_id, STACK_SIZE, peripheral_modem_task, NULL, NULL, NULL,
                PRIORITY_MODEM_TASK, 0, 0);
#endif