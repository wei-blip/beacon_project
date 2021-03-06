#include "lora_russia_railways_common.h"

#define STACK_SIZE 1024
#define PRIORITY_PROC 2
#define PRIORITY_MODEM_TASK (-2)

#if CUR_DEVICE == BASE_STATION

K_THREAD_DEFINE(proc_task_id, STACK_SIZE,
                base_station_proc_task, NULL, NULL, NULL,
                PRIORITY_PROC, 0, 0);
K_THREAD_DEFINE(modem_task_id, STACK_SIZE,
                base_station_modem_task, NULL, NULL, NULL,
                PRIORITY_MODEM_TASK, 0, 0);
#elif CUR_DEVICE == SIGNALMAN

K_THREAD_DEFINE(proc_task_id, STACK_SIZE,
                signalman_proc_task, NULL, NULL, NULL,
                PRIORITY_PROC, 0, 0);
K_THREAD_DEFINE(modem_task_id, STACK_SIZE,
                signalman_modem_task, NULL, NULL, NULL,
                PRIORITY_MODEM_TASK, 0, 0);
#elif CUR_DEVICE == BRIGADE_CHIEF

K_THREAD_DEFINE(proc_task_id, STACK_SIZE,
                brigade_chief_proc_task, NULL, NULL, NULL,
                PRIORITY_PROC, 0, 0);
K_THREAD_DEFINE(modem_task_id, STACK_SIZE,
                brigade_chief_modem_task, NULL, NULL, NULL,
                PRIORITY_MODEM_TASK, 0, 0);
#endif
