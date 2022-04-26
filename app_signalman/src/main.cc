#include "lora_rr/lora_rr_common.h"

#define STACK_SIZE 2048

#define PRIORITY_PROC 6
#define PRIORITY_APP_TASK 5
#define PRIORITY_INDICATION_TASK 7
#define PRIORITY_MODEM_TASK 5

//K_THREAD_DEFINE(proc_task_id, STACK_SIZE,
//                proc_task, NULL, NULL, NULL,
//                PRIORITY_PROC, 0, 0);
//K_THREAD_DEFINE(modem_task_id, STACK_SIZE,
//                modem_task, NULL, NULL, NULL,
//                PRIORITY_MODEM_TASK, 0, 0);
K_THREAD_DEFINE(app_task_id, STACK_SIZE,
                app_task, NULL, NULL, NULL,
                PRIORITY_APP_TASK, 0, 0);
K_THREAD_DEFINE(update_indication_task_id, STACK_SIZE,
                update_indication_task, NULL, NULL, NULL,
                PRIORITY_INDICATION_TASK, 0, 0);