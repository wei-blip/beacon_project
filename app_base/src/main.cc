#include "lora_rr/lora_rr_common.h"
#include "dwm_rr/dwm_rr_common.h"


#define STACK_SIZE 2048

#define PRIORITY_APP_TASK 5
#define PRIORITY_DWM_TASK 6


//K_THREAD_DEFINE(proc_task_id, STACK_SIZE,
//                proc_task, NULL, NULL, NULL,
//                PRIORITY_PROC, 0, 0);
//K_THREAD_DEFINE(modem_task_id, STACK_SIZE,
//                modem_task, NULL, NULL, NULL,
//                PRIORITY_MODEM_TASK, 0, 0);
K_THREAD_DEFINE(app_task_id, STACK_SIZE,
                app_task, NULL, NULL, NULL,
                PRIORITY_APP_TASK, 0, 0);
K_THREAD_DEFINE(dwm_task_id, STACK_SIZE,
                dwm_task, NULL, NULL, NULL,
                PRIORITY_DWM_TASK, 0, 0);
