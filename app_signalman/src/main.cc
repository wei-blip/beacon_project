#include "lora_rr/lora_rr_common.h"

#define STACK_SIZE 1024


K_THREAD_DEFINE(proc_task_id, STACK_SIZE,
                proc_task, NULL, NULL, NULL,
                PRIORITY_PROC, 0, 0);
K_THREAD_DEFINE(modem_task_id, STACK_SIZE,
                modem_task, NULL, NULL, NULL,
                PRIORITY_MODEM_TASK, 0, 0);

