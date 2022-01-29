#include "lora_russia_railways.h"

void SecureElementRandomNumber(uint32_t* rand_num) {
    return;
}

#define STACK_SIZE 1024
#define PRIORITY_PROC 2
#define PRIORITY_RECV 0
#define PRIORITY_SEND 0

K_THREAD_DEFINE(recv_task_id, STACK_SIZE, recv_task, NULL, NULL, NULL, PRIORITY_RECV, 0, 0);
K_THREAD_DEFINE(proc_task_id, STACK_SIZE, proc_task, NULL, NULL, NULL, PRIORITY_PROC, 0, 0);
K_THREAD_DEFINE(send_task_id, STACK_SIZE, send_task, NULL, NULL ,NULL, PRIORITY_SEND, 0, 0);