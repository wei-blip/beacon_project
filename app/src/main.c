#include "lora_russia_railways.h"

void SecureElementRandomNumber(uint32_t* rand_num) {
    return;
}

#define STACK_SIZE 1024
#define PRIORITY_PROC 2
#define PRIORITY_RECV 0

K_THREAD_DEFINE(recv_thread_id, STACK_SIZE, recv_msg, NULL, NULL, NULL, PRIORITY_RECV, 0, 0);
K_THREAD_DEFINE(proc_thread_id, STACK_SIZE, processing_data, NULL, NULL, NULL, PRIORITY_PROC, 0, 0);