//
// Created by rts on 14.03.2022.
//

#ifndef LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RUSSIA_RAILWAYS_BASE_STATION_H_
#define LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RUSSIA_RAILWAYS_BASE_STATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TX_DONE 1
#define RX_DONE 2
#define RX_TO 3
#define RX_ERR 4

#define NUMBER_OF_NODES 8
#define MAX_RANGE_M 5.0
#define NODE_TIMEOUT_MS 2000

#define SAFE_ZONE_LIMIT_METERS 5.0

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps)
 * conversion factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

#define FINAL_RX_TIMEOUT_UUS 10000

#define POLL_RX_TO_RESP_TX_DLY_UUS 1000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 300

/*rx twr_2_resp after tx twr_1_poll
 protected by responder's mp_request_at(twr_2_resp):POLL_RX_TO_RESP_TX_DLY_UUS
*/

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547.0

#define PROCESSING_INTERVAL_SEC 5
#define CLEANING_INTERVAL_SEC 10

extern const k_tid_t dwm_task_id;
extern struct k_msgq msgq_dwt_dist;

[[noreturn]] void base_station_dwm_task();

#ifdef __cplusplus
}
#endif
#endif //LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RUSSIA_RAILWAYS_BASE_STATION_H_
