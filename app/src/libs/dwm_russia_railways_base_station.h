//
// Created by rts on 14.03.2022.
//

#ifndef LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RUSSIA_RAILWAYS_BASE_STATION_H_
#define LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RUSSIA_RAILWAYS_BASE_STATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#define NUMBER_OF_NODES 8
#define MAX_RANGE_M 5.0
#define NODE_TIMEOUT_MS 2000

#define SAFE_ZONE_LIMIT_METERS 5.0

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps)
 * conversion factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

#define POLL_RX_TO_RESP_TX_DLY_UUS 1000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/*
 * rx twr_2_resp after tx twr_1_poll
 * protected by responder's mp_request_at(twr_2_resp):POLL_RX_TO_RESP_TX_DLY_UUS
*/
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
#define RESP_RX_TO_FINAL_TX_DLY_UUS 1000

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547.0

#define PROCESSING_INTERVAL_SEC 3
#define CLEANING_INTERVAL_SEC 10

typedef uint8_t worker_id_t;

extern struct k_msgq msgq_zone_info;

[[noreturn]] void base_station_dwm_task();

#ifdef __cplusplus
}
#endif
#endif //LORA_RUSSIA_RAILWAYS_SRC_LIBS_DWM_RUSSIA_RAILWAYS_BASE_STATION_H_
