/**
 * Copyright (c) 2019 - Frederic Mes, RTLOC
 * Copyright (c) 2015 - Decawave Ltd, Dublin, Ireland.
 * Copyright (c) 2021 - Home Smart Mesh
 *
 * This file is part of Zephyr-DWM1001.
 *
 *   Zephyr-DWM1001 is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Zephyr-DWM1001 is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Zephyr-DWM1001.  If not, see <https://www.gnu.org/licenses/>.
 *
 */


#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <sys/util.h>
#include <random/rand32.h>
#include <drivers/gpio.h>

#include <logging/log.h>
    LOG_MODULE_REGISTER(dwm_base_station, LOG_LEVEL_NONE);

#include "dwm_rr/dwm_rr_common.h"

//#define NUMBER_OF_NODES 8
#define NODE_TIMEOUT_MS 2000

#define MAX_RANGE_M 5.0

#define PROCESSING_INTERVAL_SEC 3

K_MUTEX_DEFINE(mtx_dwt_dist);

struct node {
  double range;
  int64_t last_seen;
};

static struct node nodes[NUMBER_OF_NODES] = {{0.0, 0}};

static void dwork_send_dist_handler(struct k_work *item);

double dist[NUMBER_OF_NODES] = {0.0};
bool active_nodes[NUMBER_OF_NODES] = {false};

[[noreturn]] void dwm_task()
{
    uint64_t resp_tx_ts = 0;
    uint64_t poll_rx_ts = 0;
    uint64_t final_rx_ts = 0;
    int rc = 0;
    enum DWM_DEVICES_e dev = RESPONDER;

    atomic_t atomic_twr_status = ATOMIC_INIT((atomic_t) msg_id_t::twr_3_final);

    const struct device *dwm_dev =  DEVICE_DT_GET(DT_INST(0, decawave_dw1000));

    msg_header_t rx_poll_msg;
    msg_header_t tx_resp_msg;
    msg_twr_final_t final_msg;

    dwt_cb_data_t dwt_cb_data = {0};
    struct k_work_delayable dwork_send_dist = {{{nullptr}}};

    while(!device_is_ready(dwm_dev)) {
        k_sleep(K_MSEC(1));
    }
    k_work_init_delayable(&dwork_send_dist, dwork_send_dist_handler);

    LOG_INF("responder_thread> starting");
    char dist_str[50] = {'\0'};

    dwt_setcallbacks(nullptr, rx_ok_cb, rx_to_cb, rx_err_cb);
    dwt_setinterrupt((DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO
        | DWT_INT_RXPTO | DWT_INT_SFDT |DWT_INT_ARFE) , 1);

    k_yield();
    k_work_schedule(&dwork_send_dist, K_SECONDS(PROCESSING_INTERVAL_SEC));
    resp_twr_1_poll_ds_twr(&atomic_twr_status);

    while (true) {
        while (k_poll(&ev_dwm[0], 1, K_NO_WAIT)) {
            k_sleep(K_MSEC(1));
        }

        if (ev_dwm[0].state != K_POLL_STATE_SIGNALED) {
            continue;
        }
        ev_dwm[0].signal->signaled = 0;
        ev_dwm[0].state = K_POLL_STATE_NOT_READY;

//        printk("Take dwt_cb_data\n");
        if (k_msgq_get(&msgq_dwt_callback_data, &dwt_cb_data, K_MSEC(1))) {
            resp_twr_1_poll_ds_twr(&atomic_twr_status);
            continue;
        }

        switch (ev_dwm[0].signal->result) {

            case RX_DONE:
                LOG_DBG("RxDone callback");
                /**
                 * Processing first received frame begin
                 * */
                if (atomic_get(&atomic_twr_status) == ((atomic_t) msg_id_t::twr_3_final)) {
//                    printk("First rx_done\n");

                    if (!check_correct_recv(&rx_poll_msg, sizeof(rx_poll_msg), dwt_cb_data.datalength)) {
//                        printk("Restart after first checking correct receive\n");
                        resp_twr_1_poll_ds_twr(&atomic_twr_status);
                        continue;
                    }

                    poll_rx_ts = get_rx_timestamp_u64();

                    tx_resp_msg = {
                      {msg_id_t::twr_2_resp,
                       (uint8_t) (rx_poll_msg.header.sequence + 1),
                       rx_poll_msg.header.dest,
                       rx_poll_msg.header.source
                      },
                      0
                    };

                    resp_twr_2_resp_ds_twr(&rx_poll_msg, poll_rx_ts, &tx_resp_msg, &atomic_twr_status);

                } else if (atomic_get(&atomic_twr_status) == ((atomic_t) msg_id_t::twr_2_resp)) {

//                    printk("Second rx_done\n");
                    if (!check_correct_recv(&final_msg, sizeof(final_msg), dwt_cb_data.datalength)) {
//                        printk("Restart after second checking correct receive\n");
                        resp_twr_1_poll_ds_twr(&atomic_twr_status);
                        continue;
                    }

                    resp_tx_ts  = get_tx_timestamp_u64();
                    final_rx_ts = get_rx_timestamp_u64();
                    while(k_mutex_lock(&mtx_dwt_dist, K_NO_WAIT)) {
                        k_sleep(K_MSEC(1));
                    }
                    resp_final_msg_poll_ds_twr(&final_msg, poll_rx_ts, resp_tx_ts, final_rx_ts, dist, active_nodes);
                    k_mutex_unlock(&mtx_dwt_dist);
//                    printk("Restart after resp_final_msg\n");
                    resp_twr_1_poll_ds_twr(&atomic_twr_status);
                }
                break;

            case RX_TO:
                LOG_DBG("RxTimeout callback");
            case RX_ERR:
                LOG_DBG("RxError callback");
//                printk("Restart transaction after rx_to rx_err!!!\n");
                resp_twr_1_poll_ds_twr(&atomic_twr_status);
                break;
        }
    }
}

void dwork_send_dist_handler(struct k_work *item)
{
    uint8_t i, cnt = 0;

    while (k_mutex_lock(&mtx_dwt_dist, K_NO_WAIT)) {
        k_sleep(K_MSEC(1));
    }

    while (i < NUMBER_OF_NODES) {
        if (active_nodes[i] && (dist[i] <= MAX_RANGE_M) ) {
            active_nodes[i] = false;
            cnt++;
        }
        i++;
    }

    k_mutex_unlock(&mtx_dwt_dist);

    k_msgq_put(&msgq_dwm_dist, &cnt, K_NO_WAIT);
    k_work_reschedule(k_work_delayable_from_work(item), K_SECONDS(PROCESSING_INTERVAL_SEC));
}