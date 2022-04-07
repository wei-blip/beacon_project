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

#include <drivers/uwb/deca_device_api.h>
#include <drivers/uwb/deca_regs.h>
#include <drivers/uwb/dw1000.h>
#include <meshposition/meshposition.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#include "dwm_russia_railways_base_station.h"

K_MSGQ_DEFINE(msgq_dwt_callback_data, sizeof(dwt_cb_data_t), 10, 1);
K_MSGQ_DEFINE(msgq_dwt_dist, sizeof(uint8_t), 10, 1);
K_MUTEX_DEFINE(mtx_dwt_dist);

struct node {
  double range;
  int64_t last_seen;
};

static struct node nodes[NUMBER_OF_NODES] = {{0.0, 0}};

struct k_poll_signal sig = K_POLL_SIGNAL_INITIALIZER(sig);
struct k_poll_event ev[1] = {
  K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                           K_POLL_MODE_NOTIFY_ONLY,
                           &sig),
};

static void dwork_send_dist_handler(struct k_work *item);

double dist[NUMBER_OF_NODES] = {0.0};
bool active_nodes[NUMBER_OF_NODES] = {false};

/* Callback functions */
static void rx_ok_cb(const dwt_cb_data_t *cb_data); /* RxDone */
static void rx_to_cb(const dwt_cb_data_t *cb_data); /* RxTimeout */
static void rx_err_cb(const dwt_cb_data_t *cb_data); /* RxError */

static inline void cb_routine(const dwt_cb_data_t* cb_data, int res)
{
    k_msgq_put(&msgq_dwt_callback_data, cb_data, K_NO_WAIT);
    k_poll_signal_raise(&sig, res);
}

void restart_twr(atomic_t *atomic_twr_status)
{
    atomic_set(atomic_twr_status, (atomic_t) msg_id_t::twr_3_final);
    k_sleep(K_MSEC(10));
    dwt_setrxtimeout(0);
    /* Activate reception immediately. */
    while (dwt_rxenable(DWT_START_RX_IMMEDIATE)) {
        k_sleep(K_MSEC(1));
    }
}

bool check_correct_recv(void *expect_msg, size_t expect_size, size_t recv_size, atomic_t *twr_status);
bool resp_twr_2_resp_ds_twr(msg_header_t *rx_poll_msg, msg_header_t *tx_resp_msg, atomic_t *twr_status);
void resp_final_msg_poll_ds_twr(msg_twr_final_t *final_msg, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                                uint64_t final_rx_ts, double *dist_src, bool *nodes);

[[noreturn]] void base_station_dwm_task()
{
    uint64_t resp_tx_ts = 0;
    uint64_t poll_rx_ts = 0;
    uint64_t final_rx_ts = 0;
    int rc = 0;

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

    k_work_schedule(&dwork_send_dist, K_SECONDS(PROCESSING_INTERVAL_SEC));
    dwt_setrxtimeout(0);
    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (true) {
        while (k_poll(&ev[0], 1, K_NO_WAIT)) {
            k_sleep(K_MSEC(1));
        }
        ev[0].signal->signaled = 0;
        ev[0].state = K_POLL_STATE_NOT_READY;
        k_msgq_get(&msgq_dwt_callback_data, &dwt_cb_data, K_FOREVER);

        switch (ev[0].signal->result) {

            case RX_DONE:
                LOG_DBG("RxDone callback");
                /**
                 * Processing first received frame begin
                 * */
                if (atomic_get(&atomic_twr_status) == ((atomic_t) msg_id_t::twr_3_final)) {

                    if (!check_correct_recv(&rx_poll_msg, sizeof(rx_poll_msg), dwt_cb_data.datalength,
                                           &atomic_twr_status)) {
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

                    if (!resp_twr_2_resp_ds_twr(&rx_poll_msg, &tx_resp_msg, &atomic_twr_status)) {
                        continue;
                    }

                } else if (atomic_get(&atomic_twr_status) == ((atomic_t) msg_id_t::twr_2_resp)) {

                    if (!check_correct_recv(&final_msg, sizeof(final_msg), dwt_cb_data.datalength,
                                            &atomic_twr_status)) {
                        continue;
                    }

                    resp_tx_ts  = get_tx_timestamp_u64();
                    final_rx_ts = get_rx_timestamp_u64();
                    resp_final_msg_poll_ds_twr(&final_msg, poll_rx_ts, resp_tx_ts, final_rx_ts, dist, active_nodes);
                    restart_twr(&atomic_twr_status);
                } else {
                    restart_twr(&atomic_twr_status);
                }
                break;

            case RX_TO:
                LOG_DBG("RxTimeout callback");
            case RX_ERR:
                LOG_DBG("RxError callback");
                restart_twr(&atomic_twr_status);
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
        active_nodes[i] = false; /* Reset nodes */
        if (dist[i++] <= SAFE_ZONE_LIMIT_METERS)
            cnt++;
    }
    k_mutex_unlock(&mtx_dwt_dist);

    k_msgq_put(&msgq_dwt_dist, &cnt, K_NO_WAIT);
    k_work_reschedule(k_work_delayable_from_work(item), K_SECONDS(PROCESSING_INTERVAL_SEC));
}

void dwork_rst_dist_handler(struct k_work *item)
{

}

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    cb_routine(cb_data, RX_DONE);
}

static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    cb_routine(cb_data, RX_TO);
}

static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    cb_routine(cb_data, RX_ERR);
}

bool resp_twr_2_resp_ds_twr(msg_header_t *rx_poll_msg, msg_header_t *tx_resp_msg, atomic_t *twr_status)
{
    bool res = false;

    if (rx_poll_msg->header.id == msg_id_t::twr_1_poll) {
        /* Set expected delay and timeout for final message reception. */
        dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
        dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

        dwt_writetxdata(sizeof(*tx_resp_msg), (uint8_t *) tx_resp_msg, 0);
        dwt_writetxfctrl(sizeof(*tx_resp_msg), 0, 1);

        if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)) {
            restart_twr(twr_status);
        } else {
            atomic_set(twr_status, (atomic_t) msg_id_t::twr_2_resp);
            res = true;
        }
    } else {
        restart_twr(twr_status);
    }

    return res;
}

bool check_correct_recv(void *expect_msg, size_t expect_size, size_t recv_size, atomic_t *twr_status)
{
    bool res = false;

    if (recv_size == expect_size) {
        dwt_readrxdata((uint8_t *) expect_msg, expect_size, 0);
        res = true;
    } else {
        restart_twr(twr_status);
    }

    return res;
}

double calc_dist_ds_twr(msg_twr_final_t *final_msg, uint32_t poll_rx_ts, uint32_t resp_tx_ts, uint32_t final_rx_ts) {
    auto Ra = (double)
      (final_msg->resp_rx_ts - final_msg->poll_tx_ts);
    auto Rb = (double)
      (final_rx_ts - resp_tx_ts);
    auto Da = (double)
      (final_msg->final_tx_ts - final_msg->resp_rx_ts);
    auto Db = (double)
      (resp_tx_ts - poll_rx_ts);
    auto tof_dtu = (int64_t)
      ((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
    double tof = (double) tof_dtu * DWT_TIME_UNITS;
    return fabs(tof * SPEED_OF_LIGHT);
}

void resp_final_msg_poll_ds_twr(msg_twr_final_t *final_msg, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                                uint64_t final_rx_ts, double *dist_src, bool *nodes) {
    static char dist_str[40] = {'\0'};
    if (final_msg->header.id == msg_id_t::twr_3_final) {
        auto poll_rx_ts_32 = (uint32_t) poll_rx_ts;
        auto resp_tx_ts_32 = (uint32_t) resp_tx_ts;
        auto final_rx_ts_32 = (uint32_t) final_rx_ts;

        double d = calc_dist_ds_twr(final_msg, poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32);

        while (k_mutex_lock(&mtx_dwt_dist, K_NO_WAIT)) {
            k_sleep(K_MSEC(1));
        }
        *(dist_src + final_msg->header.source) = d;
        *(nodes + final_msg->header.source) = true;

        sprintf(dist_str,
                "responder> dist to tag %3u: %3.2lf m\n",
                final_msg->header.source,
                dist[final_msg->header.source]);
        printk("%s", dist_str);
        k_mutex_unlock(&mtx_dwt_dist);
    }
}