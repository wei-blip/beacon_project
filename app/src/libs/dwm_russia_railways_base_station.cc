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

#include <meshposition/meshposition.h>

#include <drivers/uwb/deca_device_api.h>

#include <cstring>

// zephyr includes
#include <zephyr.h>
#include <sys/printk.h>

#include <meshposition/json.hh>
using json = nlohmann::json;

#include "dwm_russia_railways_base_station.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(dwm_base_station, LOG_LEVEL_NONE);

#define ZONE_INFO_IDLE 0
#define ZONE_INFO_FILL 1
#define ZONE_INFO_PROC 2
#define ZONE_INFO_RST 3
atomic_t atomic_zone_info_status = ATOMIC_INIT(ZONE_INFO_IDLE);

K_MSGQ_DEFINE(msgq_zone_info, sizeof(uint8_t), 10, 1);

/*
 * Worker id and him distance to base station
 * Worker id - map key;
 * Distance to base station - map value
 * */
static std::map<worker_id_t, double> zone_info = {
  {0, 0.0},
  {1, 0.0},
  {2, 0.0},
  {3, 0.0},
  {4, 0.0},
  {5, 0.0},
  {6, 0.0},
  {7, 0.0}
};

struct node {
  double range;
  int64_t last_seen;
};

static struct node nodes[NUMBER_OF_NODES] = {{0.0, 0}};

void update_node(unsigned number, double range)
{
    if (number < NUMBER_OF_NODES) {
        nodes[number].range = range;
        nodes[number].last_seen = k_uptime_get();
    }
}

void dwork_proc_handler(struct k_work *item)
{
    uint8_t cnt = 0;

    /* Wait while zone_info_status will be set on ZONE_INFO_IDLE */
    while (!atomic_cas(&atomic_zone_info_status, ZONE_INFO_IDLE, ZONE_INFO_PROC)) {
        k_sleep(K_MSEC(10));
    }

    for (int key = 0; key < NUMBER_OF_NODES; ++key) {
        if (zone_info.at(key) > SAFE_ZONE_LIMIT_METERS)
            cnt++;
    }

    /* If queue is full then clean it  */
    if (!k_msgq_num_free_get(&msgq_zone_info))
        k_msgq_purge(&msgq_zone_info);

    k_msgq_put(&msgq_zone_info, &cnt, K_NO_WAIT);
    atomic_set(&atomic_zone_info_status, ZONE_INFO_IDLE);
    k_work_reschedule(k_work_delayable_from_work(item), K_SECONDS(PROCESSING_INTERVAL_SEC));
}

void dwork_rst_handler(struct k_work *item)
{
    /* Wait while zone_info_status will be set on ZONE_INFO_IDLE */
    while (!atomic_cas(&atomic_zone_info_status, ZONE_INFO_IDLE, ZONE_INFO_RST)) {
        k_sleep(K_MSEC(10));
    }

    for (int key = 0; key < NUMBER_OF_NODES; ++key) {
        zone_info.at(key) = 0.0;
    }

    atomic_set(&atomic_zone_info_status, ZONE_INFO_IDLE);
    k_work_reschedule(k_work_delayable_from_work(item), K_SECONDS(CLEANING_INTERVAL_SEC));
}

static uint32_t reg;//force read status

[[noreturn]] void base_station_dwm_task()
{
    LOG_INF("responder_thread> starting");
    struct k_work_delayable dwork_proc = {};
//    struct k_work_delayable dwork_rst = {};

    mp_start();

    uint32_t sequence = 0;

    k_work_init_delayable(&dwork_proc, dwork_proc_handler);
//    k_work_init_delayable(&dwork_rst, dwork_rst_handler);

    k_work_schedule(&dwork_proc, K_SECONDS(PROCESSING_INTERVAL_SEC));
//    k_work_schedule(&dwork_rst, K_SECONDS(CLEANING_INTERVAL_SEC));
    while (true) {
        //APP_SET_CLEAR
        // - pulse1: 'request_at : start request resp_2 tx till sent' ;
        // - pulse2: 'receive : pending for receive final_3'
        // - pulse3: 'computing distance'

        double distance = 0.0;
        mp_rx_now();
        msg_header_t rx_poll_msg;
        if(mp_receive(msg_id_t::twr_1_poll,rx_poll_msg)){
            uint64_t poll_rx_ts = get_rx_timestamp_u64();

            mp_rx_after_tx(RESP_TX_TO_FINAL_RX_DLY_UUS,10000);

            uint32_t resp_tx_time =
              (poll_rx_ts +
                (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

            msg_header_t tx_resp_msg = {
              {msg_id_t::twr_2_resp,
               (uint8_t)(rx_poll_msg.header.sequence + 1),
               rx_poll_msg.header.dest , rx_poll_msg.header.source},0
            };
            if(mp_request_at((uint8_t*)&tx_resp_msg,
                             sizeof(msg_header_t), resp_tx_time)){
                msg_twr_final_t final_msg;
                if(mp_receive(msg_id_t::twr_3_final,final_msg)){
                    k_sleep(K_USEC(10));
                    uint64_t resp_tx_ts  = get_tx_timestamp_u64();
                    uint64_t final_rx_ts = get_rx_timestamp_u64();

                    auto poll_rx_ts_32  = (uint32_t)poll_rx_ts;
                    auto resp_tx_ts_32  = (uint32_t)resp_tx_ts;
                    auto final_rx_ts_32 = (uint32_t)final_rx_ts;

                    auto Ra = (double)
                      (final_msg.resp_rx_ts - final_msg.poll_tx_ts);
                    auto Rb = (double)
                      (final_rx_ts_32 - resp_tx_ts_32);
                    auto Da = (double)
                      (final_msg.final_tx_ts - final_msg.resp_rx_ts);
                    auto Db = (double)
                      (resp_tx_ts_32 - poll_rx_ts_32);
                    auto tof_dtu = (int64_t)
                      ((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                    auto tof = (double) tof_dtu * DWT_TIME_UNITS;
                    distance = (double) tof * SPEED_OF_LIGHT;

                    update_node(rx_poll_msg.header.source, distance);

                    if (atomic_cas(&atomic_zone_info_status, ZONE_INFO_IDLE, ZONE_INFO_FILL)) {
                        zone_info.at(rx_poll_msg.header.source) = distance; /* Set distance value for current node*/
                        atomic_set(&atomic_zone_info_status, ZONE_INFO_IDLE);
                    }

                } else {
                    LOG_WRN("mp_receive(twr_3_final) fail at rx frame %u",
                            rx_poll_msg.header.sequence);
                }
            } else {
                LOG_WRN("mp_request_at(twr_2_resp) fail at rx frame %u",
                        rx_poll_msg.header.sequence);
            }
        }

        uint32_t reg2 = mp_get_status();
        LOG_INF("responder> sequence(%u) over; statusreg = 0x%08x",
                sequence, reg2);
        sequence++;
    }
}
