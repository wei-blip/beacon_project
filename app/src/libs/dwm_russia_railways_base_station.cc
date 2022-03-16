//
// Created by rts on 14.03.2022.
//
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

#include <drivers/uwb/dw1000.h>
#include <drivers/uwb/deca_device_api.h>

#include <cstdio>
#include <cstring>

// zephyr includes
#include <zephyr.h>
#include <sys/printk.h>
#include <random/rand32.h>

#include <string>
#include <list>
#include <map>
#include <meshposition/json.hh>
using json = nlohmann::json;

#include <drivers/gpio.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(dwm_workers, LOG_LEVEL_NONE);

#include "dwm_russia_railways_base_station.h"


static bool armed = false;

#define LED_BLINK_MS 50

#define CHECK_PERIOD_MS 2000
#define NUMBER_OF_NODES 4
#define MAX_RANGE_M 5.0
#define NODE_TIMEOUT_MS 2000

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps)
 * conversion factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

#define POLL_RX_TO_RESP_TX_DLY_UUS 1000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/*rx twr_2_resp after tx twr_1_poll
 protected by responder's mp_request_at(twr_2_resp):POLL_RX_TO_RESP_TX_DLY_UUS
*/
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
#define RESP_RX_TO_FINAL_TX_DLY_UUS 1000

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547.0

struct node {
  double range;
  int64_t last_seen;
};

static struct node nodes[NUMBER_OF_NODES] = {{0.0, 0}};

int check_nodes() {
    int64_t now = k_uptime_get();
    printk("\033[2J\033[H");
    int ret = 0;
    for(int i = 0; i < NUMBER_OF_NODES; ++i) {
        int64_t delta = now - nodes[i].last_seen;
        if(nodes[i].range > MAX_RANGE_M) {
            ret = 1;
            printk("\033[93m");
        }
        if(delta > NODE_TIMEOUT_MS) {
            ret = 1;
            printk("\033[31m");
        }
        printk("%d: %3.2lf m, %lld ms ago\033[0m\n",
               i, nodes[i].range, delta);
    }
//    for(auto & node : nodes) {
//        if(node.range > MAX_RANGE_M ||
//            (now - node.last_seen) > NODE_TIMEOUT_MS) {
//            return 1;
//        }
//    }
    return ret;
}

void update_node(unsigned number, double range) {
    if (number < NUMBER_OF_NODES) {
        nodes[number].range = range;
        nodes[number].last_seen = k_uptime_get();
    }
}


static uint32_t reg;//force read status

[[noreturn]] void base_station_dwm_task()
{
    LOG_INF("responder_thread> starting");
    char dist_str[30] = {'\0'};

    mp_start();

    k_yield();
    uint32_t sequence = 0;
    while (true) {
        //APP_SET_CLEAR
        // - pulse1: 'request_at : start request resp_2 tx till sent' ;
        // - pulse2: 'receive : pending for receive final_3'
        // - pulse3: 'computing distance'
        uint32_t reg1 = mp_get_status();
        LOG_INF("responder> sequence(%u) starting ; statusreg = 0x%08x",
                sequence,reg1);
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
                    auto distance = tof * SPEED_OF_LIGHT;
                    update_node(rx_poll_msg.header.source, distance);
//                    sprintf(dist_str,
//                            "responder> dist to tag %3u: %3.2lf m\n",
//                            rx_poll_msg.header.source,
//                            distance);
//                    printk("%s", dist_str);
                }else{
                    LOG_WRN("mp_receive(twr_3_final) fail at rx frame %u",
                            rx_poll_msg.header.sequence);
                }
            }else{
                LOG_WRN("mp_request_at(twr_2_resp) fail at rx frame %u",
                        rx_poll_msg.header.sequence);
            }
        }

        uint32_t reg2 = mp_get_status();
        LOG_INF("responder> sequence(%u) over; statusreg = 0x%08x",
                sequence,reg2);
        sequence++;
    }
}
