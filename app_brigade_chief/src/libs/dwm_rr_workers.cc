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
#include <logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_NONE);

#include "dwm_rr/dwm_rr_common.h"

#define INITIATOR_ID 4

struct node {
  double range;
  int64_t last_seen;
};


/* The devicetree node identifier for the "led0" alias. */
//#define LED0_NODE DT_ALIAS(led0)

//#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
//#error "Unsupported board: led0 devicetree alias is not defined"
//#endif
//static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define LED_BLINK_MS 50

//void led_off_handler(struct k_work *work) {
//    gpio_pin_set_dt(&led, 0);
//}

//K_WORK_DELAYABLE_DEFINE(led_off_work, led_off_handler);

atomic_t atomic_twr_status = ATOMIC_INIT((atomic_t) msg_id_t::twr_3_final);

[[noreturn]] void dwm_task()
{
    uint64_t resp_rx_ts = 0;
    uint64_t poll_tx_ts = 0;
    uint64_t final_tx_ts = 0;
    int rc = 0;

//    gpio_pin_configure_dt(&led, GPIO_OUTPUT);

    uint8_t sequence = 0;

    const struct device *dwm_dev =  DEVICE_DT_GET(DT_INST(0, decawave_dw1000));

    msg_header_t tx_poll_msg = {
      .header = {
        .id = msg_id_t::twr_1_poll,
        .sequence = sequence,
        .source = INITIATOR_ID,
        .dest = RESPONDER_ID
      },
      .crc = 0
    };

    dwt_cb_data_t dwt_cb_data = {0};

    while(!device_is_ready(dwm_dev)) {
        k_sleep(K_MSEC(1));
    }

    LOG_INF("responder_thread> starting");
    char dist_str[50] = {'\0'};

    dwt_setinterrupt((DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO
      | DWT_INT_RXPTO | DWT_INT_SFDT |DWT_INT_ARFE) , 1);
    dwt_setcallbacks(tx_ok_cb, rx_ok_cb, rx_to_cb, rx_err_cb);

    init_twr_1_poll_ds_twr(&tx_poll_msg, &atomic_twr_status);


//    gpio_pin_set_dt(&led, 1);
//    k_work_schedule(&led_off_work, K_MSEC(1000));

    while (true) {
//        while (k_poll(&ev_dwm[0], 2, K_NO_WAIT)) {
//            k_sleep(K_MSEC(1));
//            printk("signal inf loop\n");
//        }
//
//        /*
//         * ind = 0 - rx
//         * ind = 1 - tx
//         * */
////        ev_dwm[0].signal->signaled = 0;
////        ev_dwm[0].state = K_POLL_STATE_NOT_READY;
//
//        switch (atomic_get(&atomic_twr_status)) {
//            case ((atomic_t) msg_id_t::twr_1_poll): /* RxDone, RxTO, RxError */
//                if (ev_dwm[0].state == K_POLL_STATE_SIGNALED) {
//                    switch (ev_dwm[0].signal->result) {
//                        ev_dwm[0].signal->signaled = 0;
//                        ev_dwm[0].state = K_POLL_STATE_NOT_READY;
//                        ev_dwm[1].signal->signaled = 0;
//                        ev_dwm[1].state = K_POLL_STATE_NOT_READY;
//
//
//                        case RX_DONE:
//                            if (!k_msgq_get(&msgq_dwt_callback_data, &dwt_cb_data, K_MSEC(1))) {
//                                init_twr_2_resp_ds_twr(dwm_dev, &tx_poll_msg, &atomic_twr_status, &dwt_cb_data);
//                            } else {
//                                /* Restart twr */
//                                init_twr_1_poll_ds_twr(&tx_poll_msg, &atomic_twr_status);
//                            }
//                            break;
//                        case RX_TO:
//                        case RX_ERR:
//                            /* Restart twr */
//                            init_twr_1_poll_ds_twr(&tx_poll_msg, &atomic_twr_status);
//                            break;
//                    }
//                }
//                break;
//            case ((atomic_t) (msg_id_t::twr_2_resp)):
//                if (ev_dwm[1].state == K_POLL_STATE_SIGNALED) {
//
//                    ev_dwm[0].signal->signaled = 0;
//                    ev_dwm[0].state = K_POLL_STATE_NOT_READY;
//                    ev_dwm[1].signal->signaled = 0;
//                    ev_dwm[1].state = K_POLL_STATE_NOT_READY;
//
//                    init_final_msg_poll_ds_twr(&atomic_twr_status);
//                }
//
//                break;
//            case ((atomic_t) msg_id_t::twr_3_final):
//                if (ev_dwm[1].state == K_POLL_STATE_SIGNALED) {
//
//                    ev_dwm[0].signal->signaled = 0;
//                    ev_dwm[0].state = K_POLL_STATE_NOT_READY;
//                    ev_dwm[1].signal->signaled = 0;
//                    ev_dwm[1].state = K_POLL_STATE_NOT_READY;
//
//                    init_twr_1_poll_ds_twr(&tx_poll_msg, &atomic_twr_status);
//                }
//
//                break;
//        }


        while (k_poll(&ev_dwm[0], 1, K_NO_WAIT)) {
            k_sleep(K_MSEC(1));
        }

        ev_dwm[0].signal->signaled = 0;
        ev_dwm[0].state = K_POLL_STATE_NOT_READY;

//        printk("Take dwt_cb_data\n");
        if (k_msgq_get(&msgq_dwt_callback_data, &dwt_cb_data, K_MSEC(1))) {
            init_twr_1_poll_ds_twr(&tx_poll_msg, &atomic_twr_status);
            continue;
        }

        switch (ev_dwm[0].signal->result) {
            case TX_DONE:
                if (atomic_get(&atomic_twr_status) == (atomic_t) (msg_id_t::twr_3_final)) {

//                    printk("First tx_done\n");
                    init_twr_1_poll_ds_twr(&tx_poll_msg, &atomic_twr_status);

                } else {
//                    gpio_pin_set_dt(&led, 1);
//                    k_work_schedule(&led_off_work, K_MSEC(LED_BLINK_MS));
//                    printk("Second tx_done\n");
                    init_final_msg_poll_ds_twr(&atomic_twr_status);
                }
                break;

            case RX_DONE:
                if (atomic_get(&atomic_twr_status) == (atomic_t) msg_id_t::twr_1_poll) {
//                    printk("twr_2_resp send\n");
                    init_twr_2_resp_ds_twr(dwm_dev, &tx_poll_msg, &atomic_twr_status, &dwt_cb_data);
                } else {
                    /* Restart twr */
//                    printk("Restart after checking rx_done\n");
                    init_twr_1_poll_ds_twr(&tx_poll_msg, &atomic_twr_status);
                }
                break;

            case RX_TO:
            case RX_ERR:
                /* Restart twr */
//                printk("Restart after rx_to rx_err\n");
                init_twr_1_poll_ds_twr(&tx_poll_msg, &atomic_twr_status);
                break;
        }
    }
}

