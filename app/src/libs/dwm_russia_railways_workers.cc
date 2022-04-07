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
LOG_MODULE_REGISTER(main, LOG_LEVEL_NONE);


/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 500

#define RESPONDER_ID 255
#define INITIATOR_ID 5

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps)
 * conversion factor.
 * 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

#define POLL_TX_TO_RESP_RX_DLY_UUS 500
#define RESP_RX_TO_FINAL_TX_DLY_UUS 2000

#define RESP_RX_TIMEOUT_UUS 10000

#define TX_DONE 1
#define RX_DONE 2
#define RX_TO 3
#define RX_ERR 4

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547.0

K_MSGQ_DEFINE(msgq_dwt_callback_data, sizeof(dwt_cb_data_t), 10, 1);
K_MUTEX_DEFINE(mtx_dwt_dist);

struct node {
  double range;
  int64_t last_seen;
};

struct k_poll_signal sig = K_POLL_SIGNAL_INITIALIZER(sig);
struct k_poll_event ev[1] = {
  K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                           K_POLL_MODE_NOTIFY_ONLY,
                           &sig),
};

/* Callback functions */
static void tx_ok_cb(const dwt_cb_data_t *cb_data); /* TxDone */
static void rx_ok_cb(const dwt_cb_data_t *cb_data); /* RxDone */
static void rx_to_cb(const dwt_cb_data_t *cb_data); /* RxTimeout */
static void rx_err_cb(const dwt_cb_data_t *cb_data); /* RxError */

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define LED_BLINK_MS 50

static void cb_routine(const dwt_cb_data_t* cb_data, int res)
{
    k_msgq_put(&msgq_dwt_callback_data, cb_data, K_NO_WAIT);
    k_poll_signal_raise(&sig, res);
}

void led_off_handler(struct k_work *work) {
    gpio_pin_set_dt(&led, 0);
}

K_WORK_DELAYABLE_DEFINE(led_off_work, led_off_handler);

[[noreturn]] void main()
{
    uint64_t resp_rx_ts = 0;
    uint64_t poll_tx_ts = 0;
    uint64_t final_tx_ts = 0;
    int rc = 0;

    gpio_pin_configure_dt(&led, GPIO_OUTPUT);

    uint8_t sequence = 0;
    atomic_t atomic_twr_status = ATOMIC_INIT((atomic_t) msg_id_t::twr_3_final);

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
    msg_header_t rx_resp_msg;
    msg_twr_final_t final_msg;

    dwt_cb_data_t dwt_cb_data = {0};

    while(!device_is_ready(dwm_dev)) {
        k_sleep(K_MSEC(1));
    }

    LOG_INF("responder_thread> starting");
    char dist_str[50] = {'\0'};

    dwt_setcallbacks(tx_ok_cb, rx_ok_cb, rx_to_cb, rx_err_cb);
    dwt_setinterrupt((DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO
      | DWT_INT_RXPTO | DWT_INT_SFDT |DWT_INT_ARFE) , 1);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    k_poll_signal_raise(&sig, TX_DONE);

    gpio_pin_set_dt(&led, 1);
    k_work_schedule(&led_off_work, K_MSEC(1000));

    while (true) {
        while (k_poll(&ev[0], 1, K_NO_WAIT)) {
            k_sleep(K_MSEC(1));
        }
        ev[0].signal->signaled = 0;
        ev[0].state = K_POLL_STATE_NOT_READY;

        switch (ev[0].signal->result) {
            case TX_DONE:
                if (atomic_get(&atomic_twr_status) == (atomic_t) (msg_id_t::twr_3_final)) {

                    dwt_writetxdata(sizeof(tx_poll_msg), (uint8_t*) &tx_poll_msg, 0);
                    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
                    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
                    atomic_set(&atomic_twr_status, (atomic_t) msg_id_t::twr_1_poll);

                } else if (atomic_get(&atomic_twr_status) == (atomic_t) (msg_id_t::twr_2_resp)) {
                    gpio_pin_set_dt(&led, 1);
                    k_work_schedule(&led_off_work, K_MSEC(LED_BLINK_MS));
                    /* TODO: case for final_twr */
                    auto rand_delay =  ((int32_t) sys_rand32_get()) >> 25;
                    k_sleep(K_MSEC(RNG_DELAY_MS + rand_delay));
                    atomic_set(&atomic_twr_status, (atomic_t) (msg_id_t::twr_3_final));
                    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                    k_poll_signal_raise(&sig, TX_DONE);
                }
//                else {
//                    /* TODO: Restart twr */
//                    auto rand_delay =  ((int32_t) sys_rand32_get()) >> 25;
//                    k_sleep(K_MSEC(RNG_DELAY_MS + rand_delay));
//                    atomic_set(&atomic_twr_status, (atomic_t) (msg_id_t::twr_3_final));
//                    k_poll_signal_raise(&sig, TX_DONE);
//                }
                break;

            case RX_DONE:
                if (atomic_get(&atomic_twr_status) == (atomic_t) msg_id_t::twr_1_poll) {
                    k_msgq_get(&msgq_dwt_callback_data, &dwt_cb_data, K_FOREVER);
                    if (dwt_cb_data.datalength == sizeof(rx_resp_msg)) {
                        /* Read receive data */
                        dwt_readrxdata((uint8_t*) &rx_resp_msg, dwt_cb_data.datalength, 0);
                        if (rx_resp_msg.header.id == msg_id_t::twr_2_resp) {
                            poll_tx_ts = get_tx_timestamp_u64();
                            resp_rx_ts = get_rx_timestamp_u64();

                            uint32_t final_tx_time =
                              (resp_rx_ts +
                                (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                            dwt_setdelayedtrxtime(final_tx_time);

                            /* Prorammed TX + antenna delay */
                            /* Host time format */
                            final_tx_ts =
                              (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8)
                                + ((struct dw1000_dev_config *) dwm_dev->config)->tx_ant_delay;

                            final_msg.header = tx_poll_msg.header;
                            final_msg.header.id = msg_id_t::twr_3_final;
                            final_msg.poll_tx_ts = (uint32_t) poll_tx_ts;
                            final_msg.resp_rx_ts = (uint32_t) resp_rx_ts;
                            final_msg.final_tx_ts = (uint32_t) final_tx_ts;

                            dwt_writetxdata(sizeof(final_msg), (uint8_t*) &final_msg, 0); /* Zero offset in TX buffer. */
                            dwt_writetxfctrl(sizeof(final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                            rc = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                            if (!rc) {
                                atomic_set(&atomic_twr_status, (atomic_t) msg_id_t::twr_2_resp);
                            } else {
                                /* TODO: Restart twr */
                                auto rand_delay =  ((int32_t) sys_rand32_get()) >> 25;
                                k_sleep(K_MSEC(RNG_DELAY_MS + rand_delay));
                                atomic_set(&atomic_twr_status, (atomic_t) (msg_id_t::twr_3_final));
                                dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                                dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                                k_poll_signal_raise(&sig, TX_DONE);
                            }
                        } else {
                            /* TODO: Restart twr */
                            auto rand_delay =  ((int32_t) sys_rand32_get()) >> 25;
                            k_sleep(K_MSEC(RNG_DELAY_MS + rand_delay));
                            atomic_set(&atomic_twr_status, (atomic_t) (msg_id_t::twr_3_final));
                            dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                            dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                            k_poll_signal_raise(&sig, TX_DONE);
                        }
                    } else {
                        /* TODO: Restart twr */
                        auto rand_delay =  ((int32_t) sys_rand32_get()) >> 25;
                        k_sleep(K_MSEC(RNG_DELAY_MS + rand_delay));
                        atomic_set(&atomic_twr_status, (atomic_t) (msg_id_t::twr_3_final));
                        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                        k_poll_signal_raise(&sig, TX_DONE);
                    }
                } else {
                    /* TODO: Restart twr */
                    auto rand_delay =  ((int32_t) sys_rand32_get()) >> 25;
                    k_sleep(K_MSEC(RNG_DELAY_MS + rand_delay));
                    atomic_set(&atomic_twr_status, (atomic_t) (msg_id_t::twr_3_final));
                    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                    k_poll_signal_raise(&sig, TX_DONE);
                }
                break;

            case RX_TO:
            case RX_ERR:
                /* TODO: Restart twr */
                auto rand_delay =  ((int32_t) sys_rand32_get()) >> 25;
                k_sleep(K_MSEC(RNG_DELAY_MS + rand_delay));
                atomic_set(&atomic_twr_status, (atomic_t) (msg_id_t::twr_3_final));
                dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                k_poll_signal_raise(&sig, TX_DONE);
                break;
        }
    }
}

static void tx_ok_cb(const dwt_cb_data_t *cb_data)
{
    k_poll_signal_raise(&sig, TX_DONE);
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




