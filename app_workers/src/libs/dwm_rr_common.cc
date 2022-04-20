#include <random/rand32.h>

#include "dwm_rr_common.h"

K_MSGQ_DEFINE(msgq_dwt_callback_data, sizeof(dwt_cb_data_t), 10, 1);
K_MSGQ_DEFINE(msgq_dwm_dist, sizeof(uint8_t), 10, 1);

struct k_poll_signal sig = K_POLL_SIGNAL_INITIALIZER(sig);
struct k_poll_event ev[1] = {
  K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                           K_POLL_MODE_NOTIFY_ONLY,
                           &sig),
};

static inline void cb_routine(const dwt_cb_data_t* cb_data, int res)
{
    k_msgq_put(&msgq_dwt_callback_data, cb_data, K_NO_WAIT);
    k_poll_signal_raise(&sig, res);
}

void tx_ok_cb(const dwt_cb_data_t *cb_data)
{
    cb_routine(cb_data, TX_DONE);
}

void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    cb_routine(cb_data, RX_DONE);
}

void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    cb_routine(cb_data, RX_TO);
}

void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    cb_routine(cb_data, RX_ERR);
}

bool check_correct_recv(void *expect_msg, size_t expect_size, size_t recv_size)
{
    bool res = false;

    if (recv_size == expect_size) {
        dwt_readrxdata((uint8_t *) expect_msg, expect_size, 0);
        res = true;
    }
    return res;
}

/*
 * Responder functions begin
 * */
void resp_twr_1_poll_ds_twr(atomic_t *atomic_twr_status)
{
    atomic_set(atomic_twr_status, (atomic_t) msg_id_t::twr_3_final);
    k_sleep(K_MSEC(10));
    dwt_setrxtimeout(0);
    /* Activate reception immediately. */
    while (dwt_rxenable(DWT_START_RX_IMMEDIATE)) {
        k_sleep(K_MSEC(1));
    }
}

void resp_twr_2_resp_ds_twr(msg_header_t *rx_poll_msg, msg_header_t *tx_resp_msg, atomic_t *twr_status)
{
    if (rx_poll_msg->header.id == msg_id_t::twr_1_poll) {
        /* Set expected delay and timeout for final message reception. */
        dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
        dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

        dwt_writetxdata(sizeof(*tx_resp_msg), (uint8_t *) tx_resp_msg, 0);
        dwt_writetxfctrl(sizeof(*tx_resp_msg), 0, 1);

        if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)) {
            resp_twr_1_poll_ds_twr(twr_status);
        } else {
            atomic_set(twr_status, (atomic_t) msg_id_t::twr_2_resp);
        }
    } else {
        resp_twr_1_poll_ds_twr(twr_status);
    }
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

        *(dist_src + final_msg->header.source) = d;
        *(nodes + final_msg->header.source) = true;

        sprintf(dist_str,
                "responder> dist to tag %3u: %3.2lf m\n",
                final_msg->header.source,
                *(dist_src + final_msg->header.source));
        printk("%s", dist_str);
    }
}
/*
 * Responder functions end
 * */


/*
 * Initiator functions begin
 * */
void init_twr_1_poll_ds_twr(msg_header_t *tx_poll_msg, atomic_t *atomic_twr_status)
{
    atomic_set(atomic_twr_status, (atomic_t) msg_id_t::twr_1_poll);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    dwt_writetxdata(sizeof(*tx_poll_msg), (uint8_t*) tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(*tx_poll_msg), 0, 1);

    while(dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)) {
        k_sleep(K_MSEC(1));
    }
}

void init_twr_2_resp_ds_twr(const struct device *dwm_dev, msg_header_t *tx_poll_msg, atomic_t *atomic_twr_status,
  dwt_cb_data_t *cb_data)
{
    int rc = 0;
    msg_header_t rx_resp_msg;
    msg_twr_final_t final_msg;
    if (check_correct_recv(&rx_resp_msg, sizeof(rx_resp_msg), cb_data->datalength)) {
            /* Read receive data */
            dwt_readrxdata((uint8_t*) &rx_resp_msg, cb_data->datalength, 0);
            if (rx_resp_msg.header.id == msg_id_t::twr_2_resp) {
                uint64_t poll_tx_ts = get_tx_timestamp_u64();
                uint64_t resp_rx_ts = get_rx_timestamp_u64();

                uint32_t final_tx_time =
                  (resp_rx_ts +
                    (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /* Prorammed TX + antenna delay */
                /* Host time format */
                uint64_t final_tx_ts =
                  (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8)
                    + ((struct dw1000_dev_config *) dwm_dev->config)->tx_ant_delay;

                final_msg.header = tx_poll_msg->header;
                final_msg.header.id = msg_id_t::twr_3_final;
                final_msg.poll_tx_ts = (uint32_t) poll_tx_ts;
                final_msg.resp_rx_ts = (uint32_t) resp_rx_ts;
                final_msg.final_tx_ts = (uint32_t) final_tx_ts;

                dwt_writetxdata(sizeof(final_msg), (uint8_t*) &final_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                rc = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                if (!rc) {
                    atomic_set(atomic_twr_status, (atomic_t) msg_id_t::twr_2_resp);
                } else {
                    /* Restart twr */
                    init_twr_1_poll_ds_twr(tx_poll_msg, atomic_twr_status);
                }
            } else {
                /* Restart twr */
                init_twr_1_poll_ds_twr(tx_poll_msg, atomic_twr_status);
            }
    } else {
        /* Restart twr */
        init_twr_1_poll_ds_twr(tx_poll_msg, atomic_twr_status);
    }
}

void init_final_msg_poll_ds_twr(atomic_t *atomic_twr_status)
{
    atomic_set(atomic_twr_status, (atomic_t) msg_id_t::twr_3_final);
    auto rand_delay =  ((int32_t) sys_rand32_get()) >> 25;
    k_msleep(RNG_DELAY_MS + rand_delay);
    k_poll_signal_raise(&sig, TX_DONE);
}
/*
 * Initiator functions end
 * */