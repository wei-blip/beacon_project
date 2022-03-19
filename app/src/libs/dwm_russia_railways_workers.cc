#include <cstdio>
#include <cstring>
#include <string>
#include <list>
#include <map>

// zephyr includes
#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <random/rand32.h>
#include <pm/pm.h>
#include <drivers/gpio.h>
#include <drivers/uwb/dw1000.h>
#include <drivers/uwb/deca_device_api.h>
#include <meshposition/meshposition.h>
#include <meshposition/json.hh>
using json = nlohmann::json;

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_NONE);


/* UWB device */
static const struct device *uwb = DEVICE_DT_GET(DT_INST(0, decawave_dw1000));

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 500

#define RESPONDER_ID 255
#define INITIATOR_ID 0

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

static uint32_t reg;//force read status

[[noreturn]] void Initiator()
{
    LOG_INF("InitiatorThread> starting with status 0x%08lx", reg);

    mp_start();
    reg = mp_get_status(); //reading reg takes 40.75 us

    k_yield();
    uint8_t sequence = 0;

    while (true) {
        //APP_SET_CLEAR
        // - pulse1: 'request-receive : tx 1st till rx resp' ;
        // - pulse2: 'send_at : tx final delayed till sent'
        auto reg1 = mp_get_status();
        LOG_INF("initiator> sequence(%u) starting ; statusreg = 0x%08lx",
                sequence, reg1);
        mp_rx_after_tx(POLL_TX_TO_RESP_RX_DLY_UUS,10000);

        msg_header_t twr_poll = {
          .header = {
            .id = msg_id_t::twr_1_poll,
            .sequence = sequence,
            .source = INITIATOR_ID,
            .dest = RESPONDER_ID
          },
          .crc = 0
        };
        mp_request(twr_poll);

        if(mp_receive(msg_id_t::twr_2_resp))
        {
            auto poll_tx_ts = get_tx_timestamp_u64(); // DWT_TIME
            auto resp_rx_ts = get_rx_timestamp_u64(); // DWT_TIME
            //tx res 9 bits (8 bits shit and 1 bit mask)
            //tranceiver time format
            uint32_t final_tx_time =
              (resp_rx_ts +
                (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            //prorammed TX + antenna delay
            uint64_t final_tx_ts = //host time format
              (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8)
                + ((struct dw1000_dev_config *) uwb->config)->tx_ant_delay;
            msg_twr_final_t twr_final;
            twr_final.header = twr_poll.header;//keep same source and dest
            twr_final.header.id = msg_id_t::twr_3_final;
            twr_final.poll_tx_ts =
              (uint32_t)poll_tx_ts;//trunc 64 bits to 32 bits
            twr_final.resp_rx_ts =
              (uint32_t)resp_rx_ts;//trunc 64 bits to 32 bits
            twr_final.final_tx_ts =
              (uint32_t)final_tx_ts;//trunc 64 bits to 32 bits
            if(mp_send_at((uint8_t*)&twr_final,
                          sizeof(msg_twr_final_t), final_tx_time))
            {
                LOG_DBG("initiator> poll_tx= 0x%08llx ; resp_rx= 0x%08llx\n",
                        poll_tx_ts, resp_rx_ts);
                LOG_DBG("initiator> final_tx(ant)= 0x%08llx ; "
                        "final_tx(chip)= 0x%04lx\n", final_tx_ts,final_tx_time);
            }else{
                LOG_WRN("mp_send_at(twr_3_final) fail at sequence %u",sequence);
            }
        }else{
            LOG_WRN("mp_receive(twr_2_resp) fail at sequence %u",sequence);
        }
        reg1 = mp_get_status();
        LOG_INF("initiator> sequence(%u) over; statusreg = 0x%08lx",
                sequence, reg1);
        sequence++;
        auto rand_delay =  ((int32_t) sys_rand32_get()) >> 25;
        k_msleep(RNG_DELAY_MS + rand_delay);
    }
}
