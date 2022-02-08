//
// Created by rts on 05.02.2022.
//

#include "lora_russia_railways_base_station.h"
#include <drivers/gpio.h>

#include <logging/log.h>
    LOG_MODULE_REGISTER(base_station);

#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

uint8_t base_station_tx_buf[MESSAGE_LEN_IN_BYTES] = {0};
uint8_t base_station_rx_buf[MESSAGE_LEN_IN_BYTES] = {0};

/// My threads ids begin
extern const k_tid_t proc_task_id;
extern const k_tid_t modem_task_id;
extern const k_tid_t start_system_id;
/// My threads ids end

static struct message_s sync_msg;
static modem_state_t recv_state;
static modem_state_t transmit_state;
static modem_state_t current_state;

/// Structure area begin
struct device* base_station_lora_dev_ptr;
struct lora_modem_config base_station_lora_cfg;

// priority queue for sending messages
K_MSGQ_DEFINE(base_station_msgq_tx_msg_prio, sizeof(struct message_s), QUEUE_LEN_IN_ELEMENTS, 1);
// none priority queue for sending messages
K_MSGQ_DEFINE(base_station_msgq_tx_msg, sizeof(struct message_s), QUEUE_LEN_IN_ELEMENTS, 1);
// queue for receiving messages
K_MSGQ_DEFINE(base_station_msgq_rx_msg, MESSAGE_LEN_IN_BYTES, QUEUE_LEN_IN_ELEMENTS, 1);
// queue for rssi values
K_MSGQ_DEFINE(base_station_msgq_rssi, sizeof(int16_t), QUEUE_LEN_IN_ELEMENTS, 2);


// periodic timer (for syncro)
struct k_timer base_station_periodic_timer;

struct k_sem sem_modem_busy;

struct message_s base_station_tx_msg = {0};
/// Structure area end


/// Enum area begin
enum receiver_addr base_station_cur_dev_addr = RECV_BASE_STATION;
enum workers_ids base_station_cur_workers_in_safe_zone = FIRST_PEOPLE_ID;
enum battery_level base_station_cur_battery_level = BATTERY_LEVEL_GOOD;
/// Enum area end


/// Function declaration area begin
void base_station_periodic_timer_handler(struct k_timer* tim); // callback for periodic_timer
static void system_init(void);
static void send_msg(void);
static void recv_msg(void);
/// Function declaration area end


/// Function definition area begin
static void system_init()
{
    //// Kernel services init begin
    k_timer_init(&base_station_periodic_timer, base_station_periodic_timer_handler, NULL);
//    k_timer_init(&calibration_timer, calibration_timer_handler, NULL);

    k_sem_init(&sem_modem_busy, SEM_LORA_BUSY_INIT_VAL, SEM_LORA_BUSY_LIM);
    /// Kernel services init end

    /// LoRa init begin
    base_station_lora_cfg.frequency = 433000000;
    base_station_lora_cfg.bandwidth = BW_125_KHZ;
    base_station_lora_cfg.datarate = SF_12;
    base_station_lora_cfg.preamble_len = 8;
    base_station_lora_cfg.coding_rate = CR_4_5;
    base_station_lora_cfg.tx_power = 0;
    base_station_lora_cfg.tx = true;

    base_station_lora_dev_ptr = DEVICE_DT_GET(DEFAULT_RADIO_NODE);

    if (!device_is_ready(base_station_lora_dev_ptr)) {
        return;
    }

    if ( lora_config(base_station_lora_dev_ptr, &base_station_lora_cfg) < 0 ) {
        return;
    }
    /// LoRa init end
}


void base_station_start_system(void)
{
    volatile int rc = -1;
    uint32_t new_msg = 0;

    system_init();

    recv_state.next = &transmit_state;
    recv_state.state = RECEIVE;

    transmit_state.next = &recv_state;
    transmit_state.state = TRANSMIT;

    current_state = recv_state;

    /// Send sync message
    sync_msg.receiver_addr = RECV_BROADCAST;
    sync_msg.sender_addr = base_station_cur_dev_addr;
    sync_msg.message_type = MESSAGE_TYPE_SYNC;
    sync_msg.direction = REQUEST;
    sync_msg.workers_in_safe_zone = 0;
    sync_msg.battery_level = BATTERY_LEVEL_GOOD;

    k_timer_start(&base_station_periodic_timer, K_NO_WAIT,K_MSEC(PERIOD_TIME_MSEC));
}


static void send_msg(void)
{
    volatile int rc = 0;
    uint32_t new_msg = 0;
    struct k_msgq* cur_queue = NULL;
//    LOG_DBG("Check queues");

    if ( base_station_msgq_tx_msg_prio.used_msgs ) {
//        LOG_DBG("Get message from priority queue");
        rc = k_msgq_get(&base_station_msgq_tx_msg_prio, &base_station_tx_msg, K_NO_WAIT);
        cur_queue = &base_station_msgq_tx_msg_prio;
    }
    else if ( base_station_msgq_tx_msg.used_msgs ) {
//        LOG_DBG("Get message from standart queue");
        rc = k_msgq_get(&base_station_msgq_tx_msg, &base_station_tx_msg, K_NO_WAIT);
        cur_queue = &base_station_msgq_tx_msg;
    }
    else {
        return;
    }

    read_write_message(&new_msg, &base_station_tx_msg, true);
    for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
        base_station_tx_buf[i] = (new_msg & (0x000000FF << i * 8) ) >> i * 8;
        base_station_tx_buf[i] = reverse(base_station_tx_buf[i]);
    }

    if (!base_station_lora_cfg.tx) {
        base_station_lora_cfg.tx = true;
        rc = lora_config(base_station_lora_dev_ptr, &base_station_lora_cfg);
        if (rc < 0) {
            LOG_DBG("Modem not configure!!!");
            k_msgq_put(cur_queue, &base_station_tx_msg, K_NO_WAIT);
            return;
        }
    }
//    LOG_DBG("Send message");
    rc = lora_send(base_station_lora_dev_ptr, base_station_tx_buf, MESSAGE_LEN_IN_BYTES);
//    LOG_DBG("Message sending");
}


static void recv_msg(void)
{
    volatile int rc = -1;
    volatile uint32_t ticks = 0;

    int16_t rssi = 0;
    int8_t snr = 0;

    if (base_station_lora_cfg.tx) {
        base_station_lora_cfg.tx = false;
        rc = lora_config(base_station_lora_dev_ptr, &base_station_lora_cfg);
        if (rc < 0) {
            return;
        }
    }

    ticks = k_ticks_to_ms_floor32(k_timer_remaining_ticks(&base_station_periodic_timer));

    rc = lora_recv(base_station_lora_dev_ptr, base_station_rx_buf, MESSAGE_LEN_IN_BYTES,
                   K_MSEC(ticks), &rssi, &snr);
    if (rc > 0) {
        k_msgq_put(&base_station_msgq_rx_msg, &base_station_rx_buf, K_NO_WAIT);
        k_msgq_put(&base_station_msgq_rssi, &rssi, K_NO_WAIT);
        k_wakeup(proc_task_id);
    }
}


_Noreturn void base_station_proc_task()
{
    uint8_t con_qual_leds_num = 0;
    int16_t rssi = 0;
    uint32_t cur_msg = 0;
    struct message_s tx_msg_proc = {0};
    struct message_s rx_msg_proc = {0};
    struct k_msgq* msgq_cur_msg_tx_ptr = &base_station_msgq_tx_msg; // Default queue
    uint8_t rx_buf_proc[MESSAGE_LEN_IN_BYTES];
    k_sleep(K_FOREVER);
    while(1) {
        if ( base_station_msgq_rx_msg.used_msgs ) {
            k_msgq_get(&base_station_msgq_rx_msg, &rx_buf_proc, K_NO_WAIT);
            k_msgq_get(&base_station_msgq_rssi, &rssi, K_NO_WAIT);
            if (rx_buf_proc[0] == rx_buf_proc[1] == rx_buf_proc[2] == 0) {
                LOG_DBG("Empty message");
                continue;
            }

            /// Processing receive data
            cur_msg = 0;
            for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
                rx_buf_proc[i] = reverse(rx_buf_proc[i]);
                cur_msg |= (rx_buf_proc[i]) << i*8;
            }
//            LOG_DBG("Incoming packet...");
            read_write_message(&cur_msg, &rx_msg_proc, false); // rx_msg struct is fill
            if ( (rx_msg_proc.receiver_addr != RECV_BROADCAST) &&
            (rx_msg_proc.receiver_addr != base_station_cur_dev_addr) ) {
                LOG_DBG("addr = 0x%02x, own addr = 0x%02x", rx_msg_proc.receiver_addr, base_station_cur_dev_addr);
                LOG_DBG("Packet is filtered");
                continue;
            }

//            LOG_DBG("Message direction");

            switch (rx_msg_proc.direction) {
                case REQUEST:
                    LOG_DBG(" REQUEST");
                    LOG_DBG("Message type:");

                    tx_msg_proc.sender_addr = rx_msg_proc.sender_addr;
                    tx_msg_proc.message_type = rx_msg_proc.message_type;
                    tx_msg_proc.workers_in_safe_zone = 0;
                    tx_msg_proc.battery_level = BATTERY_LEVEL_GOOD; // change it after
                    tx_msg_proc.receiver_addr = RECV_BROADCAST;
                    tx_msg_proc.direction = RESPONSE;

                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            tx_msg_proc.direction = REQUEST;
                            switch (rx_msg_proc.sender_addr) {
                                case SEND_BRIGADE_CHIEF:
                                    LOG_DBG("Brigade chief disabled alarm");
                                    break;
                                default:
                                    LOG_DBG("Undefined sender address for this message type");
                                    break;
                            }
                            msgq_cur_msg_tx_ptr = &base_station_msgq_tx_msg;
                            break;
                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            // TODO: On signalization, calculate workers_safe_zone
                            msgq_cur_msg_tx_ptr = &base_station_msgq_tx_msg_prio;
                            break;
                        case MESSAGE_TYPE_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = &base_station_msgq_tx_msg;
                            break;
                            // TODO: Off signalization
                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        default:
                            LOG_DBG("Not correct message type");
                            msgq_cur_msg_tx_ptr = NULL;
                    }
                    break;

                case RESPONSE:
                    LOG_DBG(" RESPONSE");
                    LOG_DBG("Message type:");
                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        case MESSAGE_TYPE_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        default:
                            LOG_DBG("Not correct message type");
                            msgq_cur_msg_tx_ptr = NULL;
                    }
                    break;

                default:
                    LOG_DBG("Not correct message direction");
                    msgq_cur_msg_tx_ptr = NULL;
            }

            if (msgq_cur_msg_tx_ptr) {
                k_msgq_put(msgq_cur_msg_tx_ptr, &tx_msg_proc, K_NO_WAIT);
            }
            con_qual_leds_num = check_rssi(&rssi);
            // ligth up leds
        }
        k_sleep(K_USEC(100));
    }
}


_Noreturn void base_station_modem_task()
{
    volatile uint32_t ticks = 0;
    k_sleep(K_FOREVER);
    while(1) {
        if (current_state.state == TRANSMIT) {
            send_msg();
            current_state = *(current_state.next);
        }
        else {
            recv_msg();
        }
        k_sleep(K_USEC(100));
    }
}


void base_station_periodic_timer_handler(struct k_timer* tim)
{
    current_state = *(current_state.next);
    static uint8_t count = 1;
    if (count == 1) {
        k_msgq_put(&base_station_msgq_tx_msg, &sync_msg, K_NO_WAIT);
        count = 0;
    }
    count++;
    k_wakeup(modem_task_id);
}
/// Function definition area end