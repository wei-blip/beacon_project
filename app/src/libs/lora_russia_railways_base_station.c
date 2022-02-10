//
// Created by rts on 05.02.2022.
//

#include "lora_russia_railways_base_station.h"
#include <drivers/gpio.h>

#include <logging/log.h>
    LOG_MODULE_REGISTER(base_station);

static bool alarm_is_active = false;

/// My threads ids begin
extern const k_tid_t proc_task_id;
extern const k_tid_t modem_task_id;
/// My threads ids end

static modem_state_t current_state;
static struct message_s sync_msg;

/// Enum area begin
static enum DEVICE_ADDR_e cur_dev_addr = BASE_STATION_ADDR;
static enum WORKERS_IDS_e cur_workers_in_safe_zone = FIRST_PEOPLE_ID;
static enum BATTERY_LEVEL_e cur_battery_level = BATTERY_LEVEL_GOOD;
/// Enum area end


/// Function declaration area begin
static void system_init(void);
static void send_msg(void);
static void recv_msg(void);
static void work_buzzer_handler(struct k_work *item);
static void periodic_timer_handler(struct k_timer* tim); // callback for periodic_timer
/// Function declaration area end


/// Function definition area begin
static void system_init(void)
{
    /// Buzzer init begin
    buzzer_dev_ptr = device_get_binding(BUZZER_GPIO_PORT);
    gpio_pin_configure(buzzer_dev_ptr, BUZZER_GPIO_PIN,(GPIO_OUTPUT | GPIO_ACTIVE_HIGH));
    /// Buzzer init end

    //// Kernel services init begin
    k_work_init(&work_buzzer, work_buzzer_handler);
    k_timer_init(&periodic_timer, periodic_timer_handler, NULL);
    /// Kernel services init end

    current_state = recv_state;

    /// Send sync message
    sync_msg.receiver_addr = BROADCAST_ADDR;
    sync_msg.sender_addr = cur_dev_addr;
    sync_msg.message_type = MESSAGE_TYPE_SYNC;
    sync_msg.direction = REQUEST;
    sync_msg.workers_in_safe_zone = 0;
    sync_msg.battery_level = BATTERY_LEVEL_GOOD;

    k_timer_start(&periodic_timer, K_NO_WAIT,K_MSEC(PERIOD_TIME_MSEC));
}


static void send_msg(void)
{
    volatile int rc = 0;
    uint32_t new_msg = 0;
    struct k_msgq* cur_queue = NULL;
//    LOG_DBG("Check queues");

    if (msgq_tx_msg_prio.used_msgs) {
//        LOG_DBG("Get message from priority queue");
        rc = k_msgq_get(&msgq_tx_msg_prio, &tx_msg, K_NO_WAIT);
        cur_queue = &msgq_tx_msg_prio;
    } else if (msgq_tx_msg.used_msgs) {
//        LOG_DBG("Get message from standart queue");
        rc = k_msgq_get(&msgq_tx_msg, &tx_msg, K_NO_WAIT);
        cur_queue = &msgq_tx_msg;
    } else {
        return;
    }

    read_write_message(&new_msg, &tx_msg, true);
    for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
        tx_buf[i] = (new_msg & (0x000000FF << i * 8) ) >> i * 8;
        tx_buf[i] = reverse(tx_buf[i]);
    }

    if (!lora_cfg.tx) {
        lora_cfg.tx = true;
        rc = lora_config(lora_dev_ptr, &lora_cfg);
        if (rc < 0) {
            LOG_DBG("Modem not configure!!!");
            k_msgq_put(cur_queue, &tx_msg, K_NO_WAIT);
            return;
        }
    }
//    LOG_DBG("Send message");
    rc = lora_send(lora_dev_ptr, tx_buf, MESSAGE_LEN_IN_BYTES);
//    LOG_DBG("Message sending");

}


static void recv_msg(void)
{
    volatile int rc = -1;
    volatile uint32_t ticks = 0;

    int16_t rssi = 0;
    int8_t snr = 0;

    if (lora_cfg.tx) {
        lora_cfg.tx = false;
        rc = lora_config(lora_dev_ptr, &lora_cfg);
        if (rc < 0) {
            return;
        }
    }

    ticks = k_ticks_to_ms_floor32(k_timer_remaining_ticks(&periodic_timer));

    rc = lora_recv(lora_dev_ptr, rx_buf, MESSAGE_LEN_IN_BYTES,
                   K_MSEC(ticks), &rssi, &snr);
    if (rc > 0) {
        k_msgq_put(&msgq_rx_msg, &rx_buf, K_NO_WAIT);
        k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);
        k_wakeup(proc_task_id);
    }
}


_Noreturn void base_station_proc_task()
{
    uint8_t leds_num = 0;
    int16_t rssi = 0;
    uint32_t cur_msg = 0;
    struct message_s tx_msg_proc = {0};
    struct message_s rx_msg_proc = {0};
    struct k_msgq* msgq_cur_msg_tx_ptr = &msgq_tx_msg; // Default queue
    uint8_t rx_buf_proc[MESSAGE_LEN_IN_BYTES];
    k_sleep(K_FOREVER);
    while(1) {
        if ( msgq_rx_msg.used_msgs ) {
            k_msgq_get(&msgq_rx_msg, &rx_buf_proc, K_NO_WAIT);
            k_msgq_get(&msgq_rssi, &rssi, K_NO_WAIT);
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
            if ( (rx_msg_proc.receiver_addr != BROADCAST_ADDR) &&
            (rx_msg_proc.receiver_addr != cur_dev_addr) ) {
                LOG_DBG("addr = 0x%02x, own addr = 0x%02x", rx_msg_proc.receiver_addr, cur_dev_addr);
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
                    tx_msg_proc.receiver_addr = BROADCAST_ADDR;
                    tx_msg_proc.direction = RESPONSE;

                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_SYNC:
                            break;
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            switch (rx_msg_proc.sender_addr) {
                                case BRIGADE_CHIEF_ADDR:
                                    LOG_DBG("Brigade chief disabled alarm");
                                    alarm_is_active = false;
                                    k_work_submit(&work_buzzer);
                                    break;
                                default:
                                    LOG_DBG("Undefined sender address for this message type");
                                    break;
                            }
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            break;
                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            // TODO: On signalization, calculate workers_safe_zone
                            alarm_is_active = true;
                            k_work_submit(&work_buzzer);
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg_prio;
                            break;
                        case MESSAGE_TYPE_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
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
            leds_num = check_rssi(rssi);
            update_indication(0, false, leds_num, true);
        }
        k_sleep(K_USEC(100));
    }
}


_Noreturn void base_station_modem_task()
{
    int8_t snr;
    int16_t rssi;
    volatile uint32_t ticks = 0;

    /// Lora config begin
    lora_cfg.frequency = 433000000;
    lora_cfg.bandwidth = BW_125_KHZ;
    lora_cfg.datarate = SF_12;
    lora_cfg.preamble_len = 8;
    lora_cfg.coding_rate = CR_4_5;
    lora_cfg.tx_power = 0;
    lora_cfg.tx = true;

    lora_dev_ptr = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    if (!device_is_ready(lora_dev_ptr)) {
        k_sleep(K_FOREVER);
    }
    if (lora_config(lora_dev_ptr, &lora_cfg) < 0) {
        k_sleep(K_FOREVER);
    }
    /// Lora config end

    system_init();

    k_sleep(K_FOREVER);
    while(1) {
        if (current_state.state == TRANSMIT) {
            send_msg();
            current_state = *current_state.next;
            recv_msg();
        }
        else {
            recv_msg();
        }
        k_sleep(K_USEC(100));
    }
}


static void periodic_timer_handler(struct k_timer* tim)
{
    current_state = *current_state.next;
    static uint8_t count = 10;
    if (count == 10) {
        k_msgq_put(&msgq_tx_msg, &sync_msg, K_NO_WAIT);
        count = 0;
    }
    count++;
    k_wakeup(modem_task_id);
}


static void work_buzzer_handler(struct k_work *item)
{
    gpio_pin_set(buzzer_dev_ptr, BUZZER_GPIO_PIN, (int)(alarm_is_active));
}
/// Function definition area end