//
// Created by rts on 07.02.2022.
//

#include "lora_russia_railways_brigade_chief.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(brigade_chief);


/// My threads ids begin
extern const k_tid_t proc_task_id;
extern const k_tid_t modem_task_id;
/// My threads ids end


static modem_state_t current_state;
/// Structure area begin
static struct message_s disable_alarm_msg;
static struct message_s left_train_passed_msg;
static struct message_s right_train_passed_msg;

static struct msg_info_s disable_alarm_msg_info;
static struct msg_info_s left_train_passed_msg_info;
static struct msg_info_s right_train_passed_msg_info;

struct gpio_callback button_disable_alarm_cb;
struct gpio_callback button_right_train_passed_cb;
struct gpio_callback button_left_train_passed_cb;

const struct device* button_disable_alarm_gpio_dev_ptr;
const struct device* button_left_train_passed_gpio_dev_ptr;
const struct device* button_right_train_passed_gpio_dev_ptr;
/// Structure area end


/// Enum area begin
static enum DEVICE_ADDR_e cur_dev_addr = BRIGADE_CHIEF_ADDR;
static enum WORKERS_IDS_e cur_workers_in_safe_zone = FIRST_PEOPLE_ID;
static enum BATTERY_LEVEL_e cur_battery_level = BATTERY_LEVEL_GOOD;
/// Enum area end


/// Function declaration area begin
void button_disable_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button_left_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button_right_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

static void send_msg(void);
static void recv_msg(void);
static void work_buzzer_handler(struct k_work *item);
static void work_msg_mngr_handler(struct k_work *item);
static void periodic_timer_handler(struct k_timer *tim);
/// Function declaration area end


//// Function definition area begin
void system_init(void)
{
    volatile int rc = -1;
    uint32_t new_msg = 0;
    int16_t rssi = 0;
    int8_t snr = 0;

    /// Buzzer GPIO init begin
    buzzer_dev_ptr = device_get_binding(BUZZER_GPIO_PORT);
    gpio_pin_configure(buzzer_dev_ptr, BUZZER_GPIO_PIN,(GPIO_OUTPUT | GPIO_ACTIVE_HIGH));
    /// Buzzer GPIO init end

    //// Init IRQ (change gpio init after tests) begin
    button_disable_alarm_gpio_dev_ptr = device_get_binding(BUTTON_DISABLE_ALARM_GPIO_PORT);
//        button_left_train_passed_gpio_dev_ptr = device_get_binding(BUTTON_LEFT_TRAIN_PASSED_GPIO_PORT);
//        button_right_train_passed_gpio_dev_ptr = device_get_binding(BUTTON_RIGHT_TRAIN_PASSED_GPIO_PORT);

    gpio_pin_configure(button_disable_alarm_gpio_dev_ptr, BUTTON_DISABLE_ALARM_GPIO_PIN,
                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_HIGH));
//    gpio_pin_configure(button_left_train_passed_gpio_dev_ptr, BUTTON_LEFT_TRAIN_PASSED_GPIO_PIN,
//                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_HIGH));
//    gpio_pin_configure(button_right_train_passed_gpio_dev_ptr, BUTTON_RIGHT_TRAIN_PASSED_GPIO_PIN,
//                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_HIGH));

    gpio_pin_interrupt_configure(button_disable_alarm_gpio_dev_ptr, BUTTON_DISABLE_ALARM_GPIO_PIN,
                                 GPIO_INT_EDGE_TO_ACTIVE);
//    gpio_pin_interrupt_configure(button_left_train_passed_gpio_dev_ptr, BUTTON_LEFT_TRAIN_PASSED_GPIO_PIN,
//                                 GPIO_INT_EDGE_TO_ACTIVE);
//    gpio_pin_interrupt_configure(button_right_train_passed_gpio_dev_ptr, BUTTON_RIGHT_TRAIN_PASSED_GPIO_PIN,
//                                 GPIO_INT_EDGE_TO_ACTIVE);


    gpio_init_callback(&button_disable_alarm_cb, button_disable_alarm_pressed_cb,
                       BIT(BUTTON_DISABLE_ALARM_GPIO_PIN));
//    gpio_init_callback(&button_right_train_passed_cb, button_right_train_pass_pressed_cb,
//                       BIT(BUTTON_RIGHT_TRAIN_PASSED_GPIO_PIN));
//    gpio_init_callback(&button_left_train_passed_cb, button_left_train_pass_pressed_cb,
//                       BIT(BUTTON_LEFT_TRAIN_PASSED_GPIO_PIN));

    gpio_add_callback(button_disable_alarm_gpio_dev_ptr, &button_disable_alarm_cb);
//    gpio_add_callback(button_left_train_passed_gpio_dev_ptr, &button_left_train_passed_cb);
//    gpio_add_callback(button_right_train_passed_gpio_dev_ptr, &button_right_train_passed_cb);
    /// Init IRQ end

    //// Kernel services init begin
    k_work_init(&work_buzzer, work_buzzer_handler);
    k_work_init(&work_msg_mngr, work_msg_mngr_handler);
    k_timer_init(&periodic_timer, periodic_timer_handler, NULL);
    /// Kernel services init end

    current_state = recv_state;

    disable_alarm_msg.receiver_addr = BASE_STATION_ADDR;
    disable_alarm_msg.sender_addr = cur_dev_addr;
    disable_alarm_msg.message_type = MESSAGE_TYPE_DISABLE_ALARM;
    disable_alarm_msg.direction = REQUEST;
    disable_alarm_msg.battery_level = cur_battery_level;
    disable_alarm_msg.workers_in_safe_zone = 0;

    disable_alarm_msg_info.msg = &disable_alarm_msg;
    disable_alarm_msg_info.msg_buf = &msgq_tx_msg_prio;
    disable_alarm_msg_info.resp_is_recv = false;
    disable_alarm_msg_info.req_is_send = false;
    disable_alarm_msg_info.cnt = 0;

    left_train_passed_msg.receiver_addr = BASE_STATION_ADDR;
    left_train_passed_msg.sender_addr = cur_dev_addr;
    left_train_passed_msg.message_type = MESSAGE_TYPE_TRAIN_PASSED;
    left_train_passed_msg.direction = REQUEST;
    left_train_passed_msg.battery_level = cur_battery_level;
    left_train_passed_msg.workers_in_safe_zone = 0;

    left_train_passed_msg_info.msg = &left_train_passed_msg;
    left_train_passed_msg_info.msg_buf = &msgq_tx_msg;
    left_train_passed_msg_info.resp_is_recv = false;
    left_train_passed_msg_info.req_is_send = false;
    left_train_passed_msg_info.cnt = 0;

    right_train_passed_msg.receiver_addr = BASE_STATION_ADDR;
    right_train_passed_msg.sender_addr = cur_dev_addr;
    right_train_passed_msg.message_type = MESSAGE_TYPE_TRAIN_PASSED;
    right_train_passed_msg.direction = REQUEST;
    right_train_passed_msg.battery_level = cur_battery_level;
    right_train_passed_msg.workers_in_safe_zone = 0;

    left_train_passed_msg_info.msg = &right_train_passed_msg;
    left_train_passed_msg_info.msg_buf = &msgq_tx_msg;
    left_train_passed_msg_info.resp_is_recv = false;
    left_train_passed_msg_info.req_is_send = false;
    left_train_passed_msg_info.cnt = 0;

    /// Receive sync message
    rc = lora_recv(lora_dev_ptr, rx_buf, MESSAGE_LEN_IN_BYTES, K_FOREVER, &rssi, &snr);
    k_sleep(K_MSEC(DELAY_TIME_MSEC));
    k_timer_start(&periodic_timer, K_MSEC(DURATION_TIME_MSEC), K_MSEC(PERIOD_TIME_MSEC));
}


static void send_msg(void)
{
    volatile int rc = 0;
    uint32_t new_msg = 0;
    struct k_msgq* cur_queue = NULL;
//    LOG_DBG("Check queues");

    if (msgq_tx_msg_prio.used_msgs) {
//        LOG_DBG("Get message from priority queue");
        k_msgq_get(&msgq_tx_msg_prio, &tx_msg, K_NO_WAIT);
        cur_queue = &msgq_tx_msg_prio;
    } else if (msgq_tx_msg.used_msgs) {
//        LOG_DBG("Get message from standart queue");
        k_msgq_get(&msgq_tx_msg, &tx_msg, K_NO_WAIT);
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
    if (current_state.state == TRANSMIT) {
        rc = lora_send(lora_dev_ptr, tx_buf, MESSAGE_LEN_IN_BYTES);
    }
    else {
        return;
    }
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
    if ( current_state.state == RECEIVE ) {
        rc = lora_recv(lora_dev_ptr, rx_buf, MESSAGE_LEN_IN_BYTES,
                       K_MSEC(ticks), &rssi, &snr);
    } else {
        return;
    }

    if (rc > 0) {
        if (IS_SYNC_MSG) {
            LOG_DBG(" REQUEST");
            LOG_DBG(" MESSAGE_TYPE_SYNC");
            k_timer_stop(&periodic_timer);
            current_state = recv_state;
            // little delay to account execution time
            k_sleep(K_MSEC(DELAY_TIME_MSEC));
            k_timer_start(&periodic_timer, K_MSEC(DURATION_TIME_MSEC), K_MSEC(PERIOD_TIME_MSEC));
        }
        k_msgq_put(&msgq_rx_msg, &rx_buf, K_NO_WAIT);
        k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);
        k_wakeup(proc_task_id);
    }
}


_Noreturn void brigade_chief_proc_task(void)
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
        if (msgq_rx_msg.used_msgs) {
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
//                    LOG_DBG("Message type:");

                    tx_msg_proc.sender_addr = cur_dev_addr;
                    tx_msg_proc.message_type = rx_msg_proc.message_type;
                    tx_msg_proc.workers_in_safe_zone = 0;
                    tx_msg_proc.direction = RESPONSE;
                    tx_msg_proc.battery_level = BATTERY_LEVEL_GOOD; // change it after
                    tx_msg_proc.receiver_addr = BASE_STATION_ADDR;

                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_SYNC:
                            break;
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            // TODO Indication for signalman and brigade chief
                            switch (rx_msg_proc.sender_addr) {
                                case BASE_STATION_ADDR:
                                    // TODO Indicate LED "Base station disabled alarm"
                                    LOG_DBG("Base station disabled alarm");
                                    break;
                                default:
                                    LOG_DBG("Undefined sender address for this message type");
                                    break;
                            }
                            break;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            /// TODO Indication for signalman and brigade chief
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            if (rx_msg_proc.sender_addr == cur_dev_addr)
                                msgq_cur_msg_tx_ptr = &msgq_tx_msg_prio; // For response message
                            else
                                msgq_cur_msg_tx_ptr = NULL; // Do nothing, because this message for base station
                            break;

                        case MESSAGE_TYPE_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = NULL; // Do nothing, because this message for base station
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
                            switch (rx_msg_proc.sender_addr) {
                                case BRIGADE_CHIEF_ADDR:
                                    // TODO Indicate LED "Brigade chief disabled alarm"
                                    LOG_DBG("Brigade chief disabled alarm");
                                    break;
                                default:
                                    LOG_DBG("Undefined sender address for this message type");
                                    break;
                            }
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
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
            update_indication(rx_msg_proc.workers_in_safe_zone, true, leds_num, true);
        }
        k_sleep(K_USEC(100));
    }
}


_Noreturn void brigade_chief_modem_task(void)
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
    lora_cfg.tx = false;

    lora_dev_ptr = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    if (!device_is_ready(lora_dev_ptr)) {
        k_sleep(K_FOREVER);
    }
    if ( lora_config(lora_dev_ptr, &lora_cfg) < 0 ) {
        k_sleep(K_FOREVER);
    }
    /// Lora config end

    system_init();

    /// Receive sync message begin
    lora_recv(lora_dev_ptr, rx_buf, MESSAGE_LEN_IN_BYTES, K_FOREVER, &rssi, &snr);
    k_sleep(K_MSEC(DELAY_TIME_MSEC));
    k_timer_start(&periodic_timer, K_MSEC(DURATION_TIME_MSEC),K_MSEC(PERIOD_TIME_MSEC));

    k_msgq_put(&msgq_rx_msg, &rx_buf, K_NO_WAIT);
    k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);
    k_wakeup(proc_task_id);
    /// Receive sync message end

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


void button_disable_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_submit(&work_buzzer);
    disable_alarm_msg_info.req_is_send = true;
    k_msgq_put(&msgq_tx_msg_prio, &disable_alarm_msg, K_NO_WAIT);
    LOG_DBG("Button alarm pressed");
}


void button_left_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_submit(&work_buzzer);
    left_train_passed_msg_info.req_is_send = true;
    k_msgq_put(&msgq_tx_msg, &left_train_passed_msg, K_NO_WAIT);
    LOG_DBG("Button left train pass pressed");
}


void button_right_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_submit(&work_buzzer);
    right_train_passed_msg_info.req_is_send = true;
    k_msgq_put(&msgq_tx_msg, &right_train_passed_msg, K_NO_WAIT);
    LOG_DBG("Button right train pass pressed");
}

static void periodic_timer_handler(struct k_timer *tim)
{
//    k_msgq_put(&peripheral_msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT);
    current_state = *current_state.next;
    k_wakeup(modem_task_id);
}


//void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
// {
//    struct message_s anti_dream_msg = {0};
//    anti_dream_msg.receiver_addr = RECV_BASE_STATION;
//    anti_dream_msg.sender_addr = cur_dev_addr;
//    anti_dream_msg.message_type = MESSAGE_TYPE_ANTI_DREAM;
//    anti_dream_msg.direction = RESPONSE;
////    tx_msg.battery_level =
//    anti_dream_msg.workers_in_safe_zone = 0;
//    k_msgq_put(&msgq_tx_msg, &anti_dream_msg, K_NO_WAIT);
//}

static void work_buzzer_handler(struct k_work *item)
{
    gpio_pin_set(buzzer_dev_ptr, BUZZER_GPIO_PIN, 1);
    k_msleep(20);
    gpio_pin_set(buzzer_dev_ptr, BUZZER_GPIO_PIN, 0);
    k_msleep(20);
}


static void work_msg_mngr_handler(struct k_work *item)
{
    if (disable_alarm_msg_info.req_is_send) {
        if ((disable_alarm_msg_info.cnt++ == WAITING_PERIOD_NUM)) {
            if (!disable_alarm_msg_info.resp_is_recv) {
                k_msgq_put(disable_alarm_msg_info.msg_buf, disable_alarm_msg_info.msg, K_NO_WAIT);
            } else {
                disable_alarm_msg_info.req_is_send = false;
                disable_alarm_msg_info.resp_is_recv = false;
            }
            disable_alarm_msg_info.cnt = 0;
        }
    } else if (left_train_passed_msg_info.req_is_send) {
        if (left_train_passed_msg_info.cnt++ == WAITING_PERIOD_NUM) {
            if (!left_train_passed_msg_info.resp_is_recv) {
                k_msgq_put(left_train_passed_msg_info.msg_buf, left_train_passed_msg_info.msg, K_NO_WAIT);
            } else {
                left_train_passed_msg_info.req_is_send = false;
                left_train_passed_msg_info.resp_is_recv = false;
            }
            left_train_passed_msg_info.cnt = 0;
        } else if (right_train_passed_msg_info.req_is_send) {
            if (right_train_passed_msg_info.cnt++ == WAITING_PERIOD_NUM) {
                if (!right_train_passed_msg_info.resp_is_recv) {
                    k_msgq_put(right_train_passed_msg_info.msg_buf, right_train_passed_msg_info.msg, K_NO_WAIT);
                } else {
                    right_train_passed_msg_info.req_is_send = false;
                    right_train_passed_msg_info.resp_is_recv = false;
                }
                right_train_passed_msg_info.cnt = 0;
            }
        }
    }
}