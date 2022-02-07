//
// Created by rts on 21.01.2022.
//
#include "lora_russia_railways_peripheral.h"
#include <drivers/gpio.h>

#include <logging/log.h>
    LOG_MODULE_REGISTER(peripheral);


#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

uint8_t peripheral_tx_buf[MESSAGE_LEN_IN_BYTES] = {0};
uint8_t peripheral_rx_buf[MESSAGE_LEN_IN_BYTES] = {0};

/// My threads ids begin
extern const k_tid_t proc_task_id;
extern const k_tid_t modem_task_id;
extern const k_tid_t start_system_id;
/// My threads ids end


static struct message_s alarm_msg;

static modem_state_t recv_state;
static modem_state_t transmit_state;
static modem_state_t current_state;


/// Structure area begin
struct device* lora_dev_ptr;
struct lora_modem_config lora_cfg;

// priority queue for sending messages
K_MSGQ_DEFINE(peripheral_msgq_tx_msg_prio, sizeof(struct message_s), QUEUE_LEN_IN_ELEMENTS, 1);
// none priority queue for sending messages
K_MSGQ_DEFINE(peripheral_msgq_tx_msg, sizeof(struct message_s), QUEUE_LEN_IN_ELEMENTS, 1);
// queue for receiving messages
K_MSGQ_DEFINE(peripheral_msgq_rx_msg, MESSAGE_LEN_IN_BYTES, QUEUE_LEN_IN_ELEMENTS, 1);
// queue for rssi values
K_MSGQ_DEFINE(peripheral_msgq_rssi, sizeof(int16_t), QUEUE_LEN_IN_ELEMENTS, 2);


// periodic timer (for syncro)
struct k_timer peripheral_periodic_timer;

struct gpio_callback button_anti_dream_cb;
struct gpio_callback button_alarm_cb;

//struct k_sem sem_modem_busy;

struct device* button_alarm_gpio_dev_ptr;
struct device* button_anti_dream_gpio_dev_ptr;

struct message_s peripheral_tx_msg = {0};
/// Structure area end


/// Enum area begin
enum receiver_addr peripheral_cur_dev_addr = RECV_SIGNALMAN_1;

enum workers_ids peripheral_cur_workers_in_safe_zone = FIRST_PEOPLE_ID;
enum battery_level peripheral_cur_battery_level = BATTERY_LEVEL_GOOD;
/// Enum area end


/// Function declaration area begin
void peripheral_periodic_timer_handler(struct k_timer* tim); // callback for periodic_timer
void peripheral_button_alarm_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins);
void peripheral_button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins);

static void send_msg(void);
static void recv_msg(void);
/// Function declaration area end


//// Function definition area begin
void peripheral_system_init()
{
    /// LoRa init begin
    lora_cfg.frequency = 433000000;
    lora_cfg.bandwidth = BW_125_KHZ;
    lora_cfg.datarate = SF_12;
    lora_cfg.preamble_len = 8;
    lora_cfg.coding_rate = CR_4_5;
    lora_cfg.tx_power = 5;
    lora_cfg.tx = false;

    lora_dev_ptr = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    if (!device_is_ready(lora_dev_ptr)) {
        k_sleep(K_FOREVER);
    }
    if ( lora_config(lora_dev_ptr, &lora_cfg) < 0 ) {
        k_sleep(K_FOREVER);
    }
    /// LoRa init end

    //// Init IRQ (change gpio init after tests) begin
    button_alarm_gpio_dev_ptr = device_get_binding(BUTTON_ALARM_GPIO_PORT);
//    button_anti_dream_gpio_dev_ptr = device_get_binding(BUTTON_ANTI_DREAM_GPIO_PORT);

    gpio_pin_configure(button_alarm_gpio_dev_ptr, BUTTON_ALARM_GPIO_PIN,
                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));
//    gpio_pin_configure(button_anti_dream_gpio_dev_ptr, BUTTON_ANTI_DREAM_GPIO_PIN,
//                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));

    gpio_pin_interrupt_configure(button_alarm_gpio_dev_ptr, BUTTON_ALARM_GPIO_PIN, GPIO_INT_EDGE_TO_ACTIVE);
//    gpio_pin_interrupt_configure(&button_anti_dream_gpio_dev_ptr, BUTTON_ANTI_DREAM_GPIO_PORT,
//                                 GPIO_INT_EDGE_RISING);

    gpio_init_callback(&button_alarm_cb, peripheral_button_alarm_pressed_cb, BIT(BUTTON_ALARM_GPIO_PIN));
//    gpio_init_callback(button_anti_dream_cb_ptr, button_anti_dream_pressed_cb,
//                       BIT(BUTTON_ANTI_DREAM_GPIO_PIN));

    gpio_add_callback(button_alarm_gpio_dev_ptr, &button_alarm_cb);
//    gpio_add_callback(button_anti_dream_gpio_dev_ptr, button_anti_dream_cb_ptr);
    /// Init IRQ end

    //// Kernel services init begin
//    k_sem_init(&sem_modem_busy, SEM_LORA_BUSY_INIT_VAL, SEM_LORA_BUSY_LIM);

    k_timer_init(&peripheral_periodic_timer, peripheral_periodic_timer_handler, NULL);
//    k_timer_init(&calibration_timer, calibration_timer_handler, NULL);
    /// Kernel services init end
}


void peripheral_start_system(void)
{
    volatile int rc = -1;
    uint32_t new_msg = 0;
    int16_t rssi = 0;
    int8_t snr = 0;

    peripheral_system_init();

    /// Filling list elements begin
    recv_state.next = &transmit_state;
    recv_state.state = RECEIVE;

    transmit_state.next = &recv_state;
    transmit_state.state = TRANSMIT;

    current_state = recv_state;
    /// Filling list elements end

    alarm_msg.receiver_addr = RECV_BASE_STATION;
    alarm_msg.sender_addr = peripheral_cur_dev_addr;
    alarm_msg.message_type = MESSAGE_TYPE_ALARM;
    alarm_msg.direction = REQUEST;
//    tx_msg.battery_level
    alarm_msg.workers_in_safe_zone = 0;

    /// Receive sync message
    rc = lora_recv(lora_dev_ptr, peripheral_rx_buf, MESSAGE_LEN_IN_BYTES, K_FOREVER, &rssi, &snr);
    k_sleep(K_MSEC(DELAY_TIME_MSEC));
    k_timer_start(&peripheral_periodic_timer, K_MSEC(SLOT_TIME_MSEC*(DEVICE_NUM-1)),
                  K_MSEC(4*SLOT_TIME_MSEC));
}

static void send_msg(void)
{
    volatile int rc = 0;
    uint32_t new_msg = 0;
    struct k_msgq* cur_queue = NULL;
//    LOG_DBG("Check queues");

    if ( peripheral_msgq_tx_msg_prio.used_msgs ) {
//        LOG_DBG("Get message from priority queue");
        k_msgq_get(&peripheral_msgq_tx_msg_prio, &peripheral_tx_msg, K_NO_WAIT);
        cur_queue = &peripheral_msgq_tx_msg_prio;
    }
    else if ( peripheral_msgq_tx_msg.used_msgs ) {
//        LOG_DBG("Get message from standart queue");
        k_msgq_get(&peripheral_msgq_tx_msg, &peripheral_tx_msg, K_NO_WAIT);
        cur_queue = &peripheral_msgq_tx_msg;
    }
    else {
        return;
    }

    read_write_message(&new_msg, &peripheral_tx_msg, true);
    for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
        peripheral_tx_buf[i] = (new_msg & (0x000000FF << i * 8) ) >> i * 8;
        peripheral_tx_buf[i] = reverse(peripheral_tx_buf[i]);
    }

    if (!lora_cfg.tx) {
        lora_cfg.tx = true;
        rc = lora_config(lora_dev_ptr, &lora_cfg);
        if (rc < 0) {
            LOG_DBG("Modem not configure!!!");
            k_msgq_put(cur_queue, &peripheral_tx_msg, K_NO_WAIT);
            return;
        }
    }
//    LOG_DBG("Send message");
    if ( current_state.state == TRANSMIT ) {
        rc = lora_send(lora_dev_ptr, peripheral_tx_buf, MESSAGE_LEN_IN_BYTES);
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

    ticks = k_ticks_to_ms_floor32(k_timer_remaining_ticks(&peripheral_periodic_timer));
    if ( current_state.state == RECEIVE ) {
        rc = lora_recv(lora_dev_ptr, peripheral_rx_buf, MESSAGE_LEN_IN_BYTES,K_MSEC(ticks), &rssi, &snr);
    }
    else {
        return;
    }
    if (rc > 0) {
        if (IS_SYNC_MSG) {
            LOG_DBG(" REQUEST");
            LOG_DBG(" MESSAGE_TYPE_SYNC");
            k_timer_stop(&peripheral_periodic_timer);
            current_state = recv_state;
            // little delay to account execution time
            k_sleep(K_MSEC(DELAY_TIME_MSEC));
            k_timer_start(&peripheral_periodic_timer, K_MSEC(SLOT_TIME_MSEC*(DEVICE_NUM-1)),
                          K_MSEC(4*SLOT_TIME_MSEC));
        }
        else {
            k_msgq_put(&peripheral_msgq_rx_msg, &peripheral_rx_buf, K_NO_WAIT);
            k_msgq_put(&peripheral_msgq_rssi, &rssi, K_NO_WAIT);
            k_wakeup(proc_task_id);
        }
    }
}


_Noreturn void peripheral_proc_task()
{
    uint8_t con_qual_leds_num = 0;
    int16_t rssi = 0;
    uint32_t cur_msg = 0;
    struct message_s tx_msg_proc = {0};
    struct message_s rx_msg_proc = {0};
    struct k_msgq* msgq_cur_msg_tx_ptr = &peripheral_msgq_tx_msg; // Default queue
    uint8_t rx_buf_proc[MESSAGE_LEN_IN_BYTES];
    k_sleep(K_FOREVER);
    while(1) {
        if ( peripheral_msgq_rx_msg.used_msgs ) {
            k_msgq_get(&peripheral_msgq_rx_msg, &rx_buf_proc, K_NO_WAIT);
            k_msgq_get(&peripheral_msgq_rssi, &rssi, K_NO_WAIT);
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
            (rx_msg_proc.receiver_addr != peripheral_cur_dev_addr) ) {
                LOG_DBG("addr = 0x%02x, own addr = 0x%02x", rx_msg_proc.receiver_addr, peripheral_cur_dev_addr);
                LOG_DBG("Packet is filtered");
                continue;
            }

//            LOG_DBG("Message direction");
            switch (rx_msg_proc.direction) {
                case REQUEST:
                    LOG_DBG(" REQUEST");
//                    LOG_DBG("Message type:");

                    tx_msg_proc.sender_addr = peripheral_cur_dev_addr;
                    tx_msg_proc.message_type = rx_msg_proc.message_type;
                    tx_msg_proc.workers_in_safe_zone = 0;
                    tx_msg_proc.direction = RESPONSE;
                    tx_msg_proc.battery_level = BATTERY_LEVEL_GOOD; // change it after
                    tx_msg_proc.receiver_addr = RECV_BASE_STATION;

                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            msgq_cur_msg_tx_ptr = &peripheral_msgq_tx_msg;
                            // TODO Indication for signalman and brigade chief
                            switch (rx_msg_proc.sender_addr) {
                                case SEND_BASE_STATION:
                                    // TODO Indicate LED "Base station disabled alarm"
                                    LOG_DBG("Base station disabled alarm");
                                    break;
                                case SEND_BRIGADE_CHIEF:
                                    // TODO Indicate LED "Brigade chief disabled alarm"
                                    LOG_DBG("Brigade chief disabled alarm");
                                    break;
                                default:
                                    LOG_DBG("Undefined sender address for this message type");
                                    break;
                            }
                            break;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = &peripheral_msgq_tx_msg;
                            /// TODO Indication for signalman and brigade chief
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            if (rx_msg_proc.sender_addr == peripheral_cur_dev_addr)
                                msgq_cur_msg_tx_ptr = &peripheral_msgq_tx_msg_prio; // For response message
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
            con_qual_leds_num = check_rssi(&rssi);
            // ligth up leds
        }
        k_sleep(K_USEC(100));
    }
}


_Noreturn void peripheral_modem_task()
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


void peripheral_button_alarm_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    LOG_DBG("Button alarm pressed");
    k_msgq_put(&peripheral_msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT);
}


void peripheral_periodic_timer_handler(struct k_timer* tim)
{
//    k_msgq_put(&peripheral_msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT);
    current_state = *current_state.next;
    k_wakeup(modem_task_id);
}

//void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
//    struct message_s anti_dream_msg = {0};
//    anti_dream_msg.receiver_addr = RECV_BASE_STATION;
//    anti_dream_msg.sender_addr = cur_dev_addr;
//    anti_dream_msg.message_type = MESSAGE_TYPE_ANTI_DREAM;
//    anti_dream_msg.direction = RESPONSE;
////    tx_msg.battery_level =
//    anti_dream_msg.workers_in_safe_zone = 0;
//    k_msgq_put(&msgq_tx_msg, &anti_dream_msg, K_NO_WAIT);
//}
/// Function definition area end
