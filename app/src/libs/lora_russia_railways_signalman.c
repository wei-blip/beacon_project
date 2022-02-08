//
// Created by rts on 21.01.2022.
//
#include "lora_russia_railways_signalman.h"
#include <drivers/gpio.h>

#include <logging/log.h>
    LOG_MODULE_REGISTER(peripheral);


/// My threads ids begin
extern const k_tid_t proc_task_id;
extern const k_tid_t modem_task_id;
extern const k_tid_t start_system_id;
/// My threads ids end


static struct message_s alarm_msg;

static modem_state_t signalman_recv_state;
static modem_state_t signalman_transmit_state;
static modem_state_t signalman_current_state;


/// Structure area begin
struct gpio_callback signalman_button_anti_dream_cb;
struct gpio_callback signalman_button_alarm_cb;
struct gpio_callback signalman_button_train_passed_cb;


struct device *signalman_button_alarm_gpio_dev_ptr;
struct device *signalman_button_anti_dream_gpio_dev_ptr;
struct device *signalman_button_train_passed_gpio_dev_ptr;
/// Structure area end


/// Enum area begin
enum receiver_addr signalman_cur_dev_addr = RECV_SIGNALMAN_1;

enum workers_ids signalman_cur_workers_in_safe_zone = FIRST_PEOPLE_ID;
enum battery_level signalman_cur_battery_level = BATTERY_LEVEL_GOOD;
/// Enum area end


/// Function declaration area begin
void signalman_periodic_timer_handler(struct k_timer *tim); // callback for periodic_timer
void signalman_button_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void signalman_button_anti_dream_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void signalman_button_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

static void send_msg(void);
static void recv_msg(void);
/// Function declaration area end


//// Function definition area begin
void signalman_system_init(void)
{
    system_init();
    //// Init IRQ (change gpio init after tests) begin
    signalman_button_alarm_gpio_dev_ptr = device_get_binding(BUTTON_ALARM_GPIO_PORT);
//    button_anti_dream_gpio_dev_ptr = device_get_binding(BUTTON_ANTI_DREAM_GPIO_PORT);
//    signalman_button_train_passed_gpio_dev_ptr = device_get_binding(BUTTON_TRAIN_PASSED_GPIO_PORT);

    gpio_pin_configure(signalman_button_alarm_gpio_dev_ptr, BUTTON_ALARM_GPIO_PIN,
                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));
//    gpio_pin_configure(button_anti_dream_gpio_dev_ptr, BUTTON_ANTI_DREAM_GPIO_PIN,
//                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));
//    gpio_pin_configure(signalman_button_train_passed_gpio_dev_ptr, BUTTON_TRAIN_PASSED_GPIO_PIN,
//                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));

    gpio_pin_interrupt_configure(signalman_button_alarm_gpio_dev_ptr, BUTTON_ALARM_GPIO_PIN,
                                 GPIO_INT_EDGE_TO_ACTIVE);
//    gpio_pin_interrupt_configure(&button_anti_dream_gpio_dev_ptr, BUTTON_ANTI_DREAM_GPIO_PORT,
//                                 GPIO_INT_EDGE_RISING);
//    gpio_pin_interrupt_configure(signalman_button_train_passed_gpio_dev_ptr, BUTTON_TRAIN_PASSED_GPIO_PIN,
//                                 GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&signalman_button_alarm_cb, signalman_button_alarm_pressed_cb,
                       BIT(BUTTON_ALARM_GPIO_PIN));
//    gpio_init_callback(button_anti_dream_cb_ptr, button_anti_dream_pressed_cb,
//                       BIT(BUTTON_ANTI_DREAM_GPIO_PIN));
//    gpio_init_callback(&signalman_button_train_passed_cb, signalman_button_train_pass_pressed_cb,
//                       BIT(BUTTON_TRAIN_PASSED_GPIO_PIN));

    gpio_add_callback(signalman_button_alarm_gpio_dev_ptr, &signalman_button_alarm_cb);
//    gpio_add_callback(button_anti_dream_gpio_dev_ptr, button_anti_dream_cb_ptr);
//    gpio_add_callback(signalman_button_train_passed_gpio_dev_ptr, &signalman_button_train_passed_cb);
    /// Init IRQ end

    /// Kernel services init begin
    k_timer_init(&periodic_timer, signalman_periodic_timer_handler, NULL);
    /// Kernel services init end

    /// Light down LEDs begin
    update_indication(0, true, 0, true);
    /// Light down LEDs end
}


void signalman_start_system(void)
{
    volatile int rc = -1;
    uint32_t new_msg = 0;
    int16_t rssi = 0;
    int8_t snr = 0;

    signalman_system_init();

    /// Filling list elements begin
    signalman_recv_state.next = &signalman_transmit_state;
    signalman_recv_state.state = RECEIVE;

    signalman_transmit_state.next = &signalman_recv_state;
    signalman_transmit_state.state = TRANSMIT;

    signalman_current_state = signalman_recv_state;
    /// Filling list elements end

    alarm_msg.receiver_addr = RECV_BASE_STATION;
    alarm_msg.sender_addr = signalman_cur_dev_addr;
    alarm_msg.message_type = MESSAGE_TYPE_ALARM;
    alarm_msg.direction = REQUEST;
//    tx_msg.battery_level
    alarm_msg.workers_in_safe_zone = 0;

    /// Receive sync message
    rc = lora_recv(lora_dev_ptr, rx_buf, MESSAGE_LEN_IN_BYTES, K_FOREVER, &rssi, &snr);
    k_sleep(K_MSEC(DELAY_TIME_MSEC));
    k_timer_start(&periodic_timer, K_MSEC(DURATION_TIME_MSEC),K_MSEC(PERIOD_TIME_MSEC));
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
    if ( signalman_current_state.state == TRANSMIT ) {
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
    if ( signalman_current_state.state == RECEIVE ) {
        rc = lora_recv(lora_dev_ptr, rx_buf, MESSAGE_LEN_IN_BYTES, K_MSEC(ticks), &rssi, &snr);
    }
    else {
        return;
    }
    if (rc > 0) {
        if (IS_SYNC_MSG) {
            LOG_DBG(" REQUEST");
            LOG_DBG(" MESSAGE_TYPE_SYNC");
            k_timer_stop(&periodic_timer);
            signalman_current_state = signalman_recv_state;
            // little delay to account execution time
            k_sleep(K_MSEC(DELAY_TIME_MSEC));
            k_timer_start(&periodic_timer, K_MSEC(DURATION_TIME_MSEC),
                          K_MSEC(PERIOD_TIME_MSEC));
        }
        k_msgq_put(&msgq_rx_msg, &rx_buf, K_NO_WAIT);
        k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);
        k_wakeup(proc_task_id);
    }
}


_Noreturn void signalman_proc_task()
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
            if ( (rx_msg_proc.receiver_addr != RECV_BROADCAST) &&
            (rx_msg_proc.receiver_addr != signalman_cur_dev_addr) ) {
                LOG_DBG("addr = 0x%02x, own addr = 0x%02x", rx_msg_proc.receiver_addr, signalman_cur_dev_addr);
                LOG_DBG("Packet is filtered");
                continue;
            }

//            LOG_DBG("Message direction");
            switch (rx_msg_proc.direction) {
                case REQUEST:
                    LOG_DBG(" REQUEST");
//                    LOG_DBG("Message type:");

                    tx_msg_proc.sender_addr = signalman_cur_dev_addr;
                    tx_msg_proc.message_type = rx_msg_proc.message_type;
                    tx_msg_proc.workers_in_safe_zone = 0;
                    tx_msg_proc.direction = RESPONSE;
                    tx_msg_proc.battery_level = BATTERY_LEVEL_GOOD; // change it after
                    tx_msg_proc.receiver_addr = RECV_BASE_STATION;

                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            msgq_cur_msg_tx_ptr = &msgq_rx_msg;
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
                            msgq_cur_msg_tx_ptr = &msgq_rx_msg;
                            /// TODO Indication for signalman and brigade chief
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            if (rx_msg_proc.sender_addr == signalman_cur_dev_addr)
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
            update_indication(rx_msg_proc.workers_in_safe_zone, true,
                              leds_num, true);
        }
        k_sleep(K_USEC(100));
    }
}


_Noreturn void signalman_modem_task()
{
    volatile uint32_t ticks = 0;
    k_sleep(K_FOREVER);
    while(1) {
        if (signalman_current_state.state == TRANSMIT) {
            send_msg();
            signalman_current_state = *(signalman_current_state.next);
        }
        else {
            recv_msg();
        }
        k_sleep(K_USEC(100));
    }
}


void signalman_button_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_submit(&work_buzzer);
    LOG_DBG("Button alarm pressed");
    k_msgq_put(&msgq_tx_msg, &alarm_msg, K_NO_WAIT);
}


void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    k_work_submit(&work_buzzer);
    LOG_DBG("Button anti-dream pressed");
}


void signalman_periodic_timer_handler(struct k_timer *tim)
{
//    k_msgq_put(&msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT);
    signalman_current_state = *signalman_current_state.next;
    k_wakeup(modem_task_id);
}
/// Function definition area end
