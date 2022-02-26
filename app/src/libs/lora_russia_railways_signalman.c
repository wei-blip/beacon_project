//
// Created by rts on 21.01.2022.
//
#include "lora_russia_railways_signalman.h"
#include <drivers/gpio.h>

#include <logging/log.h>
    LOG_MODULE_REGISTER(signalman);


/// My threads ids begin
extern const k_tid_t proc_task_id;
extern const k_tid_t modem_task_id;
/// My threads ids end

static modem_state_t current_state;

/// Structure area begin
static struct k_timer anti_dream_timer;
static struct k_work work_anti_dream;

static struct message_s alarm_msg;
static struct message_s train_passed_msg;
static struct message_s anti_dream_msg;

static struct msg_info_s alarm_msg_info;
static struct msg_info_s train_passed_msg_info;
static struct msg_info_s anti_dream_msg_info;

struct device *button_alarm_gpio_dev_ptr;
struct device *button_anti_dream_gpio_dev_ptr;
struct device *button_train_passed_gpio_dev_ptr;

struct gpio_callback button_anti_dream_cb;
struct gpio_callback button_alarm_cb;
struct gpio_callback button_train_passed_cb;
/// Structure area end


/// Enum area begin
static enum DEVICE_ADDR_e cur_dev_addr = SIGNALMAN_1_ADDR;
static enum WORKERS_IDS_e cur_workers_in_safe_zone = FIRST_PEOPLE_ID;
static enum BATTERY_LEVEL_e cur_battery_level = BATTERY_LEVEL_GOOD;
/// Enum area end


/// Function declaration area begin
void button_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button_anti_dream_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

static void send_msg(void);
static void recv_msg(void);
static void system_init(void);
static void work_buzzer_handler(struct k_work *item);
static void work_msg_mngr_handler(struct k_work *item);
static void work_anti_dream_handler(struct k_work *item);
static void periodic_timer_handler(struct k_timer *tim); // callback for periodic_timer
static void anti_dream_timer_handler(struct k_timer *tim); // callback for anti-dream timer
/// Function declaration area end


//// Function definition area begin
static void system_init(void)
{
    volatile int rc = -1;
    uint32_t new_msg = 0;
    int16_t rssi = 0;
    int8_t snr = 0;

    /// Buzzer init begin
    buzzer_dev_ptr = DEVICE_DT_GET(PWM_CTLR);
    if (!device_is_ready(buzzer_dev_ptr)) {
        LOG_DBG("Error: PWM device %s is not ready\n", buzzer_dev_ptr->name);
        k_sleep(K_FOREVER);
    }
    /// Buzzer init end

    //// Init IRQ (change gpio init after tests) begin
    button_alarm_gpio_dev_ptr = device_get_binding(BUTTON_ALARM_GPIO_PORT);
//    button_anti_dream_gpio_dev_ptr = device_get_binding(BUTTON_ANTI_DREAM_GPIO_PORT);
//    button_train_passed_gpio_dev_ptr = device_get_binding(BUTTON_TRAIN_PASSED_GPIO_PORT);

    gpio_pin_configure(button_alarm_gpio_dev_ptr, BUTTON_ALARM_GPIO_PIN,
                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));
//    gpio_pin_configure(button_anti_dream_gpio_dev_ptr, BUTTON_ANTI_DREAM_GPIO_PIN,
//                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));
//    gpio_pin_configure(button_train_passed_gpio_dev_ptr, BUTTON_TRAIN_PASSED_GPIO_PIN,
//                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));

    gpio_pin_interrupt_configure(button_alarm_gpio_dev_ptr, BUTTON_ALARM_GPIO_PIN,
                                 GPIO_INT_EDGE_TO_ACTIVE);
//    gpio_pin_interrupt_configure(&button_anti_dream_gpio_dev_ptr, BUTTON_ANTI_DREAM_GPIO_PORT,
//                                 GPIO_INT_EDGE_RISING);
//    gpio_pin_interrupt_configure(button_train_passed_gpio_dev_ptr, BUTTON_TRAIN_PASSED_GPIO_PIN,
//                                 GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button_alarm_cb, button_alarm_pressed_cb, BIT(BUTTON_ALARM_GPIO_PIN));
//    gpio_init_callback(button_anti_dream_cb_ptr, button_anti_dream_pressed_cb, BIT(BUTTON_ANTI_DREAM_GPIO_PIN));
//    gpio_init_callback(&button_train_passed_cb, button_train_pass_pressed_cb, BIT(BUTTON_TRAIN_PASSED_GPIO_PIN));

    gpio_add_callback(button_alarm_gpio_dev_ptr, &button_alarm_cb);
//    gpio_add_callback(button_anti_dream_gpio_dev_ptr, button_anti_dream_cb_ptr);
//    gpio_add_callback(button_train_passed_gpio_dev_ptr, &button_train_passed_cb);
    /// Init IRQ end

    /// Kernel services init begin
    k_work_init(&work_buzzer, work_buzzer_handler);
    k_work_init(&work_msg_mngr, work_msg_mngr_handler);
    k_work_init(&work_led_strip_blink, blink);
    k_work_init(&work_anti_dream, work_anti_dream_handler);

    k_timer_init(&periodic_timer, periodic_timer_handler, NULL);
//    k_timer_init(&anti_dream_timer, anti_dream_timer_handler, anti_dream_timer_stop_fn);
    k_timer_init(&anti_dream_timer, anti_dream_timer_handler, NULL);

    k_mutex_init(&mut_msg_info);
    k_mutex_init(&mut_buzzer_mode);
    /// Kernel services init end

    /// Light down LEDs begin
    update_indication(&led_strip_state, true, true);
    /// Light down LEDs end

    current_state = recv_state;

    /// Filling structure
    alarm_msg.receiver_addr = BASE_STATION_ADDR;
    alarm_msg.sender_addr = cur_dev_addr;
    alarm_msg.message_type = MESSAGE_TYPE_ALARM;
    alarm_msg.direction = REQUEST;
    alarm_msg.battery_level = BATTERY_LEVEL_GOOD;
    alarm_msg.workers_in_safe_zone = 0;

    alarm_msg_info.msg_buf = &msgq_tx_msg_prio;
    alarm_msg_info.req_is_send = ATOMIC_INIT(0);
    alarm_msg_info.resp_is_recv = ATOMIC_INIT(0);
    alarm_msg_info.msg = &alarm_msg;

    train_passed_msg.receiver_addr = BASE_STATION_ADDR;
    train_passed_msg.sender_addr = cur_dev_addr;
    train_passed_msg.direction = REQUEST;
    train_passed_msg.battery_level = BATTERY_LEVEL_GOOD;
    train_passed_msg.workers_in_safe_zone = 0;
    if (cur_dev_addr == SIGNALMAN_1_ADDR)
        train_passed_msg.message_type = MESSAGE_TYPE_LEFT_TRAIN_PASSED;
    else
        train_passed_msg.message_type = MESSAGE_TYPE_RIGHT_TRAIN_PASSED;

    train_passed_msg_info.msg_buf = &msgq_tx_msg;
    train_passed_msg_info.req_is_send = ATOMIC_INIT(0);
    train_passed_msg_info.resp_is_recv = ATOMIC_INIT(0);
    train_passed_msg_info.msg = &train_passed_msg;

    anti_dream_msg.receiver_addr = BASE_STATION_ADDR;
    anti_dream_msg.sender_addr = cur_dev_addr;
    anti_dream_msg.message_type = MESSAGE_TYPE_ANTI_DREAM;
    anti_dream_msg.direction = REQUEST;
    anti_dream_msg.battery_level = BATTERY_LEVEL_GOOD;
    anti_dream_msg.workers_in_safe_zone = 0;

    anti_dream_msg_info.msg_buf = &msgq_tx_msg_prio;
    anti_dream_msg_info.req_is_send = ATOMIC_INIT(0);
    anti_dream_msg_info.resp_is_recv = ATOMIC_INIT(0);
    anti_dream_msg_info.msg = &anti_dream_msg;

    buzzer_mode.single = true;
    k_work_submit(&work_buzzer);
}

static void send_msg(void)
{
    volatile int rc = 0;
    uint32_t new_msg = 0;
    enum COMMON_STRIP_COLOR_e color;
    struct k_msgq* cur_queue = NULL;
//    LOG_DBG("Check queues");
    // If mutex taken then check message into queue
    // If message present into queue then getting him from current queue
    if (!k_mutex_lock(&mut_msg_info, K_MSEC(1))) {
        if (msgq_tx_msg_prio.used_msgs) {
//        LOG_DBG("Get message from priority queue");
            k_msgq_get(&msgq_tx_msg_prio, &tx_msg, K_NO_WAIT);
            cur_queue = &msgq_tx_msg_prio;
        } else if (msgq_tx_msg.used_msgs) {
//        LOG_DBG("Get message from standart queue");
            k_msgq_get(&msgq_tx_msg, &tx_msg, K_NO_WAIT);
            cur_queue = &msgq_tx_msg;
        } else {
            k_mutex_unlock(&mut_msg_info);
            return;
        }
    } else {
        return;
    }
    k_mutex_unlock(&mut_msg_info);

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
    rc = lora_send(lora_dev_ptr, tx_buf, MESSAGE_LEN_IN_BYTES);

    if (!rc)
        color = COMMON_STRIP_COLOR_GREEN;
    else
        color = COMMON_STRIP_COLOR_RED;

    while(k_work_busy_get(&work_led_strip_blink));
    set_blink_param(color, K_MSEC(100), 5);
    k_work_submit(&work_led_strip_blink);
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
        rc = lora_recv(lora_dev_ptr, rx_buf, MESSAGE_LEN_IN_BYTES, K_MSEC(ticks), &rssi, &snr);
    }
    else {
        return;
    }
    if (rc > 0) {
//        LOG_DBG("MESSAGE RECEIVE");
        if (IS_SYNC_MSG) {
            LOG_DBG(" REQUEST");
            LOG_DBG(" MESSAGE_TYPE_SYNC");
            k_timer_stop(&periodic_timer);
            current_state = recv_state;
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
    uint8_t rssi_num = 0;
    int16_t rssi = 0;
    uint8_t rx_buf_proc[MESSAGE_LEN_IN_BYTES];
    uint32_t cur_msg = 0;
    struct message_s tx_msg_proc = {0};
    struct message_s rx_msg_proc = {0};
    struct k_msgq* msgq_cur_msg_tx_ptr = &msgq_tx_msg; // Default queue
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
                    tx_msg_proc.receiver_addr = BASE_STATION_ADDR;
                    tx_msg_proc.message_type = rx_msg_proc.message_type;
                    tx_msg_proc.workers_in_safe_zone = 0;
                    tx_msg_proc.direction = RESPONSE;
                    tx_msg_proc.battery_level = BATTERY_LEVEL_GOOD; // change it after

                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_SYNC:
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            msgq_cur_msg_tx_ptr = NULL;
//                            msgq_cur_msg_tx_ptr = &msgq_rx_msg;
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
                                msgq_cur_msg_tx_ptr = &msgq_tx_msg_prio; // For response message (priority queue)
                            else
                                msgq_cur_msg_tx_ptr = NULL; // Do nothing, because this message for base station
                            break;

                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = NULL;
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
                            switch (rx_msg_proc.sender_addr) {
                                // Because it messages retransmit from base station
                                case BRIGADE_CHIEF_ADDR:
                                    LOG_DBG("Brigade chief disabled alarm");
                                    break;
                                default:
                                    LOG_DBG("Undefined sender address for this message type");
                                    break;
                            }
                            break;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            if (rx_msg_proc.sender_addr == cur_dev_addr) {
                                /// TODO: atomic operation
                                atomic_set_bit(&alarm_msg_info.resp_is_recv, 0); // message received
                                while(k_work_busy_get(&work_led_strip_blink));
                                set_blink_param(COMMON_STRIP_COLOR_GREEN, K_MSEC(100), 5);
                                k_work_submit(&work_led_strip_blink);
                                buzzer_mode.ding_dong = true;
                                while(k_work_busy_get(&work_buzzer));
                                k_work_submit(&work_buzzer);
                            }
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            if (rx_msg_proc.sender_addr == cur_dev_addr) {
                                /// TODO: atomic operation
                                atomic_set_bit(&train_passed_msg_info.resp_is_recv, 0); // message received
                                while(k_work_busy_get(&work_led_strip_blink));
                                set_blink_param(COMMON_STRIP_COLOR_GREEN, K_MSEC(100), 5);
                                k_work_submit(&work_led_strip_blink);
                                buzzer_mode.ding_dong = true;
                                while(k_work_busy_get(&work_buzzer));
                                k_work_submit(&work_buzzer);
                            }
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
                k_mutex_lock(&mut_msg_info, K_FOREVER);
                k_msgq_put(msgq_cur_msg_tx_ptr, &tx_msg_proc, K_NO_WAIT);
                k_mutex_unlock(&mut_msg_info);
            }
            rssi_num = check_rssi(rssi);
            led_strip_state.con_status = rssi_num;
            led_strip_state.people_num = rx_msg_proc.workers_in_safe_zone;
            update_indication(&led_strip_state, true, true);
        }
        k_sleep(K_USEC(100));
    }
}


_Noreturn void signalman_modem_task()
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
        } else {
            recv_msg();
        }
        k_sleep(K_USEC(100));
    }
}


void button_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button alarm pressed");
    atomic_cas(&alarm_msg_info.req_is_send, 0, 1);
    k_work_submit(&work_msg_mngr);
}


void button_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button train pass pressed");
    atomic_cas(&train_passed_msg_info.req_is_send, 0, 1);
    k_work_submit(&work_msg_mngr);
}


void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    LOG_DBG("Button anti-dream pressed");
    k_work_submit(&work_buzzer); // disable alarm
    k_timer_stop(&anti_dream_timer);
}


static void periodic_timer_handler(struct k_timer *tim)
{
    LOG_DBG("Periodic timer handler");
//    k_msgq_put(&msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT); // for debug
    static uint8_t cnt = 0;
    if (cnt == 20) { // anti-dream
        k_work_submit(&work_anti_dream);
        cnt = 0;
    }
    cnt++;
    current_state = transmit_state;
    k_wakeup(modem_task_id);
}


static void anti_dream_timer_handler(struct k_timer *tim)
{
    // If anti_dream_timer expiry and (req_is_send == 1) putting anti_dream_msg into queue
    atomic_cas(&anti_dream_msg_info.req_is_send, 0, 1);
    k_work_submit(&work_msg_mngr);
}


//static void anti_dream_timer_stop_fn(struct k_timer *tim)
//{
//    return;
//}


static void work_buzzer_handler(struct k_work *item)
{
    k_mutex_lock(&mut_buzzer_mode, K_FOREVER);
    if (buzzer_mode.single) {
        pwm_pin_set_usec(buzzer_dev_ptr, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                         BUTTON_PRESSED_PERIOD_TIME_USEC/2U, PWM_FLAGS);
        k_sleep(K_USEC(BUTTON_PRESSED_PERIOD_TIME_USEC));
        buzzer_mode.single = false;
    } else if (buzzer_mode.continuous) {
        pwm_pin_set_usec(buzzer_dev_ptr, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                         BUTTON_PRESSED_PERIOD_TIME_USEC/2U, PWM_FLAGS);
        buzzer_mode.continuous = false;
        k_mutex_unlock(&mut_buzzer_mode);
        return;
    } else if (buzzer_mode.ding_dong) {
        uint8_t i = 0;
        while (i < 2) {
            pwm_pin_set_usec(buzzer_dev_ptr, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                             BUTTON_PRESSED_PERIOD_TIME_USEC/2U, PWM_FLAGS);
            k_sleep(K_USEC(BUTTON_PRESSED_PERIOD_TIME_USEC));
            pwm_pin_set_usec(buzzer_dev_ptr, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                             0, PWM_FLAGS);
            k_sleep(K_MSEC(90));
            i++;
        }
        buzzer_mode.ding_dong = false;
        k_mutex_unlock(&mut_buzzer_mode);
        return;
    }

    pwm_pin_set_usec(buzzer_dev_ptr, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                     0, PWM_FLAGS);
    k_mutex_unlock(&mut_buzzer_mode);
}


static void work_msg_mngr_handler(struct k_work *item)
{
    k_mutex_lock(&mut_msg_info, K_FOREVER);
    check_msg_status(&alarm_msg_info);
    check_msg_status(&train_passed_msg_info);
    check_msg_status(&anti_dream_msg_info);
    k_mutex_unlock(&mut_msg_info);

    set_color(COMMON_STRIP_COLOR_YELLOW);

    // if mut_buzzer_mode taken then work_buzzer_handler is free
    if (!k_mutex_lock(&mut_buzzer_mode, K_USEC(500))) {
        buzzer_mode.single = true;
//        while(k_work_busy_get(&work_buzzer));
        k_work_submit(&work_buzzer);
        k_mutex_unlock(&mut_buzzer_mode);
    }
}


static void work_anti_dream_handler(struct k_work *item)
{
    k_mutex_lock(&mut_buzzer_mode, K_FOREVER);
    buzzer_mode.continuous = true;
    k_work_submit(&work_buzzer);
    k_mutex_unlock(&mut_buzzer_mode);

    while(k_work_busy_get(&work_led_strip_blink));
    set_blink_param(COMMON_STRIP_COLOR_RED, K_MSEC(100), 7);
    k_work_submit(&work_led_strip_blink);

    k_timer_start(&anti_dream_timer, K_MINUTES(ANTI_DREAM_TIME_MIN), K_NO_WAIT);
}
/// Function definition area end