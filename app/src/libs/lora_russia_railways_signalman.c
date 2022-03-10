//
// Created by rts on 21.01.2022.
//
#include "lora_russia_railways_signalman.h"
#include <drivers/gpio.h>

#include <logging/log.h>
    LOG_MODULE_REGISTER(signalman);

atomic_t anti_dream_active = ATOMIC_INIT(0);

/**
 * My threads ids area begin
 * */
extern const k_tid_t proc_task_id;
extern const k_tid_t modem_task_id;
/**
 * My threads ids area end
 * */

static modem_state_t current_state;

/**
 * Structure area begin
 * */
static struct k_timer anti_dream_timer;
static struct k_work work_anti_dream;

static struct msg_info_s alarm_msg_info;
static struct msg_info_s train_passed_msg_info;
static struct msg_info_s anti_dream_msg_info;

struct gpio_callback button_anti_dream_cb;
struct gpio_callback button_alarm_cb;
struct gpio_callback button_train_passed_cb;

const static struct led_strip_indicate_s anti_dream_ind = {
    .start_led_pos = 0,
    .end_led_pos = STRIP_NUM_PIXELS,
    .led_strip_state.strip_param.color = COMMON_STRIP_COLOR_RED,
    .led_strip_state.strip_param.blink_cnt = 25,
};

/**
 * Structure area end
 * */


/**
 * Enum area begin
 * */
static uint8_t cur_workers_in_safe_zone = 5;
static enum DEVICE_ADDR_e cur_dev_addr = SIGNALMAN_1_ADDR;
//static enum WORKERS_IDS_e cur_workers_in_safe_zone = FIRST_PEOPLE_ID;
static enum BATTERY_LEVEL_e cur_battery_level = BATTERY_LEVEL_GOOD;
/**
 * Enum area end
 * */


/**
 * Function declaration area begin
 * */
void button_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button_anti_dream_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

inline static void send_msg(void);
inline static void recv_msg(void);

static void system_init(void);
static void work_buzzer_handler(struct k_work *item);
static void work_anti_dream_handler(struct k_work *item);
static void periodic_timer_handler(struct k_timer *tim); // callback for periodic_timer
static void anti_dream_timer_handler(struct k_timer *tim); // callback for anti-dream timer
/**
 * Function declaration area end
 * */


/**
 * Function definition area begin
 * */
static void system_init(void)
{
    volatile int rc = -1;
    uint32_t new_msg = 0;
    int16_t rssi = 0;
    int8_t snr = 0;
    struct led_strip_indicate_s *strip_ind = &status_ind;

    /**
     * Buzzer init begin
     * */
    buzzer_dev_ptr = DEVICE_DT_GET(PWM_CTLR);
    if (!device_is_ready(buzzer_dev_ptr)) {
        LOG_DBG("Error: PWM device %s is not ready\n", buzzer_dev_ptr->name);
        k_sleep(K_FOREVER);
    }
    /**
     * Buzzer init end
     * */

    /**
     * Init IRQ (change gpio init after tests) begin
     * */
     if (!device_is_ready(button_alarm.port)) {
         LOG_DBG("Error: button device %s is not ready\n", button_alarm.port->name);
         k_sleep(K_FOREVER);
     }

//     if (!device_is_ready(button_anti_dream.port)) {
//         LOG_DBG("Error: button device %s is not ready\n", button_anti_dream.port->name);
//         k_sleep(K_FOREVER);
//     }
//
//    if (!device_is_ready(button_train_passed.port)) {
//        LOG_DBG("Error: button device %s is not ready\n", button_train_passed.port->name);
//        k_sleep(K_FOREVER);
//    }

    gpio_pin_configure_dt(&button_alarm, GPIO_INPUT);
//    gpio_pin_configure_dt(&button_anti_dream, GPIO_INPUT);
//    gpio_pin_configure_dt(&button_train_passed, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&button_alarm, GPIO_INT_EDGE_TO_ACTIVE);
//    gpio_pin_interrupt_configure_dt(&button_anti_dream, GPIO_INT_EDGE_TO_ACTIVE);
//    gpio_pin_interrupt_configure_dt(&button_train_passed, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button_alarm_cb, button_alarm_pressed_cb, BIT(button_alarm.pin));
//    gpio_init_callback(&button_anti_dream_cb, button_anti_dream_pressed_cb, BIT(button_anti_dream.pin));
//    gpio_init_callback(&button_train_passed_cb, button_train_pass_pressed_cb, BIT(button_train_passed.pin));

    gpio_add_callback(button_alarm.port, &button_alarm_cb);
//    gpio_add_callback(button_anti_dream_gpio_dev_ptr, &button_anti_dream_cb);
//    gpio_add_callback(button_train_passed_gpio_dev_ptr, &button_train_passed_cb);
    /**
     * Init IRQ end
     * */

    /**
     * Kernel services init begin
     * */
    k_work_init(&work_buzzer, work_buzzer_handler);
    k_work_init(&work_button_pressed, work_button_pressed_handler);
    k_work_init(&work_anti_dream, work_anti_dream_handler);

    k_timer_init(&periodic_timer, periodic_timer_handler, NULL);
    k_timer_init(&anti_dream_timer, anti_dream_timer_handler, NULL);

    k_mutex_init(&mut_buzzer_mode);
    /**
     * Kernel services init end
     * */

     /* Light down LED strip */
    k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

    current_state = recv_state;

    buzzer_mode.single = true;
    k_work_submit(&work_buzzer);
}


inline static void send_msg(void)
{
    volatile int rc = 0;
    uint32_t new_msg = 0;
    enum COMMON_STRIP_COLOR_e color;
    struct k_msgq* cur_queue = NULL;
    struct led_strip_indicate_s *strip_ind = NULL;

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
    rc = lora_send(lora_dev_ptr, tx_buf, MESSAGE_LEN_IN_BYTES);

    if (!rc)
        strip_ind = &msg_send_good_ind;
    else
        strip_ind = &msg_send_bad_ind;

    k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
}


inline static void recv_msg(void)
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
    int32_t ret = 0;
    struct message_s tx_msg_proc = {0};
    struct message_s rx_msg_proc = {0};
    struct led_strip_indicate_s *strip_ind = &status_ind;
    struct k_msgq* msgq_cur_msg_tx_ptr = &msgq_tx_msg; /* Default queue */
    while(1) {
        if (k_msgq_num_used_get(&msgq_rx_msg)) {
            k_msgq_get(&msgq_rx_msg, &rx_buf_proc, K_NO_WAIT);
            k_msgq_get(&msgq_rssi, &rssi, K_NO_WAIT);
            if (IS_EMPTY_MSG) {
                LOG_DBG("Empty message");
                continue;
            }

            /**
             * Processing receive data
             * */
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
                            continue;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            /// TODO Indication for signalman and brigade chief
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            if (rx_msg_proc.sender_addr == cur_dev_addr) {
                                msgq_cur_msg_tx_ptr = &msgq_tx_msg_prio; // For response message (priority queue)
                            }
                            else {
                                msgq_cur_msg_tx_ptr = NULL; // Do nothing, because this message for base station
                                continue;
                            }
                            break;

                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = NULL;
                        default:
                            LOG_DBG("Not correct message type");
                            msgq_cur_msg_tx_ptr = NULL;
                            continue;
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
                            continue;

                        case MESSAGE_TYPE_HOMEWARD:
                            /* TODO: Make response */
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = NULL;
                            continue;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            if (rx_msg_proc.sender_addr == cur_dev_addr) {
                                strip_ind = &msg_recv_ind;
                                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

                                buzzer_mode.ding_dong = true;
                                while(k_work_busy_get(&work_buzzer)) {
                                    k_sleep(K_MSEC(10));
                                }
                                k_work_submit(&work_buzzer);
                            }
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            if (rx_msg_proc.sender_addr == cur_dev_addr) {
                                strip_ind = &msg_recv_ind;
                                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

                                buzzer_mode.ding_dong = true;
                                while(k_work_busy_get(&work_buzzer)) {
                                    k_sleep(K_MSEC(10));
                                }
                                k_work_submit(&work_buzzer);
                            }
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        default:
                            LOG_DBG("Not correct message type");
                            msgq_cur_msg_tx_ptr = NULL;
                            continue;
                    }
                    break;

                default:
                    LOG_DBG("Not correct message direction");
                    msgq_cur_msg_tx_ptr = NULL;
                    continue;
            }

            if (msgq_cur_msg_tx_ptr)
                k_msgq_put(msgq_cur_msg_tx_ptr, &tx_msg_proc, K_NO_WAIT);

            rssi_num = check_rssi(rssi);
            atomic_set(&status_ind.led_strip_state.status.con_status, rssi_num);
            atomic_set(&status_ind.led_strip_state.status.people_num, rx_msg_proc.workers_in_safe_zone);
            ret = k_poll(&event_indicate, 1, K_NO_WAIT);
            if (!ret)
                strip_ind = &status_ind;
            else if (ret == (-EAGAIN))
                strip_ind = &disable_indication;
            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

        }
        k_sleep(K_USEC(100));
    }
}


_Noreturn void signalman_modem_task()
{
    int8_t snr;
    int16_t rssi;
    volatile uint32_t ticks = 0;

    /**
     * Lora config begin
     * */
    lora_cfg.tx = false;

    lora_dev_ptr = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    if (!device_is_ready(lora_dev_ptr)) {
        k_sleep(K_FOREVER);
    }
    if ( lora_config(lora_dev_ptr, &lora_cfg) < 0 ) {
        k_sleep(K_FOREVER);
    }
    /**
     * Lora config end
     * */

    system_init();

    /**
     * Receive sync message begin
     * */
    lora_recv(lora_dev_ptr, rx_buf, MESSAGE_LEN_IN_BYTES, K_FOREVER, &rssi, &snr);
    k_sleep(K_MSEC(DELAY_TIME_MSEC));
    k_timer_start(&periodic_timer, K_MSEC(DURATION_TIME_MSEC),K_MSEC(PERIOD_TIME_MSEC));

    k_msgq_put(&msgq_rx_msg, &rx_buf, K_NO_WAIT);
    k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);
    k_wakeup(proc_task_id);
    /**
     * Receive sync message end
     * */

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
    cur_irq_gpio_ptr = &button_alarm;
    k_work_submit(&work_button_pressed);
}


void button_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button train pass pressed");
    cur_irq_gpio_ptr = &button_train_passed;

}


void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    LOG_DBG("Button anti-dream pressed");
    cur_irq_gpio_ptr = &button_anti_dream;
    k_work_submit(&work_buzzer); /* Disable alarm */
    k_timer_stop(&anti_dream_timer);
    atomic_set(&anti_dream_active, 0);
}


static void periodic_timer_handler(struct k_timer *tim)
{
    LOG_DBG("Periodic timer handler");
    struct led_strip_indicate_s *strip_ind = NULL;
//    k_msgq_put(&msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT); // for debug
    static uint8_t anti_dream_cnt = 0;
    static uint8_t indicate_cnt = 0;
    current_state = transmit_state;

    if (!k_poll(&event_indicate, 1, K_NO_WAIT)) {
        if (DISABLE_INDICATE) {
            strip_ind = &disable_indication;
            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
            event_indicate.signal->signaled = 0;
            event_indicate.state = K_POLL_STATE_NOT_READY;
            indicate_cnt = 0;
        }
        indicate_cnt++;
    }
//    if (ANTI_DREAM_START) { /* For anti-dream */
//        k_work_submit(&work_anti_dream);
//        anti_dream_cnt = 0;
//    }
    anti_dream_cnt++;
    k_wakeup(modem_task_id);
}


static void anti_dream_timer_handler(struct k_timer *tim)
{
    /* If anti_dream_timer expiry and (req_is_send == 1) putting anti_dream_msg into queue */
    atomic_cas(&anti_dream_msg_info.req_is_send, 0, 1);
//    check_msg_status(&anti_dream_msg_info);
}


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


static void work_anti_dream_handler(struct k_work *item)
{
    /* TODO: Check anti-dream status */
    if (!atomic_get(&anti_dream_active)) {
        struct led_strip_indicate_s *strip_ind = NULL;
        k_mutex_lock(&mut_buzzer_mode, K_FOREVER);
        buzzer_mode.continuous = true;
        k_work_submit(&work_buzzer);
        k_mutex_unlock(&mut_buzzer_mode);

        atomic_set(&anti_dream_active, 1);
        k_timer_start(&anti_dream_timer, K_MINUTES(ANTI_DREAM_TIME_MIN), K_MINUTES(ANTI_DREAM_TIME_MIN));

        strip_ind = &anti_dream_ind;
        k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
        k_wakeup(update_indication_task_id);;
    }
}
/**
 * Function definition area end
 * */