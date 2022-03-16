//
// Created by rts on 07.02.2022.
//

#include "lora_russia_railways_common.h"

#if CUR_DEVICE == BRIGADE_CHIEF

#include <logging/log.h>
LOG_MODULE_REGISTER(brigade_chief);


/**
 * Structure area begin
 * */
struct gpio_callback button_disable_alarm_cb;
struct gpio_callback button_right_train_passed_cb;
struct gpio_callback button_left_train_passed_cb;

static struct message_s sync_msg = {0};
/**
 * Structure area end
 * */



/**
 * Enum area begin
 * */
static uint8_t cur_workers_in_safe_zone = 3;
static enum DEVICE_ADDR_e cur_dev_addr = BRIGADE_CHIEF_ADDR;
static enum BATTERY_LEVEL_e cur_battery_level = BATTERY_LEVEL_GOOD;
/**
 * Enum area end
 * */


/**
 * Function declaration area begin
 * */
void button_disable_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button_left_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void button_right_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

static void periodic_timer_handler(struct k_timer *tim);
/**
 * Function declaration area end
 * */


/**
 * Function definition area begin
 * */
void system_init(void)
{
    volatile int rc = -1;
    uint32_t new_msg = 0;
    int16_t rssi = 0;
    int8_t snr = 0;
    struct led_strip_indicate_s *strip_ind = NULL;

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
     * Init IRQ begin
     * */
     if (!device_is_ready(button_disable_alarm.port)) {
         printk("Error: button device %s is not ready\n", button_disable_alarm.port->name);
         k_sleep(K_FOREVER);
     }

//    if (!device_is_ready(button_left_train_passed.port)) {
//        printk("Error: button device %s is not ready\n", button_left_train_passed.port->name);
//        k_sleep(K_FOREVER);
//    }
//
//    if (!device_is_ready(button_right_train_passed.port)) {
//        printk("Error: button device %s is not ready\n", button_right_train_passed.port->name);
//        k_sleep(K_FOREVER);
//    }

    gpio_pin_configure_dt(&button_disable_alarm,GPIO_INPUT);
//    gpio_pin_configure_dt(&button_left_train_passed,GPIO_INPUT);
//    gpio_pin_configure_dt(&button_right_train_passed,GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&button_disable_alarm,GPIO_INT_EDGE_TO_ACTIVE);
//    gpio_pin_interrupt_configure_dt(&button_left_train_passed,GPIO_INT_EDGE_TO_ACTIVE);
//    gpio_pin_interrupt_configure_dt(&button_right_train_passed,GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&button_disable_alarm_cb, button_disable_alarm_pressed_cb,
                       BIT(button_disable_alarm.pin));
//    gpio_init_callback(&button_right_train_passed_cb, button_right_train_pass_pressed_cb,
//                       BIT(button_left_train_passed.pin));
//    gpio_init_callback(&button_left_train_passed_cb, button_left_train_pass_pressed_cb,
//                       BIT(button_right_train_passed.pin));

    gpio_add_callback(button_disable_alarm.port, &button_disable_alarm_cb);
//    gpio_add_callback(button_left_train_passed.port, &button_left_train_passed_cb);
//    gpio_add_callback(button_right_train_passed.port, &button_right_train_passed_cb);
    /**
     * Init IRQ end
     * */

    /**
     * Kernel services init begin
     * */
    k_work_init(&work_buzzer, work_buzzer_handler);
    k_work_init(&work_button_pressed, work_button_pressed_handler);

    k_timer_init(&periodic_timer, periodic_timer_handler, NULL);
    /**
     * Kernel services init end
     * */

    sync_msg.receiver_addr = cur_dev_addr;
    sync_msg.sender_addr = cur_dev_addr;
    sync_msg.message_type = MESSAGE_TYPE_SYNC;
    sync_msg.direction = REQUEST;
    sync_msg.workers_in_safe_zone = 0;
    sync_msg.battery_level = BATTERY_LEVEL_GOOD;

    /* Light down LED strip */
    strip_ind = &status_ind;
    k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

    current_state = recv_state;

//    buzzer_mode.single = true;
    k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_SINGLE);
    k_work_submit(&work_buzzer);
}


_Noreturn void brigade_chief_proc_task(void)
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
                    tx_msg_proc.message_type = rx_msg_proc.message_type;
                    tx_msg_proc.workers_in_safe_zone = 0;
                    tx_msg_proc.direction = RESPONSE;
                    tx_msg_proc.battery_level = BATTERY_LEVEL_GOOD; // change it after
                    tx_msg_proc.receiver_addr = BASE_STATION_ADDR;

                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_SYNC:
                            msgq_cur_msg_tx_ptr = NULL;
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
                            continue;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            /// TODO Indication for signalman and brigade chief
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            msgq_cur_msg_tx_ptr = NULL;
                            continue;

                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = NULL; // Do nothing, because this message for base station
                            continue;

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
                            if (rx_msg_proc.sender_addr == cur_dev_addr) {
                                LOG_DBG("Brigade chief disabled alarm");
                                strip_ind = &msg_recv_ind;
                                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

                                k_work_submit(&work_buzzer);
                                k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_DING_DONG);
                            }
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = NULL;
                            continue;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            msgq_cur_msg_tx_ptr = NULL;
                            continue;

                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_RIGHT_TRAIN_PASSED");
                            if (rx_msg_proc.sender_addr == cur_dev_addr) {
                                strip_ind = &msg_recv_ind;
                                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

                                k_work_submit(&work_buzzer);
                                k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_DING_DONG);
                            }
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_LEFT_TRAIN_PASSED");
                            if (rx_msg_proc.sender_addr == cur_dev_addr) {
                                strip_ind = &msg_recv_ind;
                                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

                                k_work_submit(&work_buzzer);
                                k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_DING_DONG);
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
            if (msgq_cur_msg_tx_ptr) {
                k_msgq_put(msgq_cur_msg_tx_ptr, &tx_msg_proc, K_NO_WAIT);
            }

            k_msgq_get(&msgq_rssi, &rssi, K_MSEC(1));
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


_Noreturn void brigade_chief_modem_task(void)
{
    int8_t snr;
    int16_t rssi;
    int32_t rc = 0;
    struct led_strip_indicate_s *strip_ind = NULL;


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
     * Receive starting sync message begin
     * */
    lora_recv_async(lora_dev_ptr, lora_receive_cb, lora_receive_error_timeout);
    /**
     * Receive starting sync message end
     * */

    while(1) {
        rc = modem_fun();
        if (!rc) {
            strip_ind = &msg_send_good_ind;
            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
        } else if (rc < 0) {
            strip_ind = &msg_send_bad_ind;
            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
        } else {
            k_sleep(K_USEC(100));
        }
    }
}


void button_disable_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button disable alarm pressed");
    cur_irq_gpio_ptr = &button_disable_alarm;
    k_work_submit(&work_button_pressed);
}


void button_left_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button left train pass pressed");
    cur_irq_gpio_ptr = &button_left_train_passed;
    k_work_submit(&work_button_pressed);
}


void button_right_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button right train pass pressed");
    cur_irq_gpio_ptr = &button_right_train_passed;
    k_work_submit(&work_button_pressed);
}

static void periodic_timer_handler(struct k_timer *tim)
{
//    k_msgq_put(&peripheral_msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT);
    LOG_DBG("Periodic timer handler");
    struct led_strip_indicate_s *strip_ind = NULL;
    static uint8_t indicate_cnt = 0;
    static uint8_t cnt = 0;

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

//    if (cnt == (SYNC_COUNT+CURRENT_DEVICE_NUM)) {
//        k_msgq_put(&msgq_tx_msg, &sync_msg, K_NO_WAIT);
//        cnt = 0;
//    }
    cnt++;
    k_wakeup(modem_task_id);
}
/**
 * Function definition area end
 * */
#endif