//
// Created by rts on 05.02.2022.
//

#include "lora_russia_railways_common.h"

#if CUR_DEVICE == BASE_STATION
#include <logging/log.h>
    LOG_MODULE_REGISTER(base_station);


/**
 * Structure area begin
 * */
static struct message_s sync_msg;
static struct message_s home_msg;

const struct device *button_homeward_gpio_dev_ptr;

struct gpio_callback button_homeward_cb;
/**
 * Structure area end
 * */


/**
 * Enum area begin
 * */
static uint8_t cur_workers_in_safe_zone = 3;
static enum DEVICE_ADDR_e cur_dev_addr = BASE_STATION_ADDR;
static enum BATTERY_LEVEL_e cur_battery_level = BATTERY_LEVEL_GOOD;
/**
 * Enum area end
 * */


/**
 * Function declaration area begin
 * */
void button_homeward_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

inline static void send_msg(void);
inline static void recv_msg(void);

static void system_init(void);
static void periodic_timer_handler(struct k_timer* tim); // callback for periodic_timer
/**
 * Function declaration area end
 * */


/**
 * Function definition area begin
 * */
static void system_init(void)
{
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
     * Init IRQ begin
     * */
//    button_homeward_gpio_dev_ptr = device_get_binding(BUTTON_HOMEWARD_GPIO_PORT);
//
//    gpio_pin_configure(button_homeward_gpio_dev_ptr, BUTTON_HOMEWARD_GPIO_PIN,
//                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));
//
//    gpio_pin_interrupt_configure(button_homeward_gpio_dev_ptr, BUTTON_HOMEWARD_GPIO_PIN,
//                                 GPIO_INT_EDGE_TO_ACTIVE);
//
//    gpio_init_callback(&button_homeward_cb, button_homeward_pressed_cb, BIT(BUTTON_HOMEWARD_GPIO_PIN));
//
//    gpio_add_callback(button_homeward_gpio_dev_ptr, &button_homeward_cb);
    /**
     * Init IRQ end
     * */

    /**
     * Kernel services init begin
     * */
    k_work_init(&work_buzzer, work_buzzer_handler);
//    k_work_init(&work_led_strip_blink, blink);

    k_timer_init(&periodic_timer, periodic_timer_handler, NULL);
    /**
     * Kernel services init end
     * */

    /* Light down LED strip */
    k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

    current_state = recv_state;

    /**
     * Filling structure begin
     * */
    sync_msg.receiver_addr = BROADCAST_ADDR;
    sync_msg.sender_addr = cur_dev_addr;
    sync_msg.message_type = MESSAGE_TYPE_SYNC;
    sync_msg.direction = REQUEST;
    sync_msg.workers_in_safe_zone = cur_workers_in_safe_zone;
    sync_msg.battery_level = BATTERY_LEVEL_GOOD;

    home_msg.receiver_addr = BROADCAST_ADDR;
    home_msg.sender_addr = cur_dev_addr;
    home_msg.message_type = MESSAGE_TYPE_HOMEWARD;
    home_msg.direction = REQUEST;
    home_msg.workers_in_safe_zone = cur_workers_in_safe_zone;
    home_msg.battery_level = BATTERY_LEVEL_GOOD;
    /**
    * Filling structure end
    * */
    k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_SINGLE);

    k_work_submit(&work_buzzer);
    k_timer_start(&periodic_timer, K_NO_WAIT, K_MSEC(PERIOD_TIME_MSEC));
}


_Noreturn void base_station_proc_task(void)
{
    uint8_t rssi_num = 0;
    int16_t rssi = 0;
    uint8_t rx_buf_proc[MESSAGE_LEN_IN_BYTES];
    uint32_t cur_msg = 0;
    struct message_s tx_msg_proc = {0};
    struct message_s rx_msg_proc = {0};
    struct led_strip_indicate_s *strip_ind = NULL;
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
                    LOG_DBG("Message type:");

                    tx_msg_proc.sender_addr = rx_msg_proc.sender_addr;
                    tx_msg_proc.receiver_addr = BROADCAST_ADDR;
                    tx_msg_proc.message_type = rx_msg_proc.message_type;
                    tx_msg_proc.workers_in_safe_zone = cur_workers_in_safe_zone;
                    tx_msg_proc.battery_level = BATTERY_LEVEL_GOOD;
                    tx_msg_proc.direction = RESPONSE;

                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_SYNC:
                            break;

                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            switch (rx_msg_proc.sender_addr) {
                                case BRIGADE_CHIEF_ADDR:
                                    LOG_DBG("Brigade chief disabled alarm");
                                    k_work_submit(&work_buzzer);
                                    k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_IDLE);
//                                    while(k_work_busy_get(&work_buzzer)); // wait while work_buzzer is busy
//                                    k_work_submit(&work_buzzer);
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
                            k_work_submit(&work_buzzer);
                            k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_CONTINUOUS);
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg_prio;
                            break;

                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            break;

                            // TODO: Off signalization
                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
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
                            continue;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            msgq_cur_msg_tx_ptr = NULL;
                            continue;

                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = NULL;
                            continue;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = NULL;
                            continue;

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

            k_msgq_get(&msgq_rssi, &rssi, K_MSEC(1));
            rssi_num = check_rssi(rssi);
            atomic_set(&status_ind.led_strip_state.status.con_status, rssi_num);
            atomic_set(&status_ind.led_strip_state.status.people_num, rx_msg_proc.workers_in_safe_zone);
            strip_ind = &status_ind;
            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
        }
        k_sleep(K_USEC(100));
    }
}


_Noreturn void base_station_modem_task(void)
{
    /**
     * Lora config begin
     * */
    lora_dev_ptr = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    if (!device_is_ready(lora_dev_ptr)) {
        k_sleep(K_FOREVER);
    }
    /**
    * Lora config end
    * */

    /* Init system and send first sync message */
    system_init();
    k_sleep(K_FOREVER);

    while(1) {
        modem_fun();
        k_sleep(K_USEC(100));
    }
}


void button_homeward_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button homeward pressed");
}


static void periodic_timer_handler(struct k_timer* tim)
{
    LOG_DBG("Periodic timer handler");
    static uint8_t count = 10;

    current_state = transmit_state;

    if (count == (SYNC_COUNT + CURRENT_DEVICE_NUM)) {
        k_msgq_put(&msgq_tx_msg, &sync_msg, K_NO_WAIT);
        count = 0;
    }
    count++;
    k_wakeup(modem_task_id);
}
/**
 * Function definition area end
 * */
#endif