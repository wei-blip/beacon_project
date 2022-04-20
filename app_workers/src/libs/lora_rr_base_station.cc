//
// Created by rts on 05.02.2022.
//

#include "rr_common.h"

#if CUR_DEVICE == BASE_STATION

#include <logging/log.h>
    LOG_MODULE_REGISTER(base_station, LOG_LEVEL_DBG);


/**
 * Structure area begin
 * */
static struct message_s sync_msg;
static struct message_s home_msg;
static struct message_s disable_alarm_msg;

const struct device *button_homeward_gpio_dev_ptr;

struct gpio_callback button_homeward_cb;

struct k_work_delayable dwork_sync = {{{nullptr}}};
/**
 * Structure area end
 * */


/**
 * Enum area begin
 * */
static enum DEVICE_ADDR_e cur_dev_addr = BASE_STATION_ADDR;
static enum BATTERY_LEVEL_e cur_battery_level = BATTERY_LEVEL_GOOD;
/**
 * Enum area end
 * */


/**
 * Function declaration area begin
 * */
static void dwork_sync_handler(struct k_work *item);
void button_homeward_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

static void system_init();
static void periodic_timer_handler(struct k_timer* tim); // callback for periodic_timer
/**
 * Function declaration area end
 * */


/**
 * Function definition area begin
 * */
static void system_init()
{
    struct led_strip_indicate_s *strip_ind = &status_ind;
    /**
     * Buzzer init begin
     * */
    buzzer_dev = DEVICE_DT_GET(PWM_CTLR);
    if (!device_is_ready(buzzer_dev)) {
        LOG_DBG("Error: PWM device %s is not ready\n", buzzer_dev->name);
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
    k_work_init_delayable(&dwork_sync, dwork_sync_handler);
    k_timer_init(&periodic_timer, periodic_timer_handler, nullptr);
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
    sync_msg.workers_in_safe_zone = 8;
    sync_msg.battery_level = BATTERY_LEVEL_GOOD;

    home_msg.receiver_addr = BROADCAST_ADDR;
    home_msg.sender_addr = cur_dev_addr;
    home_msg.message_type = MESSAGE_TYPE_HOMEWARD;
    home_msg.direction = REQUEST;
    home_msg.workers_in_safe_zone = 8;
    home_msg.battery_level = BATTERY_LEVEL_GOOD;

    disable_alarm_msg.sender_addr = BASE_STATION_ADDR;
    disable_alarm_msg.message_type = MESSAGE_TYPE_DISABLE_ALARM;
    disable_alarm_msg.direction = REQUEST;
    disable_alarm_msg.battery_level = BATTERY_LEVEL_GOOD;
    disable_alarm_msg.workers_in_safe_zone = 0;

    /**
    * Filling structure end
    * */
    k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_SINGLE);

    k_work_schedule(&dwork_sync, K_NO_WAIT);
    k_work_submit(&work_buzzer);
    k_timer_start(&periodic_timer, K_NO_WAIT, K_MSEC(PERIOD_TIME_MSEC));
}


[[noreturn]] void base_station_proc_task(void)
{
    uint8_t rssi_num = 0;
    uint8_t workers_in_safe_zone = 8;
    uint8_t alarm_addr = 0;
    int16_t rssi = 0;
    uint8_t rx_buf[MESSAGE_LEN_IN_BYTES];
    struct message_s rx_msg = {0};
    struct message_s tx_msg = {
      .receiver_addr = BROADCAST_ADDR,
      .direction = RESPONSE,
      .battery_level = BATTERY_LEVEL_GOOD,
      .workers_in_safe_zone = workers_in_safe_zone

    };
    struct led_strip_indicate_s *strip_ind = nullptr;
    struct k_msgq* msgq_cur_msg_tx_ptr = &msgq_tx_msg; /* Default queue */

    while (true) {
        /* Always update info about workers in safe zone if it is possible */
        if (!k_msgq_get(&msgq_dwm_dist, &workers_in_safe_zone, K_NO_WAIT)) {
            sync_msg.workers_in_safe_zone = workers_in_safe_zone;
            home_msg.workers_in_safe_zone = workers_in_safe_zone;
            disable_alarm_msg.workers_in_safe_zone = workers_in_safe_zone;
        }

        if (k_msgq_num_used_get(&msgq_rx_msg)) {
            k_msgq_get(&msgq_rx_msg, &rx_buf, K_NO_WAIT);

            /**
             * Processing receive data
             * */
            /* Processing receiving data */
            if (!proc_rx_data(&msgq_rx_msg, rx_buf, sizeof(rx_buf), &rx_msg, cur_dev_addr)) {
                continue;
            }
//            LOG_DBG("Message direction");

            switch (rx_msg.direction) {
                case REQUEST:
                    LOG_DBG(" REQUEST");
                    LOG_DBG("Message type:");

                    tx_msg.sender_addr = rx_msg.sender_addr;
                    tx_msg.message_type = rx_msg.message_type;
                    tx_msg.workers_in_safe_zone = workers_in_safe_zone;

                    switch (rx_msg.message_type) {

                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            switch (rx_msg.sender_addr) {
                                case BRIGADE_CHIEF_ADDR:
                                    LOG_DBG("Brigade chief disabled alarm");
                                    k_work_submit(&work_buzzer);
                                    k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_IDLE);
                                    atomic_set(&alarm_is_active, false);
                                    break;
                                default:
                                    LOG_DBG("Undefined sender address for this message type");
                                    break;
                            }
                            /* Retransmit disable alarm message */
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            k_work_submit(&work_buzzer);
                            k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_CONTINUOUS);

                            atomic_set(&alarm_is_active, true);
                            /* Save alarm address in few railways case */
                            alarm_addr = rx_msg.sender_addr;
                            /* Retransmit alarm message */
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg_prio;
                            break;

                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            break;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            continue;

                        default:
                            LOG_DBG("Not correct message type");
                            continue;
                    }
                    break;

                case RESPONSE:
                    LOG_DBG(" RESPONSE");
                    LOG_DBG("Message type:");
                    switch (rx_msg.message_type) {
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            continue;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            continue;

                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            continue;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            continue;

                        default:
                            LOG_DBG("Not correct message type");
                            continue;
                    }

                default:
                    LOG_DBG("Not correct message direction");
                    msgq_cur_msg_tx_ptr = nullptr;
                    continue;
            }

            if (msgq_cur_msg_tx_ptr)
                k_msgq_put(msgq_cur_msg_tx_ptr, &tx_msg, K_NO_WAIT);

            k_msgq_get(&msgq_rssi, &rssi, K_MSEC(1));
            rssi_num = check_rssi(rssi);
            atomic_set(&status_ind.led_strip_state.status.con_status, rssi_num);
            atomic_set(&status_ind.led_strip_state.status.people_num, rx_msg.workers_in_safe_zone);
            strip_ind = &status_ind;
            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
        }

        if (atomic_get(&alarm_is_active)) {
            /* If number workers in safe zone equal NUMBER_OF_NODES then buzzer switching in idle mode and
             * sending message disable_alarm_msg for signalman */
            if (workers_in_safe_zone == NUMBER_OF_NODES) {

                disable_alarm_msg.workers_in_safe_zone = workers_in_safe_zone;
                k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_IDLE);
                k_work_submit(&work_buzzer);
                atomic_set(&alarm_is_active, false);

                switch (alarm_addr) {
                    case SIGNALMAN_1_ADDR:
                        disable_alarm_msg.receiver_addr = SIGNALMAN_1_ADDR;
                        k_msgq_put(&msgq_tx_msg_prio, &disable_alarm_msg, K_NO_WAIT);
                        break;
                    case SIGNALMAN_2_ADDR:
                        disable_alarm_msg.receiver_addr = SIGNALMAN_2_ADDR;
                        k_msgq_put(&msgq_tx_msg_prio, &disable_alarm_msg, K_NO_WAIT);
                        break;
                    default:
                        break;
                }

            }
        }

        k_sleep(K_MSEC(1));
    }
}


[[noreturn]] void base_station_modem_task(void)
{
    /**
     * Lora config begin
     * */

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config lora_cfg = {
      .frequency = 433000000,
      .bandwidth = BW_125_KHZ,
      .datarate = SF_12,
      .coding_rate = CR_4_5,
      .preamble_len = 8,
      .payload_len = MESSAGE_LEN_IN_BYTES,
      .fixed_len = true,
      .tx_power = 0,
      .tx = true,
    };

    if (!device_is_ready(lora_dev)) {
        k_sleep(K_FOREVER);
    }
    /**
    * Lora config end
    * */

    /* Init system and send first sync message */
    system_init();
    k_sleep(K_FOREVER);

    while(true) {
        modem_fun(lora_dev, &lora_cfg);
        k_sleep(K_USEC(100));
    }
}


void button_homeward_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button homeward pressed");
}


static void periodic_timer_handler(struct k_timer *tim)
{
    LOG_DBG("Periodic timer handler");
    current_state = transmit_state;

    k_wakeup(modem_task_id);
}


static void dwork_sync_handler(struct k_work *item)
{
    LOG_DBG("Sync delayed work handler");
    k_msgq_put(&msgq_tx_msg, &sync_msg, K_NO_WAIT);
    k_work_reschedule(k_work_delayable_from_work(item), K_MSEC(10*PERIOD_TIME_MSEC));
}
/**
 * Function definition area end
 * */
#endif