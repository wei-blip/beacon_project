//
// Created by rts on 05.02.2022.
//
#include "lora_rr/lora_rr_common.h"
#include "dwm_rr/dwm_rr_common.h"


#include <logging/log.h>
    LOG_MODULE_REGISTER(base_station, LOG_LEVEL_DBG);


/**
 * Enum area begin
 * */
const enum DEVICE_ADDR_e cur_dev_addr = BASE_STATION_ADDR;
static enum BATTERY_LEVEL_e cur_battery_level = BATTERY_LEVEL_GOOD;
/**
 * Enum area end
 * */

/**
 * Structure area begin
 * */
static struct message_s sync_msg = {
  .sender_addr = cur_dev_addr,
  .receiver_addr = BROADCAST_ADDR,
  .message_type = MESSAGE_TYPE_SYNC,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 8
};

static struct message_s home_msg = {
  .sender_addr = cur_dev_addr,
  .receiver_addr = BROADCAST_ADDR,
  .message_type = MESSAGE_TYPE_HOMEWARD,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 8
};

static struct message_s disable_alarm_msg = {
  .sender_addr = cur_dev_addr,
  .message_type = MESSAGE_TYPE_DISABLE_ALARM,
  .direction = RESPONSE,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

const struct device *button_homeward_gpio_dev_ptr;

struct gpio_callback button_homeward_cb;

struct k_work_delayable dwork_sync = {{{nullptr}}};
/**
 * Structure area end
 * */


/**
 * Function declaration area begin
 * */
static void dwork_sync_handler(struct k_work *item);
void button_homeward_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

static void system_init();
//static void periodic_timer_handler(struct k_timer* tim); // callback for periodic_timer
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
    common_kernel_services_init();
//    k_work_init_delayable(&dwork_sync, dwork_sync_handler);
    /**
     * Kernel services init end
     * */

    /* Light down LED strip */
    set_ind(&strip_ind, K_FOREVER);

    current_state = recv_state;

    /**
     * Filling structure begin
     * */

    /**
    * Filling structure end
    * */
    set_buzzer_mode(BUZZER_MODE_SINGLE);
//    k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_SINGLE);
//    k_work_submit(&work_buzzer);
//    k_work_schedule(&dwork_sync, K_NO_WAIT);
    tim_start(K_NO_WAIT, K_MSEC(PERIOD_TIME_MSEC));
}


[[noreturn]] void proc_task()
{
    uint8_t rssi_num = 0;
    uint8_t workers_in_safe_zone = 0;
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

    while (true) {
        /* Always update info about workers in safe zone if it is possible */
        if (!k_msgq_get(&msgq_dwm_dist, &workers_in_safe_zone, K_NO_WAIT)) {
            sync_msg.workers_in_safe_zone = workers_in_safe_zone;
            home_msg.workers_in_safe_zone = workers_in_safe_zone;
            disable_alarm_msg.workers_in_safe_zone = workers_in_safe_zone;
        }

        if (!radio_rx_queue_is_empty()) {
            /**
             * Processing receive data
             * */
            /* Processing receiving data */
            if (!proc_rx_data(rx_buf, sizeof(rx_buf), &rx_msg, cur_dev_addr)) {
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
                                    set_buzzer_mode(BUZZER_MODE_IDLE);
                                    atomic_set(&alarm_is_active, false);
                                    break;
                                default:
                                    LOG_DBG("Undefined sender address for this message type");
                                    break;
                            }
                            strip_ind = &msg_recv_ind;
                            set_ind(&strip_ind, K_FOREVER);
                            strip_ind = &disable_indication;
                            set_ind(&strip_ind, K_FOREVER);
                            /* Retransmit disable alarm message */
                            set_msg(&tx_msg, true);
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            strip_ind = &alarm_ind;
                            set_ind(&strip_ind, K_FOREVER);
                            set_buzzer_mode(BUZZER_MODE_CONTINUOUS);

                            atomic_set(&alarm_is_active, true);
                            /* Save alarm address in few railways case */
                            alarm_addr = rx_msg.sender_addr;
                            /* Retransmit alarm message */
                            set_msg(&tx_msg, true);
                            break;

                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            set_msg(&tx_msg, false);
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
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            break;

                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            break;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            break;

                        default:
                            LOG_DBG("Not correct message type");
                            break;
                    }

                default:
                    LOG_DBG("Not correct message direction");
                    continue;
            }

//            get_rssi(&rssi);
//            rssi_num = check_rssi(rssi);
//            atomic_set(&status_ind.led_strip_state.status.con_status, rssi_num);
//            atomic_set(&status_ind.led_strip_state.status.people_num, rx_msg.workers_in_safe_zone);
//            strip_ind = &status_ind;
//            set_ind(&strip_ind, K_FOREVER);
        }

        if (atomic_get(&alarm_is_active)) {
            /* If number workers in safe zone equal NUMBER_OF_NODES then buzzer switching in idle mode and
             * sending message disable_alarm_msg for signalman */
            if (workers_in_safe_zone == NUMBER_OF_NODES) {

                disable_alarm_msg.workers_in_safe_zone = workers_in_safe_zone;
                set_buzzer_mode(BUZZER_MODE_IDLE);
                atomic_set(&alarm_is_active, false);

                switch (alarm_addr) {
                    case SIGNALMAN_1_ADDR:
                        disable_alarm_msg.receiver_addr = SIGNALMAN_1_ADDR;
                        set_msg(&disable_alarm_msg, true);
                        break;
                    case SIGNALMAN_2_ADDR:
                        disable_alarm_msg.receiver_addr = SIGNALMAN_2_ADDR;
                        set_msg(&disable_alarm_msg, true);
                        break;
                    default:
                        break;
                }

                strip_ind = &msg_recv_ind;
                set_ind(&strip_ind, K_FOREVER);
                strip_ind = &disable_indication;
                set_ind(&strip_ind, K_FOREVER);

            }
        }

        k_sleep(K_MSEC(1));
    }
}

[[noreturn]] void modem_task()
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
        k_sleep(K_MSEC(1));
    }
}

void button_homeward_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button homeward pressed");
}

static void dwork_sync_handler(struct k_work *item)
{
    LOG_DBG("Sync delayed work handler");
    set_msg(&sync_msg, false);
    k_work_reschedule(k_work_delayable_from_work(item), K_MSEC(10*PERIOD_TIME_MSEC));
}

void work_button_pressed_handler_dev(struct gpio_dt_spec *irq_gpio)
{
    /* For this device do nothing */
}

void periodic_timer_handler(struct k_timer *tim)
{
    static uint8_t cnt = 5;
    LOG_DBG("Periodic timer handler");
    current_state = transmit_state;

    if (cnt == 5) {
        set_msg(&sync_msg, false);
        cnt = 0;
    }
    cnt++;
    k_wakeup(modem_task_id);
}
/**
 * Function definition area end
 * */
