//
// Created by rts on 21.01.2022.
//
#include "russia_railways_common.h"
#if CUR_DEVICE == SIGNALMAN

#include <logging/log.h>
    LOG_MODULE_REGISTER(signalman);

atomic_t anti_dream_active = ATOMIC_INIT(0);


/**
 * Structure area begin
 * */
static struct k_timer anti_dream_timer;
static struct k_work work_anti_dream;

struct gpio_callback button_anti_dream_cb;
struct gpio_callback button_alarm_cb;
struct gpio_callback button_train_passed_cb;

static struct led_strip_indicate_s anti_dream_ind = {
  .indication_type = INDICATION_TYPE_BLINK,
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state = {
    .strip_param = {
      .color = COMMON_STRIP_COLOR_RED,
      .blink_cnt = 25
    }
  }
};

static struct led_strip_indicate_s alarm_ind = {
  .indication_type = INDICATION_TYPE_STATIC_COLOR,
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state = {
    .strip_param = {
      .color = COMMON_STRIP_COLOR_RED,
      .blink_cnt = 0
    }
  }
};
/**
 * Structure area end
 * */


/**
 * Enum area begin
 * */
static enum DEVICE_ADDR_e cur_dev_addr = SIGNALMAN_1_ADDR;
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

static void system_init(void);
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
    buzzer_dev = DEVICE_DT_GET(PWM_CTLR);
    if (!device_is_ready(buzzer_dev)) {
        LOG_DBG("Error: PWM device %s is not ready\n", buzzer_dev->name);
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

    k_timer_init(&periodic_timer, periodic_timer_handler, nullptr);
    k_timer_init(&anti_dream_timer, anti_dream_timer_handler, nullptr);
    /**
     * Kernel services init end
     * */

     /* Light down LED strip */
    k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

    current_state = recv_state;

    k_work_submit(&work_buzzer);
    k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_SINGLE);
}


[[noreturn]] void signalman_proc_task()
{
    uint8_t rssi_num = 0;
    int16_t rssi = 0;
    uint8_t rx_buf_proc[MESSAGE_LEN_IN_BYTES];
    int32_t ret = 0;
    struct message_s tx_msg_proc = {
      .receiver_addr = BASE_STATION_ADDR,
      .direction = RESPONSE,
      .battery_level = BATTERY_LEVEL_GOOD
    };
    struct message_s rx_msg_proc = {0};
    struct led_strip_indicate_s *strip_ind = &status_ind;
    struct k_msgq* msgq_cur_msg_tx_ptr = &msgq_tx_msg; /* Default queue */

    while(true) {
        if (k_msgq_num_used_get(&msgq_rx_msg)) {

            /* Processing receiving data */
            if (!proc_recv_data(&msgq_rx_msg, rx_buf_proc, &rx_msg_proc, cur_dev_addr)) {
                continue;
            }

            /* If message from base station (send broadcast message can only base station) then alarm disabled */
            if (rx_msg_proc.receiver_addr == BROADCAST_ADDR) {
                atomic_cas(&alarm_is_active, true, false);
            }

            switch (rx_msg_proc.direction) {
                case REQUEST:
                    LOG_DBG(" REQUEST");
                    LOG_DBG("Message type:");
                    tx_msg_proc.sender_addr = cur_dev_addr;
                    tx_msg_proc.message_type = rx_msg_proc.message_type;

                    switch (rx_msg_proc.message_type) {

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            /* TODO: Make indication */
                            break;

                        default:
                            LOG_DBG("Do nothing for this message type and direction...");
                            continue;
                    }
                    break;

                case RESPONSE:
                    LOG_DBG(" RESPONSE");
                    LOG_DBG("Message type:");
                    switch (rx_msg_proc.message_type) {

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            /* TODO: Make indication if message alarm sends from second signalman */
                            if (rx_msg_proc.sender_addr == cur_dev_addr) {
                                strip_ind = &msg_recv_ind;
                                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);

                                /* Set alarm_indication if alarm response receive and
                                 * alarm_is_active still equal true */
                                if (atomic_cas(&alarm_is_active, true, true)) {
                                    strip_ind = &alarm_ind;
                                    k_msgq_put(&msgq_led_strip, &strip_ind , K_NO_WAIT);
                                }

                                k_work_submit(&work_buzzer);
                                k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_DING_DONG);
                            }
                            msgq_cur_msg_tx_ptr = nullptr;
                            break;

                        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
                        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            if (rx_msg_proc.sender_addr == cur_dev_addr) {
                                strip_ind = &msg_recv_ind;
                                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
                                k_work_submit(&work_buzzer);
                                k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_DING_DONG);
                            }
                            msgq_cur_msg_tx_ptr = nullptr;
                            break;

                        default:
                            LOG_DBG("Do nothing for this message type and direction...");
                            continue;

                    }
                    break;

                default:
                    LOG_DBG("Not correct message direction");
                    continue;
            }

            if (msgq_cur_msg_tx_ptr)
                k_msgq_put(msgq_cur_msg_tx_ptr, &tx_msg_proc, K_NO_WAIT);

            k_msgq_get(&msgq_rssi, &rssi, K_MSEC(1));
            rssi_num = check_rssi(rssi);
            atomic_set(&status_ind.led_strip_state.status.con_status, rssi_num);
            atomic_set(&status_ind.led_strip_state.status.people_num, rx_msg_proc.workers_in_safe_zone);
            ret = k_poll(&event_indicate, 1, K_NO_WAIT);

            /* Change indication only if alarm not active */
            if (atomic_cas(&alarm_is_active, false, false)) {
                if (!ret)
                    strip_ind = &status_ind;
                else if (ret == (-EAGAIN))
                    strip_ind = &disable_indication;
                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
            }

        }
        k_sleep(K_USEC(100));
    }
}


[[noreturn]] void signalman_modem_task()
{
    int32_t rc = 0;
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
      .tx = false,
    };
    struct led_strip_indicate_s *strip_ind = nullptr;

    /**
     * Lora config begin
     * */
    if (!device_is_ready(lora_dev)) {
        k_sleep(K_FOREVER);
    }
    if (lora_config(lora_dev, &lora_cfg) < 0 ) {
        k_sleep(K_FOREVER);
    }
    /**
     * Lora config end
     * */

    system_init();

    /**
     * Receive sync message begin
     * */
    lora_recv_async(lora_dev, lora_receive_cb, lora_receive_error_timeout);
    /**
     * Receive sync message end
     * */

    while(true) {
        rc = modem_fun(lora_dev, &lora_cfg);
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


void button_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button alarm pressed");
    atomic_set(&alarm_is_active, true);
    cur_irq_gpio_ptr = &button_alarm;
    k_work_submit(&work_button_pressed);
}


void button_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button train pass pressed");
    cur_irq_gpio_ptr = &button_train_passed;
    k_work_submit(&work_button_pressed);
}


void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    LOG_DBG("Button anti-dream pressed");
    cur_irq_gpio_ptr = &button_anti_dream;
    k_work_submit(&work_buzzer); /* Disable alarm */
    k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_IDLE);
    k_timer_stop(&anti_dream_timer);
    atomic_set(&anti_dream_active, 0);
    k_work_submit(&work_button_pressed);
}


static void periodic_timer_handler(struct k_timer *tim)
{
    LOG_DBG("Periodic timer handler");
    struct led_strip_indicate_s *strip_ind = nullptr;
//    k_msgq_put(&msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT); // for debug
    static uint8_t anti_dream_cnt = 0;
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
//    if (ANTI_DREAM_START) { /* For anti-dream */
//        k_work_submit(&work_anti_dream);
//        anti_dream_cnt = 0;
//    }

//    if (cnt == (SYNC_COUNT+CURRENT_DEVICE_NUM)) {
//        k_msgq_put(&msgq_tx_msg, &sync_msg, K_NO_WAIT);
//        cnt = 0;
//    }
    cnt++;
    anti_dream_cnt++;
    k_wakeup(modem_task_id);
}


static void anti_dream_timer_handler(struct k_timer *tim)
{
    /* If anti_dream_timer expiry and (req_is_send == 1) putting anti_dream_msg into queue */
//    atomic_cas(&anti_dream_msg_info.req_is_send, 0, 1);
//    check_msg_status(&anti_dream_msg_info);
}


static void work_anti_dream_handler(struct k_work *item)
{
    /* Start anti-dream mode if not active */
    if (!atomic_get(&anti_dream_active)) {
        struct led_strip_indicate_s *strip_ind = nullptr;
        k_work_submit(&work_buzzer);
        k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_CONTINUOUS);

        atomic_set(&anti_dream_active, 1);
        k_timer_start(&anti_dream_timer, K_MINUTES(ANTI_DREAM_TIME_MIN), K_MINUTES(ANTI_DREAM_TIME_MIN));

        strip_ind = &anti_dream_ind;
        k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
        k_wakeup(update_indication_task_id);
    }
}
/**
 * Function definition area end
 * */
#endif