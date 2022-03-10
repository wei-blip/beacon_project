//
// Created by rts on 05.02.2022.
//
#include "lora_russia_railways_common.h"

#if CUR_DEVICE == BASE_STATION
#elif CUR_DEVICE == SIGNALMAN

const struct gpio_dt_spec button_alarm = GPIO_DT_SPEC_GET_OR(ALARM_NODE, gpios,{0});
//const struct gpio_dt_spec button_anti_dream = GPIO_DT_SPEC_GET_OR(ANTI_DREAM_NODE, gpios,{0});
//const struct gpio_dt_spec button_train_passed = GPIO_DT_SPEC_GET_OR(TRAIN_PASSED_NODE, gpios,{0});

static const struct message_s alarm_msg = {
  .receiver_addr = BASE_STATION_ADDR,
  .sender_addr = SIGNALMAN_1_ADDR,
  .message_type = MESSAGE_TYPE_ALARM,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

static const struct message_s train_passed_msg = {
  .receiver_addr = BASE_STATION_ADDR,
  .sender_addr = SIGNALMAN_1_ADDR,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0,
  .message_type = MESSAGE_TYPE_LEFT_TRAIN_PASSED
};

static const struct message_s anti_dream_msg = {
  .receiver_addr = BASE_STATION_ADDR,
  .sender_addr = SIGNALMAN_1_ADDR,
  .message_type = MESSAGE_TYPE_ANTI_DREAM,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

#elif CUR_DEVICE == BRIGADE_CHIEF

const struct gpio_dt_spec button_disable_alarm = GPIO_DT_SPEC_GET_OR(DISABLE_ALARM_NODE, gpios,{0});
//const struct gpio_dt_spec button_right_train_passed = GPIO_DT_SPEC_GET_OR(RIGHT_TRAIN_PASSED_NODE, gpios,{0});
//const struct gpio_dt_spec button_left_train_passed = GPIO_DT_SPEC_GET_OR(LEFT_TRAIN_PASSED_NODE, gpios,{0});

static const struct message_s disable_alarm_msg = {
  .receiver_addr = BASE_STATION_ADDR,
  .sender_addr = BRIGADE_CHIEF_ADDR,
  .message_type = MESSAGE_TYPE_DISABLE_ALARM,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

static const struct message_s left_train_passed_msg = {
  .receiver_addr = BASE_STATION_ADDR,
  .sender_addr = BRIGADE_CHIEF_ADDR,
  .message_type = MESSAGE_TYPE_LEFT_TRAIN_PASSED,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

static const struct message_s right_train_passed_msg = {
  .receiver_addr = BASE_STATION_ADDR,
  .sender_addr = BRIGADE_CHIEF_ADDR,
  .message_type = MESSAGE_TYPE_RIGHT_TRAIN_PASSED,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

#endif

/**
 * Extern variable definition and initialisation begin
 * */
/* priority queue for sending messages */
K_MSGQ_DEFINE(msgq_tx_msg_prio, sizeof(struct message_s), QUEUE_LEN_IN_ELEMENTS, 1);
/* none priority queue for sending messages */
K_MSGQ_DEFINE(msgq_tx_msg, sizeof(struct message_s), QUEUE_LEN_IN_ELEMENTS, 1);
/* queue for receiving messages */
K_MSGQ_DEFINE(msgq_rx_msg, MESSAGE_LEN_IN_BYTES, QUEUE_LEN_IN_ELEMENTS, 1);
/* queue for rssi values */
K_MSGQ_DEFINE(msgq_rssi, sizeof(int16_t), QUEUE_LEN_IN_ELEMENTS, 2);

const struct device *lora_dev_ptr = {0};
const struct device *buzzer_dev_ptr = {0};

atomic_t atomic_interval_count = ATOMIC_INIT(0);
atomic_t atomic_device_active = ATOMIC_INIT(0);


struct lora_modem_config lora_cfg = {
  .frequency = 433000000,
  .bandwidth = BW_125_KHZ,
  .datarate = SF_12,
  .preamble_len = 8,
  .coding_rate = CR_4_5,
  .tx_power = 0,
  .tx = true,
  .fixed_len = true,
  .payload_len = MESSAGE_LEN_IN_BYTES
};

struct k_timer periodic_timer = {0};

struct k_work work_buzzer = {0};
struct k_work work_button_pressed = {0};

struct k_mutex mut_msg_info = {0};
struct k_mutex mut_buzzer_mode = {0};

struct gpio_dt_spec *cur_irq_gpio_ptr = {0};

struct buzzer_mode_s buzzer_mode = {0};

const modem_state_t recv_state = {
        .next = &transmit_state,
        .state = RECEIVE
};
const modem_state_t transmit_state = {
        .next = &recv_state,
        .state = TRANSMIT
};

struct message_s tx_msg = {0};

uint8_t tx_buf[MESSAGE_LEN_IN_BYTES] = {0};
uint8_t rx_buf[MESSAGE_LEN_IN_BYTES] = {0};

const struct led_strip_indicate_s msg_send_good_ind = {
    .start_led_pos = 0,
    .end_led_pos = STRIP_NUM_PIXELS,
    .led_strip_state.strip_param.color = COMMON_STRIP_COLOR_GREEN,
    .led_strip_state.strip_param.blink_cnt = 5,
    .indication_type = INDICATION_TYPE_BLINK
};

const struct led_strip_indicate_s msg_send_bad_ind = {
    .start_led_pos = 0,
    .end_led_pos = STRIP_NUM_PIXELS,
    .led_strip_state.strip_param.color = COMMON_STRIP_COLOR_RED,
    .led_strip_state.strip_param.blink_cnt = 5,
    .indication_type = INDICATION_TYPE_BLINK
};

const struct led_strip_indicate_s msg_recv_ind = {
    .start_led_pos = 0,
    .end_led_pos = STRIP_NUM_PIXELS,
    .led_strip_state.strip_param.color = COMMON_STRIP_COLOR_GREEN,
    .led_strip_state.strip_param.blink_cnt = 5,
    .indication_type = INDICATION_TYPE_BLINK
};

struct led_strip_indicate_s status_ind = {
    .start_led_pos = 0,
    .end_led_pos = STRIP_NUM_PIXELS,
    .led_strip_state.status.people_num = ATOMIC_INIT(0),
    .led_strip_state.status.con_status = ATOMIC_INIT(0),
    .indication_type = INDICATION_TYPE_STATUS_INFO
};

const struct led_strip_indicate_s disable_indication = {
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state.status.people_num = ATOMIC_INIT(0),
  .led_strip_state.status.con_status = ATOMIC_INIT(0),
  .indication_type = INDICATION_TYPE_STATUS_INFO
};

const struct led_strip_indicate_s middle_pressed_button_ind = {
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state.strip_param.color = COMMON_STRIP_COLOR_PURPLE,
  .indication_type = INDICATION_TYPE_STATIC_COLOR
};
//const struct led_strip_indicate_s *disable_indicate_ptr = &disable_indication;
/**
 * Extern variable definition and initialisation end
 * */


/**
 * Function definition area begin
 * */
void work_button_pressed_handler(struct k_work *item)
{
    bool short_pressed_is_set = false;
    bool middle_pressed_is_set = false;
    bool long_pressed_is_set = false;
    struct led_strip_indicate_s *strip_ind = NULL;
    atomic_set(&atomic_interval_count, 0);

    /* While button pressed count number of intervals */
    while (gpio_pin_get(cur_irq_gpio_ptr->port, cur_irq_gpio_ptr->pin)) {
        k_sleep(K_MSEC(INTERVAL_TIME_MS));
        atomic_inc(&atomic_interval_count);
        if ((atomic_get(&atomic_interval_count) > SHORT_PRESSED_MIN_VAL) &&
          (atomic_get(&atomic_interval_count) <= SHORT_PRESSED_MAX_VAL)) { /* Short pressed */
            /* Light up first half strip */
            if (!short_pressed_is_set) {
                strip_ind = &status_ind;
                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
                short_pressed_is_set = true;
                k_poll_signal_raise(&signal_indicate, 1);
            }
        } else if ((atomic_get(&atomic_interval_count) > MIDDLE_PRESSED_MIN_VAL) &&
          (atomic_get(&atomic_interval_count) <= MIDDLE_PRESSED_MAX_VAL)) { /* Middle pressed */
            /* Light up full strip */
            if (!middle_pressed_is_set) {
                strip_ind = &middle_pressed_button_ind;
                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
                middle_pressed_is_set = true;
            }
        } else if (atomic_get(&atomic_interval_count) > LONG_PRESSED_MIN_VAL) { /* Long pressed */
            long_pressed_is_set = true;
        }
    }

    /* Sound indicate */
    if (short_pressed_is_set) {
        if (!k_mutex_lock(&mut_buzzer_mode, K_USEC(500))) {
            buzzer_mode.single = true;
            k_mutex_unlock(&mut_buzzer_mode);
            while(k_work_busy_get(&work_buzzer)) {
                K_MSEC(10);
            }
            k_work_submit(&work_buzzer);
        }
    }

    /* Do action */
    if (short_pressed_is_set && (!middle_pressed_is_set)) { /* Short pressed */
        /* TODO: Booting device */
    } else if (middle_pressed_is_set && !long_pressed_is_set) { /* Middle pressed */
#if CUR_DEVICE == SIGNALMAN
        /* Send alarm message */
        if ((!strcmp(button_alarm.port->name, cur_irq_gpio_ptr->port->name)) &&
          (cur_irq_gpio_ptr->pin == button_alarm.pin)) {
            k_msgq_put(&msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT);
        }

        /* TODO: Uncomment this after tests */
//        /* Send train passed message */
//        if ((!strcmp(BUTTON_TRAIN_PASSED_GPIO_PORT, cur_irq_gpio.port->name)) &&
//          (cur_irq_gpio.pin == BUTTON_TRAIN_PASSED_GPIO_PIN)) {
//            k_msgq_put(&msgq_tx_msg, &train_passed_msg, K_NO_WAIT);
//        }
//
//        /* Anti-dream handler */
//        if ((!strcmp(BUTTON_ANTI_DREAM_GPIO_PORT, cur_irq_gpio.port->name)) &&
//          (cur_irq_gpio.pin == BUTTON_ANTI_DREAM_GPIO_PIN)) {
//            k_work_submit(&work_buzzer); /* Disable alarm */
//            k_timer_stop(&anti_dream_timer);
//            atomic_set(&anti_dream_active, 0);
//        }
#elif CUR_DEVICE == BRIGADE_CHIEF
        /* Send disable alarm message */
        if ((!strcmp(button_disable_alarm.port->name, cur_irq_gpio_ptr->port->name)) &&
          (cur_irq_gpio_ptr->pin == button_disable_alarm.pin)) {
            k_msgq_put(&msgq_tx_msg_prio, &disable_alarm_msg, K_NO_WAIT);
        }

//        /* TODO: Uncomment this after tests */
//        /* Send right train passed message */
//        if ((!strcmp(BUTTON_RIGHT_TRAIN_PASSED_GPIO_PORT, cur_irq_gpio.port->name)) &&
//          (cur_irq_gpio.pin == BUTTON_RIGHT_TRAIN_PASSED_GPIO_PIN)) {
//            k_msgq_put(&msgq_tx_msg, &right_train_passed_msg, K_NO_WAIT);
//        }
//
//        /* Send left train passed */
//        if ((!strcmp(BUTTON_LEFT_TRAIN_PASSED_GPIO_PORT, cur_irq_gpio.port->name)) &&
//          (cur_irq_gpio.pin == BUTTON_LEFT_TRAIN_PASSED_GPIO_PIN)) {
//            k_msgq_put(&msgq_tx_msg, &left_train_passed_msg, K_NO_WAIT);
//        }
#endif
    } else if (long_pressed_is_set) { /* Long pressed */
        /* TODO: Shut down device */
    }
}
/**
 * Function definition area end
 * */