//
// Created by rts on 05.02.2022.
//
#include "rr_common.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(common);

#if CUR_DEVICE != WORKERS
/**
 * Devices settings area begin
 * */
#if CUR_DEVICE == BASE_STATION


#elif CUR_DEVICE == SIGNALMAN

struct gpio_dt_spec button_alarm = GPIO_DT_SPEC_GET_OR(ALARM_NODE, gpios,{0});
struct gpio_dt_spec button_anti_dream = GPIO_DT_SPEC_GET_OR(ANTI_DREAM_NODE, gpios,{0});
struct gpio_dt_spec button_train_passed = GPIO_DT_SPEC_GET_OR(TRAIN_PASSED_NODE, gpios,{0});

/**
 * Message structures begin
 * */
static const struct message_s alarm_msg = {
  .sender_addr = SIGNALMAN_1_ADDR,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_ALARM,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

static const struct message_s train_passed_msg = {
  .sender_addr = SIGNALMAN_1_ADDR,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_LEFT_TRAIN_PASSED,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0,
};
/**
 * Message structures end
 * */

atomic_t anti_dream_active = ATOMIC_INIT(false);

#elif CUR_DEVICE == BRIGADE_CHIEF

struct gpio_dt_spec button_disable_alarm = GPIO_DT_SPEC_GET_OR(DISABLE_ALARM_NODE, gpios,{0});
//struct gpio_dt_spec button_right_train_passed = GPIO_DT_SPEC_GET_OR(RIGHT_TRAIN_PASSED_NODE, gpios,{0});
//struct gpio_dt_spec button_left_train_passed = GPIO_DT_SPEC_GET_OR(LEFT_TRAIN_PASSED_NODE, gpios,{0});

static const struct message_s disable_alarm_msg = {
  .sender_addr = BRIGADE_CHIEF_ADDR,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_DISABLE_ALARM,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

static const struct message_s left_train_passed_msg = {
  .sender_addr = BRIGADE_CHIEF_ADDR,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_LEFT_TRAIN_PASSED,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

static const struct message_s right_train_passed_msg = {
  .sender_addr = BRIGADE_CHIEF_ADDR,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_RIGHT_TRAIN_PASSED,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

#endif
/**
 * Devices settings area end
 * */


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

const struct device *buzzer_dev = nullptr;

atomic_t atomic_device_active = ATOMIC_INIT(0);
atomic_t alarm_is_active = ATOMIC_INIT(false);

struct k_timer periodic_timer = {{{{nullptr}}}};

struct k_work work_buzzer = {{nullptr}};
struct k_work work_button_pressed = {{nullptr}};
struct k_work_delayable dwork_enable_ind = {{{nullptr}}};

struct k_poll_signal signal_buzzer = K_POLL_SIGNAL_INITIALIZER(signal_buzzer);
struct k_poll_event event_buzzer = K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                                                   K_POLL_MODE_NOTIFY_ONLY,
                                                                   &signal_buzzer,
                                                                   0);

struct gpio_dt_spec *irq_gpio_dev = nullptr;

const modem_state_t recv_state = {
  .next =  const_cast<modem_state_s *>(&transmit_state),
  .state = RECEIVE
};

const modem_state_t transmit_state = {
  .next =  const_cast<modem_state_s *>(&recv_state),
  .state = TRANSMIT
};

modem_state_t current_state = {nullptr};
atomic_t reconfig_modem = ATOMIC_INIT(0);

//struct message_s tx_msg = {0};

//uint8_t tx_buf[MESSAGE_LEN_IN_BYTES] = {0};
//uint8_t rx_buf[MESSAGE_LEN_IN_BYTES] = {0};

/**
 * Indication structures begin
 * */
struct led_strip_indicate_s msg_send_good_ind = {
  .indication_type = INDICATION_TYPE_BLINK,
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state = {
    .strip_param = {
      .color = COMMON_STRIP_COLOR_GREEN,
      .blink_cnt = 2
      }
    },
};

struct led_strip_indicate_s msg_send_bad_ind = {
  .indication_type = INDICATION_TYPE_BLINK,
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state = {
    .strip_param = {
      .color = COMMON_STRIP_COLOR_RED,
      .blink_cnt = 2
    }
  }
};

struct led_strip_indicate_s msg_recv_ind = {
  .indication_type = INDICATION_TYPE_BLINK,
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state = {
    .strip_param = {
      .color = COMMON_STRIP_COLOR_GREEN,
      .blink_cnt = 2
    }
  }

};

struct led_strip_indicate_s status_ind = {
  .indication_type = INDICATION_TYPE_STATUS_INFO,
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state = {
    .status = {
      .con_status = ATOMIC_INIT(0),
      .people_num = ATOMIC_INIT(0)
    }
  }
};

struct led_strip_indicate_s disable_indication = {
  .indication_type = INDICATION_TYPE_STATUS_INFO,
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state = {
    .status = {
      .con_status = ATOMIC_INIT(0),
      .people_num = ATOMIC_INIT(0)
    }
  }
};

struct led_strip_indicate_s short_pressed_button_ind = {
  .indication_type = INDICATION_TYPE_STATIC_COLOR,
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS/2,
  .led_strip_state = {
    .strip_param = {
      .color = COMMON_STRIP_COLOR_PURPLE
    }
  }
};

struct led_strip_indicate_s middle_pressed_button_ind = {
  .indication_type = INDICATION_TYPE_STATIC_COLOR,
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state = {
    .strip_param = {
      .color = COMMON_STRIP_COLOR_PURPLE
    }
  }
};
/**
 * Indication structures end
 * */

/**
 * Extern variable definition and initialisation end
 * */


/**
 * Function definition area begin
 * */
void work_button_pressed_handler(struct k_work *item) {
    atomic_t atomic_interval_count = ATOMIC_INIT(0); /* Counted number of function button pressed call */
    bool short_pressed_is_set = false;
    bool middle_pressed_is_set = false;
    bool long_pressed_is_set = false;
    struct led_strip_indicate_s *strip_ind = nullptr;
    atomic_set(&atomic_interval_count, 0);

    /*
     * While button pressed count number of intervals.
     *
     * Short pressed for turn on indication
     * Middle pressed for sending messages
     * Long pressed for turn off device
     * */
    while (gpio_pin_get(irq_gpio_dev->port, irq_gpio_dev->pin)) {
        k_sleep(K_MSEC(INTERVAL_TIME_MS));
        atomic_inc(&atomic_interval_count);
        if ((atomic_get(&atomic_interval_count) > SHORT_PRESSED_MIN_VAL) &&
          (atomic_get(&atomic_interval_count) <= SHORT_PRESSED_MAX_VAL)) { /* Short pressed */
            /* Light up status */
            if (!short_pressed_is_set) {
                strip_ind = &short_pressed_button_ind;
                k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
                short_pressed_is_set = true;
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
        k_work_submit(&work_buzzer);
        k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_SINGLE);
    }

    /* Do action */
    if (short_pressed_is_set && (!middle_pressed_is_set)) { /* Short pressed */
        /* TODO: Booting device */

        /* Enable indication */
        strip_ind = &status_ind;
        k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
        k_poll_signal_raise(&signal_indicate, STANDARD_MODE_INDICATION);
        k_work_reschedule(&dwork_enable_ind, K_MINUTES(STRIP_INDICATION_TIMEOUT_MIN));
    } else if (middle_pressed_is_set && !long_pressed_is_set) { /* Middle pressed */
#if CUR_DEVICE == SIGNALMAN
        /* Send alarm message */
        if ((!strcmp(button_alarm.port->name, irq_gpio_dev->port->name)) &&
          (irq_gpio_dev->pin == button_alarm.pin)) {
            k_msgq_put(&msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT);
        }

        /* TODO: Uncomment this after tests */
        /* Send train passed message */
        if ((!strcmp(button_train_passed.port->name, irq_gpio_dev->port->name)) &&
          (irq_gpio_dev->pin == button_train_passed.pin)) {
            k_msgq_put(&msgq_tx_msg, &train_passed_msg, K_NO_WAIT);
        }

        /* Anti-dream handler */
        if ((!strcmp(button_anti_dream.port->name, irq_gpio_dev->port->name)) &&
          (irq_gpio_dev->pin == button_anti_dream.pin)) {
            /* Disable alarm */
            k_poll_signal_raise(&signal_buzzer, BUZZER_MODE_IDLE);
            k_work_submit(&work_buzzer);

            /* Enable indication */
            strip_ind = &status_ind;
            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
            k_poll_signal_raise(&signal_indicate, STANDARD_MODE_INDICATION);
            k_work_reschedule(&dwork_enable_ind, K_MINUTES(STRIP_INDICATION_TIMEOUT_MIN));

            atomic_set(&anti_dream_active, false);
        }
#elif CUR_DEVICE == BRIGADE_CHIEF
        /* Send disable alarm message */
        if ((!strcmp(button_disable_alarm.port->name, irq_gpio_dev->port->name)) &&
          (irq_gpio_dev->pin == button_disable_alarm.pin)) {
            k_msgq_put(&msgq_tx_msg_prio, &disable_alarm_msg, K_NO_WAIT);
        }

//        /* TODO: Uncomment this after tests */
//        /* Send right train passed message */
//        if ((!strcmp(BUTTON_RIGHT_TRAIN_PASSED_GPIO_PORT, irq_gpio_dev.port->name)) &&
//          (irq_gpio_dev.pin == BUTTON_RIGHT_TRAIN_PASSED_GPIO_PIN)) {
//            k_msgq_put(&msgq_tx_msg, &right_train_passed_msg, K_NO_WAIT);
//        }
//
//        /* Send left train passed */
//        if ((!strcmp(BUTTON_LEFT_TRAIN_PASSED_GPIO_PORT, irq_gpio_dev.port->name)) &&
//          (irq_gpio_dev.pin == BUTTON_LEFT_TRAIN_PASSED_GPIO_PIN)) {
//            k_msgq_put(&msgq_tx_msg, &left_train_passed_msg, K_NO_WAIT);
//        }
#endif
    } else if (long_pressed_is_set) { /* Long pressed */
        /* TODO: Shut down device */
    }
}

void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr) {
    static struct k_spinlock spin;
    static k_spinlock_key_t key;
    volatile uint16_t len = size;

    key = k_spin_lock(&spin);

    /*
     * Compare first byte in receive message and 13
     * 13 means thar received message has the following parameters:
     * SENDER_ADDR = BASE_STATION, RECV_ADDR = BROADCAST, MESSAGE_TYPE = SYNC
     * */
    if ((*data) == 13) {
        LOG_DBG(" REQUEST");
        LOG_DBG(" MESSAGE_TYPE_SYNC");
        k_timer_stop(&periodic_timer);
        k_sleep(K_MSEC(DELAY_TIME_MSEC));
        k_timer_start(&periodic_timer, K_MSEC(DURATION_TIME_MSEC), K_MSEC(PERIOD_TIME_MSEC));
    }

    if ((size != MESSAGE_LEN_IN_BYTES) || is_empty_msg(data, size)) {
        LOG_DBG("Reconfig modem");
        atomic_set(&reconfig_modem, 1);
        k_spin_unlock(&spin, key);
        return;
    }
    k_msgq_put(&msgq_rx_msg, data, K_NO_WAIT);
    k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);
    k_spin_unlock(&spin, key);
}

void lora_receive_error_timeout(const struct device *dev) {
    static struct k_spinlock spin;
    static k_spinlock_key_t key;
    key = k_spin_lock(&spin);

    /* Restart receive */
    lora_recv_async(dev, nullptr, nullptr);
    lora_recv_async(dev, lora_receive_cb, lora_receive_error_timeout);

    k_spin_unlock(&spin, key);
}

void work_buzzer_handler(struct k_work *item) {
    uint8_t i = 0;

    /* Wait while signal will be raised */
    while (k_poll(&event_buzzer, 1, K_NO_WAIT)) {
        k_sleep(K_MSEC(5));
    }

    switch (event_buzzer.signal->result) {
        case BUZZER_MODE_CONTINUOUS:
            pwm_pin_set_usec(buzzer_dev, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                             BUTTON_PRESSED_PERIOD_TIME_USEC / 2U, PWM_FLAGS);
            break;
        case BUZZER_MODE_DING_DONG:
            i = 0;
            while (i < 2) {
                pwm_pin_set_usec(buzzer_dev, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                                 BUTTON_PRESSED_PERIOD_TIME_USEC / 2U, PWM_FLAGS);
                k_sleep(K_USEC(2 * BUTTON_PRESSED_PERIOD_TIME_USEC));
                pwm_pin_set_usec(buzzer_dev, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                                 0, PWM_FLAGS);
                k_sleep(K_USEC(2 * BUTTON_PRESSED_PERIOD_TIME_USEC));
                i++;
            }
            break;
        case BUZZER_MODE_SINGLE:
            pwm_pin_set_usec(buzzer_dev, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                             BUTTON_PRESSED_PERIOD_TIME_USEC / 2U, PWM_FLAGS);
            k_sleep(K_USEC(BUTTON_PRESSED_PERIOD_TIME_USEC));
        case BUZZER_MODE_IDLE:
        default:
            pwm_pin_set_usec(buzzer_dev, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                             0, PWM_FLAGS);
            break;
    }
    event_buzzer.signal->signaled = 0;
    event_buzzer.state = K_POLL_STATE_NOT_READY;
}


bool proc_rx_data(struct k_msgq *msgq, uint8_t *recv_data, size_t len, struct message_s *rx_msg, uint8_t cur_dev_addr)
{
    uint32_t cur_msg = 0;

    k_msgq_get(msgq, recv_data, K_NO_WAIT);

    for (uint8_t i = 0; i < len; ++i) {
        recv_data[i] = reverse(recv_data[i]);
        cur_msg |= (recv_data[i]) << i*8;
    }

    read_write_message(&cur_msg, rx_msg, false); /* fill rx_msg struct */
    if ( (rx_msg->receiver_addr != BROADCAST_ADDR) &&
      (rx_msg->receiver_addr != cur_dev_addr) ) {
        LOG_DBG("addr = 0x%02x, own addr = 0x%02x", rx_msg->receiver_addr, cur_dev_addr);
        LOG_DBG("Packet is filtered");
        return false;
    }

    return true;
}


bool proc_tx_data(struct k_msgq *msgq, uint8_t *tx_data, size_t len, struct message_s *tx_msg)
{
    uint32_t new_msg = 0;
    /* Check messages into queue
     * Beginning check priority queue, after check standard queue
     * If queue's is not empty receiving will be stopped */
    if (k_msgq_num_used_get(&msgq_tx_msg_prio)) {
        k_msgq_get(&msgq_tx_msg_prio, tx_msg, K_NO_WAIT);
        msgq = &msgq_tx_msg_prio;
    } else if (k_msgq_num_used_get(&msgq_tx_msg)) {
        k_msgq_get(&msgq_tx_msg, tx_msg, K_NO_WAIT);
        msgq = &msgq_tx_msg;
    } else {
        /* Return 1 */
        current_state = *current_state.next;
        return false;
    }

    read_write_message(&new_msg, tx_msg, true);
    for (uint8_t i = 0; i < len; ++i) {
        *(tx_data + i) = (new_msg & (0x000000FF << i * 8) ) >> i * 8;
        *(tx_data + i) = reverse(*(tx_data + i));
    }
    return true;
}


void dwork_enable_ind_handler(struct k_work *item)
{
    struct led_strip_indicate_s *strip_ind = nullptr;

    if (!k_poll(&event_indicate, 1, K_NO_WAIT)) {
        strip_ind = &disable_indication;
        k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
        event_indicate.signal->signaled = 0;
        event_indicate.state = K_POLL_STATE_NOT_READY;
    }
}
#endif
/**
 * Function definition area end
 * */