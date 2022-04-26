//
// Created by rts on 05.02.2022.
//
#include "lora_rr/lora_rr_common.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(lora_rr_common, LOG_LEVEL_DBG);


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

K_MUTEX_DEFINE(mtx_buzzer);

const struct device *buzzer_dev = nullptr;

atomic_t atomic_device_active = ATOMIC_INIT(0);
atomic_t alarm_is_active = ATOMIC_INIT(false);

struct k_timer periodic_timer = {{{{nullptr}}}};

struct k_work work_buzzer = {{nullptr}};
struct k_work work_button_pressed = {{nullptr}};
struct k_work_delayable dwork_disable_ind = {{{nullptr}}};

struct k_poll_signal signal_buzzer = K_POLL_SIGNAL_INITIALIZER(signal_buzzer);
struct k_poll_event event_buzzer = K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                                                   K_POLL_MODE_NOTIFY_ONLY,
                                                                   &signal_buzzer,
                                                                   0);

struct k_poll_signal sig_tx_mode = K_POLL_SIGNAL_INITIALIZER(sig_tx_mode);
struct k_poll_signal sig_proc_rx_data = K_POLL_SIGNAL_INITIALIZER(sig_proc_rx_data);
struct k_poll_signal sig_rx_mode = K_POLL_SIGNAL_INITIALIZER(sig_rx_mode);

struct k_poll_event event_app[APPLICATION_EVENTS_NUM] = {
  K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                  K_POLL_MODE_NOTIFY_ONLY,
                                  &sig_tx_mode,
                                  0),
  K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                  K_POLL_MODE_NOTIFY_ONLY,
                                  &sig_proc_rx_data,
                                  0),
  K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                  K_POLL_MODE_NOTIFY_ONLY,
                                  &sig_rx_mode,
                                  0)
};

bool active_events[APPLICATION_EVENTS_NUM] = {false};

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

struct led_strip_indicate_s alarm_ind = {
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
 * Indication structures end
 * */

/**
 * Extern variable definition and initialisation end
 * */


/**
 * Function definition area begin
 * */
int32_t modem_fun(const struct device *lora_dev, struct lora_modem_config* lora_cfg)
{
    int32_t rc = 1;
    struct k_msgq *cur_queue = nullptr;
    static struct k_spinlock spin;
    static k_spinlock_key_t key;
    static message_s tx_msg = {0};
    static uint8_t tx_buf[MESSAGE_LEN_IN_BYTES] = {0};

    if (!proc_tx_data(cur_queue, tx_buf, sizeof(tx_buf), &tx_msg)) {
        start_rx(lora_dev, lora_cfg);
        return 1;
    }
    key = k_spin_lock(&spin);

    /* Stop receiving */
    lora_recv_async(lora_dev, nullptr, nullptr);

    lora_cfg->tx = true;
    rc = lora_config(lora_dev, lora_cfg);

    if (rc < 0) {
        k_msgq_put(cur_queue, &tx_msg, K_NO_WAIT);
        current_state = recv_state;
//            atomic_set(&reconfig_modem, 1);
        start_rx(lora_dev, lora_cfg);
        k_spin_unlock(&spin, key);
        return rc;
    }

    rc = lora_send(lora_dev, tx_buf, MESSAGE_LEN_IN_BYTES);
    /*
     * If message not transmit then putting back him into queue and reconfig modem
     * */
    if (rc < 0) {
        k_msgq_put(cur_queue, &tx_msg, K_NO_WAIT);
    }
    start_rx(lora_dev, lora_cfg);
//        atomic_set(&reconfig_modem, 1);

    if (tx_msg.message_type == MESSAGE_TYPE_SYNC)
        rc = 1;

    current_state = recv_state;

    k_spin_unlock(&spin, key);
    return rc;
}

void start_rx(const struct device *lora_dev, struct lora_modem_config* lora_cfg)
{
    volatile int rc = 0;
    lora_recv_async(lora_dev, nullptr, nullptr);
    lora_cfg->tx = false;
    rc = lora_config(lora_dev, lora_cfg);
    if (!rc) {
        rc = lora_recv_async(lora_dev, lora_receive_cb, lora_receive_error_timeout);
        if (rc < 0) {
            k_poll_signal_raise(&sig_rx_mode, 1);
        }
    } else {
        k_poll_signal_raise(&sig_rx_mode, 1);
    }
}

void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr) {
    static struct k_spinlock spin;
    static k_spinlock_key_t key;
    volatile int rc = 0;

    key = k_spin_lock(&spin);

    if ((size != MESSAGE_LEN_IN_BYTES) || is_empty_msg(data, size)) {
        LOG_DBG("Reconfig modem");
        k_poll_signal_raise(&sig_rx_mode, 1);
        k_spin_unlock(&spin, key);
        return;
    }

    /*
     * Compare first byte in receive message and 13
     * 13 means thar received message has the following parameters:
     * SENDER_ADDR = BASE_STATION, RECV_ADDR = BROADCAST, MESSAGE_TYPE = SYNC
     * */
    if ((*data) == 13) {
        LOG_DBG(" REQUEST");
        LOG_DBG(" MESSAGE_TYPE_SYNC");
        k_timer_stop(&periodic_timer);
        k_timer_start(&periodic_timer, K_MSEC(DURATION_TIME_MSEC + DELAY_TIME_MSEC), K_MSEC(PERIOD_TIME_MSEC));
    }

    if (!k_msgq_num_free_get(&msgq_rx_msg)) {
        k_msgq_purge(&msgq_rx_msg);
    }
    k_msgq_put(&msgq_rx_msg, data, K_NO_WAIT);

    if (!k_msgq_num_free_get(&msgq_rssi)) {
        k_msgq_purge(&msgq_rssi);
    }
    k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);

    rc = k_poll_signal_raise(&sig_proc_rx_data, 1);
    k_spin_unlock(&spin, key);
}

void lora_receive_error_timeout(const struct device *dev) {
    static struct k_spinlock spin;
    static k_spinlock_key_t key;
    key = k_spin_lock(&spin);

    /* Restart receive */
    k_poll_signal_raise(&sig_rx_mode, 1);

    k_spin_unlock(&spin, key);
}

void set_ind(led_strip_indicate_s **ind, k_timeout_t duration_min)
{
    if ((duration_min.ticks != K_FOREVER.ticks)) {
        if (!indicate_is_enabled()) {
            enable_ind();
            k_work_schedule(&dwork_disable_ind, duration_min);
        }
    }

    if (!k_msgq_num_free_get(&msgq_led_strip)) {
        k_msgq_purge(&msgq_led_strip);
    }
    k_msgq_put(&msgq_led_strip, ind, K_NO_WAIT);
}

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
        if (!atomic_get(&alarm_is_active)) {
            strip_ind = &status_ind;
            set_ind(&strip_ind, K_MINUTES(STRIP_INDICATION_TIMEOUT_MIN));
        } else {
            strip_ind = &alarm_ind;
            set_ind(&strip_ind, K_FOREVER);
        }

    } else if (middle_pressed_is_set && !long_pressed_is_set) { /* Middle pressed */
        /*  */
        work_button_pressed_handler_dev(irq_gpio_dev);

    } else if (long_pressed_is_set) { /* Long pressed */
        /* TODO: Shut down device */
    }
}

void common_kernel_services_init()
{
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

    k_work_init(&work_buzzer, work_buzzer_handler);
    k_work_init(&work_button_pressed, work_button_pressed_handler);
    k_work_init_delayable(&dwork_disable_ind, dwork_disable_ind_handler); /* For enable and disable indication */

    k_timer_init(&periodic_timer, periodic_timer_handler, nullptr);
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
            k_sleep(K_USEC(BUTTON_PRESSED_PERIOD_TIME_USEC));
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

bool proc_rx_data(uint8_t *recv_data, size_t len, struct message_s *rx_msg, uint8_t cur_dev_addr)
{
    uint32_t cur_msg = 0;

    k_msgq_get(&msgq_rx_msg, recv_data, K_NO_WAIT);

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

void dwork_disable_ind_handler(struct k_work *item)
{
    struct led_strip_indicate_s *strip_ind = nullptr;
    disable_ind();
    k_sleep(K_MSEC(500));
    if (!atomic_get(&alarm_is_active)) {
        strip_ind = &disable_indication;
        set_ind(&strip_ind, K_FOREVER);
    } else {
        strip_ind = &alarm_ind;
        set_ind(&strip_ind, K_FOREVER);
    }
}

bool radio_rx_queue_is_empty()
{
    return (k_msgq_num_used_get(&msgq_rx_msg) == 0);
}

void set_rssi(int16_t *rssi)
{
    k_msgq_put(&msgq_rssi, rssi, K_NO_WAIT);
}

void get_rssi(int16_t *rssi)
{
    k_msgq_get(&msgq_rssi, rssi, K_MSEC(1));
}

void set_msg(struct message_s *msg, bool prio)
{
    if (prio)
        k_msgq_put(&msgq_tx_msg_prio, msg, K_NO_WAIT);
    else
        k_msgq_put(&msgq_tx_msg, msg, K_NO_WAIT);
}

void tim_start(k_timeout_t duration, k_timeout_t period)
{
    k_timer_start(&periodic_timer, duration, period);
}

void tim_restart(k_timeout_t duration, k_timeout_t period)
{
    k_timer_stop(&periodic_timer);
    k_timer_start(&periodic_timer, duration, period);
}

void set_buzzer_mode(uint8_t buzzer_mode)
{
    uint8_t i = 0;
    /* Wait while signal will be raised */
    while (k_mutex_lock(&mtx_buzzer, K_NO_WAIT)) {
        k_sleep(K_MSEC(5));
    }

    switch (buzzer_mode) {
        case BUZZER_MODE_CONTINUOUS:
            pwm_pin_set_usec(buzzer_dev, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                             BUTTON_PRESSED_PERIOD_TIME_USEC / 2U, PWM_FLAGS);
            k_sleep(K_USEC(BUTTON_PRESSED_PERIOD_TIME_USEC));
            break;
        case BUZZER_MODE_DING_DONG:
            i = 0;
            while (i < 2) {
                pwm_pin_set_usec(buzzer_dev, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                                 BUTTON_PRESSED_PERIOD_TIME_USEC / 2U, PWM_FLAGS);
                k_sleep(K_USEC(BUTTON_PRESSED_PERIOD_TIME_USEC));
                pwm_pin_set_usec(buzzer_dev, PWM_CHANNEL, BUTTON_PRESSED_PERIOD_TIME_USEC,
                                 0, PWM_FLAGS);
                k_sleep(K_USEC(BUTTON_PRESSED_PERIOD_TIME_USEC));
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
    k_mutex_unlock(&mtx_buzzer);
}

void irq_routine(struct gpio_dt_spec *dev)
{
    irq_gpio_dev = dev;
    k_work_submit(&work_button_pressed);
}

void periodic_timer_handler(struct k_timer *tim)
{
    LOG_DBG("Periodic timer handler");
    k_poll_signal_raise(&sig_tx_mode, 1);
    current_state = transmit_state;
}

int8_t wait_app_event()
{
    if (k_poll(event_app, APPLICATION_EVENTS_NUM, K_NO_WAIT)) {
        return (-1);
    }

    for (int8_t i = 0; i < APPLICATION_EVENTS_NUM; ++i) {
        if (event_app[i].state == K_POLL_STATE_SIGNALED) {
            /* Processing only one event */
            /* Reset event */
            event_app[i].signal->signaled = 0;
            event_app[i].state = K_POLL_STATE_NOT_READY;
            return i;
        }
    }
    return (-1);
}

int8_t get_app_event()
{
    for (int8_t i = 0; i < APPLICATION_EVENTS_NUM; ++i) {
        if (active_events[i]) {
            active_events[i] = false;
            return i;
        }
    }
    return -1;
}
/**
 * Function definition area end
 * */