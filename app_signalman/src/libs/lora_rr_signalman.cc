//
// Created by rts on 21.01.2022.
//
#include "lora_rr/lora_rr_common.h"

#include <logging/log.h>
    LOG_MODULE_REGISTER(signalman);

#define ANTI_DREAM_PERIOD (10*PERIOD_TIME_MSEC)

/*
 * SW0 - alarm button
 * SW1 - anti-dream button
 * SW2 - train passed button
 * */
#define ALARM_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(ALARM_NODE, okay)
#error "Unsupported board: alarm_sw devicetree alias is not defined"
#endif

//#define ANTI_DREAM_NODE	DT_ALIAS(sw1)
//#if !DT_NODE_HAS_STATUS(ANTI_DREAM_NODE, okay)
//#error "Unsupported board: anti_dream_sw devicetree alias is not defined"
//#endif
//
//#define TRAIN_PASSED_NODE DT_ALIAS(sw2)
//#if !DT_NODE_HAS_STATUS(TRAIN_PASSED_NODE, okay)
//#error "Unsupported board: train_passed_sw alias is not defined"
//#endif


/**
 * Structure area begin
 * */
struct gpio_dt_spec button_alarm = GPIO_DT_SPEC_GET_OR(ALARM_NODE, gpios,{nullptr});
//struct gpio_dt_spec button_anti_dream = GPIO_DT_SPEC_GET_OR(ANTI_DREAM_NODE, gpios,{nullptr});
//struct gpio_dt_spec button_train_passed = GPIO_DT_SPEC_GET_OR(TRAIN_PASSED_NODE, gpios,{nullptr});

struct gpio_callback button_alarm_cb;
//struct gpio_callback button_anti_dream_cb;
//struct gpio_callback button_train_passed_cb;

static struct k_work_delayable dwork_anti_dream = {{{nullptr}}};

atomic_t anti_dream_active = ATOMIC_INIT(false);

/**
 * Enum area begin
 * */
const enum DEVICE_ADDR_e cur_dev_addr = SIGNALMAN_1_ADDR;
static enum BATTERY_LEVEL_e cur_battery_level = BATTERY_LEVEL_GOOD;
/**
 * Enum area end
 * */

static struct message_s alarm_msg = {
  .sender_addr = cur_dev_addr,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_ALARM,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

static struct message_s train_passed_msg = {
  .sender_addr = cur_dev_addr,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_LEFT_TRAIN_PASSED,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0,
};

static struct message_s anti_dream_msg = {
  .sender_addr = cur_dev_addr,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_ANTI_DREAM,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

static struct led_strip_indicate_s anti_dream_ind = {
  .indication_type = INDICATION_TYPE_BLINK,
  .start_led_pos = 0,
  .end_led_pos = STRIP_NUM_PIXELS,
  .led_strip_state = {
    .strip_param = {
      .color = COMMON_STRIP_COLOR_RED,
      .blink_cnt = 11
    }
  }
};
/**
 * Structure area end
 * */


/**
 * Function declaration area begin
 * */
void button_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
//void button_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
//void button_anti_dream_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

static void system_init();
static void dwork_anti_dream_handler(struct k_work *item);
static void anti_dream_timer_handler(struct k_timer *tim); // callback for anti-dream timer
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
     * Init IRQ (change gpio init after tests) begin
     * */
     if (!device_is_ready(button_alarm.port)) {
         LOG_DBG("Error: button device %s is not ready\n", button_alarm.port->name);
         k_sleep(K_FOREVER);
     }
//
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
//    gpio_add_callback(button_anti_dream.port, &button_anti_dream_cb);
//    gpio_add_callback(button_train_passed.port, &button_train_passed_cb);
    /**
     * Init IRQ end
     * */

    /**
     * Kernel services init begin
     * */
    common_kernel_services_init();
//    k_work_init_delayable(&dwork_anti_dream, dwork_anti_dream_handler); /* For anti-dream */
    /**
     * Kernel services init end
     * */

     /* Light down LED strip */
    set_ind(&strip_ind, K_FOREVER);

//    k_work_schedule(&dwork_anti_dream, K_MSEC(ANTI_DREAM_PERIOD));
    set_buzzer_mode(BUZZER_MODE_SINGLE);
}

[[noreturn]] void app_task()
{
    int8_t event = 0;
    uint8_t rssi_num = 0;
    int32_t rc = 0;
    uint8_t rx_buf[MESSAGE_LEN_IN_BYTES];
    struct message_s rx_msg = {0};

    /**
    * Lora config begin
    * */
    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config lora_cfg = {
      .frequency = 433000000,
      .bandwidth = BW_125_KHZ,
      .datarate = SF_10,
      .coding_rate = CR_4_5,
      .preamble_len = 8,
      .payload_len = MESSAGE_LEN_IN_BYTES,
      .fixed_len = true,
      .tx_power = 0,
      .tx = false,
    };

    if (!device_is_ready(lora_dev)) {
        k_sleep(K_FOREVER);
    }

    if ( lora_config(lora_dev, &lora_cfg) < 0 ) {
        k_sleep(K_FOREVER);
    }
    /**
    * Lora config end
    * */

    struct led_strip_indicate_s *strip_ind = nullptr;

    /* Init system and put first sync message into queue */
    system_init();

    lora_recv_async(lora_dev, lora_receive_cb, lora_receive_error_timeout);

    uint32_t cnt = 0;

    while(true) {
        do {
            k_sleep(K_MSEC(1));
            event = wait_app_event();
            /* Bad practices */
            cnt++;
            if (cnt == 20*PERIOD_TIME_MSEC) {
                printk("Bad practices lora is worked\n");
                cnt = 0;
                event = EVENT_RX_MODE;
            }
        } while (event < 0);

        switch (event) {
            case EVENT_TX_MODE:
                LOG_DBG("Tx mode event");
                rc = start_tx(lora_dev, &lora_cfg);
                if (!rc) {
                    strip_ind = &msg_send_good_ind;
                    set_ind(&strip_ind, K_FOREVER);

                } else if (rc < 0) {
                    strip_ind = &msg_send_bad_ind;
                    set_ind(&strip_ind, K_FOREVER);

                    if (atomic_get(&alarm_is_active)) {
                        strip_ind = &alarm_ind;
                        set_ind(&strip_ind, K_FOREVER);
                    }

                }
                break;
            case EVENT_PROC_RX_DATA:
                LOG_DBG("Processing data event");
                /* Processing receiving data */
                if (!proc_rx_data(rx_buf, sizeof(rx_buf), &rx_msg, cur_dev_addr)) {
                    continue;
                }
                /* Continue only if anti-dream not active */
                if (!atomic_get(&anti_dream_active)) {
                    analysis_fun(&rx_msg, &status_ind);
                }
                cnt = 0;
                break;
            case EVENT_RX_MODE:
                LOG_DBG("Rx mode event");
                cnt = 0;
                start_rx(lora_dev, &lora_cfg);
                break;

            default:
                LOG_DBG("Event error!!!");
                break;
        }
    }
}

void analysis_fun(struct message_s *rx_msg, void *dev_data)
{
    int16_t rssi = 0;
    struct message_s tx_msg = {
      .receiver_addr = BASE_STATION_ADDR,
      .direction = RESPONSE,
      .battery_level = BATTERY_LEVEL_GOOD
    };

    static struct led_strip_indicate_s *strip_ind = &status_ind;

    switch (rx_msg->direction) {
        case REQUEST:
            tx_msg.sender_addr = cur_dev_addr;
            tx_msg.message_type = rx_msg->message_type;
            request_analysis(rx_msg, &tx_msg, strip_ind);
            break;

        case RESPONSE:
            response_analysis(rx_msg, &tx_msg, strip_ind);
            break;

        default:
            LOG_DBG("Not correct message direction");
            break;
    }


    get_rssi(&rssi);
    ((led_strip_indicate_s*)(dev_data))->led_strip_state.status.con_status = check_rssi(rssi);
    ((led_strip_indicate_s*)(dev_data))->led_strip_state.status.people_num = rx_msg->workers_in_safe_zone;
    /* Change indication only if alarm not active */
    if (!atomic_get(&alarm_is_active)) {
        if (indicate_is_enabled()) {
            strip_ind = &status_ind;
            set_ind(&strip_ind, K_MINUTES(STRIP_INDICATION_TIMEOUT_MIN));
        }
    }

}

void response_analysis(const struct message_s *rx_msg, struct message_s *tx_msg, struct led_strip_indicate_s *strip_ind)
{
    LOG_DBG(" RESPONSE");
    LOG_DBG("Message type:");
    switch (rx_msg->message_type) {

        case MESSAGE_TYPE_ALARM:
            LOG_DBG(" MESSAGE_TYPE_ALARM");
            /* TODO: Make indication if message alarm sends from second signalman */
            if (rx_msg->sender_addr == cur_dev_addr) {
                strip_ind = &msg_recv_ind;
                set_ind(&strip_ind, K_FOREVER);

                set_buzzer_mode(BUZZER_MODE_DING_DONG);

                atomic_set(&alarm_is_active, true);
                strip_ind = &alarm_ind;
                set_ind(&strip_ind, K_FOREVER);
            }
            break;

        case MESSAGE_TYPE_DISABLE_ALARM:
            /* Signalize that alarm is disabled */
            atomic_set(&alarm_is_active, false);
            strip_ind = &msg_recv_ind;
            set_ind(&strip_ind, K_FOREVER);
            strip_ind = &status_ind;
            set_ind(&strip_ind, K_MINUTES(STRIP_INDICATION_TIMEOUT_MIN));
            break;

        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
            if (rx_msg->sender_addr == cur_dev_addr) {
                strip_ind = &msg_recv_ind;
                set_ind(&strip_ind, K_MINUTES(STRIP_INDICATION_TIMEOUT_MIN));
                set_buzzer_mode(BUZZER_MODE_DING_DONG);
            }
            break;

        default:
            LOG_DBG("Do nothing for this message type and direction...");
            break;

    }
}

void request_analysis(const struct message_s *rx_msg, struct message_s *tx_msg, struct led_strip_indicate_s *strip_ind)
{
    static bool is_first = true;

    LOG_DBG(" REQUEST");
    LOG_DBG("Message type:");
    tx_msg->sender_addr = cur_dev_addr;
    tx_msg->message_type = rx_msg->message_type;

    switch (rx_msg->message_type) {
        case MESSAGE_TYPE_SYNC:
            if (is_first) {
                strip_ind = &status_ind;
                set_ind(&strip_ind, K_MINUTES(STRIP_INDICATION_TIMEOUT_MIN));
                is_first = false;
            }
            break;

        case MESSAGE_TYPE_HOMEWARD:
            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
            set_msg(tx_msg, false);
            /* TODO: Make indication */
            break;

        default:
            LOG_DBG("Do nothing for this message type and direction...");
            break;
    }
}

void button_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button alarm pressed");
    /* Return if anti-dream started */
    if (atomic_get(&anti_dream_active)) {
        return;
    }

    button_irq_routine(&button_alarm);
}

//void button_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
//{
//    LOG_DBG("Button train pass pressed");
//    /* Return if anti-dream started */
//    if (atomic_get(&anti_dream_active)) {
//        return;
//    }
//
//    irq_routine(&button_train_passed);
//}
//
//void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
//{
//    LOG_DBG("Button anti-dream pressed");
//    irq_routine(&button_anti_dream);
//}

//static void dwork_anti_dream_handler(struct k_work *item)
//{
//    /*
//     * Start anti-dream mode if it is not active.
//     * Else anti-dream already started and anti_dream_active == true, then send anti-dream message on base station.
//     * Don't start anti-dream mode if started alarm mode
//     * */
//    struct led_strip_indicate_s *strip_ind = nullptr;
//
//    if (!atomic_get(&alarm_is_active)) {
//        if (!atomic_get(&anti_dream_active)) {
//            set_buzzer_mode(BUZZER_MODE_CONTINUOUS);
//
//            strip_ind = &anti_dream_ind;
//            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
//            strip_ind = &alarm_ind;
//            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
//            k_wakeup(update_indication_task_id);
//        } else {
//            /* If anti-dream mode not stopped restart indication */
//            set_msg(&anti_dream_msg, true);
//            strip_ind = &anti_dream_ind;
//            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
//            strip_ind = &alarm_ind;
//            k_msgq_put(&msgq_led_strip, &strip_ind, K_NO_WAIT);
//        }
//    }
//    k_work_reschedule(k_work_delayable_from_work(item), K_MSEC(ANTI_DREAM_PERIOD));
//}

void work_button_pressed_handler_dev(struct gpio_dt_spec *irq_gpio)
{
    struct led_strip_indicate_s *strip_ind = nullptr;
    /* For this device do nothing */
    /* Send alarm message */
    if ((!strcmp(button_alarm.port->name, irq_gpio->port->name)) &&
      (irq_gpio->pin == button_alarm.pin)) {
        set_msg(&alarm_msg, true);
    }

//    /* TODO: Check indication duration */
//    /* Send train passed message */
//    if ((!strcmp(button_train_passed.port->name, irq_gpio->port->name)) &&
//      (irq_gpio->pin == button_train_passed.pin)) {
//        set_msg(&train_passed_msg, false);
//    }
//
//    /* Anti-dream handler */
//    if ((!strcmp(button_anti_dream.port->name, irq_gpio->port->name)) &&
//      (irq_gpio->pin == button_anti_dream.pin)) {
//        /* Disable alarm */
//        set_buzzer_mode(BUZZER_MODE_IDLE);
//
//        /* Enable indication */
//        strip_ind = &status_ind;
//        set_ind(&strip_ind, K_MINUTES(STRIP_INDICATION_TIMEOUT_MIN));
//        atomic_set(&anti_dream_active, false);
//    }
}
/**
 * Function definition area end
 * */
