//
// Created by rts on 07.02.2022.
//

#include "lora_rr/lora_rr_common.h"

#if CUR_DEVICE == BRIGADE_CHIEF

#include <logging/log.h>
LOG_MODULE_REGISTER(brigade_chief);

/*
 * SW0 - disable alarm button
 * SW1 - left train passed button
 * SW2 - right train passed button
 * */
#define DISABLE_ALARM_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(DISABLE_ALARM_NODE, okay)
#error "Unsupported board: disable_alarm_sw devicetree alias is not defined"
#endif

//#define LEFT_TRAIN_PASSED_NODE	DT_ALIAS(sw1)
//#if !DT_NODE_HAS_STATUS(LEFT_TRAIN_PASSED_NODE, okay)
//#error "Unsupported board: left_train_passed_sw devicetree alias is not defined"
//#endif
//
//#define RIGHT_TRAIN_PASSED_NODE DT_ALIAS(sw2)
//#if !DT_NODE_HAS_STATUS(RIGHT_TRAIN_PASSED_NODE, okay)
//#error "Unsupported board: right_train_passed_sw alias is not defined"
//#endif

static struct message_s disable_alarm_msg = {
  .sender_addr = BRIGADE_CHIEF_ADDR,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_DISABLE_ALARM,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

static struct message_s left_train_passed_msg = {
  .sender_addr = BRIGADE_CHIEF_ADDR,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_LEFT_TRAIN_PASSED,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};

static struct message_s right_train_passed_msg = {
  .sender_addr = BRIGADE_CHIEF_ADDR,
  .receiver_addr = BASE_STATION_ADDR,
  .message_type = MESSAGE_TYPE_RIGHT_TRAIN_PASSED,
  .direction = REQUEST,
  .battery_level = BATTERY_LEVEL_GOOD,
  .workers_in_safe_zone = 0
};


struct gpio_dt_spec button_disable_alarm = GPIO_DT_SPEC_GET_OR(DISABLE_ALARM_NODE, gpios,{0});
//struct gpio_dt_spec button_right_train_passed = GPIO_DT_SPEC_GET_OR(RIGHT_TRAIN_PASSED_NODE, gpios,{0});
//struct gpio_dt_spec button_left_train_passed = GPIO_DT_SPEC_GET_OR(LEFT_TRAIN_PASSED_NODE, gpios,{0});

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
const enum DEVICE_ADDR_e cur_dev_addr = BRIGADE_CHIEF_ADDR;
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
/**
 * Function declaration area end
 * */


/**
 * Function definition area begin
 * */
void system_init()
{
    struct led_strip_indicate_s *strip_ind = nullptr;

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
    common_kernel_services_init();
    /**
     * Kernel services init end
     * */

    /* Light down LED strip */
    strip_ind = &status_ind;
    set_ind(&strip_ind, K_FOREVER);

    current_state = recv_state;

    set_buzzer_mode(BUZZER_MODE_SINGLE);
}

[[noreturn]] void app_task()
{
    int8_t event = 0;
    uint8_t rssi_num = 0;
    int32_t rc = 0;

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
                rc = modem_fun(lora_dev, &lora_cfg);
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
                proc_fun(&status_ind);
                cnt = 0;
                break;
            case EVENT_RX_MODE:
                LOG_DBG("Rx mode event");
                start_rx(lora_dev, &lora_cfg);
                cnt = 0;
                break;

            default:
                LOG_DBG("Event error!!!");
                break;
        }
    }
}

void proc_fun(void *dev_data)
{
    int16_t rssi = 0;
    struct message_s tx_msg = {
      .receiver_addr = BASE_STATION_ADDR,
      .direction = RESPONSE,
      .battery_level = BATTERY_LEVEL_GOOD
    };
    static struct message_s rx_msg = {0};
    static struct led_strip_indicate_s *strip_ind = &status_ind;
    static uint8_t rx_buf[MESSAGE_LEN_IN_BYTES];


    /* Processing receiving data */
    if (!proc_rx_data(rx_buf, sizeof(rx_buf), &rx_msg, cur_dev_addr)) {
        return;
    }

    switch (rx_msg.direction) {
        case REQUEST:
            tx_msg.sender_addr = cur_dev_addr;
            tx_msg.message_type = rx_msg.message_type;

            request_analysis(&rx_msg, &tx_msg, strip_ind);
            break;

        case RESPONSE:
            response_analysis(&rx_msg, &tx_msg, strip_ind);
            break;

        default:
            LOG_DBG("Not correct message direction");
            break;
    }


    get_rssi(&rssi);
    ((led_strip_indicate_s*)(dev_data))->led_strip_state.status.con_status = check_rssi(rssi);
    ((led_strip_indicate_s*)(dev_data))->led_strip_state.status.people_num = rx_msg.workers_in_safe_zone;
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
        case MESSAGE_TYPE_DISABLE_ALARM:
            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
            atomic_set(&alarm_is_active, false);

            if (rx_msg->sender_addr == cur_dev_addr) {
                LOG_DBG("Brigade chief disabled alarm");
            } else {
                LOG_DBG("Base station disabled alarm");
            }

            strip_ind = &msg_recv_ind;
            set_ind(&strip_ind, K_FOREVER);
            strip_ind = &status_ind;
            set_ind(&strip_ind, K_MINUTES(STRIP_INDICATION_TIMEOUT_MIN));

            set_buzzer_mode(BUZZER_MODE_DING_DONG);
            break;

        case MESSAGE_TYPE_ALARM:
            LOG_DBG(" MESSAGE_TYPE_ALARM");
            /* Then received response on alarm message from base station */
            strip_ind = &alarm_ind;
            set_ind(&strip_ind, K_FOREVER);
            atomic_set(&alarm_is_active, true);
            break;

        case MESSAGE_TYPE_RIGHT_TRAIN_PASSED:
            LOG_DBG(" MESSAGE_TYPE_RIGHT_TRAIN_PASSED");
            if (rx_msg->sender_addr == cur_dev_addr) {
                strip_ind = &msg_recv_ind;
                set_ind(&strip_ind, K_FOREVER);

                set_buzzer_mode(BUZZER_MODE_DING_DONG);
            }
            break;

        case MESSAGE_TYPE_LEFT_TRAIN_PASSED:
            LOG_DBG(" MESSAGE_TYPE_LEFT_TRAIN_PASSED");
            if (rx_msg->sender_addr == cur_dev_addr) {
                strip_ind = &msg_recv_ind;
                set_ind(&strip_ind, K_FOREVER);

                set_buzzer_mode(BUZZER_MODE_DING_DONG);
            }
            break;

        default:
            LOG_DBG("Do nothing for this message type and direction ...");
            break;
    }
}

void request_analysis(const struct message_s *rx_msg, struct message_s *tx_msg, struct led_strip_indicate_s *strip_ind)
{
    static bool is_first = true;

    LOG_DBG(" REQUEST");
    LOG_DBG("Message type:");
    switch (rx_msg->message_type) {
        case MESSAGE_TYPE_SYNC:
            if (is_first) {
                strip_ind = &status_ind;
                set_ind(&strip_ind, K_MINUTES(STRIP_INDICATION_TIMEOUT_MIN));
                is_first = false;
            }
            break;

        case MESSAGE_TYPE_DISABLE_ALARM:
            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
            switch (rx_msg->sender_addr) {
                case BASE_STATION_ADDR:
                    LOG_DBG("Base station disabled alarm");
                    break;
                default:
                    LOG_DBG("Undefined sender address for this message type");
                    break;
            }

            break;

        case MESSAGE_TYPE_HOMEWARD:
            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
            /* TODO: Make indication */
            set_msg(tx_msg, false);
            break;

        default:
            LOG_DBG("Do nothing for this message type and direction ...");
            break;
    }
}

void button_disable_alarm_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("Button disable alarm pressed");
    irq_routine(&button_disable_alarm);
}

//void button_left_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
//{
//    LOG_DBG("Button left train pass pressed");
//    irq_routine(&button_left_train_passed);
//}
//
//void button_right_train_pass_pressed_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
//{
//    LOG_DBG("Button right train pass pressed");
//    irq_routine(&button_right_train_passed);
//}

void work_button_pressed_handler_dev(struct gpio_dt_spec *irq_gpio)
{
    struct led_strip_indicate_s *strip_ind = nullptr;
    /* Send disable alarm message */
    if ((!strcmp(button_disable_alarm.port->name, irq_gpio->port->name)) &&
      (irq_gpio->pin == button_disable_alarm.pin)) {
        set_msg(&disable_alarm_msg, true);
    }

//    /* Send right train passed message */
//    if ((!strcmp(button_right_train_passed.port->name, irq_gpio->port->name)) &&
//      (irq_gpio->pin == button_right_train_passed.pin)) {
//        set_msg(&right_train_passed_msg, false);
//    }
//
//    /* Send left train passed */
//    if ((!strcmp(button_left_train_passed.port->name, irq_gpio->port->name)) &&
//      (irq_gpio->pin == button_left_train_passed.pin)) {
//        set_msg(&left_train_passed_msg, false);
//    }
}
/**
 * Function definition area end
 * */
#endif