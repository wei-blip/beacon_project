//
// Created by rts on 21.01.2022.
//
#include "lora_russia_railways.h"
#include <drivers/gpio.h>

#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

/// Array area begin
uint8_t tx_buf[MESSAGE_LEN_IN_BYTES];
//uint8_t rx_buf[MESSAGE_LEN_IN_BYTES];
/// Array area end


/// Structure area begin
struct message_s rx_msg;
struct message_s tx_msg;

struct device* lora_dev_ptr;
struct lora_modem_config lora_cfg;

struct k_work sender;

K_MSGQ_DEFINE(msgq_rx_buf, MESSAGE_LEN_IN_BYTES, QUEUE_LEN_IN_ELEMENTS, 1);
K_MSGQ_DEFINE(msgq_rssi, sizeof(int16_t), QUEUE_LEN_IN_ELEMENTS, 2);

#ifdef BASE_STATION
struct k_timer control_timer;
struct k_sem sem_anti_dream_msg;
#else
struct gpio_callback button_anti_dream_cb;
struct gpio_callback button_alarm_cb;
#endif
struct k_sem sem_proc_data;
struct k_queue queue_rssi;
struct device* button_alarm_gpio_dev_ptr;
struct device* button_anti_dream_gpio_dev_ptr;
/// Structure area end


/// Enum area begin
enum receiver_addr cur_dev_addr = RECV_SIGNALMAN_1;
enum people_in_safe_zone_ids people_in_safe_zone = FIRST_PEOPLE_ID;
/// Enum area end


/// Function declaration area begin
static void fill_msg_bit_field(uint32_t* msg_ptr, uint8_t field_val, uint8_t field_len, uint8_t* pos);
static uint8_t reverse(uint8_t input);
static void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write);
static void extract_msg_bit_field(const uint32_t* msg_ptr, uint8_t *field_val, uint8_t field_len, uint8_t* pos);
static uint8_t check_rssi(const int16_t rssi);

#ifdef BASE_STATION
void control_timer_handler(struct k_timer* tim); // callback for timer
#else
void button_alarm_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins);
void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins);
#endif
/// Function declaration area end


//// Function definition area begin
#ifdef BASE_STATION
int system_init(uint8_t tim_duration_min, unsigned int sem_anti_dream_init_val, unsigned int sem_anti_dream_lim,
                unsigned int sem_proc_data_init_val, unsigned int sem_proc_data_lim) {
    /// LoRa init begin
    lora_cfg.frequency = 433000000;
    lora_cfg.bandwidth = BW_125_KHZ;
    lora_cfg.datarate = SF_10;
    lora_cfg.preamble_len = 8;
    lora_cfg.coding_rate = CR_4_5;
    lora_cfg.tx_power = 5;
    lora_cfg.tx = false;

    lora_dev_ptr = DEVICE_DT_GET(DEFAULT_RADIO_NODE);

    if (!device_is_ready(lora_dev_ptr)) {
        return -1;
    }
    if ( lora_config(lora_dev_ptr, &lora_cfg) < 0 ) {
        return -1;
    }
    /// LoRa init end

    //// Kernel services init begin
    k_timer_init(&control_timer, control_timer_handler, NULL);
    k_timer_start(&control_timer, K_SECONDS(tim_duration_min * 60),
                  K_SECONDS(tim_duration_min * 60));

    k_sem_init(&sem_anti_dream_msg, sem_anti_dream_init_val, sem_anti_dream_lim);
    k_sem_init(&sem_proc_data, sem_proc_data_init_val, sem_proc_data_lim);

    k_queue_init(&queue_rssi);

    k_work_init(&sender, send_msg);
    /// Kernel services init end
    return 0;
}
#else
int system_init(unsigned int sem_proc_data_init_val, unsigned int sem_proc_data_lim) {
    /// LoRa init begin
    lora_cfg.frequency = 433000000;
    lora_cfg.bandwidth = BW_125_KHZ;
    lora_cfg.datarate = SF_10;
    lora_cfg.preamble_len = 8;
    lora_cfg.coding_rate = CR_4_5;
    lora_cfg.tx_power = 5;
    lora_cfg.tx = false;

    lora_dev_ptr = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    if (!device_is_ready(lora_dev_ptr)) {
        return -1;
    }
    if ( lora_config(lora_dev_ptr, &lora_cfg) < 0 ) {
        return -1;
    }
    /// LoRa init end

    //// Init IRQ (change gpio init after tests) begin
    button_alarm_gpio_dev_ptr = device_get_binding(BUTTON_ALARM_GPIO_PORT);
//    button_anti_dream_gpio_dev_ptr = device_get_binding(BUTTON_ANTI_DREAM_GPIO_PORT);

    gpio_pin_configure(button_alarm_gpio_dev_ptr, BUTTON_ALARM_GPIO_PIN,
                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));
//    gpio_pin_configure(button_anti_dream_gpio_dev_ptr, BUTTON_ANTI_DREAM_GPIO_PIN,
//                       (GPIO_INPUT | GPIO_PUSH_PULL | GPIO_ACTIVE_LOW));

    gpio_pin_interrupt_configure(button_alarm_gpio_dev_ptr, BUTTON_ALARM_GPIO_PIN, GPIO_INT_EDGE_TO_ACTIVE);
//    gpio_pin_interrupt_configure(&button_anti_dream_gpio_dev_ptr, BUTTON_ANTI_DREAM_GPIO_PORT,
//                                 GPIO_INT_EDGE_RISING);

    gpio_init_callback(&button_alarm_cb, button_alarm_pressed_cb, BIT(BUTTON_ALARM_GPIO_PIN));
//    gpio_init_callback(button_anti_dream_cb_ptr, button_anti_dream_pressed_cb,
//                       BIT(BUTTON_ANTI_DREAM_GPIO_PIN));

    gpio_add_callback(button_alarm_gpio_dev_ptr, &button_alarm_cb);
//    gpio_add_callback(button_anti_dream_gpio_dev_ptr, button_anti_dream_cb_ptr);
    /// Init IRQ end

    //// Kernel services init begin
    k_sem_init(&sem_proc_data, sem_proc_data_init_val, sem_proc_data_lim);

    k_queue_init(&queue_rssi);

    k_work_init(&sender, send_msg);
    // Threads init
//    volatile k_tid_t t1 = k_thread_create(&thread_data_proc, &thread_stack_data_proc, STACK_SIZE, processing_data,
//                    0, 0, 0,
//                    THREAD_PRIORITY, 0, K_NO_WAIT);
//    volatile k_tid_t t2 = k_thread_create(&thread_recv, &thread_stack_recv, STACK_SIZE, recv_msg,
//                    0, 0, 0,
//                    THREAD_PRIORITY, 0, K_NO_WAIT);
//    k_thread_start(&thread_data_proc);
//    k_thread_start(&thread_recv);
    /// Kernel services init end
    return 0;
}
#endif
static void fill_msg_bit_field(uint32_t* msg_ptr, uint8_t field_val, uint8_t field_len, uint8_t* pos) {
    uint8_t start_pos = *pos;
    while ( *pos < start_pos + field_len ) {
        *msg_ptr |= ( field_val & BIT((*pos) - start_pos) ) << start_pos;
        (*pos)++;
    }
}

static void extract_msg_bit_field(const uint32_t* msg_ptr, uint8_t* field_val, uint8_t field_len, uint8_t* pos) {
    uint8_t start_pos = *pos;
    while ( *pos < start_pos + field_len ) {
        (*field_val) |= ( (*msg_ptr) & BIT((*pos) ) ) >> start_pos;
        (*pos)++;
    }
}

static uint8_t reverse(uint8_t input) {
    uint8_t output;
    uint8_t bit = 0;
    uint8_t pos = 0;
    while( pos < 7 ) {
        bit = input & BIT(0);
        output |= bit;
        output = output << 1;
        input = input >> 1;
        pos++;
    }
    bit = input & BIT(0);
    output |= bit;
    return output;
}

int send_msg() {
    int rc;
    uint32_t new_msg = 0;
    read_write_message(&new_msg, &tx_msg, true);
    for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
        tx_buf[i] = (new_msg & (0x000000FF << i*8) ) >> i*8;
        tx_buf[i] = reverse(tx_buf[i]);
    }
    if (!lora_cfg.tx) {
        lora_cfg.tx = true;
        lora_config(lora_dev_ptr, &lora_cfg);
    }
    rc = lora_send(lora_dev_ptr, tx_buf, MESSAGE_LEN_IN_BYTES);
    return rc;
}

void recv_msg(void) {
#ifdef BASE_STATION
    system_init(TIMER_DURATION_MIN, SEM_ANTI_DREAM_INIT_VAL, SEM_ANTI_DREAM_LIM,
                SEM_PROC_DATA_INIT_VAL, SEM_PROC_DATA_LIM);

#else
    system_init(SEM_PROC_DATA_INIT_VAL, SEM_PROC_DATA_LIM);
#endif
    uint8_t rx_msg_[MESSAGE_LEN_IN_BYTES] = {0};
    int rc = 0;
    int16_t rssi = 0;
    int8_t snr = 0;
    uint8_t ind = 0;
    while(1) {
        printk("thread_1\n");
        if (lora_dev_ptr) {
            if (lora_cfg.tx) {
                lora_cfg.tx = false;
                lora_config(lora_dev_ptr, &lora_cfg);
            }
            rc = lora_recv(lora_dev_ptr, rx_msg_, MESSAGE_LEN_IN_BYTES, K_FOREVER, &rssi, &snr);
            if (rc > 0) {
                k_msgq_put(&msgq_rx_buf, &rx_msg_,K_NO_WAIT);
                k_queue_append(&queue_rssi, &rssi);
            }
#ifdef BASE_STATION
            if (!k_sem_take(&sem_anti_dream_msg, K_NO_WAIT)) {
                /// Create message for signalman 1
                tx_msg.receiver_addr = RECV_SIGNALMAN_1;
                tx_msg.sender_addr = cur_dev_addr;
                //    tx_msg.battery_level = ?;
                tx_msg.people_in_safe_zone = people_in_safe_zone;
                tx_msg.direction = REQUEST;
                tx_msg.message_type = MESSAGE_TYPE_ANTI_DREAM;
                send_msg(&tx_msg);  // it may need to be done several times
                /// Create message for signalman 2
                tx_msg.receiver_addr = RECV_SIGNALMAN_2;
                send_msg(&tx_msg); // it may need to be done several times
        }
#endif
        }
    }
}

void processing_data() {
    uint8_t con_qual_leds_num = 0;
    uint8_t rx_buf[MESSAGE_LEN_IN_BYTES] = {0};
    int16_t rssi = 0;
    uint32_t cur_msg = 0;
    while(1) {
        printk("thread_2\n");
        if ( k_msgq_num_free_get(&msgq_rx_buf) != QUEUE_LEN_IN_ELEMENTS ) {
            k_msgq_get(&msgq_rx_buf, &rx_buf,K_NO_WAIT);
            k_msgq_get(&msgq_rssi, &rssi, K_NO_WAIT);
//            int16_t* rssi_ptr = k_queue_get(&queue_rssi, K_NO_WAIT);
            //// Processing receive data
            for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
                rx_buf[i] = reverse(rx_buf[i]);
                cur_msg |= (rx_buf[i]) << i*8;
            }
            read_write_message(&cur_msg, &rx_msg, false); // rx_msg struct is fill
            if ( rx_msg.receiver_addr != cur_dev_addr )
                continue;
            con_qual_leds_num = check_rssi(&rssi);
            // ligth up leds
        }
        k_sleep(K_MSEC(1));
    }
}

static void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write) {
    uint8_t pos = 0;
    for (int cur_field = 0; cur_field < MESSAGE_FIELD_NUMBER; ++cur_field) {
        switch (cur_field) {
            case SENDER_ADDR:
                write ? fill_msg_bit_field(new_msg, msg_ptr->sender_addr, SENDER_ADDR_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->sender_addr, SENDER_ADDR_FIELD_LEN, &pos);
                break;
            case RECEIVER_ADDR:
                write ? fill_msg_bit_field(new_msg, msg_ptr->receiver_addr, RECEIVER_ADDR_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->receiver_addr, RECEIVER_ADDR_FIELD_LEN, &pos);
                break;
            case MESSAGE_TYPE:
                write ? fill_msg_bit_field(new_msg, msg_ptr->message_type, MESSAGE_TYPE_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->message_type, MESSAGE_TYPE_FIELD_LEN, &pos);
                break;
            case DIRECTION:
                write ? fill_msg_bit_field(new_msg, msg_ptr->direction, DIRECTION_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->direction, DIRECTION_FIELD_LEN, &pos);
                break;
            case BATTERY:
                write ? fill_msg_bit_field(new_msg, msg_ptr->battery_level, BATTERY_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->battery_level, BATTERY_FIELD_LEN, &pos);
                break;
            case PEOPLE_IN_SAFE_ZONE:
                write ? fill_msg_bit_field(new_msg, msg_ptr->people_in_safe_zone, PEOPLE_IN_SAFE_ZONE_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->people_in_safe_zone, PEOPLE_IN_SAFE_ZONE_FIELD_LEN, &pos);
                break;
            default:
                break;
        }
    }
}

static uint8_t check_rssi(const int16_t rssi) {
    if ( rssi >= CONNECTION_QUALITY_RSSI_1 ) {
        return LIGHT_UP_EIGHT;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_1) && (rssi >= CONNECTION_QUALITY_RSSI_2) ) {
        return LIGHT_UP_SEVEN;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_2) && (rssi >= CONNECTION_QUALITY_RSSI_3) ) {
        return LIGHT_UP_SIX;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_3) && (rssi >= CONNECTION_QUALITY_RSSI_4) ) {
        return LIGHT_UP_FIVE;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_4) && (rssi >= CONNECTION_QUALITY_RSSI_5) ) {
        return LIGHT_UP_FOUR;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_5) && (rssi >= CONNECTION_QUALITY_RSSI_6) ) {
        return LIGHT_UP_THREE;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_6) && (rssi >= CONNECTION_QUALITY_RSSI_7) ) {
        return LIGHT_UP_TWO;
    }
    else if ( (rssi < CONNECTION_QUALITY_RSSI_7) && (rssi >= CONNECTION_QUALITY_RSSI_8) ) {
        return LIGHT_UP_ONE;
    }
    else if ( rssi < CONNECTION_QUALITY_RSSI_8 ) {
        return LIGHT_UP_ZERO;
    }
}
#ifdef BASE_STATION
void control_timer_handler(struct k_timer* tim) {
    volatile int rc = 0;
    printk("timer handler");
    k_sem_give(&sem_anti_dream_msg);
//    k_timer_start
}

#else
void button_alarm_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    printk("button alarm pressed");
    tx_msg.receiver_addr = RECV_BASE_STATION;
    tx_msg.sender_addr = cur_dev_addr;
    tx_msg.message_type = MESSAGE_TYPE_ALARM;
    tx_msg.direction = REQUEST;
//    tx_msg.battery_level
    tx_msg.people_in_safe_zone = 0;
    k_work_submit(&sender);
}

void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    tx_msg.receiver_addr = RECV_BASE_STATION;
    tx_msg.sender_addr = cur_dev_addr;
    tx_msg.message_type = MESSAGE_TYPE_ANTI_DREAM;
    tx_msg.direction = RESPONSE;
//    tx_msg.battery_level =
    tx_msg.people_in_safe_zone = 0;
    k_work_submit(&sender);
}
#endif
/// Function definition area end
