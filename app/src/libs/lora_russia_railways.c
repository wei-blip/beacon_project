//
// Created by rts on 21.01.2022.
//
#include "lora_russia_railways.h"
#include <drivers/gpio.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(main);

#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

//volatile uint32_t timeout_msec = SLOT_TIME_MSEC + SLOT_TIME_MSEC*CURRENT_DEVICE_NUM;

/// My threads ids begin
extern const k_tid_t recv_task_id;
extern const k_tid_t send_task_id;
extern const k_tid_t proc_task_id;
/// My threads ids end


/// Structure area begin
struct message_s sync_msg;
struct message_s rx_msg;

struct device* lora_dev_ptr;
struct lora_modem_config lora_cfg;

// priority queue for sending messages
K_MSGQ_DEFINE(msgq_tx_msg_prio, sizeof(struct message_s), QUEUE_LEN_IN_ELEMENTS, 1);
// none priority queue for sending messages
K_MSGQ_DEFINE(msgq_tx_msg, sizeof(struct message_s), QUEUE_LEN_IN_ELEMENTS, 1);
// queue for receiving messages
K_MSGQ_DEFINE(msgq_rx_msg, MESSAGE_LEN_IN_BYTES, QUEUE_LEN_IN_ELEMENTS, 1);
// queue for rssi values
K_MSGQ_DEFINE(msgq_rssi, sizeof(int16_t), QUEUE_LEN_IN_ELEMENTS, 2);


#ifdef PERIPHERAL
// starting session timeout timer
struct k_timer session_timeout_timer;
#endif
// periodic timer (for syncro)
struct k_timer periodic_timer;


#ifdef PERIPHERAL
struct gpio_callback button_anti_dream_cb;
struct gpio_callback button_alarm_cb;
#endif

struct k_sem sem_stop_recv;
struct k_sem sem_lora_busy;
struct k_sem sem_cur_device_slot;

struct device* button_alarm_gpio_dev_ptr;
struct device* button_anti_dream_gpio_dev_ptr;
/// Structure area end


/// Enum area begin
enum receiver_addr cur_dev_addr = RECV_SIGNALMAN_1;
enum workers_ids cur_workers_in_safe_zone = FIRST_PEOPLE_ID;
enum battery_level cur_battery_level = BATTERY_LEVEL_GOOD;
/// Enum area end


/// Function declaration area begin
static void fill_msg_bit_field(uint32_t* msg_ptr, uint8_t field_val, uint8_t field_len, uint8_t* pos);
static uint8_t reverse(uint8_t input);
static void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write);
static void extract_msg_bit_field(const uint32_t* msg_ptr, uint8_t *field_val, uint8_t field_len, uint8_t* pos);
static uint8_t check_rssi(const int16_t rssi);

void periodic_timer_handler(struct k_timer* tim); // callback for timer
void session_timeout_timer_handler(struct k_timer* tim); // tx timeout callback timer

#ifdef PERIPHERAL
void button_alarm_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins);
void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins);
#endif
/// Function declaration area end


//// Function definition area begin
#ifdef BASE_STATION
void system_init() {
    struct message_s sync_msg = {0};
    /// LoRa init begin
    lora_cfg.frequency = 433000000;
    lora_cfg.bandwidth = BW_125_KHZ;
    lora_cfg.datarate = SF_10;
    lora_cfg.preamble_len = 8;
    lora_cfg.coding_rate = CR_4_5;
    lora_cfg.tx_power = 5;
    lora_cfg.tx = true;

    lora_dev_ptr = DEVICE_DT_GET(DEFAULT_RADIO_NODE);

    if (!device_is_ready(lora_dev_ptr)) {
        return;
    }
    if ( lora_config(lora_dev_ptr, &lora_cfg) < 0 ) {
        return;
    }
    /// LoRa init end

    //// Kernel services init begin
    k_timer_init(&periodic_timer, periodic_timer_handler, NULL);
//    k_timer_init(&tx_slot_timer, tx_slot_timer_handler, NULL);

//    k_sem_init(&sem_anti_dream_msg, SEM_ANTI_DREAM_INIT_VAL, SEM_ANTI_DREAM_LIM);
//    k_sem_init(&sem_proc_data, sem_proc_data_init_val, sem_proc_data_lim);
    k_sem_init(&sem_lora_busy, SEM_LORA_BUSY_INIT_VAL, SEM_LORA_BUSY_LIM);
//    k_sem_init(&sem_cur_device_slot, SEM_CUR_DEVICE_SLOT_INIT_VAL, SEM_CUR_DEVICE_SLOT_LIM);
//    k_sem_init(&sem_stop_recv, SEM_STOP_RECV_INIT_VAL, SEM_STOP_RECV_LIM);


//    k_work_init(&sender, send_msg);
    /// Kernel services init end

    /// Send sync message
    sync_msg.receiver_addr = RECV_BROADCAST;
    sync_msg.sender_addr = cur_dev_addr;
    sync_msg.message_type = MESSAGE_TYPE_SYNC;
    sync_msg.direction = REQUEST;
    sync_msg.workers_in_safe_zone = 0;
    sync_msg.battery_level = BATTERY_LEVEL_GOOD;
    k_msgq_put(&msgq_tx_msg, &sync_msg, K_NO_WAIT);

    k_timer_start(&periodic_timer, K_MSEC(4*SLOT_TIME_MSEC), K_MSEC(4*SLOT_TIME_MSEC));
//    k_timer_start(&tx_slot_timer, K_MSEC(SLOT_TIME_MSEC), K_NO_WAIT);
}
#else
void system_init() {
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
        k_sleep(K_FOREVER);
    }
    if ( lora_config(lora_dev_ptr, &lora_cfg) < 0 ) {
        k_sleep(K_FOREVER);
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
    k_sem_init(&sem_lora_busy, SEM_LORA_BUSY_INIT_VAL, SEM_LORA_BUSY_LIM);
//    k_sem_init(&sem_stop_recv, SEM_STOP_RECV_INIT_VAL, SEM_STOP_RECV_LIM);

    k_timer_init(&periodic_timer, periodic_timer_handler, NULL);
    k_timer_init(&session_timeout_timer, session_timeout_timer_handler, NULL);
    /// Kernel services init end
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


_Noreturn void send_task(void) {
    uint8_t tx_buf[MESSAGE_LEN_IN_BYTES] = {0};
    struct message_s tx_msg = {0};
    volatile int rc = 0;
    uint32_t new_msg = 0;
    while(1) {
//        if(!k_sem_count_get(&sem_stop_recv)) {  // if semaphore free this thread go to sleep, because recv_task should take this semaphore
//            k_wakeup(recv_task_id);
//            k_sleep(K_FOREVER);
//        }
        LOG_DBG("Check queues");
        if ( k_msgq_num_used_get(&msgq_tx_msg_prio) ) {
            k_msgq_get(&msgq_tx_msg_prio, &tx_msg, K_NO_WAIT);
        }
        else if ( k_msgq_num_used_get(&msgq_tx_msg) ) {
            k_msgq_get(&msgq_tx_msg, &tx_msg, K_NO_WAIT);
        }
        else {
            k_thread_resume(recv_task_id);
            k_sleep(K_FOREVER);
            continue;
        }
//        rc = k_msgq_get(&msgq_tx_msg_prio, &tx_msg, K_NO_WAIT);
        read_write_message(&new_msg, &tx_msg, true);
        for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
            tx_buf[i] = (new_msg & (0x000000FF << i*8) ) >> i*8;
            tx_buf[i] = reverse(tx_buf[i]);
        }
        LOG_DBG("Take semaphore sem_lora_busy");
        k_sem_take(&sem_lora_busy, K_FOREVER);
        if (!lora_cfg.tx) {
            lora_cfg.tx = true;
            rc = lora_config(lora_dev_ptr, &lora_cfg);
        }
        LOG_DBG("Send message");
        rc = lora_send(lora_dev_ptr, tx_buf, MESSAGE_LEN_IN_BYTES);
        LOG_DBG("Give semaphore sem_lora_busy");
        k_sem_give(&sem_lora_busy);
//        k_sem_give(&sem_stop_recv);
        k_wakeup(recv_task_id);
        k_sleep(K_FOREVER);
    }
}


_Noreturn void recv_task(void) {
#ifdef BASE_STATION
    system_init();

#else
    system_init();

#endif
    volatile int rc = 0;
    volatile uint32_t ticks = 0;
    uint8_t rx_msg[MESSAGE_LEN_IN_BYTES] = {0};
    int16_t rssi = 0;
    int8_t snr = 0;
    uint8_t ind = 0;
#ifdef PERIPHERAL
    // Receive first sync message
    LOG_DBG("Start sync");
    rc = lora_recv(lora_dev_ptr, rx_msg, MESSAGE_LEN_IN_BYTES, K_FOREVER, &rssi, &snr);
    if (rc > 0) {
        k_msgq_put(&msgq_rx_msg, &rx_msg, K_NO_WAIT);
        k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);
        k_sleep(K_FOREVER);
    }
#endif

    while(1) {
//        rc = k_sem_take(&sem_stop_recv, K_NO_WAIT);
//        if ( rc < 0 ) {
//            k_sleep(K_FOREVER);
//        }
        LOG_DBG("Take semaphore sem_lora_busy");
        k_sem_take(&sem_lora_busy, K_FOREVER);
        if (lora_cfg.tx) {
            lora_cfg.tx = false;
            lora_config(lora_dev_ptr, &lora_cfg);
        }
        ticks = k_ticks_to_ms_floor32(k_timer_remaining_ticks(&periodic_timer));
//        while( ticks < 3*SLOT_TIME_MSEC ) {
        LOG_DBG("Start receiving");
        rc = lora_recv(lora_dev_ptr, rx_msg, MESSAGE_LEN_IN_BYTES, K_MSEC(ticks), &rssi, &snr);
        if (rc > 0) {
            k_msgq_put(&msgq_rx_msg, &rx_msg, K_NO_WAIT);
            k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);
        }
//            ticks = k_ticks_to_ms_floor32(k_timer_remaining_ticks(&periodic_timer));
//        }
        LOG_DBG("Give semaphore sem_lora_busy");
        k_sem_give(&sem_lora_busy);
//        k_sleep(K_FOREVER);
//        k_sem_give(&sem_stop_recv);
    }
}


_Noreturn void proc_task() {
    uint8_t con_qual_leds_num = 0;
    uint8_t rx_buf[MESSAGE_LEN_IN_BYTES] = {0};
    int16_t rssi = 0;
    uint32_t cur_msg = 0;
    struct message_s tx_msg = {0};
    while(1) {
        if ( k_msgq_num_free_get(&msgq_rx_msg) != QUEUE_LEN_IN_ELEMENTS ) {
            k_msgq_get(&msgq_rx_msg, &rx_buf, K_NO_WAIT);
            k_msgq_get(&msgq_rssi, &rssi, K_NO_WAIT);

            //// Processing receive data
            for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
                rx_buf[i] = reverse(rx_buf[i]);
                cur_msg |= (rx_buf[i]) << i*8;
            }
            LOG_DBG("Incoming packet...");
            read_write_message(&cur_msg, &rx_msg, false); // rx_msg struct is fill
            if ( (rx_msg.receiver_addr != RECV_BROADCAST) && (rx_msg.receiver_addr != cur_dev_addr) ) {
                LOG_DBG("addr = 0x%02x, own addr = 0x%02x", rx_msg.receiver_addr, cur_dev_addr);
                LOG_DBG("Packet is filtered");
                continue;
            }
#ifdef PERIPHERAL
            LOG_DBG("Message type:");
            switch (rx_msg.message_type) {
                case MESSAGE_TYPE_SYNC:
                    LOG_DBG(" MESSAGE_TYPE_SYNC");
                    k_timer_stop(&periodic_timer);
                    k_timer_start(&session_timeout_timer, K_MSEC(DEVICE_SESSION_TIMEOUT_MSEC), K_NO_WAIT);
                    break;
                case MESSAGE_TYPE_ALARM:
                    LOG_DBG(" MESSAGE_TYPE_ALARM");
                    tx_msg.receiver_addr = rx_msg.sender_addr;
                    tx_msg.sender_addr = cur_dev_addr;
                    tx_msg.message_type = rx_msg.message_type;
                    tx_msg.direction = RESPONSE;
                    tx_msg.workers_in_safe_zone = cur_workers_in_safe_zone;
                    tx_msg.battery_level = cur_battery_level;
                    k_msgq_put(&msgq_tx_msg_prio, &tx_msg, K_NO_WAIT);
                    break;
//                case MESSAGE_TYPE_ALL_IN_SAFE_ZONE:
//                case MESSAGE_TYPE_PEOPLE_IN_SAFE_ZONE:
                case MESSAGE_TYPE_DISABLE_ALARM:
                    LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                    // Indication for signalman and brigade chief
                    switch (rx_msg.sender_addr) {
                        case SEND_BASE_STATION:
                            // Indicate LED "Base station disabled alarm"
                            LOG_DBG("Base station disabled alarm");
                            break;
                        case SEND_BRIGADE_CHIEF:
                            // Indicate LED "Brigade chief disabled alarm"
                            LOG_DBG("Brigade chief disabled alarm");
                            break;
                        default:
                            LOG_DBG("Undefined sender address for this message type");
                            break;
                    }
                case MESSAGE_TYPE_HOMEWARD:
                    LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                    // Indication for signalman and brigade chief
                case MESSAGE_TYPE_RESET_DEVICE:
                    LOG_DBG(" MESSAGE_TYPE_RESET_DEVICE");
                    // Something that reboot system
//                case MESSAGE_TYPE_ANTI_DREAM:
//                    LOG_DBG(" MESSAGE_TYPE_ANTI_DREAM");
                case MESSAGE_TYPE_TRAIN_PASSED:
                    LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                    break;
            }
#endif
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
                write ? fill_msg_bit_field(new_msg, msg_ptr->workers_in_safe_zone, PEOPLE_IN_SAFE_ZONE_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->workers_in_safe_zone, PEOPLE_IN_SAFE_ZONE_FIELD_LEN, &pos);
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


void periodic_timer_handler(struct k_timer* tim) {
    LOG_DBG("Periodic timer handler");
    LOG_DBG("Take semaphore sem_lora_busy");
    if ( !k_sem_take(&sem_lora_busy, K_FOREVER) ) {
        k_thread_suspend(recv_task_id);
    }
    LOG_DBG("Give semaphore sem_lora_busy");
    k_sem_give(&sem_lora_busy);
#ifdef BASE_STATION
    static uint8_t count = 0;
    if (count == 10) {
        k_msgq_put(&msgq_tx_msg, &sync_msg, K_NO_WAIT);
        count = 0;
    }
    count++;
#endif
    k_wakeup(send_task_id);
}

#ifdef PERIPHERAL
void session_timeout_timer_handler(struct k_timer* tim) {
    LOG_DBG("Session timer handler");
    k_wakeup(send_task_id);
    k_timer_start(&periodic_timer, K_MSEC(4*SLOT_TIME_MSEC), K_MSEC(4*SLOT_TIME_MSEC));
//    k_sem_give(&sem_stop_recv);
}

void button_alarm_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    LOG_DBG("Button alarm pressed");
    struct message_s alarm_msg = {0};
    alarm_msg.receiver_addr = RECV_BASE_STATION;
    alarm_msg.sender_addr = cur_dev_addr;
    alarm_msg.message_type = MESSAGE_TYPE_ALARM;
    alarm_msg.direction = REQUEST;
//    tx_msg.battery_level
    alarm_msg.workers_in_safe_zone = 0;
    k_msgq_put(&msgq_tx_msg_prio, &alarm_msg, K_NO_WAIT);
}

void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
    struct message_s anti_dream_msg = {0};
    anti_dream_msg.receiver_addr = RECV_BASE_STATION;
    anti_dream_msg.sender_addr = cur_dev_addr;
    anti_dream_msg.message_type = MESSAGE_TYPE_ANTI_DREAM;
    anti_dream_msg.direction = RESPONSE;
//    tx_msg.battery_level =
    anti_dream_msg.workers_in_safe_zone = 0;
    k_msgq_put(&msgq_tx_msg, &anti_dream_msg, K_NO_WAIT);
}
#endif
/// Function definition area end
