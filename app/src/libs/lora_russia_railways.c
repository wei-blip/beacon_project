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

uint8_t tx_buf[MESSAGE_LEN_IN_BYTES] = {0};
uint8_t rx_buf[MESSAGE_LEN_IN_BYTES] = {0};

/// My threads ids begin
extern const k_tid_t proc_task_id;
extern const k_tid_t modem_task_id;
extern const k_tid_t start_system_id;
/// My threads ids end

static modem_state_t start_state;
static modem_state_t transmit_state;
static modem_state_t receive_1_state;
static modem_state_t receive_2_state;
static modem_state_t receive_3_state;
static modem_state_t current_state;

/// Structure area begin
struct message_s sync_msg;

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
// queue for function pointer
K_MSGQ_DEFINE(msgq_fun_pointer, sizeof( void (*)(void) ), QUEUE_LEN_IN_ELEMENTS, 4);


// periodic timer (for syncro)
struct k_timer periodic_timer;
//struct k_timer calibration_timer;


#ifdef PERIPHERAL
struct gpio_callback button_anti_dream_cb;
struct gpio_callback button_alarm_cb;
#endif

struct k_sem sem_lora_busy;

struct device* button_alarm_gpio_dev_ptr;
struct device* button_anti_dream_gpio_dev_ptr;

struct message_s tx_msg = {0};
/// Structure area end


/// Enum area begin
#ifdef BASE_STATION
enum receiver_addr cur_dev_addr = RECV_BASE_STATION;
#else
enum receiver_addr cur_dev_addr = RECV_SIGNALMAN_1;
#endif

enum workers_ids cur_workers_in_safe_zone = FIRST_PEOPLE_ID;
enum battery_level cur_battery_level = BATTERY_LEVEL_GOOD;
/// Enum area end


/// Function declaration area begin
static void fill_msg_bit_field(uint32_t* msg_ptr, uint8_t field_val, uint8_t field_len, uint8_t* pos);
static uint8_t reverse(uint8_t input);
static void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write);
static void extract_msg_bit_field(const uint32_t* msg_ptr, uint8_t *field_val, uint8_t field_len, uint8_t* pos);
static uint8_t check_rssi(const int16_t rssi);
static void start_sync(void);

void periodic_timer_handler(struct k_timer* tim); // callback for periodic_timer
void calibration_timer_handler(struct k_timer* tim); // callback for calibration_timer

#ifdef PERIPHERAL
void button_alarm_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins);
void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins);
#endif
/// Function declaration area end


//// Function definition area begin
#ifdef BASE_STATION
void system_init() {
    struct message_s sync_msg = {0};
    //// Kernel services init begin
    k_timer_init(&periodic_timer, periodic_timer_handler, NULL);
    k_timer_init(&calibration_timer, calibration_timer_handler, NULL);

    k_sem_init(&sem_lora_busy, SEM_LORA_BUSY_INIT_VAL, SEM_LORA_BUSY_LIM);
    /// Kernel services init end

    /// LoRa init begin
    lora_cfg.frequency = 433000000;
    lora_cfg.bandwidth = BW_125_KHZ;
    lora_cfg.datarate = SF_12;
    lora_cfg.preamble_len = 8;
    lora_cfg.coding_rate = CR_4_5;
    lora_cfg.tx_power = 0;
    lora_cfg.tx = true;

    lora_dev_ptr = DEVICE_DT_GET(DEFAULT_RADIO_NODE);

    if (!device_is_ready(lora_dev_ptr)) {
        return;
    }

    if ( lora_config(lora_dev_ptr, &lora_cfg) < 0 ) {
        return;
    }
    /// LoRa init end
}
#else
void system_init() {
    /// LoRa init begin
    lora_cfg.frequency = 433000000;
    lora_cfg.bandwidth = BW_125_KHZ;
    lora_cfg.datarate = SF_12;
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

    k_timer_init(&periodic_timer, periodic_timer_handler, NULL);
//    k_timer_init(&calibration_timer, calibration_timer_handler, NULL);
    /// Kernel services init end
}
#endif


static void fill_msg_bit_field(uint32_t* msg_ptr, uint8_t field_val, uint8_t field_len, uint8_t* pos) {
    uint8_t start_pos = *pos;
    while ( *pos < start_pos + field_len ) {
        *msg_ptr &= ( ~BIT(*pos) ); // clear bit
        *msg_ptr |= ( field_val & BIT((*pos) - start_pos) ) << start_pos;
        (*pos)++;
    }
}


static void extract_msg_bit_field(const uint32_t* msg_ptr, uint8_t* field_val, uint8_t field_len, uint8_t* pos) {
    uint8_t start_pos = *pos;
    while ( *pos < start_pos + field_len ) {
        *field_val &= ( ~BIT((*pos) - start_pos) ); // clear bit
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

#ifdef BASE_STATION
void start_system(void) {
    volatile int rc = -1;
    uint32_t new_msg = 0;

    system_init();
    k_sleep(K_SECONDS(2));

    /// Filling list elements begin
    start_state.next = &transmit_state;
    start_state.trancieve_fun = NULL;

    receive_1_state.next = &receive_2_state;
    receive_1_state.trancieve_fun = recv_msg;

    receive_2_state.next = &receive_3_state;
    receive_2_state.trancieve_fun = recv_msg;

    receive_3_state.next = &transmit_state;
    receive_3_state.trancieve_fun = recv_msg;

    transmit_state.next = &receive_1_state;
    transmit_state.trancieve_fun = send_msg;
    /// Filling list elements end

    /// Send sync message
    sync_msg.receiver_addr = RECV_BROADCAST;
    sync_msg.sender_addr = cur_dev_addr;
    sync_msg.message_type = MESSAGE_TYPE_SYNC;
    sync_msg.direction = REQUEST;
    sync_msg.workers_in_safe_zone = 0;
    sync_msg.battery_level = BATTERY_LEVEL_GOOD;
    k_timer_start(&periodic_timer, K_MSEC(10000), K_NO_WAIT);
    read_write_message(&new_msg, &sync_msg, true);
    for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
        tx_buf[i] = (new_msg & (0x000000FF << i*8) ) >> i*8;
        tx_buf[i] = reverse(tx_buf[i]);
    }
//    volatile uint64_t ticks = k_ticks_to_ms_floor64(k_timer_remaining_ticks(&periodic_timer));
    rc = lora_send(lora_dev_ptr, tx_buf, MESSAGE_LEN_IN_BYTES);
//    volatile uint64_t ticks_1 = k_ticks_to_ms_floor64(k_timer_remaining_ticks(&periodic_timer));
    current_state = receive_1_state;
    k_sleep(K_MSEC(150));
    k_timer_start(&periodic_timer, K_MSEC(PERIODIC_TIMER_DURATION_MSEC),
                  K_MSEC(PERIODIC_TIMER_DURATION_MSEC));
}
#else
void start_system(void) {
    volatile int rc = -1;
    uint32_t new_msg = 0;
    int16_t rssi = 0;
    int8_t snr = 0;

    system_init();

    /// Filling list elements begin
    start_state.next = &receive_1_state;
    start_state.trancieve_fun = NULL;

    receive_1_state.next = &receive_2_state;
    receive_1_state.trancieve_fun = recv_msg;

    receive_2_state.next = &receive_3_state;
    receive_2_state.trancieve_fun = recv_msg;

    receive_3_state.next = &transmit_state;
    receive_3_state.trancieve_fun = recv_msg;

    transmit_state.next = &receive_1_state;
    transmit_state.trancieve_fun = send_msg;

    current_state = start_state;
    /// Filling list elements end

    /// Send sync message
//    k_timer_start(&periodic_timer, K_MSEC(PERIODIC_TIMER_DURATION_MSEC), K_MSEC(PERIODIC_TIMER_DURATION_MSEC));
    rc = lora_recv(lora_dev_ptr, rx_buf, MESSAGE_LEN_IN_BYTES, K_FOREVER, &rssi, &snr);
//    if ( (rc > 0) ) {
//        k_msgq_put(&msgq_rx_msg, &rx_buf, K_NO_WAIT);
//        k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);
//        k_wakeup(proc_task_id);
//    }
    current_state = transmit_state;
    k_sleep(K_MSEC(150));
    k_timer_start(&periodic_timer, K_MSEC(PERIODIC_TIMER_DURATION_MSEC),
                  K_MSEC(PERIODIC_TIMER_DURATION_MSEC));
}
#endif

void send_msg(void) {
    volatile int rc = 0;
    uint32_t new_msg = 0;
    struct k_msgq* cur_queue = NULL;
//    LOG_DBG("Check queues");

    if ( k_msgq_num_used_get(&msgq_tx_msg_prio) ) {
//        LOG_DBG("Get message from priority queue");
        rc = k_msgq_get(&msgq_tx_msg_prio, &tx_msg, K_NO_WAIT);
        cur_queue = &msgq_tx_msg_prio;
    }
    else if ( k_msgq_num_used_get(&msgq_tx_msg) ) {
//        LOG_DBG("Get message from standart queue");
        rc = k_msgq_get(&msgq_tx_msg, &tx_msg, K_NO_WAIT);
        cur_queue = &msgq_tx_msg;
    }
    else {
        return;
    }

    read_write_message(&new_msg, &tx_msg, true);
    for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
        tx_buf[i] = (new_msg & (0x000000FF << i*8) ) >> i*8;
        tx_buf[i] = reverse(tx_buf[i]);
    }

    if (!lora_cfg.tx) {
        lora_cfg.tx = true;
        rc = lora_config(lora_dev_ptr, &lora_cfg);
        if (rc < 0) {
            LOG_DBG("Modem not configure!!!");
            k_msgq_put(cur_queue, &tx_msg, K_NO_WAIT);
            return;
        }
    }
//    LOG_DBG("Send message");
    rc = lora_send(lora_dev_ptr, tx_buf, MESSAGE_LEN_IN_BYTES);
//    LOG_DBG("Message sending");
}


void recv_msg(void) {
    volatile int rc = -1;
    volatile uint32_t ticks = 0;
//    uint8_t rx_msg[MESSAGE_LEN_IN_BYTES] = {0};
    int16_t rssi = 0;
    int8_t snr = 0;

    if (lora_cfg.tx) {
        lora_cfg.tx = false;
        rc = lora_config(lora_dev_ptr, &lora_cfg);
        if (rc < 0) {
            return;
        }
    }

//    LOG_DBG("Start receiving");
    rc = lora_recv(lora_dev_ptr, rx_buf, MESSAGE_LEN_IN_BYTES,K_MSEC(RECV_TIME_MSEC), &rssi, &snr);
    if (rc > 0) {
        k_msgq_put(&msgq_rx_msg, &rx_buf, K_NO_WAIT);
        k_msgq_put(&msgq_rssi, &rssi, K_NO_WAIT);
        k_wakeup(proc_task_id);
    }
}


_Noreturn void modem_task() {
    void (*f_ptr) (void) = NULL;
    volatile int rc = 0;
    while(1) {
        if (k_msgq_num_used_get(&msgq_fun_pointer)) {
            rc = k_msgq_get(&msgq_fun_pointer, &f_ptr, K_NO_WAIT);
            if (rc < 0) {
                continue;
            }
            f_ptr();
        }
        k_sleep(K_USEC(100));
    }
}

_Noreturn void proc_task() {
    uint8_t con_qual_leds_num = 0;
    int16_t rssi = 0;
    uint32_t cur_msg = 0;
    struct message_s tx_msg_proc = {0};
    struct message_s rx_msg_proc = {0};
    struct k_msgq* msgq_cur_msg_tx_ptr = &msgq_tx_msg; // Default queue
    uint8_t rx_buf_proc[MESSAGE_LEN_IN_BYTES];
    while(1) {
        if ( k_msgq_num_used_get(&msgq_rx_msg) ) {
            k_msgq_get(&msgq_rx_msg, &rx_buf_proc, K_NO_WAIT);
            k_msgq_get(&msgq_rssi, &rssi, K_NO_WAIT);
            if (rx_buf_proc[0] == rx_buf_proc[1] == rx_buf_proc[2] == 0) {
                LOG_DBG("Empty message");
                continue;
            }

            /// Processing receive data
            cur_msg = 0;
            for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
                rx_buf_proc[i] = reverse(rx_buf_proc[i]);
                cur_msg |= (rx_buf_proc[i]) << i*8;
            }
//            LOG_DBG("Incoming packet...");
            read_write_message(&cur_msg, &rx_msg_proc, false); // rx_msg struct is fill
            if ( (rx_msg_proc.receiver_addr != RECV_BROADCAST) && (rx_msg_proc.receiver_addr != cur_dev_addr) ) {
                LOG_DBG("addr = 0x%02x, own addr = 0x%02x", rx_msg_proc.receiver_addr, cur_dev_addr);
                LOG_DBG("Packet is filtered");
                continue;
            }

//            LOG_DBG("Message direction");
#ifdef BASE_STATION

            switch (rx_msg_proc.direction) {
                case REQUEST:
                    LOG_DBG(" REQUEST");
                    LOG_DBG("Message type:");

                    tx_msg_proc.sender_addr = rx_msg_proc.sender_addr;
                    tx_msg_proc.message_type = rx_msg_proc.message_type;
                    tx_msg_proc.workers_in_safe_zone = 0;
                    tx_msg_proc.battery_level = BATTERY_LEVEL_GOOD; // change it after
                    tx_msg_proc.receiver_addr = RECV_BROADCAST;
                    tx_msg_proc.direction = RESPONSE;

                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            tx_msg_proc.direction = REQUEST;
                            switch (rx_msg_proc.sender_addr) {
                                case SEND_BRIGADE_CHIEF:
                                    LOG_DBG("Brigade chief disabled alarm");
                                    break;
                                default:
                                    LOG_DBG("Undefined sender address for this message type");
                                    break;
                            }
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            break;
                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            // TODO: On signalization, calculate workers_safe_zone
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg_prio;
                            break;
                        case MESSAGE_TYPE_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            break;
                            // TODO: Off signalization
                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        default:
                            LOG_DBG("Not correct message type");
                            msgq_cur_msg_tx_ptr = NULL;
                    }
                    break;

                case RESPONSE:
                    LOG_DBG(" RESPONSE");
                    LOG_DBG("Message type:");
                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        case MESSAGE_TYPE_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;
                        default:
                            LOG_DBG("Not correct message type");
                            msgq_cur_msg_tx_ptr = NULL;
                    }
                    break;

                default:
                    LOG_DBG("Not correct message direction");
                    msgq_cur_msg_tx_ptr = NULL;
            }

#endif

#ifdef PERIPHERAL

            switch (rx_msg_proc.direction) {
                case REQUEST:
                    LOG_DBG(" REQUEST");
//                    LOG_DBG("Message type:");

                    tx_msg_proc.sender_addr = cur_dev_addr;
                    tx_msg_proc.message_type = rx_msg_proc.message_type;
                    tx_msg_proc.workers_in_safe_zone = 0;
                    tx_msg_proc.direction = RESPONSE;
                    tx_msg_proc.battery_level = BATTERY_LEVEL_GOOD; // change it after
                    tx_msg_proc.receiver_addr = RECV_BASE_STATION;

                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_SYNC:
                            LOG_DBG(" MESSAGE_TYPE_SYNC");
                            k_timer_stop(&periodic_timer);
                            // little delay to account execution time
                            k_sleep(K_MSEC(100));
                            k_timer_start(&periodic_timer, K_MSEC(PERIODIC_TIMER_DURATION_MSEC),
                                           K_MSEC(PERIODIC_TIMER_DURATION_MSEC));
                            current_state = transmit_state;
                            k_msgq_purge(&msgq_fun_pointer);
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            // TODO Indication for signalman and brigade chief
                            switch (rx_msg_proc.sender_addr) {
                                case SEND_BASE_STATION:
                                    // TODO Indicate LED "Base station disabled alarm"
                                    LOG_DBG("Base station disabled alarm");
                                    break;
                                case SEND_BRIGADE_CHIEF:
                                    // TODO Indicate LED "Brigade chief disabled alarm"
                                    LOG_DBG("Brigade chief disabled alarm");
                                    break;
                                default:
                                    LOG_DBG("Undefined sender address for this message type");
                                    break;
                            }
                            break;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = &msgq_tx_msg;
                            /// TODO Indication for signalman and brigade chief
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            if (rx_msg_proc.sender_addr == cur_dev_addr)
                                msgq_cur_msg_tx_ptr = &msgq_tx_msg_prio; // For response message
                            else
                                msgq_cur_msg_tx_ptr = NULL; // Do nothing, because this message for base station
                            break;

                        case MESSAGE_TYPE_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = NULL; // Do nothing, because this message for base station
                            break;

                        default:
                            LOG_DBG("Not correct message type");
                            msgq_cur_msg_tx_ptr = NULL;
                    }
                    break;

                case RESPONSE:
                    LOG_DBG(" RESPONSE");
                    LOG_DBG("Message type:");
                    switch (rx_msg_proc.message_type) {
                        case MESSAGE_TYPE_DISABLE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_DISABLE_ALARM");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        case MESSAGE_TYPE_HOMEWARD:
                            LOG_DBG(" MESSAGE_TYPE_HOMEWARD");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        case MESSAGE_TYPE_ALARM:
                            LOG_DBG(" MESSAGE_TYPE_ALARM");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        case MESSAGE_TYPE_TRAIN_PASSED:
                            LOG_DBG(" MESSAGE_TYPE_TRAIN_PASSED");
                            msgq_cur_msg_tx_ptr = NULL;
                            break;

                        default:
                            LOG_DBG("Not correct message type");
                            msgq_cur_msg_tx_ptr = NULL;
                    }
                    break;

                default:
                    LOG_DBG("Not correct message direction");
                    msgq_cur_msg_tx_ptr = NULL;
            }
#endif
            if (msgq_cur_msg_tx_ptr) {
                k_msgq_put(msgq_cur_msg_tx_ptr, &tx_msg_proc, K_NO_WAIT);
            }
            con_qual_leds_num = check_rssi(&rssi);
            // ligth up leds
        }
        k_sleep(K_USEC(100));
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
//    LOG_DBG("Periodic timer handler");
    current_state = (*current_state.next);
#ifdef BASE_STATION
    static uint8_t count = 1;
    if (count == 36) {      // should divide on 4
        sync_msg.receiver_addr = RECV_BROADCAST;
        sync_msg.sender_addr = cur_dev_addr;
        sync_msg.message_type = MESSAGE_TYPE_SYNC;
        sync_msg.direction = REQUEST;
        sync_msg.workers_in_safe_zone = 0;
        sync_msg.battery_level = BATTERY_LEVEL_GOOD;
        k_msgq_put(&msgq_tx_msg, &sync_msg, K_NO_WAIT);
        count = 0;
    }
    count++;
#endif
    k_msgq_put(&msgq_fun_pointer,&(current_state.trancieve_fun), K_NO_WAIT);
    k_wakeup(modem_task_id);
}

//void calibration_timer_handler(struct k_timer* tim) {
////    LOG_DBG("Start timer handler");
//    current_state = (*current_state.next);
//    k_timer_start(&periodic_timer, K_MSEC(PERIODIC_TIMER_DURATION_MSEC),
//                  K_MSEC(PERIODIC_TIMER_DURATION_MSEC));
//    k_msgq_put(&msgq_fun_pointer,&(current_state.trancieve_fun), K_NO_WAIT);
//    k_wakeup(modem_task_id);
//}


#ifdef PERIPHERAL
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


//void button_anti_dream_pressed_cb(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
//    struct message_s anti_dream_msg = {0};
//    anti_dream_msg.receiver_addr = RECV_BASE_STATION;
//    anti_dream_msg.sender_addr = cur_dev_addr;
//    anti_dream_msg.message_type = MESSAGE_TYPE_ANTI_DREAM;
//    anti_dream_msg.direction = RESPONSE;
////    tx_msg.battery_level =
//    anti_dream_msg.workers_in_safe_zone = 0;
//    k_msgq_put(&msgq_tx_msg, &anti_dream_msg, K_NO_WAIT);
//}
#endif
/// Function definition area end
