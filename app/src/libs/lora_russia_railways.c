//
// Created by rts on 21.01.2022.
//
#include "lora_russia_railways.h"
#include <zephyr.h>

#define SEM_ANTI_DREAM_INIT_VAL  0
#define SEM_ANTI_DREAM_LIM 1
#define SEM_PROC_DATA_INIT_VAL 0
#define SEM_PROC_DATA_LIM 1

//// Array area begin ////
extern uint8_t tx_buf[MESSAGE_LEN_IN_BYTES];
extern uint8_t rx_buf[MESSAGE_LEN_IN_BYTES];
//// Array area

//// Structure area begin ////
extern struct message_s rx_msg;
extern struct message_s tx_msg;

struct k_timer* control_timer_ptr;
struct k_sem sem_anti_dream_msg;
struct k_sem sem_proc_data;
struct k_queue queue_msg;
//// Structure area end ////

//// Enum area begin ////
enum receiver_addr cur_dev_addr = RECV_SIGNALMAN_1;
enum people_in_safe_zone_ids people_in_safe_zone = FIRST_PEOPLE_ID;

enum CONNECTION_QUALITY_RSSI {
    CONNECTION_QUALITY_RSSI_1 = -50,
    CONNECTION_QUALITY_RSSI_2 = -60,
    CONNECTION_QUALITY_RSSI_3 = -70,
    CONNECTION_QUALITY_RSSI_4 = -80,
    CONNECTION_QUALITY_RSSI_5 = -90,
    CONNECTION_QUALITY_RSSI_6 = -100,
    CONNECTION_QUALITY_RSSI_7 = -110,
    CONNECTION_QUALITY_RSSI_8 = -120
};

enum LIGHT_UP_LEDS {
    LIGHT_UP_ZERO = 0,
    LIGHT_UP_ONE = 1,
    LIGHT_UP_TWO = 2,
    LIGHT_UP_THREE = 3,
    LIGHT_UP_FOUR = 4,
    LIGHT_UP_FIVE = 5,
    LIGHT_UP_SIX = 6,
    LIGHT_UP_SEVEN = 7,
    LIGHT_UP_EIGHT = 8
};
//// Enum area end ////

//// Function declaration area begin ////
static void fill_msg_bit_field(uint32_t* msg_ptr, uint8_t field_val, uint8_t field_len, uint8_t* pos);
static uint8_t reverse(uint8_t input);
static void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write);
static void extract_msg_bit_field(uint32_t* msg_ptr, uint8_t *field_val, uint8_t field_len, uint8_t* pos);
static uint8_t check_rssi(const int16_t rssi);

void control_timer_handler(struct k_timer* tim); // callback for timer
//// Function declaration area end ////

//// Fuction definition area begin ////
int system_init(uint8_t tim_duration_min, unsigned int sem_anti_dream_init_val, unsigned int sem_anti_dream_lim,
                unsigned int sem_proc_data_init_val, unsigned int sem_proc_data_lim) {
    k_timer_init(control_timer_ptr, control_timer_handler, NULL);
    k_timer_start(control_timer_ptr, K_SECONDS(tim_duration_min*60), K_FOREVER);
    k_sem_init(&sem_anti_dream_msg, sem_anti_dream_init_val, sem_anti_dream_lim);
    k_sem_init(&sem_proc_data, sem_proc_data_init_val, sem_proc_data_lim);
    k_queue_init(&queue_msg);
    if (!device_is_ready(lora_dev_ptr)) {
        return -1;
    }
}

static void fill_msg_bit_field(uint32_t* msg_ptr, uint8_t field_val, uint8_t field_len, uint8_t* pos) {
    uint8_t start_pos = *pos;
    while ( *pos < start_pos + field_len ) {
        *msg_ptr |= ( field_val & BIT((*pos) - start_pos) ) << start_pos;
        (*pos)++;
    }
}

static void extract_msg_bit_field(uint32_t* msg_ptr, uint8_t* field_val, uint8_t field_len, uint8_t* pos) {
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

int send_msg(struct message_s* msg_ptr) {
    int rc;
    uint32_t new_msg = 0;
    read_write_message(&new_msg, msg_ptr, true);
    for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
        tx_buf[i] = (new_msg & (0x000000FF << i*8) ) >> i*8;
        tx_buf[i] = reverse(tx_buf[i]);
    }
//    rc = lora_send(lora_dev, tx_buf, MESSAGE_LEN_IN_BYTES);
    return rc;
}

void recv_msg(void) {
//    lora_recv(lora_dev, data, MAX_DATA_LEN, K_FOREVER,
//              &rssi, &snr);
    k_queue_
    if (k_sem_take(&sem_anti_dream_msg, K_NO_WAIT)) {
        //// Create message for signalman 1
        tx_msg.receiver_addr = RECV_SIGNALMAN_1;
        tx_msg.sender_addr = cur_dev_addr;
    //    tx_msg.battery_level = ?;
        tx_msg.people_in_safe_zone = people_in_safe_zone;
        tx_msg.direction = REQUEST;
        tx_msg.message_type = MESSAGE_TYPE_ANTI_DREAM;
        send_msg(&tx_msg);  // it may need to be done several times
        //// Create message for signalman 2
        tx_msg.receiver_addr = RECV_SIGNALMAN_2;
        tx_msg.sender_addr = cur_dev_addr;
        //    tx_msg.battery_level = ?;
        tx_msg.people_in_safe_zone = people_in_safe_zone;
        tx_msg.direction = REQUEST;
        tx_msg.message_type = MESSAGE_TYPE_ANTI_DREAM;
        send_msg(&tx_msg); // it may need to be done several times
    }
}

void processing_data() {
    int8_t snr = 0;
    uint8_t con_qual_leds_num = 0;
    int16_t rssi = 0;
    uint32_t cur_msg = 0;
    for (uint8_t i = 0; i < MESSAGE_LEN_IN_BYTES; ++i) {
        rx_buf[i] = reverse(rx_buf[i]);
        cur_msg |= (rx_buf[i]) << i*8;
    }
    read_write_message(&cur_msg, &rx_msg, false); // rx_msg struct is fill
    if ( rx_msg.receiver_addr != cur_dev_addr )
        return;
    con_qual_leds_num = check_rssi(rssi);
    while(1) {
        if (k_queue_is_empty(&queue_msg) ) {
            //// Processing receive data
        }
    }
}

static void read_write_message(uint32_t* new_msg, struct message_s* msg_ptr, bool write) {
    uint8_t pos = 0;
    for (int cur_field = 0; cur_field < MESSAGE_FIELD_NUMBER; ++cur_field) {
        switch (cur_field) {
            case SENDER_ADDR:
                write ? fill_msg_bit_field(new_msg, msg_ptr->sender_addr, SENDER_ADDR_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->sender_addr, SENDER_ADDR_FIELD_LEN, &pos);
//                if (write) {
//                    fill_msg_bit_field(new_msg, msg_ptr->sender_addr, SENDER_ADDR_FIELD_LEN, &pos);
//                }
//                else {
//                    extract_msg_bit_field(new_msg, &msg_ptr->sender_addr, SENDER_ADDR_FIELD_LEN, &pos);
//                }
                break;
            case RECEIVER_ADDR:
                write ? fill_msg_bit_field(new_msg, msg_ptr->receiver_addr, RECEIVER_ADDR_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->receiver_addr, RECEIVER_ADDR_FIELD_LEN, &pos);
//                if (write) {
//                    fill_msg_bit_field(new_msg, msg_ptr->receiver_addr, RECEIVER_ADDR_FIELD_LEN, &pos);
//                }
//                else {
//                    extract_msg_bit_field(new_msg, &msg_ptr->receiver_addr, RECEIVER_ADDR_FIELD_LEN, &pos);
//                }
                break;
            case MESSAGE_TYPE:
                write ? fill_msg_bit_field(new_msg, msg_ptr->message_type, MESSAGE_TYPE_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->message_type, MESSAGE_TYPE_FIELD_LEN, &pos);
//                if (write) {
//                    fill_msg_bit_field(new_msg, msg_ptr->message_type, MESSAGE_TYPE_FIELD_LEN, &pos);
//                }
//                else {
//                    extract_msg_bit_field(new_msg, &msg_ptr->message_type, MESSAGE_TYPE_FIELD_LEN, &pos);
//                }
                break;
            case DIRECTION:
                write ? fill_msg_bit_field(new_msg, msg_ptr->direction, DIRECTION_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->direction, DIRECTION_FIELD_LEN, &pos);
//                if (write) {
//                    fill_msg_bit_field(new_msg, msg_ptr->direction, DIRECTION_FIELD_LEN, &pos);
//                }
//                else {
//                    extract_msg_bit_field(new_msg, &msg_ptr->direction, DIRECTION_FIELD_LEN, &pos);
//                }
                break;
            case BATTERY:
                write ? fill_msg_bit_field(new_msg, msg_ptr->battery_level, BATTERY_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->battery_level, BATTERY_FIELD_LEN, &pos);
//                if (write) {
//                    fill_msg_bit_field(new_msg, msg_ptr->battery_level, BATTERY_FIELD_LEN, &pos);
//                }
//                else {
//                    extract_msg_bit_field(new_msg, &msg_ptr->battery_level, BATTERY_FIELD_LEN, &pos);
//                }
                break;
            case PEOPLE_IN_SAFE_ZONE:
                write ? fill_msg_bit_field(new_msg, msg_ptr->people_in_safe_zone, PEOPLE_IN_SAFE_ZONE_FIELD_LEN, &pos) :
                    extract_msg_bit_field(new_msg, &msg_ptr->people_in_safe_zone, PEOPLE_IN_SAFE_ZONE_FIELD_LEN, &pos);
//                if (write) {
//                    fill_msg_bit_field(new_msg, msg_ptr->people_in_safe_zone, PEOPLE_IN_SAFE_ZONE_FIELD_LEN, &pos);
//                }
//                else {
//                    extract_msg_bit_field(new_msg, &msg_ptr->people_in_safe_zone, PEOPLE_IN_SAFE_ZONE_FIELD_LEN, &pos);
//                }
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

void control_timer_handler(struct k_timer* tim) {
    k_timer_stop(control_timer_ptr); // if it long-time function
    k_sem_give(&sem_anti_dream_msg);
}
//// Fuction definition area end ////