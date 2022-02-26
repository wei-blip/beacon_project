//
// Created by rts on 05.02.2022.
//
#include "lora_russia_railways_common.h"


/// Extern variable definition and initialisation begin
// priority queue for sending messages
K_MSGQ_DEFINE(msgq_tx_msg_prio, sizeof(struct message_s), QUEUE_LEN_IN_ELEMENTS, 1);
// none priority queue for sending messages
K_MSGQ_DEFINE(msgq_tx_msg, sizeof(struct message_s), QUEUE_LEN_IN_ELEMENTS, 1);
// queue for receiving messages
K_MSGQ_DEFINE(msgq_rx_msg, MESSAGE_LEN_IN_BYTES, QUEUE_LEN_IN_ELEMENTS, 1);
// queue for rssi values
K_MSGQ_DEFINE(msgq_rssi, sizeof(int16_t), QUEUE_LEN_IN_ELEMENTS, 2);

const struct device *lora_dev_ptr = {0};
const struct device *buzzer_dev_ptr = {0};

struct lora_modem_config lora_cfg = {0};

struct k_timer periodic_timer = {0};

struct k_work work_buzzer = {0};
struct k_work work_msg_mngr = {0};
struct k_work work_led_strip_blink = {0};

struct k_mutex mut_msg_info = {0};
struct k_mutex mut_buzzer_mode = {0};

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
/// Extern variable definition and initialisation end


void fill_msg_bit_field(uint32_t *msg_ptr, const uint8_t field_val, uint8_t field_len, uint8_t *pos) {
    uint8_t start_pos = *pos;
    while ( *pos < start_pos + field_len ) {
        *msg_ptr &= ( ~BIT(*pos) ); // clear bit
        *msg_ptr |= ( field_val & BIT((*pos) - start_pos) ) << start_pos;
        (*pos)++;
    }
}


void extract_msg_bit_field(const uint32_t *msg_ptr, uint8_t *field_val, uint8_t field_len, uint8_t *pos)
{
    uint8_t start_pos = *pos;
    while ( *pos < start_pos + field_len ) {
        *field_val &= ( ~BIT((*pos) - start_pos) ); // clear bit
        (*field_val) |= ( (*msg_ptr) & BIT((*pos) ) ) >> start_pos;
        (*pos)++;
    }
}


uint8_t reverse(uint8_t input)
{
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


void read_write_message(uint32_t *new_msg, struct message_s *msg_ptr, bool write)
{
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


uint8_t check_rssi(const int16_t rssi)
{
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


void check_msg_status(struct msg_info_s *msg_info)
{
    // Check response status
    // If response will receive - set flags on false (good transfer)
    // Else if check count status, if cnt == WAITING_PERIOD_NUM - message retransmit (bad transfer)
//    if (atomic_get(&(msg_info->req_is_send))) {
//        if (atomic_get(&(msg_info->resp_is_recv))) {
//            atomic_clear(&(msg_info->req_is_send));
//            atomic_clear(&(msg_info->resp_is_recv));
//            printk("        Message transmit!!!\n");
//        } else {
//            printk("        Message retransmit!!!\n");
//            k_msgq_put(msg_info->msg_buf, msg_info->msg, K_NO_WAIT);
//        }
//    }
    if (atomic_get((&msg_info->req_is_send))) {
      k_msgq_put(msg_info->msg_buf, msg_info->msg, K_NO_WAIT);
      atomic_clear(&(msg_info->req_is_send));
    }
}