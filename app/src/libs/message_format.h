//
// Created by rts on 20.01.2022.
//
#include <zephyr.h>
#include <zephyr/types.h>

#ifndef RADIO_SIGNALMAN_MESSAGE_FORMAT_H
#define RADIO_SIGNALMAN_MESSAGE_FORMAT_H

#define MESSAGE_LENGTH 24
#define MESSAGE_FIELD_NUMBER 6
#define MESSAGE_LEN_IN_BYTES 3

struct message_s {
    uint8_t sender_addr;
    uint8_t receiver_addr;
    uint8_t message_type;
    uint8_t direction;
    uint8_t battery_level;
    uint8_t workers_in_safe_zone;
};

enum MESSAGE_FIELD_POSITIONS_e {
    SENDER_ADDR = 0,
    RECEIVER_ADDR = 1,
    MESSAGE_TYPE = 2,
    DIRECTION = 3,
    BATTERY = 4,
    PEOPLE_IN_SAFE_ZONE = 5
};

#define SENDER_ADDR_FIELD_LEN 2
#define RECEIVER_ADDR_FIELD_LEN 3
#define MESSAGE_TYPE_FIELD_LEN 4
#define DIRECTION_FIELD_LEN 1
#define BATTERY_FIELD_LEN 1
#define PEOPLE_IN_SAFE_ZONE_FIELD_LEN 8

//#define SENDER_ADDR_START_POS 0
//#define RECEIVER_ADDR_START_POS 2
//#define MESSAGE_TYPE_START_POS 5
//#define DIRECTION_START_POS 9
//#define BATTERY_START_POS 10
//#define PEOPLE_IN_SAFE_ZONE_START_POS 11

enum DEVICE_ADDR_e {
    BASE_STATION_ADDR = 0x00,
    SIGNALMAN_1_ADDR = 0x01,
    SIGNALMAN_2_ADDR = 0x02,
    BRIGADE_CHIEF_ADDR = 0x03,
    BROADCAST_ADDR = 0x04
};

enum sender_addr {
    SEND_SIGNALMAN_1 = 0x01,
    SEND_SIGNALMAN_2 = 0x02,
    SEND_BASE_STATION = 0x03,
    SEND_BRIGADE_CHIEF = 0x04
};

enum receiver_addr {
    RECV_BROADCAST = 0x00,
    RECV_SIGNALMAN_1 = 0x01,
    RECV_SIGNALMAN_2 = 0x02,
    RECV_BASE_STATION = 0x03,
    RECV_BRIGADE_CHIEF = 0x04,
    RECV_ONLY_SIGNALMANS = 0x05
};

enum message_type {
    MESSAGE_TYPE_ALARM = 0x00,
    MESSAGE_TYPE_DISABLE_ALARM = 0x01,
//    MESSAGE_TYPE_ANTI_DREAM = 0x02,
    MESSAGE_TYPE_HOMEWARD = 0x02,
//    MESSAGE_TYPE_ALL_IN_SAFE_ZONE = 0x04,
    MESSAGE_TYPE_TRAIN_PASSED = 0x03,
//    MESSAGE_TYPE_PEOPLE_IN_SAFE_ZONE = 0x05,
//    MESSAGE_TYPE_RESET_DEVICE = 0x04,
    MESSAGE_TYPE_SYNC = 0x04
};

enum message_direction {
    RESPONSE = 0x00,
    REQUEST = 0x01
};

enum battery_level {
    BATTERY_LEVEL_GOOD = 0x00,
    BATTERY_LEVEL_CRITICAL = 0x01
};

enum workers_ids {
    FIRST_PEOPLE_ID = 0x01,
    SECOND_PEOPLE_ID = 0x02,
    THIRD_PEOPLE_ID = 0x04,
    FOURTH_PEOPLE_ID = 0x08,
    FIVE_PEOPLE_ID = 0x10,
    SIXTH_PEOPLE_ID = 0x20,
    SEVENTH_PEOPLE_ID = 0x40,
    EIGHTH_PEOPLE_ID = 0x80
};

#endif //MESSAGE_FORMAT_H
