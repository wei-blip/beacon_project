#include <devicetree.h>
#include <device.h>
#include <errno.h>
#include <sys/printk.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/lora.h>
#include "lora_russia_railways.h"
#include <string.h>


#define MS_SLEEP 1000

//#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
//#include <logging/log.h>
//LOG_MODULE_REGISTER(rzd_project);

//#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)

//BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
//             "No default LoRa radio specified in DT");

void SecureElementRandomNumber(uint32_t* rand_num) {
    return;
}
#define MAX_DATA_LEN 10
char data[MAX_DATA_LEN] = {'h', 'e', 'l', 'l', 'o', 'w', 'o', 'r', 'l', 'd'};

extern uint8_t rx_buf[MESSAGE_LEN_IN_BYTES];
extern uint8_t tx_buf[MESSAGE_LEN_IN_BYTES];

struct message_s msg;

void main() {
//    printk("Start application\n");
//    const struct device* lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
//    struct lora_modem_config config;
//    int ret;
//
//    config.frequency = 434000000;
//    config.bandwidth = BW_125_KHZ;
//    config.datarate = SF_10;
//    config.preamble_len = 8;
//    config.coding_rate = CR_4_5;
//    config.tx_power = 0;
//    config.tx = true;
//
//    if (!device_is_ready(lora_dev)) {
//        printk("Radio not ready");
//        return;
//    }
//
//    ret = lora_config(lora_dev, &config);
//    if (ret < 0) {
//        printk(" LoRa config failed");
//        return;
//    }
//
////    char data[30] = {0};
////    strcpy(&data[0], "Hello from FieldSense!");
//    while(1) {
//        ret = lora_send(lora_dev, data, MAX_DATA_LEN);
//        if (ret < 0) {
//            printk("%d Operation failed error code: \n", ret);
//            return;
//        }
//        printk(" Data sent!\n");
//        k_sleep(K_MSEC(1000));O
//    }
    msg.sender_addr = SEND_BASE_STATION;
    msg.receiver_addr = RECV_BROADCAST;
    msg.message_type = MESSAGE_TYPE_CONNECTION_QUALITY_TEST;
    msg.direction = REQUEST;
    msg.battery_level = BATTERY_LEVEL_GOOD;
    msg.people_in_safe_zone = FIRST_PEOPLE_ID | SECOND_PEOPLE_ID | EIGHTH_PEOPLE_ID | FIVE_PEOPLE_ID;
    send_msg(&msg);
    memcpy(rx_buf, tx_buf, MESSAGE_LEN_IN_BYTES);
    recv_msg();
    while(1);
}