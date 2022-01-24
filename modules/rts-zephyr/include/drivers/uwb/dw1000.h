#ifndef ZEPHYR_DRIVERS_SENSOR_DW1000_DW1000_H_
#define ZEPHYR_DRIVERS_SENSOR_DW1000_DW1000_H_

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <sys/util.h>
#include "drivers/uwb/deca_device_api.h"

/* config */
struct dw1000_dev_config {
    const char *irq_port;
    uint8_t irq_pin;
    gpio_dt_flags_t irq_flags;
    const char *rst_port;
    uint8_t rst_pin;
    gpio_dt_flags_t rst_flags;
    const char *spi_port;
    uint8_t spi_cs_pin;
    gpio_dt_flags_t spi_cs_flags;
    const char *spi_cs_port;
    uint32_t spi_freq;
    uint8_t spi_slave;
    uint16_t tx_ant_delay;
    uint16_t rx_ant_delay;
};

/* data */
struct dw1000_dev_data {
    const struct device *irq_gpio;
    const struct device *rst_gpio;
    const struct device *spi;
    struct spi_cs_control spi_cs;
    struct spi_config *spi_cfg;
    struct spi_config spi_cfg_slow;
    struct spi_config spi_cfg_fast;
    struct gpio_callback gpio_cb;
    struct k_work irq_cb_work;
    dwt_config_t phy_cfg;
};

/* api */
struct dw1000_dev_api {

};

#endif /* ZEPHYR_DRIVERS_SENSOR_DW1000_DW1000_H_ */
