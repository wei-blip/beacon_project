#include <logging/log.h>
LOG_MODULE_REGISTER(dw1000, LOG_LEVEL_INF);
#include <errno.h>
#include <kernel.h>
#include <arch/cpu.h>
#include <debug/stack.h>
#include <device.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/uwb/deca_device_api.h>

#if DT_HAS_COMPAT_STATUS_OKAY(decawave_dw1000)
#include "drivers/uwb/dw1000.h"
#define DT_DRV_COMPAT decawave_dw1000
#else
#error No DW1000 instance in device tree.
#endif

#define DWT_SPI_SLOW_FREQ 2000000U

static const struct dw1000_dev_config dw1000_0_config = {
    .irq_port = DT_INST_GPIO_LABEL(0, int_gpios),
    .irq_pin = DT_INST_GPIO_PIN(0, int_gpios),
    .irq_flags = DT_INST_GPIO_FLAGS(0, int_gpios),
    .rst_port = DT_INST_GPIO_LABEL(0, reset_gpios),
    .rst_pin = DT_INST_GPIO_PIN(0, reset_gpios),
    .rst_flags = DT_INST_GPIO_FLAGS(0, reset_gpios),
    .spi_port = DT_INST_BUS_LABEL(0),
    .spi_freq  = DT_INST_PROP(0, spi_max_frequency),
    .spi_slave = DT_INST_REG_ADDR(0),
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
    .spi_cs_port = DT_INST_SPI_DEV_CS_GPIOS_LABEL(0),
	.spi_cs_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0),
	.spi_cs_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0),
#endif
    .tx_ant_delay = DT_INST_PROP_OR(0, tx_ant_delay, 0x4042),
    .rx_ant_delay = DT_INST_PROP_OR(0, rx_ant_delay, 0x4042),
};

static void dwt_set_spi_slow(struct dw1000_dev_data *ctx, const uint32_t freq)
{
    ctx->spi_cfg_slow.frequency = freq;
    ctx->spi_cfg = &ctx->spi_cfg_slow;
}

static void dwt_set_spi_fast(struct dw1000_dev_data *ctx)
{
    ctx->spi_cfg = &ctx->spi_cfg_fast;
}

static int dwt_spi_read(struct dw1000_dev_data *ctx,
                        uint16_t hdr_len, const uint8_t *hdr_buf,
                        uint32_t data_len, uint8_t *data)
{
    const struct spi_buf tx_buf = {
        .buf = (uint8_t *)hdr_buf,
        .len = hdr_len
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };
    struct spi_buf rx_buf[2] = {
        {
            .buf = NULL,
            .len = hdr_len,
        },
        {
            .buf = (uint8_t *)data,
            .len = data_len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = 2
    };

    LOG_DBG("spi read, header length %u, data length %u",
            (uint16_t)hdr_len, (uint32_t)data_len);
    LOG_HEXDUMP_DBG(hdr_buf, (uint16_t)hdr_len, "rd: header");

    if (spi_transceive(ctx->spi, ctx->spi_cfg, &tx, &rx)) {
        LOG_ERR("SPI transfer failed");
        return -EIO;
    }

    LOG_HEXDUMP_DBG(data, (uint32_t)data_len, "rd: data");

    return 0;
}

static int dwt_spi_write(struct dw1000_dev_data *ctx,
                         uint16_t hdr_len, const uint8_t *hdr_buf,
                         uint32_t data_len, const uint8_t *data)
{
    struct spi_buf buf[2] = {
        {.buf = (uint8_t *)hdr_buf, .len = hdr_len},
        {.buf = (uint8_t *)data, .len = data_len}
    };
    struct spi_buf_set buf_set = {.buffers = buf, .count = 2};

    LOG_DBG("spi write, header length %u, data length %u",
            (uint16_t)hdr_len, (uint32_t)data_len);
    LOG_HEXDUMP_DBG(hdr_buf, (uint16_t)hdr_len, "wr: header");
    LOG_HEXDUMP_DBG(data, (uint32_t)data_len, "wr: data");

    if (spi_write(ctx->spi, ctx->spi_cfg, &buf_set)) {
        LOG_ERR("SPI read failed");
        return -EIO;
    }

    return 0;
}

static void dwt_gpio_callback(const struct device *dev,
                              struct gpio_callback *cb, uint32_t pins)
{
    struct dw1000_dev_data *ctx = CONTAINER_OF(cb,
        struct dw1000_dev_data, gpio_cb);

    LOG_DBG("IRQ callback triggered %p", ctx);
    k_work_submit(&ctx->irq_cb_work);
}

static ALWAYS_INLINE void dwt_setup_int(const struct device *dev, bool en)
{
    struct dw1000_dev_data *ctx = dev->data;
    const struct dw1000_dev_config *cfg = dev->config;
    unsigned int flags = en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;
    gpio_pin_interrupt_configure(ctx->irq_gpio, cfg->irq_pin, flags);
}

static void dwt_irq_work_handler(struct k_work *item)
{
    dwt_isr();
}

static struct dw1000_dev_data dwt_0_data = {
    .phy_cfg = {
        .chan = CONFIG_DW1000_CHANNEL_NO,
#if defined(CONFIG_DW1000_PRF_16M)
        .prf = DWT_PRF_16M,
#endif
#if defined(CONFIG_DW1000_PRF_64M)
        .prf = DWT_PRF_64M,
#endif
#if defined(CONFIG_DW1000_PLEN_4096)
        .txPreambLength = DWT_PLEN_4096,
#endif
#if defined(CONFIG_DW1000_PLEN_2048)
        .txPreambLength = DWT_PLEN_2048,
#endif
#if defined(CONFIG_DW1000_PLEN_1536)
        .txPreambLength = DWT_PLEN_1536,
#endif
#if defined(CONFIG_DW1000_PLEN_1024)
        .txPreambLength = DWT_PLEN_1024,
#endif
#if defined(CONFIG_DW1000_PLEN_512)
        .txPreambLength = DWT_PLEN_512,
#endif
#if defined(CONFIG_DW1000_PLEN_256)
        .txPreambLength = DWT_PLEN_256,
#endif
#if defined(CONFIG_DW1000_PLEN_128)
        .txPreambLength = DWT_PLEN_128,
#endif
#if defined(CONFIG_DW1000_PLEN_64)
        .txPreambLength = DWT_PLEN_64,
#endif
#if defined(CONFIG_DW1000_RX_PAC8)
        .rxPAC = DWT_PAC8,
#endif
#if defined(CONFIG_DW1000_RX_PAC16)
        .rxPAC = DWT_PAC16,
#endif
#if defined(CONFIG_DW1000_RX_PAC32)
        .rxPAC = DWT_PAC32,
#endif
#if defined(CONFIG_DW1000_RX_PAC64)
        .rxPAC = DWT_PAC64,
#endif
        .txCode = CONFIG_DW1000_TX_PREAMBLE_CODE,
        .rxCode = CONFIG_DW1000_RX_PREAMBLE_CODE,
#if defined(CONFIG_DW1000_SFD_STD)
        .nsSFD = 0,
#endif
#if defined(CONFIG_DW1000_SFD_NON_STD)
        .nsSFD = 1,
#endif
#if defined(CONFIG_DW1000_BR_110K)
        .dataRate = DWT_BR_110K,
#endif
#if defined(CONFIG_DW1000_BR_850K)
        .dataRate = DWT_BR_850K,
#endif
#if defined(CONFIG_DW1000_BR_6M8)
        .dataRate = DWT_BR_6M8,
#endif
#if defined(CONFIG_DW1000_PHR_MODE_STD)
        .phrMode = DWT_PHRMODE_STD,
#endif
#if defined(CONFIG_DW1000_PHR_MODE_EXT)
        .phrMode = DWT_PHRMODE_EXT,
#endif
        .sfdTO = CONFIG_DW1000_SFD_TIMEOUT
    }
};

void deca_sleep(unsigned int time_ms)
{
    (void) k_msleep((int) time_ms);
}

decaIrqStatus_t decamutexon(void)
{
    return 0;
}

void decamutexoff(decaIrqStatus_t s) {

}

int writetospi(uint16 headerLength, const uint8 * headerBuffer,
               uint32 bodyLength, const uint8 * bodyBuffer) {
    struct dw1000_dev_data *ctx = &dwt_0_data;
    return dwt_spi_write(ctx, headerLength, headerBuffer,
                         bodyLength, bodyBuffer);
}

int readfromspi(uint16 headerLength, const uint8 * headerBuffer,
                uint32 readLength, uint8 * readBuffer) {
    struct dw1000_dev_data *ctx = &dwt_0_data;
    return dwt_spi_read(ctx, headerLength, headerBuffer,
                        readLength, readBuffer);
}

static int dwt_hw_reset(const struct device *dev)
{
    struct dw1000_dev_data *ctx = dev->data;
    const struct dw1000_dev_config *cfg = dev->config;

    if (gpio_pin_configure(ctx->rst_gpio, cfg->rst_pin,
                           GPIO_OUTPUT_ACTIVE | cfg->rst_flags)) {
        LOG_ERR("Failed to configure GPIO pin %u", cfg->rst_pin);
        return -EINVAL;
    }

    k_sleep(K_MSEC(1));
    gpio_pin_set(ctx->rst_gpio, cfg->rst_pin, 0);
    k_sleep(K_MSEC(5));

    if (gpio_pin_configure(ctx->rst_gpio, cfg->rst_pin,
                           GPIO_INPUT | cfg->rst_flags)) {
        LOG_ERR("Failed to configure GPIO pin %u", cfg->rst_pin);
        return -EINVAL;
    }

    return 0;
}

static int dw1000_init(const struct device *dev)
{
    struct dw1000_dev_data *ctx = dev->data;
    const struct dw1000_dev_config *cfg = dev->config;

    LOG_INF("Initialize DW1000 Transceiver");

    /* SPI config */
    ctx->spi_cfg_slow.operation = SPI_WORD_SET(8);
    ctx->spi_cfg_slow.frequency = DWT_SPI_SLOW_FREQ;
    ctx->spi_cfg_slow.slave = cfg->spi_slave;

    ctx->spi_cfg_fast.operation = SPI_WORD_SET(8);
    ctx->spi_cfg_fast.frequency = cfg->spi_freq;
    ctx->spi_cfg_fast.slave = cfg->spi_slave;

    ctx->spi = device_get_binding((char *)cfg->spi_port);
    if (!ctx->spi) {
        LOG_ERR("SPI master port %s not found", cfg->spi_port);
        return -EINVAL;
    }

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
    ctx->spi_cs.gpio_dev =
		device_get_binding((char *)cfg->spi_cs_port);
	if (!ctx->spi_cs.gpio_dev) {
		LOG_ERR("SPI CS port %s not found", cfg->spi_cs_port);
		return -EINVAL;
	}

	ctx->spi_cs.gpio_pin = cfg->spi_cs_pin;
	ctx->spi_cs.gpio_dt_flags = cfg->spi_cs_flags;
	ctx->spi_cfg_slow.cs = &ctx->spi_cs;
	ctx->spi_cfg_fast.cs = &ctx->spi_cs;
#endif

    dwt_set_spi_slow(ctx, DWT_SPI_SLOW_FREQ);

    /* Initialize IRQ GPIO */
    ctx->irq_gpio = device_get_binding((char *)cfg->irq_port);
    if (!ctx->irq_gpio) {
        LOG_ERR("GPIO port %s not found", cfg->irq_port);
        return -EINVAL;
    }

    if (gpio_pin_configure(ctx->irq_gpio, cfg->irq_pin,
                           GPIO_INPUT | cfg->irq_flags)) {
        LOG_ERR("Unable to configure GPIO pin %u", cfg->irq_pin);
        return -EINVAL;
    }

    gpio_init_callback(&(ctx->gpio_cb), dwt_gpio_callback,
                       BIT(cfg->irq_pin));

    if (gpio_add_callback(ctx->irq_gpio, &(ctx->gpio_cb))) {
        LOG_ERR("Failed to add IRQ callback");
        return -EINVAL;
    }

    /* Initialize RESET GPIO */
    ctx->rst_gpio = device_get_binding(cfg->rst_port);
    if (ctx->rst_gpio == NULL) {
        LOG_ERR("Could not get GPIO port for RESET");
        return -EIO;
    }

    if (gpio_pin_configure(ctx->rst_gpio, cfg->rst_pin,
                           GPIO_INPUT | cfg->rst_flags)) {
        LOG_ERR("Unable to configure GPIO pin %u", cfg->rst_pin);
        return -EINVAL;
    }

    LOG_INF("GPIO and SPI configured");

    int init_cfg = DWT_LOADNONE;
#if defined(CONFIG_DW1000_STARTUP_LOADUCODE)
    init_cfg |= DWT_LOADUCODE;
#endif
#if defined(CONFIG_DW1000_STARTUP_DW_WAKE_UP)
    init_cfg |= DWT_DW_WAKE_UP;
#endif
#if defined(CONFIG_DW1000_STARTUP_DW_WUP_NO_UCODE)
    init_cfg |= DWT_DW_WUP_NO_UCODE;
#endif
#if defined(CONFIG_DW1000_STARTUP_DW_WUP_RD_OTPREV)
    init_cfg |= DWT_DW_WUP_RD_OTPREV;
#endif
#if defined(CONFIG_DW1000_STARTUP_READ_OTP_PID)
    init_cfg |= DWT_READ_OTP_PID;
#endif
#if defined(CONFIG_DW1000_STARTUP_READ_OTP_LID)
    init_cfg |= DWT_READ_OTP_LID;
#endif
#if defined(CONFIG_DW1000_STARTUP_READ_OTP_BAT)
    init_cfg |= DWT_READ_OTP_BAT;
#endif
#if defined(CONFIG_DW1000_STARTUP_READ_OTP_TMP)
    init_cfg |= DWT_READ_OTP_TMP;
#endif

    dwt_hw_reset(dev);

    if (dwt_initialise(init_cfg) == DWT_ERROR) {
        LOG_ERR("Failed to initialize DW1000");
        return -EIO;
    }

    dwt_set_spi_fast(ctx);
    dwt_configure(&ctx->phy_cfg);
    dwt_setrxantennadelay(cfg->rx_ant_delay);
    dwt_settxantennadelay(cfg->tx_ant_delay);

    k_work_init(&ctx->irq_cb_work, dwt_irq_work_handler);

    dwt_setup_int(dev, true);

    LOG_INF("DW1000 device initialized and configured");

    return 0;
}

#ifdef CONFIG_PM_DEVICE
int dw1000_pm_ctrl(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;
    struct dw1000_dev_data *ctx = dev->data;
    const struct dw1000_dev_config *cfg = dev->config;
    uint16_t mode = DWT_PRESRV_SLEEP | DWT_CONFIG;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/* Put the chip into sleep mode */
#if defined(CONFIG_DW1000_STARTUP_LOADUCODE)
        mode |= DWT_LOADUCODE;
#endif
        dwt_configuresleep(mode, DWT_WAKE_CS | DWT_SLP_EN);
        dwt_entersleep();
		break;
	case PM_DEVICE_ACTION_RESUME:
		/* Wake up chip */
        if(dwt_readdevid() != DWT_DEVICE_ID) {
            gpio_pin_set(ctx->spi_cs.gpio_dev, ctx->spi_cs.gpio_pin, 0);
            k_sleep(K_MSEC(1));
            gpio_pin_set(ctx->spi_cs.gpio_dev, ctx->spi_cs.gpio_pin, 1);
            k_sleep(K_MSEC(5));
        }
        dwt_setrxantennadelay(cfg->rx_ant_delay);
        dwt_settxantennadelay(cfg->tx_ant_delay);
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */


static struct dw1000_dev_api dwt_radio_api = {

};

DEVICE_DT_INST_DEFINE(0, dw1000_init, dw1000_pm_ctrl,
                      &dwt_0_data, &dw1000_0_config,
                      POST_KERNEL, CONFIG_DW1000_INIT_PRIO,
                      &dwt_radio_api);
