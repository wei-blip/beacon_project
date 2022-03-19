#include <device.h>
#include <toolchain.h>

/* 1 : /soc/rcc@40021000:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/clocks/pll)
 * Supported:
 *   - (/soc/adc@50040000)
 *   - (/soc/adc@50040100)
 *   - (/soc/can@40006400)
 *   - (/soc/dac@40007400)
 *   - (/soc/dma@40020000)
 *   - (/soc/dma@40020400)
 *   - (/soc/flash-controller@40022000)
 *   - (/soc/i2c@40005400)
 *   - (/soc/i2c@40005800)
 *   - (/soc/i2c@40005c00)
 *   - (/soc/i2c@40008400)
 *   - (/soc/quadspi@a0001000)
 *   - /soc/rng@50060800
 *   - (/soc/rtc@40002800)
 *   - /soc/serial@40004400
 *   - (/soc/serial@40004800)
 *   - (/soc/serial@40004c00)
 *   - (/soc/serial@40008000)
 *   - (/soc/serial@40013800)
 *   - /soc/spi@40003800
 *   - (/soc/spi@40003c00)
 *   - /soc/spi@40013000
 *   - (/soc/timers@40000000)
 *   - (/soc/timers@40000400)
 *   - (/soc/timers@40001000)
 *   - (/soc/timers@40007c00)
 *   - (/soc/timers@40012c00)
 *   - (/soc/timers@40014000)
 *   - (/soc/timers@40014400)
 *   - (/soc/usb@40006800)
 *   - (/soc/watchdog@40002c00)
 *   - /soc/pin-controller@48000000/gpio@48000000
 *   - /soc/pin-controller@48000000/gpio@48000400
 *   - /soc/pin-controller@48000000/gpio@48000800
 *   - /soc/pin-controller@48000000/gpio@48000c00
 *   - /soc/pin-controller@48000000/gpio@48001000
 *   - /soc/pin-controller@48000000/gpio@48001c00
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_rcc_40021000[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 12, 9, 7, 3, 4, 11, 6, 2, 10, 5, 13, DEVICE_HANDLE_ENDS };

/* 2 : /soc/pin-controller@48000000/gpio@48001c00:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48001c00[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 3 : /soc/pin-controller@48000000/gpio@48001000:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48001000[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 4 : /soc/pin-controller@48000000/gpio@48000c00:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000c00[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 5 : /soc/pin-controller@48000000/gpio@48000800:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 * Supported:
 *   - (/gpio_keys/button)
 *   - (/gpio_keys/button_0)
 *   - /soc/spi@40003800
 *   - /soc/spi@40013000/sx1278@0
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000800[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 12, 15, DEVICE_HANDLE_ENDS };

/* 6 : /soc/pin-controller@48000000/gpio@48000400:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 * Supported:
 *   - /soc/spi@40013000
 *   - /soc/spi@40003800/dw1000@0
 *   - /soc/spi@40013000/sx1278@0
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000400[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 14, 13, 15, DEVICE_HANDLE_ENDS };

/* 7 : /soc/pin-controller@48000000/gpio@48000000:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 * Supported:
 *   - (/leds/led_0)
 *   - /soc/spi@40003800/dw1000@0
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000000[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 14, DEVICE_HANDLE_ENDS };

/* 8 : /soc/interrupt-controller@40010400:
 * Direct Dependencies:
 *   - (/soc)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_interrupt_controller_40010400[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 9 : /soc/rng@50060800:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/soc/interrupt-controller@e000e100)
 *   - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_rng_50060800[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 10 : /soc/serial@40004400:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/soc/interrupt-controller@e000e100)
 *   - /soc/rcc@40021000
 *   - (/soc/pin-controller@48000000/usart2_rx_pa3)
 *   - (/soc/pin-controller@48000000/usart2_tx_pa2)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40004400[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 11 : /soc/timers@40000000/pwm:
 * Direct Dependencies:
 *   - (/soc/timers@40000000)
 *   - (/soc/pin-controller@48000000/tim2_ch1_pa0)
 *   - (/soc/pin-controller@48000000/tim2_ch2_pa1)
 * Supported:
 *   - /ws2812
 *   - (/pwmleds/external_pwm_led)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_timers_40000000_S_pwm[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 16, DEVICE_HANDLE_ENDS };

/* 12 : /soc/spi@40003800:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/soc/interrupt-controller@e000e100)
 *   - /soc/rcc@40021000
 *   - /soc/pin-controller@48000000/gpio@48000800
 *   - (/soc/pin-controller@48000000/spi2_miso_pb14)
 *   - (/soc/pin-controller@48000000/spi2_mosi_pb15)
 *   - (/soc/pin-controller@48000000/spi2_sck_pb13)
 * Supported:
 *   - /soc/spi@40003800/dw1000@0
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40003800[] = { 5, 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 14, DEVICE_HANDLE_ENDS };

/* 13 : /soc/spi@40013000:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/soc/interrupt-controller@e000e100)
 *   - /soc/rcc@40021000
 *   - /soc/pin-controller@48000000/gpio@48000400
 *   - (/soc/pin-controller@48000000/spi1_miso_pa6)
 *   - (/soc/pin-controller@48000000/spi1_mosi_pa7)
 *   - (/soc/pin-controller@48000000/spi1_sck_pa5)
 * Supported:
 *   - /soc/spi@40013000/sx1278@0
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40013000[] = { 6, 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 15, DEVICE_HANDLE_ENDS };

/* 14 : /soc/spi@40003800/dw1000@0:
 * Direct Dependencies:
 *   - /soc/spi@40003800
 *   - /soc/pin-controller@48000000/gpio@48000000
 *   - /soc/pin-controller@48000000/gpio@48000400
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40003800_S_dw1000_0[] = { 12, 7, 6, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 15 : /soc/spi@40013000/sx1278@0:
 * Direct Dependencies:
 *   - /soc/spi@40013000
 *   - /soc/pin-controller@48000000/gpio@48000400
 *   - /soc/pin-controller@48000000/gpio@48000800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40013000_S_sx1278_0[] = { 5, 6, 13, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 16 : /ws2812:
 * Direct Dependencies:
 *   - (/)
 *   - /soc/timers@40000000/pwm
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_ws2812[] = { 11, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };
