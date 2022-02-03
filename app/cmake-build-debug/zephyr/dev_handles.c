#include <device.h>
#include <toolchain.h>

/* 1 : /soc/rcc@40021000:
 * - (/soc)
 * - (/clocks/pll)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_rcc_40021000[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 2 : /soc/pin-controller@48000000/gpio@48001c00:
 * - (/soc/pin-controller@48000000)
 * - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48001c00[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 3 : /soc/pin-controller@48000000/gpio@48001000:
 * - (/soc/pin-controller@48000000)
 * - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48001000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 4 : /soc/pin-controller@48000000/gpio@48000c00:
 * - (/soc/pin-controller@48000000)
 * - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000c00[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 5 : /soc/pin-controller@48000000/gpio@48000800:
 * - (/soc/pin-controller@48000000)
 * - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000800[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 6 : /soc/pin-controller@48000000/gpio@48000400:
 * - (/soc/pin-controller@48000000)
 * - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 7 : /soc/pin-controller@48000000/gpio@48000000:
 * - (/soc/pin-controller@48000000)
 * - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 8 : /soc/interrupt-controller@40010400:
 * - (/soc)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_interrupt_controller_40010400[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 9 : /soc/serial@40004400:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40021000
 * - (/soc/pin-controller@48000000/usart2_rx_pa3)
 * - (/soc/pin-controller@48000000/usart2_tx_pa2)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40004400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 10 : sysinit:
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_sys_init_sys_clock_driver_init0[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 11 : /soc/spi@40013000:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40021000
 * - /soc/pin-controller@48000000/gpio@48000400
 * - (/soc/pin-controller@48000000/spi1_miso_pa6)
 * - (/soc/pin-controller@48000000/spi1_mosi_pa7)
 * - (/soc/pin-controller@48000000/spi1_sck_pa5)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40013000[] = { 1, 6, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 12 : /soc/spi@40013000/sx1278@0:
 * - /soc/spi@40013000
 * - /soc/pin-controller@48000000/gpio@48000000
 * - /soc/pin-controller@48000000/gpio@48000400
 * - /soc/pin-controller@48000000/gpio@48000800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40013000_S_sx1278_0[] = { 11, 7, 5, 6, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };
