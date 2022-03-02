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
 *   - /soc/dma@40020000
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
 *   - /soc/serial@40013800
 *   - (/soc/spi@40003800)
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
__devicehdl_DT_N_S_soc_S_rcc_40021000[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 11, 3, 2, 12, 13, 6, 10, 7, 4, 8, 5, DEVICE_HANDLE_ENDS };

/* 2 : /soc/dma@40020000:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/soc/interrupt-controller@e000e100)
 *   - /soc/rcc@40021000
 * Supported:
 *   - /soc/serial@40013800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_dma_40020000[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 12, DEVICE_HANDLE_ENDS };

/* 3 : /soc/pin-controller@48000000/gpio@48001c00:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48001c00[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 4 : /soc/pin-controller@48000000/gpio@48001000:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48001000[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 5 : /soc/pin-controller@48000000/gpio@48000c00:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000c00[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 6 : /soc/pin-controller@48000000/gpio@48000800:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 * Supported:
 *   - (/gpio_keys/button)
 *   - /soc/spi@40013000/sx1278@0
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000800[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 14, DEVICE_HANDLE_ENDS };

/* 7 : /soc/pin-controller@48000000/gpio@48000400:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 * Supported:
 *   - /soc/spi@40013000
 *   - /soc/spi@40013000/sx1278@0
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000400[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 13, 14, DEVICE_HANDLE_ENDS };

/* 8 : /soc/pin-controller@48000000/gpio@48000000:
 * Direct Dependencies:
 *   - (/soc/pin-controller@48000000)
 *   - /soc/rcc@40021000
 * Supported:
 *   - (/leds/led_0)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_48000000_S_gpio_48000000[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 9 : /soc/interrupt-controller@40010400:
 * Direct Dependencies:
 *   - (/soc)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_interrupt_controller_40010400[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 10 : /soc/rng@50060800:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/soc/interrupt-controller@e000e100)
 *   - /soc/rcc@40021000
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_rng_50060800[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 11 : /soc/serial@40004400:
 * Direct Dependencies:
 *   - (/soc)
 *   - (/soc/interrupt-controller@e000e100)
 *   - /soc/rcc@40021000
 *   - (/soc/pin-controller@48000000/usart2_rx_pa3)
 *   - (/soc/pin-controller@48000000/usart2_tx_pa2)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40004400[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 12 : /soc/serial@40013800:
 * Direct Dependencies:
 *   - (/soc)
 *   - /soc/dma@40020000
 *   - (/soc/interrupt-controller@e000e100)
 *   - /soc/rcc@40021000
 *   - (/soc/pin-controller@48000000/usart1_rx_pa10)
 *   - (/soc/pin-controller@48000000/usart1_tx_pa9)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40013800[] = { 1, 2, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

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
__devicehdl_DT_N_S_soc_S_spi_40013000[] = { 7, 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 14, DEVICE_HANDLE_ENDS };

/* 14 : /soc/spi@40013000/sx1278@0:
 * Direct Dependencies:
 *   - /soc/spi@40013000
 *   - /soc/pin-controller@48000000/gpio@48000400
 *   - /soc/pin-controller@48000000/gpio@48000800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40013000_S_sx1278_0[] = { 13, 6, 7, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };
