//
// Created by rts on 01.03.2022.
//
#ifndef SERVICE_DEVICE_SRC_LIBS_SERVICE_DEVICE_COMMON_H_
#define SERVICE_DEVICE_SRC_LIBS_SERVICE_DEVICE_COMMON_H_

#include <device.h>
#include <devicetree.h>
#include <errno.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <drivers/uart.h>
#include <drivers/lora.h>
#include <drivers/gpio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <random/rand32.h>

/**
 * Define area begin
 * */

#define STACK_SIZE 1024
#define THREAD_PRIORITY 0

#define UART_RX_BUF_LEN 16
#define UART_TX_BUF_LEN 64
#define RADIO_BUF_LEN 4
#define TIMEOUT 0
#define TX_TIMEOUT_US 0

#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

#define STATE_NODE	DT_ALIAS(state)
#if !DT_NODE_HAS_STATUS(STATE_NODE, okay)
#error "Unsupported board: state devicetree alias is not defined"
#endif

#define COMMAND_TYPE_STOP_RECEIVE_SESSION "STOP"
#define COMMAND_TYPE_GET_CFG "GET_CFG"
#define COMMAND_TYPE_PER "PER"
#define COMMAND_TYPE_SET_FREQ "SET_FREQ"
#define COMMAND_TYPE_INCR_FREQ "FREQ+"
#define COMMAND_TYPE_DECR_FREQ "FREQ-"
#define COMMAND_TYPE_SET_SF "SET_SF"

#define PER_VALUE_10 "10"
#define PER_VALUE_100 "100"
#define PER_VALUE_1000 "1000"

#define DR_SF_12 12
#define DR_SF_11 11
#define DR_SF_10 10
#define DR_SF_9 9
#define DR_SF_8 8
#define DR_SF_7 7
#define DR_SF_6 6

#define FREQUENCY_STEP_HZ 100000

#define CURRENT_UART_DEVICE "UART_1"

#define RECV_TIMEOUT_SEC 30

#define STATE_IDLE 0
#define STATE_START_PER_MEAS 1
#define STATE_PER_MEAS_RUN 2
#define STATE_RECV 3
#define STATE_TRANSMIT 4
#define STATE_GET_CFG 5
#define STATE_SET_FREQ 6
#define STATE_INCR_FREQ 7
#define STATE_DECR_FREQ 8
#define STATE_SET_SF 9
/**
 * Define area end
 * */

extern uint8_t radio_buf_rx[RADIO_BUF_LEN];
extern uint8_t radio_buf_tx[RADIO_BUF_LEN];

extern atomic_t atomic_cur_state;
extern atomic_t atomic_per_num;
extern atomic_t atomic_uart_tx_done;

extern struct gpio_dt_spec state_bluetooth;

struct parsed_frame_s {
  const char *cmd_ptr;
  uint32_t arg;
};

struct print_data_elem_s {
  int8_t snr;
  int16_t rssi;
  uint32_t packet_num;
};

/**
 * Function  area begin
 * */
void print_modem_cfg(const struct device *dev, uint8_t *buf_tx, struct lora_modem_config *cfg);
void incr_decr_modem_frequency(const struct device *lora_dev, struct lora_modem_config *lora_cfg, bool incr,
  const struct device *uart_dev, uint8_t buf_tx);
void print_per_status(const struct device *dev, uint8_t *buf_tx, int ret, struct print_data_elem_s *print_data);
int change_modem_frequency(const struct device *dev, struct lora_modem_config *cfg, uint32_t new_freq_khz);
int change_modem_datarate(const struct device *dev, struct lora_modem_config *cfg, enum lora_datarate new_dr);
void per_meas(const struct device *lora_dev, struct lora_modem_config *lora_cfg,
             const struct device *uart_dev, uint8_t *buf_tx);

void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr);
void lora_rx_error_timeout_cb(void);
/**
 * Function  area end
 * */

#endif //SERVICE_DEVICE_SRC_LIBS_SERVICE_DEVICE_COMMON_H_
