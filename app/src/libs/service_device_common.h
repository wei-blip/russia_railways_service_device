//
// Created by rts on 01.03.2022.
//

#ifndef SERVICE_DEVICE_SRC_LIBS_SERVICE_DEVICE_COMMON_H_
#define SERVICE_DEVICE_SRC_LIBS_SERVICE_DEVICE_COMMON_H_

#define COMMAND_TYPE_START_TRANSMIT "START"
#define COMMAND_TYPE_STOP_RECEIVE_SESSION "STOP"
#define COMMAND_TYPE_PER "PER"

#define PER_VALUE_10 "10"
#define PER_VALUE_100 "100"
#define PER_VALUE_1000 "1000"

#define CURRENT_UART_DEVICE "UART_1"

#define RECV_TIMEOUT_SEC 30

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
void uart_config(const char* dev_name, uint32_t baudrate,
                 uint8_t data_bits, uint8_t flow_ctrl, uint8_t parity, uint8_t stop_bits);
void event_cb(const struct device* dev, struct uart_event* evt, void* user_data);
void work_uart_data_proc_handler(struct k_work *item);
void system_init(void);
/**
 * Function  area end
 * */

#endif //SERVICE_DEVICE_SRC_LIBS_SERVICE_DEVICE_COMMON_H_
