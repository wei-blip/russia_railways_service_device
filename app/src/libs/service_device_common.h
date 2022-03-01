//
// Created by rts on 01.03.2022.
//

#ifndef SERVICE_DEVICE_SRC_LIBS_SERVICE_DEVICE_COMMON_H_
#define SERVICE_DEVICE_SRC_LIBS_SERVICE_DEVICE_COMMON_H_

#define COMMAND_TYPE_START_TRANSMIT "START"
#define COMMAND_TYPE_STOP_SESSION "STOP"
#define COMMAND_TYPE_PER "PER"

#define PER_VALUE_10 "10"
#define PER_VALUE_100 "100"
#define PER_VALUE_1000 "1000"

#define RECV_TIMEOUT_MIN 1

struct parsed_frame_s {
  const char *cmd_ptr;
  uint8_t arg;
};

struct print_data_elem_s {
  int8_t snr;
  int16_t rssi;
  atomic_val_t packet_num;
};

#endif //SERVICE_DEVICE_SRC_LIBS_SERVICE_DEVICE_COMMON_H_
