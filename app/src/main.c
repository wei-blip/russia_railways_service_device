#include <devicetree.h>
#include <device.h>
#include <errno.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <drivers/uart.h>
#include <drivers/lora.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <random/rand32.h>

#include "service_device_common.h"

//// Define area begin ////
#define UART_BUF_LEN_RX 8
#define UART_BUF_LEN_TX 32
#define RADIO_BUF_LEN 4
#define TIMEOUT 0

#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");
//// Define area end ////


#include <logging/log.h>
LOG_MODULE_REGISTER(service_device);

uint32_t per_num = 0;

uint8_t uart_buf_tx[UART_BUF_LEN_TX] = {0};
uint8_t uart_buf_rx[UART_BUF_LEN_RX] = {0};
uint8_t radio_buf_rx[RADIO_BUF_LEN] = {0};
uint8_t radio_buf_tx[RADIO_BUF_LEN] = {0};

K_MSGQ_DEFINE(msgq_radio_recv_data, sizeof(radio_buf_rx), 10, 1);
K_MSGQ_DEFINE(msgq_radio_send_data, sizeof(radio_buf_tx), 10, 1);
K_MSGQ_DEFINE(msgq_uart_recv_data, sizeof(uart_buf_rx), 10, 1);

//// Structs, typedefs and enums area begin ////
atomic_t atomic_packet_count = ATOMIC_INIT(0);

atomic_t atomic_cur_state = ATOMIC_INIT(0);
const atomic_t atomic_idle_state = ATOMIC_INIT(0);
const atomic_t atomic_per_meas_state = ATOMIC_INIT(1);
const atomic_t atomic_recv_state = ATOMIC_INIT(2);

const struct device *uart_dev;
const struct device *lora_dev;
struct k_work work_uart_data_proc;
struct k_work work_radio_recv;
struct k_work work_per_meas;

struct k_mutex mut_radio_busy;
//// Structs and enums area end ////

//// Function  area begin ////
void uart_config(const char* dev_name, uint32_t baudrate,
                 uint8_t data_bits, uint8_t flow_ctrl, uint8_t parity, uint8_t stop_bits);
void event_cb(const struct device* dev, struct uart_event* evt, void* user_data);
void work_uart_data_proc_handler(struct k_work *item);
void system_init(void);
//// Function  area end ////

void main(void) {
    system_init();
    while(1);
}


void uart_config(const char* dev_name, uint32_t baudrate, uint8_t data_bits,
                 uint8_t flow_ctrl, uint8_t parity, uint8_t stop_bits) {
    int ret;
    uart_dev = device_get_binding(dev_name);
    if (!device_is_ready(uart_dev)) {
        LOG_DBG("Device not ready");
    }
    struct uart_config uart_cfg = {
      .baudrate = baudrate,
      .data_bits = data_bits,
      .flow_ctrl = flow_ctrl,
      .parity = parity,
      .stop_bits = stop_bits
    };
    ret = uart_configure(uart_dev, &uart_cfg);
    if(ret < 0) {
        printk(" Error uart_configure %d\n", ret);
        return;
    }
}


void event_cb(const struct device* dev, struct uart_event* evt, void* user_data) {
    switch (evt->type) {
        case UART_TX_ABORTED:
            break;
        case UART_TX_DONE:
            printk(" Data send\n");
            uart_rx_enable(uart_dev, uart_buf_rx, UART_BUF_LEN_RX, 0);
            break;
        case UART_RX_BUF_RELEASED:
            break;
        case UART_RX_BUF_REQUEST:
            break;
        case UART_RX_DISABLED:
            break;
        case UART_RX_RDY:
            printk(" Data receive\n");
            break;
        case UART_RX_STOPPED:
            break;
    }
}


void work_uart_data_proc_handler(struct k_work *item)
{
    struct parsed_frame_s parsed_frame;
    parsed_frame.cmd_ptr = strtok((char*)(&uart_buf_rx), "="); /* select current command */
    parsed_frame.arg = atoi(strtok((char*)(&uart_buf_rx), ".")); /* select command argument */

    if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_START_TRANSMIT)) {
        atomic_set(&atomic_cur_state, atomic_get(&atomic_idle_state));
    } else if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_PER)) {
        atomic_set(&atomic_cur_state, atomic_get(&atomic_per_meas_state));
        atomic_set(&atomic_packet_count, 0);
        per_num = parsed_frame.arg;
    } else if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_STOP_SESSION)) {
        atomic_set(&atomic_cur_state, atomic_get(&atomic_recv_state));
    }
}


void work_radio_per_meas_handler(struct k_work *item)
{
    int8_t snr = 0;
    int16_t rssi = 0;
    int32_t ret = 0;
    atomic_val_t per_meas_state = atomic_get(&atomic_per_meas_state);
    struct print_data_elem_s print_data = {0};
    while (atomic_get(&atomic_cur_state) == per_meas_state) {
        uint8_t i = 0;
        while (i < per_num) {
            sys_rand_get(radio_buf_tx, RADIO_BUF_LEN);
            ret = lora_send(lora_dev, radio_buf_tx, RADIO_BUF_LEN);
            ret = lora_recv(lora_dev, radio_buf_rx, RADIO_BUF_LEN, K_MINUTES(RECV_TIMEOUT_MIN), &rssi, &snr);
            if (ret > 0) { /*If receive successful change print_data */
                if (!strcmp(radio_buf_tx, radio_buf_rx)) {
                    atomic_inc(&atomic_packet_count);
                    print_data.rssi = rssi;
                    print_data.snr = snr;
                    print_data.packet_num = atomic_get(&atomic_packet_count);
                }
            }
            k_msgq_put(&msgq_radio_recv_data, &print_data, K_NO_WAIT);
            i++;
        }
        atomic_set(&atomic_cur_state, atomic_get(&atomic_idle_state));
    }
}


void work_radio_recv_handler(struct k_work *item)
{
    int8_t snr = 0;
    int16_t rssi = 0;
    int32_t ret = 0;
    atomic_val_t recv_state = atomic_get(&atomic_recv_state);
    while (atomic_get(&atomic_cur_state) == recv_state) {
        ret = lora_recv(lora_dev, radio_buf_rx, RADIO_BUF_LEN, K_MINUTES(RECV_TIMEOUT_MIN), &rssi, &snr);
        if (ret > 0) {
            k_msgq_put(&msgq_radio_recv_data, radio_buf_rx, K_NO_WAIT);
        }
    }
}


void system_init(void)
{
    struct lora_modem_config lora_cfg = {0};
    int err = 0;

    k_work_init(&work_per_meas, work_radio_per_meas_handler);
    k_work_init(&work_radio_recv, work_radio_recv_handler);
    k_work_init(&work_uart_data_proc, work_uart_data_proc_handler);

    k_mutex_init(&mut_radio_busy);

    uart_dev = device_get_binding("UART_1");
    if (!device_is_ready(uart_dev)) {
        LOG_DBG("Device not ready: %s", uart_dev->name);
        k_sleep(K_FOREVER);
    }
    uart_callback_set(uart_dev, event_cb, NULL);

    lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    if (!device_is_ready(lora_dev)) {
        LOG_DBG("Device not ready: %s", lora_dev->name);
        k_sleep(K_FOREVER);
    }

    lora_cfg.tx = false;
    lora_cfg.frequency = 433000000;
    lora_cfg.bandwidth = BW_125_KHZ;
    lora_cfg.datarate = SF_12;
    lora_cfg.preamble_len = 8;
    lora_cfg.coding_rate = CR_4_5;
    lora_cfg.tx_power = 0;
    err = lora_config(lora_dev, &lora_cfg);
    if (err < 0) {
        LOG_DBG("Device not configure: %s", lora_dev->name);
        k_sleep(K_FOREVER);
    }
}


_Noreturn void task_radio_data_proc_handler(void)
{
    int32_t ret = 0;
    struct print_data_elem_s print_data = {0};
    atomic_val_t atomic_prev_val;
    char buf[64] = {0};
    while(1) {
        if (k_msgq_num_used_get(&msgq_radio_recv_data)) {
            ret = k_msgq_get(&msgq_radio_recv_data, &print_data, K_NO_WAIT);
            if (!ret) {
                if (atomic_cas(&print_data.packet_num, atomic_prev_val, atomic_prev_val)) { /* If packet_num equal prev_val */
                    sprintf(buf, "Packet %d is missing!!!\r\n", (int)atomic_prev_val);
                    /* TODO: out into terminal */
                } else {
                    sprintf(buf, "Packet %d is received\r\n rssi: %d\r\n snr: %d\r\n", (int)print_data.packet_num,
                            print_data.rssi, print_data.snr);
                }
            } else {
                continue;
            }

        } else {
            k_sleep(K_MSEC(1));
        }
    }
}

