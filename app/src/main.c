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

/**
 * Define area begin
 * */
#define UART_RX_BUF_LEN 16
#define UART_TX_BUF_LEN 64
#define RADIO_BUF_LEN 4
#define TIMEOUT 0

#define STACK_SIZE 1024
#define THREAD_PRIORITY 0

#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");
/**
 * Define area end
 * */


#include <logging/log.h>
LOG_MODULE_REGISTER(service_device);

extern const k_tid_t task_uart_data_proc_id;
extern const k_tid_t task_radio_data_proc_id;
extern const k_tid_t task_radio_data_recv_id;

uint32_t per_num = 0;

uint8_t uart_buf_tx[UART_TX_BUF_LEN] = {0};
uint8_t uart_buf_rx[UART_RX_BUF_LEN] = {0};
uint8_t radio_buf_rx[RADIO_BUF_LEN] = {0};
uint8_t radio_buf_tx[RADIO_BUF_LEN] = {0};

K_MSGQ_DEFINE(msgq_radio_recv_data, sizeof(struct print_data_elem_s), 10, 1);
K_MSGQ_DEFINE(msgq_radio_send_data, sizeof(radio_buf_tx), 10, 1);
K_MSGQ_DEFINE(msgq_uart_recv_data, sizeof(uart_buf_rx), 10, 1);

/**
 * Structs, typedefs and enums area begin
 * */
atomic_t atomic_packet_count = ATOMIC_INIT(0);
atomic_t atomic_uart_busy = ATOMIC_INIT(0);

const atomic_t atomic_idle_state = ATOMIC_INIT(0);
const atomic_t atomic_per_meas_state = ATOMIC_INIT(1);
const atomic_t atomic_recv_state = ATOMIC_INIT(2);

atomic_t atomic_cur_state = atomic_recv_state;

const struct device *uart_dev;
const struct device *lora_dev;
//struct k_work work_radio_recv;
struct k_work work_per_meas;

struct k_mutex mut_radio_busy;

struct lora_modem_config lora_cfg = {0};
/**
 * Structs and enums area end
 * */


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
            break;
        case UART_RX_BUF_RELEASED:
            break;
        case UART_RX_BUF_REQUEST:
            break;
        case UART_RX_DISABLED:
            k_wakeup(task_uart_data_proc_id);
            break;
        case UART_RX_RDY:
            uart_rx_disable(uart_dev);
            break;
        case UART_RX_STOPPED:
            break;
    }
}


void task_uart_data_proc(void)
{
    struct parsed_frame_s parsed_frame;
    /* select current command */
    parsed_frame.cmd_ptr = strtok((char*)(&uart_buf_rx), "=");
    /* select command argument */
    parsed_frame.arg = atoi(strtok((char*)(&uart_buf_rx[strlen(parsed_frame.cmd_ptr) + 1]), "."));

    if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_PER)) {
        atomic_set(&atomic_cur_state, atomic_get(&atomic_per_meas_state));
        atomic_set(&atomic_packet_count, 0);
        per_num = parsed_frame.arg;
        k_work_submit(&work_per_meas);
    } else if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_STOP_RECEIVE_SESSION)) {
        atomic_set(&atomic_cur_state, atomic_get(&atomic_idle_state));
    }
    /* Start UART receive */
    uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
}


void work_radio_per_meas_handler(struct k_work *item)
{
    volatile int8_t snr = 0;
    volatile int16_t rssi = 0;
    volatile int32_t ret = 0;
    atomic_val_t per_meas_state = atomic_get(&atomic_per_meas_state);
    struct print_data_elem_s print_data = {0};
    while(1) {
        uint8_t i = 0;
        while (i < per_num) {
            if (atomic_get(&atomic_cur_state) == per_meas_state) {
                sys_rand_get(radio_buf_tx, RADIO_BUF_LEN);
                atomic_inc(&atomic_packet_count);
                lora_cfg.tx = true;
                lora_config(lora_dev, &lora_cfg);
                ret = lora_send(lora_dev, radio_buf_tx, RADIO_BUF_LEN);
                lora_cfg.tx = false;
                lora_config(lora_dev, &lora_cfg);
                ret = lora_recv(lora_dev, radio_buf_rx, RADIO_BUF_LEN, K_SECONDS(RECV_TIMEOUT_SEC), &rssi, &snr);
                if (ret > 0) { /*If receive successful change print_data */
                    if (!strcmp(radio_buf_tx, radio_buf_rx)) {
                        print_data.rssi = rssi;
                        print_data.snr = snr;
                    }
                } else {
                    print_data.rssi = 0;
                    print_data.snr = 0;
                }
                print_data.packet_num = atomic_get(&atomic_packet_count);
                k_msgq_put(&msgq_radio_recv_data, &print_data, K_NO_WAIT);
                i++;
            } else {
                return;
            }
        }
        atomic_set(&atomic_cur_state, atomic_get(&atomic_idle_state));
    }
}


void task_radio_data_recv(void)
{
    int8_t snr = 0;
    int16_t rssi = 0;
    volatile int32_t ret = 0;
    volatile atomic_val_t recv_state = atomic_get(&atomic_recv_state);
    volatile atomic_val_t per_meas_state = atomic_get(&atomic_per_meas_state);
    struct print_data_elem_s print_data = {0};
    k_sleep(K_FOREVER);
    if (atomic_cas(&atomic_cur_state, recv_state, recv_state)) {
        ret = lora_recv(lora_dev, radio_buf_rx, RADIO_BUF_LEN, K_SECONDS(RECV_TIMEOUT_SEC), &rssi, &snr);
//        ret = 1;
        if (ret > 0) {
            lora_cfg.tx = true;
            lora_config(lora_dev, &lora_cfg);
            lora_send(lora_dev, radio_buf_rx, RADIO_BUF_LEN);
            lora_cfg.tx = false;
            lora_config(lora_dev, &lora_cfg);
        }
        k_sleep(K_MSEC(1));
    } else if (atomic_cas(&atomic_cur_state, per_meas_state, per_meas_state)) {
        uint8_t i = 0;
        while (i < per_num) {
            if (atomic_get(&atomic_cur_state) == per_meas_state) {
                sys_rand_get(radio_buf_tx, RADIO_BUF_LEN);
                atomic_inc(&atomic_packet_count);
                lora_cfg.tx = true;
                lora_config(lora_dev, &lora_cfg);
                ret = lora_send(lora_dev, radio_buf_tx, RADIO_BUF_LEN);
                lora_cfg.tx = false;
                lora_config(lora_dev, &lora_cfg);
                ret = lora_recv(lora_dev, radio_buf_rx, RADIO_BUF_LEN, K_SECONDS(RECV_TIMEOUT_SEC), &rssi, &snr);
                if (ret > 0) { /*If receive successful change print_data */
                    if (!strcmp(radio_buf_tx, radio_buf_rx)) {
                        print_data.rssi = rssi;
                        print_data.snr = snr;
                    }
                } else {
                    print_data.rssi = 0;
                    print_data.snr = 0;
                }
                print_data.packet_num = atomic_get(&atomic_packet_count);
                k_msgq_put(&msgq_radio_recv_data, &print_data, K_NO_WAIT);
                i++;
            } else {
                return;
            }
        }
        atomic_set(&atomic_cur_state, atomic_get(&atomic_idle_state));
    }
}


void system_init(void)
{
    int err = 0;

    k_work_init(&work_per_meas, work_radio_per_meas_handler);
//    k_work_init(&work_radio_recv, work_radio_recv_handler);

    k_mutex_init(&mut_radio_busy);

    uart_dev = device_get_binding(CURRENT_UART_DEVICE);
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

    /* Start radio receive */
    k_wakeup(task_radio_data_recv_id);
    /* Start UART receive */
    uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
}


_Noreturn void task_radio_data_proc(void)
{
    int32_t ret = 0;
    struct print_data_elem_s print_data = {0};
    system_init();
    while(1) {
        if (k_msgq_num_used_get(&msgq_radio_recv_data)) {
            ret = k_msgq_get(&msgq_radio_recv_data, &print_data, K_NO_WAIT);
            if (!ret) {
                if (print_data.rssi == 0) { /* If packet_num equal prev_val */
                    /* Wait while UART busy. UART will be released in UART callback */
                    sprintf(uart_buf_tx, "Packet %ld is missing!!!\n", print_data.packet_num);
                    uart_tx(uart_dev, uart_buf_tx, strlen("Packet %d is missing!!!\r"),
                            TIMEOUT);
                } else {
                    /* Wait while UART busy. UART will be released in UART callback */
                    sprintf(uart_buf_tx, "Packet %ld is received\n rssi: %d\n snr: %d\n", print_data.packet_num,
                            print_data.rssi, print_data.snr);
                    uart_tx(uart_dev, uart_buf_tx, strlen("Packet %d is received\n rssi: %d\n snr: %d\n"),
                            TIMEOUT);
                }
            } else {
                continue;
            }
        } else {
            k_sleep(K_MSEC(10));
        }
    }
}

K_THREAD_DEFINE(task_radio_data_proc_id, STACK_SIZE, task_radio_data_proc, NULL, NULL, NULL,
                THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(task_uart_data_proc_id, STACK_SIZE, task_uart_data_proc, NULL, NULL, NULL,
                THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(task_radio_data_recv_id, STACK_SIZE, task_radio_data_recv, NULL, NULL, NULL,
                -2, 0, 0);