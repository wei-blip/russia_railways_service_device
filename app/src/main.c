#include "service_device_common.h"


#include <logging/log.h>
LOG_MODULE_REGISTER(service_device, LOG_LEVEL_DBG);


uint8_t uart_buf_tx[UART_TX_BUF_LEN] = {0};
uint8_t uart_buf_rx[UART_RX_BUF_LEN] = {0};

/**
 * Structs, typedefs and enums area begin
 * */

atomic_t atomic_freq = ATOMIC_INIT(433000000);
atomic_t atomic_sf = ATOMIC_INIT(SF_12);

struct k_work work_uart_data_proc = {0};
struct gpio_callback state_bluetooth_cb;

/**
 * Structs and enums area end
 * */

void state_bluetooth_active_cb (const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
}

void event_cb(const struct device* dev, struct uart_event* evt, void* user_data) {
    switch (evt->type) {
        case UART_TX_ABORTED:
            break;
        case UART_TX_DONE:
            atomic_set(&atomic_uart_tx_done, true);
            break;
        case UART_RX_BUF_RELEASED:
        case UART_RX_BUF_REQUEST:
            break;
        case UART_RX_DISABLED:
            k_work_submit(&work_uart_data_proc);
            break;
        case UART_RX_RDY:
            uart_rx_disable(dev);
            break;
        case UART_RX_STOPPED:
            break;
    }
}


void work_uart_data_proc_handler(struct k_work *item)
{
    struct parsed_frame_s parsed_frame = {0};
    /* Select current command */
    parsed_frame.cmd_ptr = strtok((char*)(&uart_buf_rx), "=");
    /* Select command argument */
    parsed_frame.arg = atoi(strtok((char*)(&uart_buf_rx[strlen(parsed_frame.cmd_ptr) + 1]), "\n"));

    if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_PER)) { /* Set PER measurement */

        atomic_set(&atomic_cur_state, STATE_START_PER_MEAS);
        atomic_set(&atomic_per_num, (atomic_val_t)parsed_frame.arg); /* Set packet num */

    } else if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_STOP_RECEIVE_SESSION)) { /* Stop PER measurement */

        atomic_set(&atomic_cur_state, STATE_IDLE);

    } else if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_GET_CFG)) { /* Get lora modem configuration parameters */

        atomic_set(&atomic_cur_state, STATE_GET_CFG);

    } else if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_INCR_FREQ)) { /* Increment current frequency on 100 kHz */

        atomic_set(&atomic_cur_state, STATE_INCR_FREQ);

    } else if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_DECR_FREQ)) { /* Decrement current frequency on 100 kHz */

        atomic_set(&atomic_cur_state, STATE_DECR_FREQ);

    } else if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_SET_FREQ)) { /* Change frequency value in lora configuration param */

        atomic_set(&atomic_freq, (long)parsed_frame.arg);
        atomic_set(&atomic_cur_state, STATE_SET_FREQ);

    } else if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_SET_SF)) { /* Change SF value in lora configuration param */

        switch (parsed_frame.arg) {
            case 12:
                atomic_set(&atomic_sf, SF_12);
                break;
            case 11:
                atomic_set(&atomic_sf, SF_11);
                break;
            case 10:
                atomic_set(&atomic_sf, SF_10);
                break;
            case 9:
                atomic_set(&atomic_sf, SF_9);
                break;
            case 8:
                atomic_set(&atomic_sf, SF_8);
                break;
            case 7:
                atomic_set(&atomic_sf, SF_7);
                break;
            case 6:
                atomic_set(&atomic_sf, SF_6);
                break;
        }
        atomic_set(&atomic_cur_state, STATE_SET_SF);

    } else {
        atomic_set(&atomic_cur_state, STATE_IDLE);
    }
}


_Noreturn void common_task(void)
{
    int8_t snr = 0;
    int16_t rssi = 0;
    volatile int32_t ret = 0;
    struct print_data_elem_s print_data = {0};

    struct lora_modem_config lora_cfg = {
      .tx = false,
      .frequency = atomic_get(&atomic_freq),
      .bandwidth = BW_125_KHZ,
      .datarate = atomic_get(&atomic_sf),
      .preamble_len = 8,
      .coding_rate = CR_4_5,
      .tx_power = 20
    };
    const struct device *lora_dev;
    const struct device *uart_dev;

    /**
     * Radio initialization area begin
     * */
    /* Init radio */
    lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    if (!device_is_ready(lora_dev)) {
        LOG_DBG("Device not ready: %s", lora_dev->name);
        k_sleep(K_FOREVER);
    }

    ret = lora_config(lora_dev, &lora_cfg);
    if (ret < 0) {
        LOG_DBG("Device not configure: %s", lora_dev->name);
        k_sleep(K_FOREVER);
    }
    /**
    * Radio initialization area end
    * */

    /**
     * UART initialization area begin
     * */
    k_work_init(&work_uart_data_proc, work_uart_data_proc_handler);

    /* Init UART*/
    uart_dev = device_get_binding(CURRENT_UART_DEVICE);
    if (!device_is_ready(uart_dev)) {
        LOG_DBG("Device not ready: %s", uart_dev->name);
        k_sleep(K_FOREVER);
    }
    uart_callback_set(uart_dev, event_cb, NULL);
    /**
    * UART initialization area end
    * */


    while(1) {
        if (atomic_cas(&atomic_cur_state, STATE_RECV, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            lora_recv_async(lora_dev, lora_receive_cb, lora_rx_error_timeout_cb);

        } else if (atomic_cas(&atomic_cur_state, STATE_TRANSMIT, STATE_IDLE)) {

            lora_cfg.tx = true;
            lora_config(lora_dev, &lora_cfg);
            lora_send(lora_dev, radio_buf_tx, RADIO_BUF_LEN);
            atomic_cas(&atomic_cur_state, STATE_TRANSMIT, STATE_RECV);

        } else if (atomic_cas(&atomic_cur_state, STATE_START_PER_MEAS, STATE_PER_MEAS_RUN)) {

            memcpy(uart_buf_tx, "Wait while previous receive complete...\n",
                   strlen("Wait while previous receive complete...\n"));
            uart_tx(uart_dev, uart_buf_tx, strlen("Wait while previous receive complete...\n"), TIMEOUT);
            per_meas(lora_dev, &lora_cfg, uart_dev, uart_buf_tx);
            /* Change currently state on STATE_IDLE if it still equals STATE_PER_MEAS_RUN
             * Else do nothing*/
            atomic_cas(&atomic_cur_state, STATE_PER_MEAS_RUN, STATE_IDLE);

        } else if (atomic_cas(&atomic_cur_state, STATE_GET_CFG, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            print_modem_cfg(uart_dev, uart_buf_tx, &lora_cfg);

        } else if (atomic_cas(&atomic_cur_state, STATE_INCR_FREQ, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            if (!incr_decr_modem_frequency(lora_dev, &lora_cfg, true)) {
                print_modem_cfg(uart_dev, uart_buf_tx, &lora_cfg);
            } else {
                memcpy(uart_buf_tx, "Failed to change frequency!!!\n",
                       strlen("Failed to change frequency!!!\n"));
                uart_tx(uart_dev, uart_buf_tx, strlen(uart_buf_tx), TIMEOUT);
            }

        } else if (atomic_cas(&atomic_cur_state, STATE_DECR_FREQ, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            if (!incr_decr_modem_frequency(lora_dev, &lora_cfg, false)) {
                print_modem_cfg(uart_dev, uart_buf_tx, &lora_cfg);
            } else {
                memcpy(uart_buf_tx, "Failed to change frequency!!!\n",
                       strlen("Failed to change frequency!!!\n"));
                uart_tx(uart_dev, uart_buf_tx, strlen(uart_buf_tx), TIMEOUT);
            }

        } else if (atomic_cas(&atomic_cur_state, STATE_SET_FREQ, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            if (change_modem_frequency(lora_dev, &lora_cfg, atomic_get(&atomic_freq))) {
                print_modem_cfg(uart_dev, uart_buf_tx, &lora_cfg);
            } else {
                memcpy(uart_buf_tx, "Failed to change frequency!!!\n",
                       strlen("Failed to change frequency!!!\n"));
                uart_tx(uart_dev, uart_buf_tx, strlen(uart_buf_tx), TIMEOUT);
            }

        } else if (atomic_cas(&atomic_cur_state, STATE_SET_SF, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            if (change_modem_datarate(lora_dev, &lora_cfg, atomic_get(&atomic_sf))) {
                print_modem_cfg(uart_dev, uart_buf_tx, &lora_cfg);
            } else {
                memcpy(uart_buf_tx, "Failed to change SF value!!!\n",
                       strlen("Failed to change SF value!!!\n"));
                uart_tx(uart_dev, uart_buf_tx, strlen(uart_buf_tx), TIMEOUT);
            }

        } else {
            k_sleep(K_MSEC(10));
        }
    }
}

K_THREAD_DEFINE(common_task_id, STACK_SIZE, common_task, NULL, NULL, NULL,
                -2, 0, 0);