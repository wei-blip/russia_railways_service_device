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

/**
 * Structs and enums area end
 * */

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
    static const struct device* uart_dev;
    struct parsed_frame_s parsed_frame = {0};
    /* Select current command */
    parsed_frame.cmd_ptr = strtok((char*)(&uart_buf_rx), "=");
    /* Select command argument */
    parsed_frame.arg = atoi(strtok((char*)(&uart_buf_rx[strlen(parsed_frame.cmd_ptr) + 1]), "\n"));

    if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_PER)) { /* Set PER measurement */

        atomic_set(&atomic_cur_state, STATE_START_PER_MEAS);
        atomic_set(&atomic_per_num, (atomic_val_t)parsed_frame.arg); /* Set packet num */

    } else if (!strcmp(parsed_frame.cmd_ptr, COMMAND_TYPE_STOP_RECEIVE_SESSION)) { /* Stop receive */

        atomic_set(&atomic_cur_state, STATE_STOP);
        uart_dev = device_get_binding(CURRENT_UART_DEVICE);
        send_to_terminal(uart_dev, "Wait while started transaction will be ended\n");

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
            case DR_SF_12:
                atomic_set(&atomic_sf, SF_12);
                break;
            case DR_SF_11:
                atomic_set(&atomic_sf, SF_11);
                break;
            case DR_SF_10:
                atomic_set(&atomic_sf, SF_10);
                break;
            case DR_SF_9:
                atomic_set(&atomic_sf, SF_9);
                break;
            case DR_SF_8:
                atomic_set(&atomic_sf, SF_8);
                break;
            case DR_SF_7:
                atomic_set(&atomic_sf, SF_7);
                break;
            case DR_SF_6:
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
    volatile int32_t ret = 0;
    struct print_data_elem_s print_data = {0};

    struct lora_modem_config lora_cfg = {
      .tx = false,
      .frequency = atomic_get(&atomic_freq),
      .bandwidth = BW_125_KHZ,
      .datarate = atomic_get(&atomic_sf),
      .preamble_len = 8,
      .coding_rate = CR_4_5,
      .tx_power = 0
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
        if (atomic_cas(&atomic_cur_state, STATE_RECV, STATE_IDLE)) { /* This is start state */

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            lora_recv_async(lora_dev, lora_receive_cb, lora_rx_error_timeout_cb);

        } else if (atomic_cas(&atomic_cur_state, STATE_TRANSMIT, STATE_IDLE)) {

            /* Receive stopped into callback function */
            lora_cfg.tx = true;
            lora_config(lora_dev, &lora_cfg);
            lora_send(lora_dev, radio_buf_tx, RADIO_BUF_LEN);
            lora_recv_async(lora_dev, lora_receive_cb, lora_rx_error_timeout_cb); /* Restart receive */
            atomic_cas(&atomic_cur_state, STATE_TRANSMIT, STATE_RECV);

        } else if (atomic_cas(&atomic_cur_state, STATE_START_PER_MEAS, STATE_PER_MEAS_RUN)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            per_meas(lora_dev, &lora_cfg, uart_dev, uart_buf_tx);
            /* Change currently state on STATE_IDLE if it still equals STATE_PER_MEAS_RUN
             * Else do nothing*/
            atomic_cas(&atomic_cur_state, STATE_PER_MEAS_RUN, STATE_IDLE);

        } else if (atomic_cas(&atomic_cur_state, STATE_GET_CFG, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            print_modem_cfg(uart_dev, &lora_cfg);

        } else if (atomic_cas(&atomic_cur_state, STATE_INCR_FREQ, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            incr_decr_modem_frequency(lora_dev, &lora_cfg, true, uart_dev, uart_buf_tx);

        } else if (atomic_cas(&atomic_cur_state, STATE_DECR_FREQ, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            incr_decr_modem_frequency(lora_dev, &lora_cfg, false, uart_dev, uart_buf_tx);

        } else if (atomic_cas(&atomic_cur_state, STATE_SET_FREQ, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            change_modem_frequency(lora_dev, &lora_cfg, atomic_get(&atomic_freq), uart_dev, uart_buf_tx);

        } else if (atomic_cas(&atomic_cur_state, STATE_SET_SF, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            change_modem_datarate(lora_dev, &lora_cfg, atomic_get(&atomic_sf), uart_dev, uart_buf_tx);

        } else if (atomic_cas(&atomic_cur_state, STATE_STOP, STATE_IDLE)) {

            /* Start UART receive */
            uart_rx_enable(uart_dev, uart_buf_rx, UART_RX_BUF_LEN, TIMEOUT);
            stop_session(lora_dev, uart_dev, uart_buf_tx);

        } else {
            k_sleep(K_MSEC(10));
        }
    }
}

K_THREAD_DEFINE(common_task_id, STACK_SIZE, common_task, NULL, NULL, NULL,
                0, 0, 0);