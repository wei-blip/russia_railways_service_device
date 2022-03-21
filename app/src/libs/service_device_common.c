//
// Created by rts on 17.03.2022.
//
#include "service_device_common.h"

uint8_t radio_buf_rx[RADIO_BUF_LEN] = {0};
uint8_t radio_buf_tx[RADIO_BUF_LEN] = {0};

atomic_t atomic_cur_state = ATOMIC_INIT(STATE_RECV);
atomic_t atomic_per_num = ATOMIC_INIT(0);
atomic_t atomic_uart_tx_done = ATOMIC_INIT(true);


static inline void wait_uart(void)
{
    while (!atomic_get(&atomic_uart_tx_done)) {
        k_sleep(K_MSEC(1));
    }
}

void per_meas(const struct device *lora_dev, struct lora_modem_config *lora_cfg,
  const struct device *uart_dev, uint8_t *buf_tx)
{
    int8_t snr = 0;
    int16_t rssi;
    uint32_t i = 0;
    int32_t ret = 0;
    atomic_val_t per_num = atomic_get(&atomic_per_num);
    atomic_t atomic_packet_count = ATOMIC_INIT(0);
    struct print_data_elem_s print_data = {0};

    sprintf(buf_tx, "Start PER measurement...\n");
    send_to_terminal(uart_dev, buf_tx);

    /* If receive is running then stop it */
    lora_recv_async(lora_dev, NULL, NULL);

    while (i < per_num) {
        if (atomic_cas(&atomic_cur_state, STATE_PER_MEAS_RUN, STATE_PER_MEAS_RUN)) {
            atomic_inc(&atomic_packet_count);
            sys_rand_get(radio_buf_tx, RADIO_BUF_LEN);

            lora_cfg->tx = true;
            lora_config(lora_dev, lora_cfg);
            k_sleep(K_MSEC(500));
            lora_send(lora_dev, radio_buf_tx, RADIO_BUF_LEN);
            lora_cfg->tx = false;
            lora_config(lora_dev, lora_cfg);

            ret = lora_recv(lora_dev, radio_buf_rx, RADIO_BUF_LEN,
                            K_SECONDS(RECV_TIMEOUT_SEC), &rssi, &snr);

            print_data.packet_num = atomic_get(&atomic_packet_count);
            print_data.rssi = rssi;
            print_data.snr = snr;

            print_per_status(uart_dev, buf_tx, ret, &print_data);
            i++;
        } else {
            break;
        }
    }
}

void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr)
{
    memcpy(data, radio_buf_tx, size);
    lora_recv_async(dev, NULL, NULL);
    atomic_set(&atomic_cur_state, STATE_TRANSMIT);
}

void lora_rx_error_timeout_cb(void)
{

}

void change_modem_datarate(const struct device *lora_dev, struct lora_modem_config *lora_cfg, enum lora_datarate new_dr,
  const struct device *uart_dev, uint8_t *buf_tx)
{
    lora_recv_async(lora_dev, NULL, NULL);
    lora_cfg->datarate = new_dr;
    if (!lora_config(lora_dev, lora_cfg)) {
        print_modem_cfg(uart_dev, lora_cfg);
    } else {
        send_to_terminal(uart_dev, "Failed to change modem datarate!!!\n");
    }
}


void change_modem_frequency(const struct device *lora_dev, struct lora_modem_config *lora_cfg, uint32_t new_freq_khz,
  const struct device *uart_dev, uint8_t *buf_tx)
{
    lora_recv_async(lora_dev, NULL, NULL);
    lora_cfg->frequency = (new_freq_khz*1000);
    if (!lora_config(lora_dev, lora_cfg)) {
        print_modem_cfg(uart_dev, lora_cfg);
    } else {
        send_to_terminal(uart_dev, "Failed to change modem frequency!!!\n");
    }
}


void incr_decr_modem_frequency(const struct device *lora_dev, struct lora_modem_config *lora_cfg, bool incr,
  const struct device *uart_dev, uint8_t *buf_tx)
{
    lora_recv_async(lora_dev, NULL, NULL);
    if (incr) {
        lora_cfg->frequency += FREQUENCY_STEP_HZ;
    } else {
        lora_cfg->frequency -= FREQUENCY_STEP_HZ;
    }

    if (!lora_config(lora_dev, lora_cfg)) {
        print_modem_cfg(uart_dev, lora_cfg);
    } else {
        send_to_terminal(uart_dev, "Failed to change modem frequency!!!\n");
    }
}


void print_modem_cfg(const struct device *dev, struct lora_modem_config *cfg)
{
    static uint8_t cfg_buf[UART_TX_BUF_LEN] = {0};

    sprintf(cfg_buf, "Current modem configuration:\n");
    send_to_terminal(dev, cfg_buf);

    sprintf(cfg_buf, "Frequency: %lu kHz\n", (cfg->frequency)/1000);
    send_to_terminal(dev, cfg_buf);

    switch (cfg->bandwidth) {
        case BW_125_KHZ:
            sprintf(cfg_buf, "Bandwidth: %u kHz\n", 125);
            break;
        case BW_250_KHZ:
            sprintf(cfg_buf, "Bandwidth: %u kHz\n", 250);
            break;
        case BW_500_KHZ:
            sprintf(cfg_buf, "Bandwidth: %u kHz\n", 500);
            break;
    }
    send_to_terminal(dev, cfg_buf);

    switch (cfg->datarate) {
        case SF_6:
            sprintf(cfg_buf, "Datarate: SF_6\n");
            break;
        case SF_7:
            sprintf(cfg_buf, "Datarate: SF_7\n");
            break;
        case SF_8:
            sprintf(cfg_buf, "Datarate: SF_8\n");
            break;
        case SF_9:
            sprintf(cfg_buf, "Datarate: SF_9\n");
            break;
        case SF_10:
            sprintf(cfg_buf, "Datarate: SF_10\n");
            break;
        case SF_11:
            sprintf(cfg_buf, "Datarate: SF_11\n");
            break;
        case SF_12:
            sprintf(cfg_buf, "Datarate: SF_12\n");
            break;
    }
    send_to_terminal(dev, cfg_buf);

    switch (cfg->coding_rate) {
        case CR_4_5:
            sprintf(cfg_buf, "Coding rate: 4/5\n");
            break;
        case CR_4_6:
            sprintf(cfg_buf, "Coding rate: 4/6\n");
            break;
        case CR_4_7:
            sprintf(cfg_buf, "Coding rate: 4/7\n");
            break;
        case CR_4_8:
            sprintf(cfg_buf, "Coding rate: 4/8\n");
            break;
    }
    send_to_terminal(dev, cfg_buf);

    sprintf(cfg_buf, "Preamble length: %u symb\n", cfg->preamble_len);
    send_to_terminal(dev, cfg_buf);

    if (cfg->fixed_len) {
        sprintf(cfg_buf, "Fixed length: true\n");
        send_to_terminal(dev, cfg_buf);

        sprintf(cfg_buf, "Payload length: %u\n", cfg->payload_len);
        send_to_terminal(dev, cfg_buf);
    } else {
        sprintf(cfg_buf, "Fixed length: false\n");
        send_to_terminal(dev, cfg_buf);
    }

    sprintf(cfg_buf, "TX power: %d dBm\n", cfg->tx_power);
    send_to_terminal(dev, cfg_buf);
}


void print_per_status(const struct device *uart_dev, uint8_t *buf_tx, int ret, struct print_data_elem_s *print_data)
{
    static uint8_t per_buf[UART_TX_BUF_LEN] = {0};
    if (ret) {
        sprintf(per_buf, "Packet %lu/%lu is missing!!!\n", print_data->packet_num, atomic_per_num);
        send_to_terminal(uart_dev, per_buf);
    } else {
        sprintf(per_buf, "Packet %lu/%lu is received\n rssi: %d\n snr: %d\n", print_data->packet_num,
                atomic_per_num, print_data->rssi, print_data->snr);
        send_to_terminal(uart_dev, per_buf);
    }
}


void stop_session(const struct device *lora_dev, const struct device *uart_dev, uint8_t *buf_tx) {
    lora_recv_async(lora_dev, NULL, NULL);
    send_to_terminal(uart_dev, "Session stopped, modem released.\n");
}
