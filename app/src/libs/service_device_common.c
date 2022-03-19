//
// Created by rts on 17.03.2022.
//
#include "service_device_common.h"

uint8_t radio_buf_rx[RADIO_BUF_LEN] = {0};
uint8_t radio_buf_tx[RADIO_BUF_LEN] = {0};

atomic_t atomic_cur_state = ATOMIC_INIT(STATE_RECV);
atomic_t atomic_per_num = ATOMIC_INIT(0);
atomic_t atomic_uart_tx_done = ATOMIC_INIT(true);

struct gpio_dt_spec state_bluetooth = GPIO_DT_SPEC_GET_OR(STATE_NODE, gpios, {0});

static inline bool uart_sends(void)
{
    return atomic_get(&atomic_uart_tx_done);
}

static inline void send_to_terminal(const struct device *dev, uint8_t *buf_tx)
{
    atomic_set(&atomic_uart_tx_done, false);
    uart_tx(dev, buf_tx, strlen(buf_tx), TX_TIMEOUT_US);
    while (!atomic_get(&atomic_uart_tx_done)) {
        k_sleep(K_MSEC(10));
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

int change_modem_datarate(const struct device *dev, struct lora_modem_config *cfg, enum lora_datarate new_dr)
{
    cfg->datarate = new_dr;
    return lora_config(dev, cfg);
}


int change_modem_frequency(const struct device *dev, struct lora_modem_config *cfg, uint32_t new_freq_khz)
{
    cfg->frequency = (new_freq_khz*1000);
    return lora_config(dev, cfg);
}


void incr_decr_modem_frequency(const struct device *lora_dev, struct lora_modem_config *lora_cfg, bool incr,
  const struct device *uart_dev, uint8_t buf_tx)
{
    if (incr) {
        cfg->frequency = cfg->frequency + FREQUENCY_STEP_HZ;
    } else {
        cfg->frequency = cfg->frequency - FREQUENCY_STEP_HZ;
    }
    return lora_config(dev, cfg);
}


void print_modem_cfg(const struct device *dev, uint8_t *buf_tx, struct lora_modem_config *cfg)
{
    sprintf(buf_tx, "Current modem configuration:\n");
    send_to_terminal(dev, buf_tx);

    sprintf(buf_tx, "Frequency: %lu \n", cfg->frequency);
    send_to_terminal(dev, buf_tx);

    switch (cfg->bandwidth) {
        case BW_125_KHZ:
            sprintf(buf_tx, "Bandwidth: %u kHz\n", 125);
            break;
        case BW_250_KHZ:
            sprintf(buf_tx, "Bandwidth: %u kHz\n", 250);
            break;
        case BW_500_KHZ:
            sprintf(buf_tx, "Bandwidth: %u kHz\n", 500);
            break;
    }
    send_to_terminal(dev, buf_tx);

    switch (cfg->datarate) {
        case SF_6:
            sprintf(buf_tx, "Datarate: SF_6\n");
            break;
        case SF_7:
            sprintf(buf_tx, "Datarate: SF_7\n");
            break;
        case SF_8:
            sprintf(buf_tx, "Datarate: SF_8\n");
            break;
        case SF_9:
            sprintf(buf_tx, "Datarate: SF_9\n");
            break;
        case SF_10:
            sprintf(buf_tx, "Datarate: SF_10\n");
            break;
        case SF_11:
            sprintf(buf_tx, "Datarate: SF_11\n");
            break;
        case SF_12:
            sprintf(buf_tx, "Datarate: SF_12\n");
            break;
    }
    send_to_terminal(dev, buf_tx);

    switch (cfg->coding_rate) {
        case CR_4_5:
            sprintf(buf_tx, "Coding rate: 4/5\n");
            break;
        case CR_4_6:
            sprintf(buf_tx, "Coding rate: 4/6\n");
            break;
        case CR_4_7:
            sprintf(buf_tx, "Coding rate: 4/7\n");
            break;
        case CR_4_8:
            sprintf(buf_tx, "Coding rate: 4/8\n");
            break;
    }
    send_to_terminal(dev, buf_tx);

    sprintf(buf_tx, "Preamble length: %u\n", cfg->preamble_len);
    send_to_terminal(dev, buf_tx);

    if (cfg->fixed_len) {
        sprintf(buf_tx, "Fixed length: true\n");
        send_to_terminal(dev, buf_tx);

        sprintf(buf_tx, "Payload length: %u\n", cfg->payload_len);
        send_to_terminal(dev, buf_tx);
    } else {
        sprintf(buf_tx, "Fixed length: false\n");
        send_to_terminal(dev, buf_tx);
    }

    sprintf(buf_tx, "TX power: %d dBm\n", cfg->tx_power);
    send_to_terminal(dev, buf_tx);
}


void print_per_status(const struct device *uart_dev, uint8_t *buf_tx, int ret, struct print_data_elem_s *print_data)
{

    if (ret) {
        sprintf(buf_tx, "Packet %lu/%lu is missing!!!\n", print_data->packet_num, atomic_per_num);
        send_to_terminal(uart_dev, buf_tx);
    } else {
        sprintf(buf_tx, "Packet %lu/%lu is received\n rssi: %d\n snr: %d\n", print_data->packet_num,
                atomic_per_num, print_data->rssi, print_data->snr);
        send_to_terminal(uart_dev, buf_tx);
    }
}
