/*
 *  nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 *
 */
#include "nrf24l01p.h"

static void cs_high()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

static void cs_low()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

void ce_high()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

void ce_low()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}

static uint8_t read_register(uint8_t reg)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint8_t status;
    uint8_t read_val;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, &read_val, 1, 2000);
    cs_high();

    return read_val;
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t status;
    uint8_t write_val = value;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, &write_val, 1, 2000);
    cs_high();

    return write_val;
}

/* nRF24L01+ Main Functions */
void nrf24l01p_init(channel MHz, air_data_rate bps)
{
    // Power down first for clean initialization
    ce_low();
    nrf24l01p_power_down();
    HAL_Delay(100);  // Allow chip to settle

    // Reset all constant settings
    nrf24l01p_reset();

    // Apply parameter-based settings:
    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);

    // Power up the chip, wait for oscillator to settle
    nrf24l01p_power_up();
    HAL_Delay(2);
}

void nRF24L01_Set_Rx_Addr(uint8_t addr[5], uint8_t Pipe)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | Pipe;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, addr, 5, 2000);
    cs_high();
}

void nRF24L01_Set_Tx_Addr(uint8_t addr[5])
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_TX_ADDR;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, addr, 5, 2000);
    cs_high();
}

void read_tx_address_register(uint8_t *buffer, uint8_t length)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | NRF24L01P_REG_TX_ADDR;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, buffer, length, 2000); // Fill the buffer
    cs_high();
}

void read_rx_address_register(uint8_t pipe, uint8_t *buffer, uint8_t length)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | pipe;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, buffer, length, 2000); // Fill the buffer
    cs_high();
}

void nrf24l01p_rx_receive(uint8_t *rx_payload)
{
    nrf24l01p_read_rx_fifo(rx_payload);
    // nrf24l01p_clear_rx_dr();
}

void nrf24l01p_tx_transmit(uint8_t *tx_payload)
{
    // Enhanced transmit function with proper timing
    ce_low();
    
    // Ensure TX FIFO is empty
    nrf24l01p_flush_tx_fifo();
    
    // Write payload
    nrf24l01p_write_tx_fifo(tx_payload);
    
    // Pulse CE for transmission
    ce_high();
    HAL_Delay(1);  // Minimum 10µs high pulse
    ce_low();
}

void nrf24l01p_tx_irq()
{
    ce_low();

    nrf24l01p_clear_tx_ds();
    nrf24l01p_clear_max_rt();

    ce_high();
}

/* nRF24L01+ Sub Functions */
void nrf24l01p_reset()
{
    // Reset pins
    cs_high();
    ce_low();

    // Set base CONFIG register with CRC length 1 (bit2 = 0)
    write_register(NRF24L01P_REG_CONFIG, 0x08);

    // Enable auto acknowledgement on all pipes (previously was disabled with 0x00)
    write_register(NRF24L01P_REG_EN_AA, 0x3F);
    
    // Enable RX pipes 0 and 1 (keep as before or adjust as needed)
    write_register(NRF24L01P_REG_EN_RXADDR, 0x03);

    // Set address width to 5 bytes (5-2 = 3)
    write_register(NRF24L01P_REG_SETUP_AW, 0x03);

    // Set auto retransmit: (constant; e.g., 0x03)
    write_register(NRF24L01P_REG_SETUP_RETR, 0x03);

    // Do not set RF_CH here as it is parameter-based in init

    // Set base RF setup (TX output power constant _0dBm remains, air data rate will be applied later)
    write_register(NRF24L01P_REG_RF_SETUP, 0x06);

    // Clear interrupts in STATUS register
    write_register(NRF24L01P_REG_STATUS, 0x7E);

    // Set payload width for pipes 0 and 1 to the constant value
    write_register(NRF24L01P_REG_RX_PW_P0, NRF24L01P_PAYLOAD_LENGTH);
    write_register(NRF24L01P_REG_RX_PW_P1, NRF24L01P_PAYLOAD_LENGTH);

    // Clear payload widths for pipes 2 to 5
    write_register(NRF24L01P_REG_RX_PW_P2, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P3, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P4, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P5, 0x00);

    // Set FIFO status and disable dynamic payload and features
    write_register(NRF24L01P_REG_FIFO_STATUS, 0x11);
    write_register(NRF24L01P_REG_DYNPD, 0x00);
    write_register(NRF24L01P_REG_FEATURE, 0x00);

    nrf24l01p_flush_rx_fifo();
    nrf24l01p_flush_tx_fifo();
}

void nrf24l01p_read_all_registers(void)
{
    // Read CONFIG register
    uint8_t config = read_register(NRF24L01P_REG_CONFIG);
    // Set a breakpoint here to inspect the value of 'config'

    // Read EN_AA register
    uint8_t en_aa = read_register(NRF24L01P_REG_EN_AA);
    // Set a breakpoint here to inspect the value of 'en_aa'

    // Read EN_RXADDR register
    uint8_t en_rxaddr = read_register(NRF24L01P_REG_EN_RXADDR);
    // Set a breakpoint here to inspect the value of 'en_rxaddr'

    // Read SETUP_AW register
    uint8_t setup_aw = read_register(NRF24L01P_REG_SETUP_AW);
    // Set a breakpoint here to inspect the value of 'setup_aw'

    // Read SETUP_RETR register
    uint8_t setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);
    // Set a breakpoint here to inspect the value of 'setup_retr'

    // Read RF_CH register
    uint8_t rf_ch = read_register(NRF24L01P_REG_RF_CH);
    // Set a breakpoint here to inspect the value of 'rf_ch'

    // Read RF_SETUP register
    uint8_t rf_setup = read_register(NRF24L01P_REG_RF_SETUP);
    // Set a breakpoint here to inspect the value of 'rf_setup'

    // Read STATUS register
    uint8_t status = read_register(NRF24L01P_REG_STATUS);
    // Set a breakpoint here to inspect the value of 'status'

    // Read RX_PW_P0 register
    uint8_t rx_pw_p0 = read_register(NRF24L01P_REG_RX_PW_P0);
    // Set a breakpoint here to inspect the value of 'rx_pw_p0'

    // Read RX_PW_P1 register
    uint8_t rx_pw_p1 = read_register(NRF24L01P_REG_RX_PW_P1);
    // Set a breakpoint here to inspect the value of 'rx_pw_p1'

    // Read RX_PW_P2 register
    uint8_t rx_pw_p2 = read_register(NRF24L01P_REG_RX_PW_P2);
    // Set a breakpoint here to inspect the value of 'rx_pw_p2'

    // Read RX_PW_P3 register
    uint8_t rx_pw_p3 = read_register(NRF24L01P_REG_RX_PW_P3);
    // Set a breakpoint here to inspect the value of 'rx_pw_p3'

    // Read RX_PW_P4 register
    uint8_t rx_pw_p4 = read_register(NRF24L01P_REG_RX_PW_P4);
    // Set a breakpoint here to inspect the value of 'rx_pw_p4'

    // Read RX_PW_P5 register
    uint8_t rx_pw_p5 = read_register(NRF24L01P_REG_RX_PW_P5);
    // Set a breakpoint here to inspect the value of 'rx_pw_p5'

    // Read FIFO_STATUS register
    uint8_t fifo_status = read_register(NRF24L01P_REG_FIFO_STATUS);
    // Set a breakpoint here to inspect the value of 'fifo_status'

    // Read DYNPD register
    uint8_t dynpd = read_register(NRF24L01P_REG_DYNPD);
    // Set a breakpoint here to inspect the value of 'dynpd'

    // Read FEATURE register
    uint8_t feature = read_register(NRF24L01P_REG_FEATURE);
    // Set a breakpoint here to inspect the value of 'feature'

    // Read TX address
    uint8_t tx_address[5] = {0}; // Maximum address width is 5
    read_tx_address_register(tx_address, 5);
    // Set a breakpoint here to inspect 'tx_address'

    // Read RX address for pipe 0
    uint8_t rx_address_p0[5] = {0}; // Maximum address width is 5
    read_rx_address_register(NRF24L01P_REG_RX_ADDR_P0, rx_address_p0, 5);
    // Set a breakpoint here to inspect 'rx_address_p0'

    // Read RX address for pipe 1
    uint8_t rx_address_p1[5] = {0}; // Maximum address width is 5
    read_rx_address_register(NRF24L01P_REG_RX_ADDR_P1, rx_address_p1, 5);

    int res = 0;
}

void nrf24l01p_prx_mode()
{
    ce_low();
    // Set PRIM_RX bit
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 0;
    write_register(NRF24L01P_REG_CONFIG, new_config);
    HAL_Delay(2); // Allow mode switch to settle
    ce_high();
}

void nrf24l01p_ptx_mode()
{
    ce_low();
    // Clear PRIM_RX bit
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;
    write_register(NRF24L01P_REG_CONFIG, new_config);
    HAL_Delay(2); // Allow mode switch to settle
    ce_high();
}

uint8_t nrf24l01p_read_rx_fifo(uint8_t *rx_payload)
{
    uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, rx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high();

    return status;
}

uint8_t nrf24l01p_write_tx_fifo(uint8_t *tx_payload)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high();

    ce_high();

    return status;
}

void nrf24l01p_flush_rx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_RX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

void nrf24l01p_flush_tx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_TX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

uint8_t nrf24l01p_get_status()
{
    uint8_t command = NRF24L01P_CMD_NOP;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();

    return status;
}

uint8_t nrf24l01p_get_fifo_status()
{
    return read_register(NRF24L01P_REG_FIFO_STATUS);
}

void nrf24l01p_rx_set_payload_widths(uint8_t bytes, uint8_t Pipe)
{
    write_register(Pipe, bytes);
}

void nrf24l01p_clear_rx_dr()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x40;

    write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_tx_ds()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x20;

    write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_max_rt()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x10;

    write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_power_up()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 1;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFD;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_crc_length(length bytes)
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);

    switch (bytes)
    {
    // CRCO bit in CONFIG resiger set 0
    case 1:
        new_config &= 0xFB;
        break;
    // CRCO bit in CONFIG resiger set 1
    case 2:
        new_config |= 1 << 2;
        break;
    }

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_address_widths(widths bytes)
{
    write_register(NRF24L01P_REG_SETUP_AW, bytes - 2);
}

void nrf24l01p_auto_retransmit_count(count cnt)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);

    // Reset ARC register 0
    new_setup_retr &= 0xF0;
    new_setup_retr |= cnt;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_auto_retransmit_delay(delay us)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);

    // Reset ARD register 0
    new_setup_retr &= 0x0F;
    new_setup_retr |= ((us / 250) - 1) << 4;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_set_rf_channel(channel MHz)
{
    uint16_t new_rf_ch = MHz - 2400;
    write_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

void nrf24l01p_set_rf_tx_output_power(output_power dBm)
{
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (dBm << 1);

    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_set_rf_air_data_rate(air_data_rate bps)
{
    // Set value to 0
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xD7;

    switch (bps)
    {
    case _1Mbps:
        break;
    case _2Mbps:
        new_rf_setup |= 1 << 3;
        break;
    case _250kbps:
        new_rf_setup |= 1 << 5;
        break;
    }
    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

uint8_t nrf24l01p_is_initialized()
{
    // Read the CONFIG register
    uint8_t config_value = read_register(NRF24L01P_REG_CONFIG);

    // Check if the PWR_UP bit is set
    return (config_value & NRF24L01P_PWR_UP_BIT) != 0;
}