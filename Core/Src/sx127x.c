#include "sx127x.h"
#include "stm32f4xx_hal.h"

#define SX127X_SPI_TIMEOUT_MS 100u
#define SX127X_TX_TIMEOUT_MS  2000u

static SPI_HandleTypeDef *spi;

uint8_t SX127X_ReadRegister(uint8_t reg) {
  uint8_t txData = reg & 0x7F;
  uint8_t rxData = 0;

  HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_RESET);
  (void)HAL_SPI_Transmit(spi, &txData, 1, SX127X_SPI_TIMEOUT_MS);
  (void)HAL_SPI_Receive(spi, &rxData, 1, SX127X_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_SET);

  return rxData;
}

void SX127X_WriteRegister(uint8_t reg, uint8_t data) {
  uint8_t txData[2] = {reg | 0x80, data};

  HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_RESET);
  (void)HAL_SPI_Transmit(spi, txData, 2, SX127X_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_SET);
}

static void SX127X_Reset(void) {
  HAL_GPIO_WritePin(SX127X_RESET_PORT, SX127X_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(SX127X_RESET_PORT, SX127X_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(10);
}

bool SX127X_Init(SPI_HandleTypeDef *spi_port) {
  if (spi_port == NULL) {
    return false;
  }

  spi = spi_port;

  SX127X_Reset();
  HAL_Delay(10);

  // Check version
  if (SX127X_ReadRegister(REG_VERSION) != SX127X_VERSION) {
    return false;
  }
  
  for (int i = 0; i < 3; i++) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    HAL_Delay(100);
  }
  HAL_Delay(1000);

  // Initialize LoRa mode
  SX127X_Sleep(); // Switch to sleep mode to set LoRa mode
  SX127X_WriteRegister(REG_OP_MODE, 0x80); // Set LoRa mode

  // Configure default settings
  SX127X_SetModeStandby();
  SX127X_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0);
  SX127X_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0);
  SX127X_WriteRegister(REG_LNA, 0x23);            // High gain LNA
  SX127X_WriteRegister(REG_MODEM_CONFIG_3, 0x04); // AGC auto on
  SX127X_WriteRegister(REG_PA_RAMP, 0x08); // Low datarate optimize off, PA ramp-up time 50 μs
  
  SX127X_SetTxPower(17, false); // Set PA boost
  SX127X_SetFrequency(915.0);   // Default to 915 MHz

  // Configure for maximum reliability
  SX127X_SetSpreadingFactor(7);
  SX127X_SetSignalBandwidth(125E3);
  SX127X_SetCodingRate4(5);
  SX127X_SetPreambleLength(8);

  // Set frequency to match transmitter (commonly 915MHz for this band)
  SX127X_SetFrequency(915.0);

  // Configure radio parameters
  SX127X_SetSpreadingFactor(7);
  SX127X_SetSignalBandwidth(125E3);
  SX127X_SetCodingRate4(5);
  SX127X_SetPreambleLength(8);

  // Start receiving
  SX127X_SetModeRx();

  return true;
}

void SX127X_SetFrequency(float freq) {
  uint32_t frf = (freq * 1000000.0) / 61.03515625;
  SX127X_WriteRegister(REG_FRF_MSB, (frf >> 16) & 0xFF);
  SX127X_WriteRegister(REG_FRF_MID, (frf >> 8) & 0xFF);
  SX127X_WriteRegister(REG_FRF_LSB, frf & 0xFF);
}

void SX127X_SetTxPower(int8_t power, bool useRFO) {
  if (useRFO) {
    if (power > 14)
      power = 14;
    if (power < -1)
      power = -1;
    SX127X_WriteRegister(REG_PA_CONFIG, 0x70 | (power + 1));
  } else {
    if (power > 20)
      power = 20;
    if (power < 2)
      power = 2;
    SX127X_WriteRegister(REG_PA_CONFIG, PA_BOOST | (power - 2));
  }
}

void SX127X_SetSpreadingFactor(uint8_t sf) {
  if (sf < 6)
    sf = 6;
  else if (sf > 12)
    sf = 12;

  if (sf == 6) {
    SX127X_WriteRegister(REG_DETECTION_OPTIMIZE, 0xC5);
    SX127X_WriteRegister(REG_DETECTION_THRESHOLD, 0x0C);
  } else {
    SX127X_WriteRegister(REG_DETECTION_OPTIMIZE, 0xC3);
    SX127X_WriteRegister(REG_DETECTION_THRESHOLD, 0x0A);
  }

  SX127X_WriteRegister(REG_MODEM_CONFIG_2,
                       (SX127X_ReadRegister(REG_MODEM_CONFIG_2) & 0x0F) |
                           ((sf << 4) & 0xF0));
}

void SX127X_SetSignalBandwidth(uint32_t sbw) {
  uint8_t bw;

  if (sbw <= 7.8E3)
    bw = 0;
  else if (sbw <= 10.4E3)
    bw = 1;
  else if (sbw <= 15.6E3)
    bw = 2;
  else if (sbw <= 20.8E3)
    bw = 3;
  else if (sbw <= 31.25E3)
    bw = 4;
  else if (sbw <= 41.7E3)
    bw = 5;
  else if (sbw <= 62.5E3)
    bw = 6;
  else if (sbw <= 125E3)
    bw = 7;
  else if (sbw <= 250E3)
    bw = 8;
  else
    bw = 9;

  SX127X_WriteRegister(REG_MODEM_CONFIG_1,
                       (SX127X_ReadRegister(REG_MODEM_CONFIG_1) & 0x0F) |
                           (bw << 4));
}

void SX127X_SetCodingRate4(uint8_t denominator) {
  if (denominator < 5)
    denominator = 5;
  else if (denominator > 8)
    denominator = 8;

  uint8_t cr = denominator - 4;
  SX127X_WriteRegister(REG_MODEM_CONFIG_1,
                       (SX127X_ReadRegister(REG_MODEM_CONFIG_1) & 0xF1) |
                           (cr << 1));
}

void SX127X_SetPreambleLength(uint16_t length) {
  SX127X_WriteRegister(REG_PREAMBLE_MSB, (length >> 8) & 0xFF);
  SX127X_WriteRegister(REG_PREAMBLE_LSB, length & 0xFF);
}

bool SX127X_Send(uint8_t *buf, uint8_t len) {
  if (spi == NULL || buf == NULL || len == 0u) {
    return false;
  }

  SX127X_SetModeStandby();
  SX127X_WriteRegister(REG_FIFO_ADDR_PTR, 0);

  // Write data to FIFO
  HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_RESET);
  uint8_t reg = REG_FIFO | 0x80;
  if (HAL_SPI_Transmit(spi, &reg, 1, SX127X_SPI_TIMEOUT_MS) != HAL_OK ||
      HAL_SPI_Transmit(spi, buf, len, SX127X_SPI_TIMEOUT_MS) != HAL_OK) {
    HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_SET);
    SX127X_SetModeRx();
    return false;
  }
  HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_SET);

  // Set payload length
  SX127X_WriteRegister(REG_PAYLOAD_LENGTH, len);

  // Start transmission
  SX127X_SetModeTx();

  // Wait for TX done
  const uint32_t tx_started = HAL_GetTick();
  while ((SX127X_ReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0u) {
    if ((HAL_GetTick() - tx_started) >= SX127X_TX_TIMEOUT_MS) {
      SX127X_SetModeRx();
      return false;
    }
    HAL_Delay(1);
  }

  // Clear IRQ flags
  SX127X_WriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  SX127X_SetModeRx();
  return true;
}

bool SX127X_Receive(uint8_t *buf, uint8_t capacity, uint8_t *len) {
  if (spi == NULL || buf == NULL || len == NULL || capacity == 0u) {
    return false;
  }

  SX127X_SetModeRx();

  if ((SX127X_ReadRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) == 0) {
    return false;
  }

  // Check for CRC error
  if (SX127X_ReadRegister(REG_IRQ_FLAGS) & IRQ_PAYLOAD_CRC_ERROR_MASK) {
    SX127X_WriteRegister(REG_IRQ_FLAGS, IRQ_PAYLOAD_CRC_ERROR_MASK);
    return false;
  }

  // Get packet length
  const uint8_t packet_length = SX127X_ReadRegister(REG_RX_NB_BYTES);
  *len = packet_length;

  if (packet_length > capacity) {
    SX127X_WriteRegister(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);
    return false;
  }

  // Reset FIFO pointer
  SX127X_WriteRegister(REG_FIFO_ADDR_PTR,
                       SX127X_ReadRegister(REG_FIFO_RX_CURRENT_ADDR));

  // Read packet
  HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_RESET);
  uint8_t reg = REG_FIFO & 0x7F;
  if (HAL_SPI_Transmit(spi, &reg, 1, SX127X_SPI_TIMEOUT_MS) != HAL_OK ||
      HAL_SPI_Receive(spi, buf, packet_length, SX127X_SPI_TIMEOUT_MS) != HAL_OK) {
    HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_SET);
    SX127X_WriteRegister(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);
    return false;
  }
  HAL_GPIO_WritePin(SX127X_NSS_PORT, SX127X_NSS_PIN, GPIO_PIN_SET);

  // Clear IRQ
  SX127X_WriteRegister(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);

  return true;
}

void SX127X_Sleep(void) { SX127X_WriteRegister(REG_OP_MODE, MODE_SLEEP); }

void SX127X_SetModeStandby(void) {
  SX127X_WriteRegister(REG_OP_MODE, MODE_STDBY);
}

void SX127X_SetModeRx(void) {
  SX127X_WriteRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);
}

void SX127X_SetModeTx(void) { SX127X_WriteRegister(REG_OP_MODE, MODE_TX); }

uint8_t SX127X_GetIrqFlags(void) {
  return SX127X_ReadRegister(REG_IRQ_FLAGS);
}

void SX127X_ClearIrqFlags(void) {
  SX127X_WriteRegister(REG_IRQ_FLAGS, 0xFFu);
}

int16_t SX127X_GetPacketRssi(void) {
  return (SX127X_ReadRegister(REG_PKT_RSSI_VALUE) - (RSSI_OFFSET_HF_PORT));
}

float SX127X_GetPacketSnr(void) {
  return ((int8_t)SX127X_ReadRegister(REG_PKT_SNR_VALUE)) * 0.25;
}
