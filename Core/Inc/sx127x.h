#ifndef SX127X_H
#define SX127X_H

#include "main.h"
#include <stdbool.h>
#include <string.h>

// Pin Definitions
#define SX127X_NSS_PORT     GPIOC
#define SX127X_NSS_PIN      GPIO_PIN_7

#define SX127X_RESET_PORT   GPIOC
#define SX127X_RESET_PIN    GPIO_PIN_6

#define SX127X_VERSION 0x12

// SX127X Register Addresses
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_PA_RAMP 0x0A
#define REG_OCP 0x0B
#define REG_LNA 0x0C
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1A
#define REG_MODEM_CONFIG_1 0x1D
#define REG_MODEM_CONFIG_2 0x1E
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_FREQ_ERROR_MSB 0x28
#define REG_FREQ_ERROR_MID 0x29
#define REG_FREQ_ERROR_LSB 0x2A
#define REG_RSSI_WIDEBAND 0x2C
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_INVERTIQ 0x33
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_INVERTIQ2 0x3B
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4D

// Operating Modes
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06

// PA Config
#define PA_BOOST 0x80

// IRQ Flags
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK 0x40

#define RF_MID_BAND_THRESHOLD 525E6
#define RSSI_OFFSET_HF_PORT 157
#define RSSI_OFFSET_LF_PORT 164

#define MAX_PKT_LENGTH 255

// Function prototypes
bool SX127X_Init(SPI_HandleTypeDef *spi_port);
void SX127X_SetFrequency(float freq);
void SX127X_SetTxPower(int8_t power, bool useRFO);
void SX127X_SetSpreadingFactor(uint8_t sf);
void SX127X_SetSignalBandwidth(uint32_t sbw);
void SX127X_SetCodingRate4(uint8_t denominator);
void SX127X_SetPreambleLength(uint16_t length);
bool SX127X_Send(uint8_t *buf, uint8_t len);
bool SX127X_Receive(uint8_t *buf, uint8_t capacity, uint8_t *len);
int16_t SX127X_GetPacketRssi(void);
float SX127X_GetPacketSnr(void);
void SX127X_Sleep(void);
void SX127X_SetModeStandby(void);
void SX127X_SetModeRx(void);
void SX127X_SetModeTx(void);
uint8_t SX127X_GetIrqFlags(void);
void SX127X_ClearIrqFlags(void);
uint8_t SX127X_ReadRegister(uint8_t reg);
void SX127X_WriteRegister(uint8_t reg, uint8_t data);

#endif // SX127X_H
