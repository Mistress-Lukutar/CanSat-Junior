/**
 * @file SX1268_LoRa.h
 * @brief Header-only library for E22_400MM22S LoRa module based on SX1268 chip
 * @author Nate Hunter
 * @date 2025-09-25
 * @version v1.0.0
 */

#ifndef SX1268_LORA_H
#define SX1268_LORA_H

#include <Arduino.h>
#include <SPI.h>

// ========================================
// Command Opcodes (from datasheet)
// ========================================

/** @defgroup OPERATIONAL_COMMANDS Operational Mode Commands */
#define SX1268_CMD_SET_SLEEP 0x84
#define SX1268_CMD_SET_STANDBY 0x80
#define SX1268_CMD_SET_FS 0xC1
#define SX1268_CMD_SET_TX 0x83
#define SX1268_CMD_SET_RX 0x82
#define SX1268_CMD_SET_REGULATOR_MODE 0x96
#define SX1268_CMD_CALIBRATE 0x89
#define SX1268_CMD_CALIBRATE_IMAGE 0x98
#define SX1268_CMD_SET_PA_CONFIG 0x95
#define SX1268_CMD_SET_RX_TX_FALLBACK_MODE 0x93

/** @defgroup REGISTER_COMMANDS Register and Buffer Access Commands */
#define SX1268_CMD_WRITE_REGISTER 0x0D
#define SX1268_CMD_READ_REGISTER 0x1D
#define SX1268_CMD_WRITE_BUFFER 0x0E
#define SX1268_CMD_READ_BUFFER 0x1E

/** @defgroup DIO_IRQ_COMMANDS DIO and IRQ Control Commands */
#define SX1268_CMD_SET_DIO_IRQ_PARAMS 0x08
#define SX1268_CMD_GET_IRQ_STATUS 0x12
#define SX1268_CMD_CLEAR_IRQ_STATUS 0x02
#define SX1268_CMD_SET_DIO2_AS_RF_SWITCH_CTRL 0x9D
#define SX1268_CMD_SET_DIO3_AS_TCXO_CTRL 0x97

/** @defgroup RF_COMMANDS RF, Modulation and Packet Commands */
#define SX1268_CMD_SET_RF_FREQUENCY 0x86
#define SX1268_CMD_SET_PACKET_TYPE 0x8A
#define SX1268_CMD_GET_PACKET_TYPE 0x11
#define SX1268_CMD_SET_TX_PARAMS 0x8E
#define SX1268_CMD_SET_MODULATION_PARAMS 0x8B
#define SX1268_CMD_SET_PACKET_PARAMS 0x8C
#define SX1268_CMD_SET_BUFFER_BASE_ADDRESS 0x8F

/** @defgroup STATUS_COMMANDS Status Commands */
#define SX1268_CMD_GET_STATUS 0xC0
#define SX1268_CMD_GET_RX_BUFFER_STATUS 0x13
#define SX1268_CMD_GET_PACKET_STATUS 0x14
#define SX1268_CMD_GET_RSSI_INST 0x15
#define SX1268_CMD_GET_STATS 0x10
#define SX1268_CMD_RESET_STATS 0x00

// ========================================
// Configuration Constants
// ========================================

/** @defgroup PACKET_TYPE Packet Type Definitions */
#define SX1268_PACKET_TYPE_GFSK 0x00
#define SX1268_PACKET_TYPE_LORA 0x01

/** @defgroup STANDBY_MODE Standby Mode Definitions */
#define SX1268_STANDBY_RC 0x00
#define SX1268_STANDBY_XOSC 0x01

/** @defgroup REGULATOR_MODE Regulator Mode Definitions */
#define SX1268_REGULATOR_LDO 0x00
#define SX1268_REGULATOR_DC_DC 0x01

/** @defgroup LORA_SF LoRa Spreading Factor */
#define SX1268_LORA_SF5 0x05
#define SX1268_LORA_SF6 0x06
#define SX1268_LORA_SF7 0x07
#define SX1268_LORA_SF8 0x08
#define SX1268_LORA_SF9 0x09
#define SX1268_LORA_SF10 0x0A
#define SX1268_LORA_SF11 0x0B
#define SX1268_LORA_SF12 0x0C

/** @defgroup LORA_BW LoRa Bandwidth */
#define SX1268_LORA_BW_7 0x00
#define SX1268_LORA_BW_10 0x08
#define SX1268_LORA_BW_15 0x01
#define SX1268_LORA_BW_20 0x09
#define SX1268_LORA_BW_31 0x02
#define SX1268_LORA_BW_41 0x0A
#define SX1268_LORA_BW_62 0x03
#define SX1268_LORA_BW_125 0x04
#define SX1268_LORA_BW_250 0x05
#define SX1268_LORA_BW_500 0x06

/** @defgroup LORA_CR LoRa Coding Rate */
#define SX1268_LORA_CR_4_5 0x01
#define SX1268_LORA_CR_4_6 0x02
#define SX1268_LORA_CR_4_7 0x03
#define SX1268_LORA_CR_4_8 0x04

/** @defgroup IRQ_MASK IRQ Mask Definitions */
#define SX1268_IRQ_TX_DONE 0x0001
#define SX1268_IRQ_RX_DONE 0x0002
#define SX1268_IRQ_PREAMBLE_DETECTED 0x0004
#define SX1268_IRQ_SYNC_WORD_VALID 0x0008
#define SX1268_IRQ_HEADER_VALID 0x0010
#define SX1268_IRQ_HEADER_ERR 0x0020
#define SX1268_IRQ_CRC_ERR 0x0040
#define SX1268_IRQ_CAD_DONE 0x0080
#define SX1268_IRQ_CAD_DETECTED 0x0100
#define SX1268_IRQ_TIMEOUT 0x0200

/** @defgroup RAMP_TIME Ramp Time Definitions */
#define SX1268_RAMP_10_US 0x00
#define SX1268_RAMP_20_US 0x01
#define SX1268_RAMP_40_US 0x02
#define SX1268_RAMP_80_US 0x03
#define SX1268_RAMP_200_US 0x04
#define SX1268_RAMP_800_US 0x05
#define SX1268_RAMP_1700_US 0x06
#define SX1268_RAMP_3400_US 0x07

/** @defgroup CALIBRATION Calibration Settings */
#define SX1268_CALIB_RC64K 0x01
#define SX1268_CALIB_RC13M 0x02
#define SX1268_CALIB_PLL 0x04
#define SX1268_CALIB_ADC_PULSE 0x08
#define SX1268_CALIB_ADC_BULK_N 0x10
#define SX1268_CALIB_ADC_BULK_P 0x20
#define SX1268_CALIB_IMAGE 0x40

/** @defgroup REGISTERS Important Register Addresses */
#define SX1268_REG_LORA_SYNC_WORD_MSB 0x0740
#define SX1268_REG_LORA_SYNC_WORD_LSB 0x0741
#define SX1268_REG_RANDOM_NUMBER_0 0x0819
#define SX1268_REG_RX_GAIN 0x08AC
#define SX1268_REG_OCP_CONFIGURATION 0x08E7
#define SX1268_REG_XTA_TRIM 0x0911
#define SX1268_REG_XTB_TRIM 0x0912

/** @defgroup TIMING Timing Constants */
#define SX1268_RESET_PULSE_WIDTH_US 100
#define SX1268_WAKEUP_TIME_MS 10
#define SX1268_MAX_TIMEOUT_MS 5000

/** @defgroup POWER_LEVELS Default Power Levels */
#define SX1268_MAX_POWER_DBM 22
#define SX1268_MIN_POWER_DBM -9
#define SX1268_DEFAULT_POWER_DBM 14

/** @defgroup BUFFER_SIZE Buffer Size Definitions */
#define SX1268_MAX_PAYLOAD_LENGTH 255
#define SX1268_BUFFER_SIZE 256

// ========================================
// Structure Definitions
// ========================================

/**
 * @brief Pin configuration structure for SX1268 module
 */
typedef struct {
  uint8_t cs_pin;    ///< SPI Chip Select pin
  uint8_t busy_pin;  ///< BUSY status pin
  uint8_t rxen_pin;  ///< RX Enable pin
  uint8_t txen_pin;  ///< TX Enable pin
  uint8_t nrst_pin;  ///< Reset pin (active low)
  uint8_t dio1_pin;  ///< DIO1 interrupt pin
  uint8_t dio2_pin;  ///< DIO2 interrupt pin
} SX1268_PinConfig;

/**
 * @brief LoRa configuration structure
 */
typedef struct {
  uint32_t frequency;        ///< RF frequency in Hz
  uint8_t spreading_factor;  ///< Spreading factor (SF5-SF12)
  uint8_t bandwidth;         ///< Signal bandwidth
  uint8_t coding_rate;       ///< Coding rate
  uint8_t power;             ///< TX power in dBm
  uint16_t preamble_length;  ///< Preamble length in symbols
  bool explicit_header;      ///< Header mode (true=explicit, false=implicit)
  bool crc_enable;           ///< CRC enable flag
  bool invert_iq;            ///< IQ inversion flag
} SX1268_LoRaConfig;

/**
 * @brief Main SX1268 device structure
 */
typedef struct {
  SX1268_PinConfig pins;     ///< Pin configuration
  SX1268_LoRaConfig config;  ///< LoRa configuration
  SPIClass* spi_instance;    ///< SPI instance pointer
  uint32_t spi_frequency;    ///< SPI frequency in Hz
  bool initialized;          ///< Initialization flag
} SX1268_Device;

// ========================================
// Function Declarations and Implementations
// ========================================

/**
 * @brief Initialize SX1268 device with default pin configuration
 * @param dev Pointer to the device structure
 * @return true on success, false on failure
 */
bool SX1268_Init(SX1268_Device* dev) {
  if (!dev) return false;

  // Set default pin configuration (PC1-BUSY, PC3-RXEN, PD4-TXEN, PD7-NRST, PD2-DIO1, PD3-DIO2)
  dev->pins.cs_pin = 8;     // PB0
  dev->pins.busy_pin = A1;  // PC1
  dev->pins.rxen_pin = A3;  // PC3
  dev->pins.txen_pin = 4;   // PD4
  dev->pins.nrst_pin = 7;   // PD7
  dev->pins.dio1_pin = 2;   // PD2
  dev->pins.dio2_pin = 3;   // PD3

  // Set default LoRa configuration
  dev->config.frequency = 433000000;  // 433 MHz
  dev->config.spreading_factor = SX1268_LORA_SF7;
  dev->config.bandwidth = SX1268_LORA_BW_125;
  dev->config.coding_rate = SX1268_LORA_CR_4_5;
  dev->config.power = 14;  // 14 dBm
  dev->config.preamble_length = 8;
  dev->config.explicit_header = true;
  dev->config.crc_enable = true;
  dev->config.invert_iq = false;

  dev->spi_instance = &SPI;
  dev->spi_frequency = 8000000;  // 8 MHz SPI
  dev->initialized = false;

  return true;
}

/**
 * @brief Initialize SX1268 device with custom pin configuration
 * @param dev Pointer to the device structure
 * @param pins Pointer to pin configuration structure
 * @return true on success, false on failure
 */
bool SX1268_InitWithPins(SX1268_Device* dev, const SX1268_PinConfig* pins) {
  if (!dev || !pins) return false;

  // Copy pin configuration
  dev->pins = *pins;

  // Set default LoRa configuration
  dev->config.frequency = 433000000;  // 433 MHz
  dev->config.spreading_factor = SX1268_LORA_SF7;
  dev->config.bandwidth = SX1268_LORA_BW_125;
  dev->config.coding_rate = SX1268_LORA_CR_4_5;
  dev->config.power = 14;  // 14 dBm
  dev->config.preamble_length = 8;
  dev->config.explicit_header = true;
  dev->config.crc_enable = true;
  dev->config.invert_iq = false;

  dev->spi_instance = &SPI;
  dev->spi_frequency = 8000000;  // 8 MHz SPI
  dev->initialized = false;

  return true;
}

/**
 * @brief Wait for BUSY pin to go low
 * @param dev Pointer to the device structure
 * @return true if BUSY went low within timeout, false otherwise
 */
bool SX1268_WaitOnBusy(SX1268_Device* dev) {
  if (!dev) return false;

  uint32_t start_time = millis();
  while (digitalRead(dev->pins.busy_pin) && (millis() - start_time) < SX1268_MAX_TIMEOUT_MS) {
    delay(1);
  }
  return !digitalRead(dev->pins.busy_pin);
}

/**
 * @brief Write a command to the SX1268
 * @param dev Pointer to the device structure
 * @param cmd Command opcode
 * @param data Pointer to command data
 * @param len Length of command data
 * @return true on success, false on failure
 */
bool SX1268_WriteCommand(SX1268_Device* dev, uint8_t cmd, const uint8_t* data, uint8_t len) {
  if (!dev || !SX1268_WaitOnBusy(dev)) return false;

  dev->spi_instance->beginTransaction(SPISettings(dev->spi_frequency, MSBFIRST, SPI_MODE0));
  digitalWrite(dev->pins.cs_pin, LOW);

  dev->spi_instance->transfer(cmd);
  for (uint8_t i = 0; i < len; i++) {
    dev->spi_instance->transfer(data[i]);
  }

  digitalWrite(dev->pins.cs_pin, HIGH);
  dev->spi_instance->endTransaction();

  return true;
}

/**
 * @brief Read a command response from the SX1268
 * @param dev Pointer to the device structure
 * @param cmd Command opcode
 * @param data Pointer to buffer for response data
 * @param len Length of response data expected
 * @return true on success, false on failure
 */
bool SX1268_ReadCommand(SX1268_Device* dev, uint8_t cmd, uint8_t* data, uint8_t len) {
  if (!dev || !data || !SX1268_WaitOnBusy(dev)) return false;

  dev->spi_instance->beginTransaction(SPISettings(dev->spi_frequency, MSBFIRST, SPI_MODE0));
  digitalWrite(dev->pins.cs_pin, LOW);

  dev->spi_instance->transfer(cmd);
  dev->spi_instance->transfer(0x00);  // NOP for status
  for (uint8_t i = 0; i < len; i++) {
    data[i] = dev->spi_instance->transfer(0x00);
  }

  digitalWrite(dev->pins.cs_pin, HIGH);
  dev->spi_instance->endTransaction();

  return true;
}

/**
 * @brief Write to a register
 * @param dev Pointer to the device structure
 * @param address Register address
 * @param data Pointer to data to write
 * @param len Length of data
 * @return true on success, false on failure
 */
bool SX1268_WriteRegister(SX1268_Device* dev, uint16_t address, const uint8_t* data, uint8_t len) {
  if (!dev || !data) return false;

  uint8_t cmd_data[2 + len];
  cmd_data[0] = (address >> 8) & 0xFF;
  cmd_data[1] = address & 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    cmd_data[2 + i] = data[i];
  }

  return SX1268_WriteCommand(dev, SX1268_CMD_WRITE_REGISTER, cmd_data, 2 + len);
}

/**
 * @brief Read from a register
 * @param dev Pointer to the device structure
 * @param address Register address
 * @param data Pointer to buffer for read data
 * @param len Length of data to read
 * @return true on success, false on failure
 */
bool SX1268_ReadRegister(SX1268_Device* dev, uint16_t address, uint8_t* data, uint8_t len) {
  if (!dev || !data || !SX1268_WaitOnBusy(dev)) return false;

  dev->spi_instance->beginTransaction(SPISettings(dev->spi_frequency, MSBFIRST, SPI_MODE0));
  digitalWrite(dev->pins.cs_pin, LOW);

  dev->spi_instance->transfer(SX1268_CMD_READ_REGISTER);
  dev->spi_instance->transfer((address >> 8) & 0xFF);
  dev->spi_instance->transfer(address & 0xFF);
  dev->spi_instance->transfer(0x00);  // NOP for status
  for (uint8_t i = 0; i < len; i++) {
    data[i] = dev->spi_instance->transfer(0x00);
  }

  digitalWrite(dev->pins.cs_pin, HIGH);
  dev->spi_instance->endTransaction();

  return true;
}

/**
 * @brief Write data to buffer
 * @param dev Pointer to the device structure
 * @param offset Buffer offset
 * @param data Pointer to data to write
 * @param len Length of data
 * @return true on success, false on failure
 */
bool SX1268_WriteBuffer(SX1268_Device* dev, uint8_t offset, const uint8_t* data, uint8_t len) {
  if (!dev || !data) return false;

  uint8_t cmd_data[1 + len];
  cmd_data[0] = offset;
  for (uint8_t i = 0; i < len; i++) {
    cmd_data[1 + i] = data[i];
  }

  return SX1268_WriteCommand(dev, SX1268_CMD_WRITE_BUFFER, cmd_data, 1 + len);
}

/**
 * @brief Read data from buffer
 * @param dev Pointer to the device structure
 * @param offset Buffer offset
 * @param data Pointer to buffer for read data
 * @param len Length of data to read
 * @return true on success, false on failure
 */
bool SX1268_ReadBuffer(SX1268_Device* dev, uint8_t offset, uint8_t* data, uint8_t len) {
  if (!dev || !data || !SX1268_WaitOnBusy(dev)) return false;

  dev->spi_instance->beginTransaction(SPISettings(dev->spi_frequency, MSBFIRST, SPI_MODE0));
  digitalWrite(dev->pins.cs_pin, LOW);

  dev->spi_instance->transfer(SX1268_CMD_READ_BUFFER);
  dev->spi_instance->transfer(offset);
  dev->spi_instance->transfer(0x00);  // NOP for status
  for (uint8_t i = 0; i < len; i++) {
    data[i] = dev->spi_instance->transfer(0x00);
  }

  digitalWrite(dev->pins.cs_pin, HIGH);
  dev->spi_instance->endTransaction();

  return true;
}

/**
 * @brief Reset the SX1268 chip
 * @param dev Pointer to the device structure
 * @return true on success, false on failure
 */
bool SX1268_Reset(SX1268_Device* dev) {
  if (!dev) return false;

  digitalWrite(dev->pins.nrst_pin, LOW);
  delay(SX1268_RESET_PULSE_WIDTH_US / 1000 + 1);
  digitalWrite(dev->pins.nrst_pin, HIGH);
  delay(SX1268_WAKEUP_TIME_MS);

  return SX1268_WaitOnBusy(dev);
}

/**
 * @brief Set device to standby mode
 * @param dev Pointer to the device structure
 * @param mode Standby mode (RC or XOSC)
 * @return true on success, false on failure
 */
bool SX1268_SetStandby(SX1268_Device* dev, uint8_t mode) {
  if (!dev) return false;
  return SX1268_WriteCommand(dev, SX1268_CMD_SET_STANDBY, &mode, 1);
}

/**
 * @brief Set packet type
 * @param dev Pointer to the device structure
 * @param packet_type Packet type (GFSK or LoRa)
 * @return true on success, false on failure
 */
bool SX1268_SetPacketType(SX1268_Device* dev, uint8_t packet_type) {
  if (!dev) return false;
  return SX1268_WriteCommand(dev, SX1268_CMD_SET_PACKET_TYPE, &packet_type, 1);
}

/**
 * @brief Set RF frequency
 * @param dev Pointer to the device structure
 * @param frequency Frequency in Hz
 * @return true on success, false on failure
 */
bool SX1268_SetRfFrequency(SX1268_Device* dev, uint32_t frequency) {
  if (!dev) return false;

  // Calculate frequency register value: RF_Freq = frequency * 2^25 / F_XTAL
  uint32_t freq_reg = ((uint64_t)frequency * (1UL << 25)) / 32000000UL;

  uint8_t data[4];
  data[0] = (freq_reg >> 24) & 0xFF;
  data[1] = (freq_reg >> 16) & 0xFF;
  data[2] = (freq_reg >> 8) & 0xFF;
  data[3] = freq_reg & 0xFF;

  return SX1268_WriteCommand(dev, SX1268_CMD_SET_RF_FREQUENCY, data, 4);
}

/**
 * @brief Set TX parameters
 * @param dev Pointer to the device structure
 * @param power Output power in dBm
 * @param ramp_time Ramp time setting
 * @return true on success, false on failure
 */
bool SX1268_SetTxParams(SX1268_Device* dev, int8_t power, uint8_t ramp_time) {
  if (!dev) return false;

  uint8_t data[2];
  data[0] = power;
  data[1] = ramp_time;

  return SX1268_WriteCommand(dev, SX1268_CMD_SET_TX_PARAMS, data, 2);
}

/**
 * @brief Set LoRa modulation parameters
 * @param dev Pointer to the device structure
 * @param sf Spreading factor
 * @param bw Bandwidth
 * @param cr Coding rate
 * @param ldro Low data rate optimization
 * @return true on success, false on failure
 */
bool SX1268_SetLoRaModulationParams(SX1268_Device* dev, uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro) {
  if (!dev) return false;

  uint8_t data[8] = { sf, bw, cr, ldro, 0x00, 0x00, 0x00, 0x00 };
  return SX1268_WriteCommand(dev, SX1268_CMD_SET_MODULATION_PARAMS, data, 8);
}

/**
 * @brief Set LoRa packet parameters
 * @param dev Pointer to the device structure
 * @param preamble_length Preamble length in symbols
 * @param header_type Header type (explicit/implicit)
 * @param payload_length Payload length in bytes
 * @param crc_enable CRC enable flag
 * @param invert_iq IQ inversion flag
 * @return true on success, false on failure
 */
bool SX1268_SetLoRaPacketParams(SX1268_Device* dev, uint16_t preamble_length, uint8_t header_type,
                                uint8_t payload_length, uint8_t crc_enable, uint8_t invert_iq) {
  if (!dev) return false;

  uint8_t data[9];
  data[0] = (preamble_length >> 8) & 0xFF;
  data[1] = preamble_length & 0xFF;
  data[2] = header_type;
  data[3] = payload_length;
  data[4] = crc_enable;
  data[5] = invert_iq;
  data[6] = 0x00;
  data[7] = 0x00;
  data[8] = 0x00;

  return SX1268_WriteCommand(dev, SX1268_CMD_SET_PACKET_PARAMS, data, 9);
}

/**
 * @brief Set buffer base addresses
 * @param dev Pointer to the device structure
 * @param tx_base_addr TX buffer base address
 * @param rx_base_addr RX buffer base address
 * @return true on success, false on failure
 */
bool SX1268_SetBufferBaseAddress(SX1268_Device* dev, uint8_t tx_base_addr, uint8_t rx_base_addr) {
  if (!dev) return false;

  uint8_t data[2] = { tx_base_addr, rx_base_addr };
  return SX1268_WriteCommand(dev, SX1268_CMD_SET_BUFFER_BASE_ADDRESS, data, 2);
}

/**
 * @brief Set DIO IRQ parameters
 * @param dev Pointer to the device structure
 * @param irq_mask IRQ mask
 * @param dio1_mask DIO1 mask
 * @param dio2_mask DIO2 mask
 * @param dio3_mask DIO3 mask
 * @return true on success, false on failure
 */
bool SX1268_SetDioIrqParams(SX1268_Device* dev, uint16_t irq_mask, uint16_t dio1_mask,
                            uint16_t dio2_mask, uint16_t dio3_mask) {
  if (!dev) return false;

  uint8_t data[8];
  data[0] = (irq_mask >> 8) & 0xFF;
  data[1] = irq_mask & 0xFF;
  data[2] = (dio1_mask >> 8) & 0xFF;
  data[3] = dio1_mask & 0xFF;
  data[4] = (dio2_mask >> 8) & 0xFF;
  data[5] = dio2_mask & 0xFF;
  data[6] = (dio3_mask >> 8) & 0xFF;
  data[7] = dio3_mask & 0xFF;

  return SX1268_WriteCommand(dev, SX1268_CMD_SET_DIO_IRQ_PARAMS, data, 8);
}

/**
 * @brief Get IRQ status
 * @param dev Pointer to the device structure
 * @param irq_status Pointer to store IRQ status
 * @return true on success, false on failure
 */
bool SX1268_GetIrqStatus(SX1268_Device* dev, uint16_t* irq_status) {
  if (!dev || !irq_status) return false;

  uint8_t data[2];
  if (!SX1268_ReadCommand(dev, SX1268_CMD_GET_IRQ_STATUS, data, 2)) return false;

  *irq_status = ((uint16_t)data[0] << 8) | data[1];
  return true;
}

/**
 * @brief Clear IRQ status
 * @param dev Pointer to the device structure
 * @param irq_clear_mask IRQ clear mask
 * @return true on success, false on failure
 */
bool SX1268_ClearIrqStatus(SX1268_Device* dev, uint16_t irq_clear_mask) {
  if (!dev) return false;

  uint8_t data[2];
  data[0] = (irq_clear_mask >> 8) & 0xFF;
  data[1] = irq_clear_mask & 0xFF;

  return SX1268_WriteCommand(dev, SX1268_CMD_CLEAR_IRQ_STATUS, data, 2);
}

/**
 * @brief Set device to TX mode
 * @param dev Pointer to the device structure
 * @param timeout Timeout in 15.625us steps (0 = no timeout)
 * @return true on success, false on failure
 */
bool SX1268_SetTx(SX1268_Device* dev, uint32_t timeout) {
  if (!dev) return false;

  uint8_t data[3];
  data[0] = (timeout >> 16) & 0xFF;
  data[1] = (timeout >> 8) & 0xFF;
  data[2] = timeout & 0xFF;

  return SX1268_WriteCommand(dev, SX1268_CMD_SET_TX, data, 3);
}

/**
 * @brief Set device to RX mode
 * @param dev Pointer to the device structure
 * @param timeout Timeout in 15.625us steps (0 = single mode, 0xFFFFFF = continuous mode)
 * @return true on success, false on failure
 */
bool SX1268_SetRx(SX1268_Device* dev, uint32_t timeout) {
  if (!dev) return false;

  uint8_t data[3];
  data[0] = (timeout >> 16) & 0xFF;
  data[1] = (timeout >> 8) & 0xFF;
  data[2] = timeout & 0xFF;

  return SX1268_WriteCommand(dev, SX1268_CMD_SET_RX, data, 3);
}

/**
 * @brief Get RX buffer status
 * @param dev Pointer to the device structure
 * @param payload_length Pointer to store payload length
 * @param rx_start_buffer_pointer Pointer to store RX start buffer pointer
 * @return true on success, false on failure
 */
bool SX1268_GetRxBufferStatus(SX1268_Device* dev, uint8_t* payload_length, uint8_t* rx_start_buffer_pointer) {
  if (!dev || !payload_length || !rx_start_buffer_pointer) return false;

  uint8_t data[2];
  if (!SX1268_ReadCommand(dev, SX1268_CMD_GET_RX_BUFFER_STATUS, data, 2)) return false;

  *payload_length = data[0];
  *rx_start_buffer_pointer = data[1];
  return true;
}

/**
 * @brief Get packet status (LoRa)
 * @param dev Pointer to the device structure
 * @param rssi_pkt Pointer to store RSSI packet value
 * @param snr_pkt Pointer to store SNR packet value
 * @param signal_rssi_pkt Pointer to store signal RSSI packet value
 * @return true on success, false on failure
 */
bool SX1268_GetLoRaPacketStatus(SX1268_Device* dev, uint8_t* rssi_pkt, uint8_t* snr_pkt, uint8_t* signal_rssi_pkt) {
  if (!dev || !rssi_pkt || !snr_pkt || !signal_rssi_pkt) return false;

  uint8_t data[3];
  if (!SX1268_ReadCommand(dev, SX1268_CMD_GET_PACKET_STATUS, data, 3)) return false;

  *rssi_pkt = data[0];
  *snr_pkt = data[1];
  *signal_rssi_pkt = data[2];
  return true;
}

/**
 * @brief Set PA configuration
 * @param dev Pointer to the device structure
 * @param pa_duty_cycle PA duty cycle
 * @param hp_max HP max setting
 * @param device_sel Device selection (always 0x00)
 * @param pa_lut PA LUT (always 0x01)
 * @return true on success, false on failure
 */
bool SX1268_SetPaConfig(SX1268_Device* dev, uint8_t pa_duty_cycle, uint8_t hp_max, uint8_t device_sel, uint8_t pa_lut) {
  if (!dev) return false;

  uint8_t data[4] = { pa_duty_cycle, hp_max, device_sel, pa_lut };
  return SX1268_WriteCommand(dev, SX1268_CMD_SET_PA_CONFIG, data, 4);
}

/**
 * @brief Set regulator mode
 * @param dev Pointer to the device structure
 * @param mode Regulator mode (LDO or DC-DC)
 * @return true on success, false on failure
 */
bool SX1268_SetRegulatorMode(SX1268_Device* dev, uint8_t mode) {
  if (!dev) return false;
  return SX1268_WriteCommand(dev, SX1268_CMD_SET_REGULATOR_MODE, &mode, 1);
}

/**
 * @brief Calibrate device
 * @param dev Pointer to the device structure
 * @param calib_param Calibration parameters
 * @return true on success, false on failure
 */
bool SX1268_Calibrate(SX1268_Device* dev, uint8_t calib_param) {
  if (!dev) return false;
  return SX1268_WriteCommand(dev, SX1268_CMD_CALIBRATE, &calib_param, 1);
}

/**
 * @brief Calibrate image rejection
 * @param dev Pointer to the device structure
 * @param freq1 Frequency 1 parameter
 * @param freq2 Frequency 2 parameter
 * @return true on success, false on failure
 */
bool SX1268_CalibrateImage(SX1268_Device* dev, uint8_t freq1, uint8_t freq2) {
  if (!dev) return false;

  uint8_t data[2] = { freq1, freq2 };
  return SX1268_WriteCommand(dev, SX1268_CMD_CALIBRATE_IMAGE, data, 2);
}

/**
 * @brief Configure RF switch control using DIO2
 * @param dev Pointer to the device structure  
 * @return true on success, false on failure
 */
bool SX1268_ConfigureRfSwitch(SX1268_Device* dev) {
  if (!dev) return false;

  // Configure DIO2 as RF switch control
  uint8_t enable = 0x01;
  if (!SX1268_WriteCommand(dev, SX1268_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &enable, 1)) {
    return false;
  }

  // Set up external RF switch control pins if defined
  if (dev->pins.rxen_pin != 0xFF) {
    pinMode(dev->pins.rxen_pin, OUTPUT);
    digitalWrite(dev->pins.rxen_pin, LOW);
  }

  if (dev->pins.txen_pin != 0xFF) {
    pinMode(dev->pins.txen_pin, OUTPUT);
    digitalWrite(dev->pins.txen_pin, LOW);
  }

  return true;
}

/**
 * @brief Control external RF switch
 * @param dev Pointer to the device structure
 * @param tx_mode true for TX mode, false for RX mode
 */
void SX1268_SetRfSwitch(SX1268_Device* dev, bool tx_mode) {
  if (!dev) return;

  if (tx_mode) {
    // TX mode: enable TX, disable RX
    if (dev->pins.txen_pin != 0xFF) digitalWrite(dev->pins.txen_pin, HIGH);
    if (dev->pins.rxen_pin != 0xFF) digitalWrite(dev->pins.rxen_pin, LOW);
  } else {
    // RX mode: enable RX, disable TX
    if (dev->pins.rxen_pin != 0xFF) digitalWrite(dev->pins.rxen_pin, HIGH);
    if (dev->pins.txen_pin != 0xFF) digitalWrite(dev->pins.txen_pin, LOW);
  }
}

/**
 * @brief Apply LoRa configuration to the device
 * @param dev Pointer to the device structure
 * @return true on success, false on failure
 */
bool SX1268_ApplyLoRaConfig(SX1268_Device* dev) {
  if (!dev) return false;

  // Set LoRa packet type
  if (!SX1268_SetPacketType(dev, SX1268_PACKET_TYPE_LORA)) return false;

  // Set RF frequency
  if (!SX1268_SetRfFrequency(dev, dev->config.frequency)) return false;

  // Set TX parameters
  if (!SX1268_SetTxParams(dev, dev->config.power, SX1268_RAMP_200_US)) return false;

  // Set LoRa modulation parameters
  uint8_t ldro = (dev->config.spreading_factor > SX1268_LORA_SF10 && dev->config.bandwidth <= SX1268_LORA_BW_125) ? 0x01 : 0x00;
  if (!SX1268_SetLoRaModulationParams(dev, dev->config.spreading_factor,
                                      dev->config.bandwidth, dev->config.coding_rate, ldro)) return false;

  // Set LoRa packet parameters
  uint8_t header_type = dev->config.explicit_header ? 0x00 : 0x01;
  uint8_t crc_enable = dev->config.crc_enable ? 0x01 : 0x00;
  uint8_t invert_iq = dev->config.invert_iq ? 0x01 : 0x00;
  if (!SX1268_SetLoRaPacketParams(dev, dev->config.preamble_length, header_type,
                                  SX1268_MAX_PAYLOAD_LENGTH, crc_enable, invert_iq)) return false;

  // Set buffer base addresses
  if (!SX1268_SetBufferBaseAddress(dev, 0x00, 0x00)) return false;

  // Set LoRa sync word for public network (0x3444) or private network (0x1424)
  uint8_t sync_word_msb = 0x34;
  uint8_t sync_word_lsb = 0x44;
  if (!SX1268_WriteRegister(dev, SX1268_REG_LORA_SYNC_WORD_MSB, &sync_word_msb, 1)) return false;
  if (!SX1268_WriteRegister(dev, SX1268_REG_LORA_SYNC_WORD_LSB, &sync_word_lsb, 1)) return false;

  return true;
}

/**
 * @brief Begin the SX1268 LoRa module
 * @param dev Pointer to the device structure
 * @return true on success, false on failure
 */
bool SX1268_Begin(SX1268_Device* dev) {
  if (!dev) return false;

  // Configure GPIO pins
  pinMode(dev->pins.cs_pin, OUTPUT);
  digitalWrite(dev->pins.cs_pin, HIGH);

  pinMode(dev->pins.busy_pin, INPUT);

  if (dev->pins.nrst_pin != 0xFF) {
    pinMode(dev->pins.nrst_pin, OUTPUT);
    digitalWrite(dev->pins.nrst_pin, HIGH);
  }

  if (dev->pins.dio1_pin != 0xFF) {
    pinMode(dev->pins.dio1_pin, INPUT);
  }

  if (dev->pins.dio2_pin != 0xFF) {
    pinMode(dev->pins.dio2_pin, INPUT);
  }

  // Initialize SPI
  dev->spi_instance->begin();

  // Reset the device
  if (!SX1268_Reset(dev)) return false;

  // Set to standby mode
  if (!SX1268_SetStandby(dev, SX1268_STANDBY_RC)) return false;

  // Set regulator mode to DC-DC for better efficiency
  if (!SX1268_SetRegulatorMode(dev, SX1268_REGULATOR_DC_DC)) return false;

  // Calibrate all blocks
  uint8_t calib_all = SX1268_CALIB_RC64K | SX1268_CALIB_RC13M | SX1268_CALIB_PLL | SX1268_CALIB_ADC_PULSE | SX1268_CALIB_ADC_BULK_N | SX1268_CALIB_ADC_BULK_P | SX1268_CALIB_IMAGE;
  if (!SX1268_Calibrate(dev, calib_all)) return false;

  // Image calibration for 430-440 MHz band (adjust as needed)
  if (!SX1268_CalibrateImage(dev, 0x6B, 0x6F)) return false;

  // Configure PA for +14 dBm operation
  if (!SX1268_SetPaConfig(dev, 0x04, 0x06, 0x00, 0x01)) return false;

  // Configure RF switch
  if (!SX1268_ConfigureRfSwitch(dev)) return false;

  // Apply LoRa configuration
  if (!SX1268_ApplyLoRaConfig(dev)) return false;

  // Set up IRQ configuration
  uint16_t irq_mask = SX1268_IRQ_TX_DONE | SX1268_IRQ_RX_DONE | SX1268_IRQ_TIMEOUT | SX1268_IRQ_CRC_ERR;
  uint16_t dio1_mask = SX1268_IRQ_TX_DONE | SX1268_IRQ_RX_DONE | SX1268_IRQ_TIMEOUT | SX1268_IRQ_CRC_ERR;
  if (!SX1268_SetDioIrqParams(dev, irq_mask, dio1_mask, 0x0000, 0x0000)) return false;

  dev->initialized = true;
  return true;
}

/**
 * @brief Send data using LoRa
 * @param dev Pointer to the device structure
 * @param data Pointer to data to send
 * @param len Length of data to send
 * @param timeout_ms Timeout in milliseconds
 * @return true on success, false on failure
 */
bool SX1268_SendData(SX1268_Device* dev, const uint8_t* data, uint8_t len, uint32_t timeout_ms) {
  if (!dev || !data || len == 0 || len > SX1268_MAX_PAYLOAD_LENGTH || !dev->initialized) return false;

  // Clear any pending IRQs
  if (!SX1268_ClearIrqStatus(dev, 0xFFFF)) return false;

  // Write data to buffer
  if (!SX1268_WriteBuffer(dev, 0x00, data, len)) return false;

  // Update packet parameters with actual payload length
  uint8_t header_type = dev->config.explicit_header ? 0x00 : 0x01;
  uint8_t crc_enable = dev->config.crc_enable ? 0x01 : 0x00;
  uint8_t invert_iq = dev->config.invert_iq ? 0x01 : 0x00;
  if (!SX1268_SetLoRaPacketParams(dev, dev->config.preamble_length, header_type,
                                  len, crc_enable, invert_iq)) return false;

  // Set RF switch to TX mode
  SX1268_SetRfSwitch(dev, true);

  // Set device to TX mode
  uint32_t timeout_steps = timeout_ms * 64;  // Convert ms to 15.625us steps (approximately)
  if (!SX1268_SetTx(dev, timeout_steps)) return false;

  // Wait for TX done or timeout
  uint32_t start_time = millis();
  uint16_t irq_status = 0;

  while ((millis() - start_time) < timeout_ms) {
    if (SX1268_GetIrqStatus(dev, &irq_status)) {
      if (irq_status & SX1268_IRQ_TX_DONE) {
        // TX completed successfully
        SX1268_ClearIrqStatus(dev, SX1268_IRQ_TX_DONE);
        SX1268_SetRfSwitch(dev, false);  // Disable TX
        return true;
      }

      if (irq_status & SX1268_IRQ_TIMEOUT) {
        // TX timeout
        SX1268_ClearIrqStatus(dev, SX1268_IRQ_TIMEOUT);
        SX1268_SetRfSwitch(dev, false);  // Disable TX
        return false;
      }
    }
    delay(1);
  }

  // Timeout occurred
  SX1268_SetRfSwitch(dev, false);  // Disable TX
  return false;
}

/**
 * @brief Receive data using LoRa (blocking)
 * @param dev Pointer to the device structure
 * @param data Pointer to buffer for received data
 * @param max_len Maximum length of data buffer
 * @param received_len Pointer to store actual received length
 * @param timeout_ms Timeout in milliseconds (0 = single mode, 0xFFFFFFFF = continuous)
 * @return true on success, false on failure
 */
bool SX1268_ReceiveData(SX1268_Device* dev, uint8_t* data, uint8_t max_len, uint8_t* received_len, uint32_t timeout_ms) {
  if (!dev || !data || !received_len || max_len == 0 || !dev->initialized) return false;

  *received_len = 0;

  // Clear any pending IRQs
  if (!SX1268_ClearIrqStatus(dev, 0xFFFF)) return false;

  // Set RF switch to RX mode
  SX1268_SetRfSwitch(dev, false);

  // Set device to RX mode
  uint32_t rx_timeout = 0xFFFFFF;  // Continuous mode by default
  if (timeout_ms != 0xFFFFFFFF && timeout_ms != 0) {
    rx_timeout = timeout_ms * 64;  // Convert ms to 15.625us steps (approximately)
  } else if (timeout_ms == 0) {
    rx_timeout = 0;  // Single mode
  }

  if (!SX1268_SetRx(dev, rx_timeout)) return false;

  // Wait for RX done, CRC error, or timeout
  uint32_t start_time = millis();
  uint16_t irq_status = 0;
  uint32_t actual_timeout = (timeout_ms == 0xFFFFFFFF) ? 0xFFFFFFFF : timeout_ms;

  while (actual_timeout == 0xFFFFFFFF || (millis() - start_time) < actual_timeout) {
    if (SX1268_GetIrqStatus(dev, &irq_status)) {
      if (irq_status & SX1268_IRQ_RX_DONE) {
        // Check for CRC error
        if (irq_status & SX1268_IRQ_CRC_ERR) {
          SX1268_ClearIrqStatus(dev, SX1268_IRQ_RX_DONE | SX1268_IRQ_CRC_ERR);
          return false;  // CRC error
        }

        // RX completed successfully
        uint8_t payload_len, rx_start_ptr;
        if (!SX1268_GetRxBufferStatus(dev, &payload_len, &rx_start_ptr)) {
          SX1268_ClearIrqStatus(dev, SX1268_IRQ_RX_DONE);
          return false;
        }

        // Limit payload length to buffer size
        if (payload_len > max_len) payload_len = max_len;

        // Read received data
        if (!SX1268_ReadBuffer(dev, rx_start_ptr, data, payload_len)) {
          SX1268_ClearIrqStatus(dev, SX1268_IRQ_RX_DONE);
          return false;
        }

        *received_len = payload_len;
        SX1268_ClearIrqStatus(dev, SX1268_IRQ_RX_DONE);
        return true;
      }

      if (irq_status & SX1268_IRQ_TIMEOUT) {
        // RX timeout
        SX1268_ClearIrqStatus(dev, SX1268_IRQ_TIMEOUT);
        return false;
      }

      if (irq_status & SX1268_IRQ_CRC_ERR) {
        // CRC error only
        SX1268_ClearIrqStatus(dev, SX1268_IRQ_CRC_ERR);
        return false;
      }
    }
    delay(1);
  }

  // Timeout occurred
  return false;
}

/**
 * @brief Check if data is available for reading
 * @param dev Pointer to the device structure
 * @return true if data is available, false otherwise
 */
bool SX1268_Available(SX1268_Device* dev) {
  if (!dev || !dev->initialized) return false;

  uint16_t irq_status = 0;
  if (!SX1268_GetIrqStatus(dev, &irq_status)) return false;

  return (irq_status & SX1268_IRQ_RX_DONE) && !(irq_status & SX1268_IRQ_CRC_ERR);
}

/**
 * @brief Get RSSI of last received packet
 * @param dev Pointer to the device structure
 * @param rssi Pointer to store RSSI value in dBm
 * @return true on success, false on failure
 */
bool SX1268_GetLastPacketRssi(SX1268_Device* dev, int16_t* rssi) {
  if (!dev || !rssi || !dev->initialized) return false;

  uint8_t rssi_pkt, snr_pkt, signal_rssi_pkt;
  if (!SX1268_GetLoRaPacketStatus(dev, &rssi_pkt, &snr_pkt, &signal_rssi_pkt)) return false;

  *rssi = -(int16_t)rssi_pkt / 2;  // Convert to dBm
  return true;
}

/**
 * @brief Get SNR of last received packet
 * @param dev Pointer to the device structure
 * @param snr Pointer to store SNR value in dB
 * @return true on success, false on failure
 */
bool SX1268_GetLastPacketSnr(SX1268_Device* dev, int8_t* snr) {
  if (!dev || !snr || !dev->initialized) return false;

  uint8_t rssi_pkt, snr_pkt, signal_rssi_pkt;
  if (!SX1268_GetLoRaPacketStatus(dev, &rssi_pkt, &snr_pkt, &signal_rssi_pkt)) return false;

  *snr = (int8_t)snr_pkt / 4;  // Convert to dB (SNR is in two's complement format multiplied by 4)
  return true;
}

/**
 * @brief Set LoRa configuration
 * @param dev Pointer to the device structure
 * @param config Pointer to LoRa configuration structure
 * @return true on success, false on failure
 */
bool SX1268_SetLoRaConfig(SX1268_Device* dev, const SX1268_LoRaConfig* config) {
  if (!dev || !config) return false;

  dev->config = *config;

  if (dev->initialized) {
    return SX1268_ApplyLoRaConfig(dev);
  }

  return true;
}

/**
 * @brief Put device into sleep mode
 * @param dev Pointer to the device structure
 * @param warm_start true for warm start (retain configuration), false for cold start
 * @return true on success, false on failure
 */
bool SX1268_Sleep(SX1268_Device* dev, bool warm_start) {
  if (!dev) return false;

  // Disable RF switch
  SX1268_SetRfSwitch(dev, false);

  uint8_t sleep_config = warm_start ? 0x04 : 0x00;  // bit 2 = warm start
  return SX1268_WriteCommand(dev, SX1268_CMD_SET_SLEEP, &sleep_config, 1);
}

/**
 * @brief Wake up device from sleep mode
 * @param dev Pointer to the device structure
 * @return true on success, false on failure
 */
bool SX1268_Wakeup(SX1268_Device* dev) {
  if (!dev) return false;

  // Toggle NSS to wake up device
  digitalWrite(dev->pins.cs_pin, LOW);
  delay(1);
  digitalWrite(dev->pins.cs_pin, HIGH);
  delay(SX1268_WAKEUP_TIME_MS);

  return SX1268_WaitOnBusy(dev);
}

#endif  // SX1268_LORA_H
