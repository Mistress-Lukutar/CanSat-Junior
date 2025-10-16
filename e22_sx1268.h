/**
 * @file e22_sx1268.h
 * @brief E22-400MM22S LoRa module driver for SX1268 chip (with PROGMEM logs & runtime status read)
 * @author Mistress Lukutar
 * @date 2025-10-04
 * @version v1.0.0
 *
 * Uses SX1268 datasheet as reference for GetStatus and other commands
 */

#ifndef E22_SX1268_H
#define E22_SX1268_H

#include <Arduino.h>
#include <SPI.h>

/* ===== Debug control ===== */
/* Define E22_DEBUG to enable Serial logging of SPI transactions and states (stored in flash). */
//#define E22_DEBUG

#ifdef E22_DEBUG
/* Compact PROGMEM-friendly logging macros */
#define E22_P(str) Serial.print(F(str))
#define E22_PL(str) Serial.println(F(str))
#define E22_PHEX(val) Serial.print(val, HEX)
#define E22_PDEC(val) Serial.print(val)
#define E22_PCH(val) Serial.print(val)
#define E22_PSP() Serial.print(' ')
#else
#define E22_P(str)
#define E22_PL(str)
#define E22_PHEX(val)
#define E22_PDEC(val)
#define E22_PCH(val)
#define E22_PSP()
#endif

/* ===== SX1268 Command Opcodes ===== */
#define SX1268_CMD_SET_SLEEP 0x84
#define SX1268_CMD_SET_STANDBY 0x80
#define SX1268_CMD_SET_FS 0xC1
#define SX1268_CMD_SET_TX 0x83
#define SX1268_CMD_SET_RX 0x82
#define SX1268_CMD_SET_RF_FREQUENCY 0x86
#define SX1268_CMD_SET_PACKET_TYPE 0x8A
#define SX1268_CMD_SET_MODULATION_PARAMS 0x8B
#define SX1268_CMD_SET_PACKET_PARAMS 0x8C
#define SX1268_CMD_SET_TX_PARAMS 0x8E
#define SX1268_CMD_SET_BUFFER_BASE_ADDRESS 0x8F
#define SX1268_CMD_SET_DIO_IRQ_PARAMS 0x08
#define SX1268_CMD_GET_IRQ_STATUS 0x12
#define SX1268_CMD_CLEAR_IRQ_STATUS 0x02
#define SX1268_CMD_WRITE_BUFFER 0x0E
#define SX1268_CMD_READ_BUFFER 0x1E
#define SX1268_CMD_GET_RX_BUFFER_STATUS 0x13
#define SX1268_CMD_GET_PACKET_STATUS 0x14
#define SX1268_CMD_SET_REGULATOR_MODE 0x96
#define SX1268_CMD_CALIBRATE_IMAGE 0x98
#define SX1268_CMD_SET_PA_CONFIG 0x95
#define SX1268_CMD_SET_DIO2_AS_RF_SWITCH 0x9D

/* ===== Status / Register access opcodes ===== */
#define SX1268_CMD_WRITE_REGISTER 0x0D
#define SX1268_CMD_READ_REGISTER 0x1D
#define SX1268_CMD_GET_STATUS 0xC0
#define SX1268_CMD_GET_RSSI_INST 0x15
#define SX1268_CMD_GET_DEVICE_ERRORS 0x17

/* ===== Standby Modes ===== */
#define SX1268_STANDBY_RC 0x00
#define SX1268_STANDBY_XOSC 0x01

/* ===== Packet Types ===== */
#define SX1268_PACKET_TYPE_GFSK 0x00
#define SX1268_PACKET_TYPE_LORA 0x01

/* ===== Regulator Modes ===== */
#define SX1268_REGULATOR_LDO 0x00
#define SX1268_REGULATOR_DC_DC 0x01

/* ===== IRQ Masks ===== */
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
#define SX1268_IRQ_ALL 0x03FF

/* ===== LoRa Spreading Factors ===== */
#define SX1268_LORA_SF5 0x05
#define SX1268_LORA_SF6 0x06
#define SX1268_LORA_SF7 0x07
#define SX1268_LORA_SF8 0x08
#define SX1268_LORA_SF9 0x09
#define SX1268_LORA_SF10 0x0A
#define SX1268_LORA_SF11 0x0B
#define SX1268_LORA_SF12 0x0C

/* ===== LoRa Bandwidths ===== */
#define SX1268_LORA_BW_125 0x04
#define SX1268_LORA_BW_250 0x05
#define SX1268_LORA_BW_500 0x06

/* ===== LoRa Coding Rates ===== */
#define SX1268_LORA_CR_4_5 0x01
#define SX1268_LORA_CR_4_6 0x02
#define SX1268_LORA_CR_4_7 0x03
#define SX1268_LORA_CR_4_8 0x04

/* ===== Other Constants ===== */
#define SX1268_MAX_PAYLOAD_LENGTH 255
#define SX1268_XTAL_FREQ 32000000UL
#define SX1268_DEFAULT_TIMEOUT_MS 1000

/* ===== Known register addresses ===== */
/* LoRa Sync Word register address (two bytes) */
#define REG_LR_SYNCWORD 0x0740
#define REG_LR_SYNCWORD_DEFAULT_MSB 0x14
#define REG_LR_SYNCWORD_DEFAULT_LSB 0x24

/**
 * @brief E22-400MM22S device configuration structure
 */
typedef struct {
  SPIClass *spi;             // SPI interface pointer
  uint8_t nss_pin;           // NSS (chip select) pin
  uint8_t busy_pin;          // BUSY pin
  uint8_t nrst_pin;          // RESET pin
  uint8_t dio1_pin;          // DIO1 interrupt pin
  uint8_t dio2_pin;          // DIO2 pin (optional)
  uint8_t txen_pin;          // TX enable pin (E22 specific)
  uint8_t rxen_pin;          // RX enable pin (E22 specific)
  uint32_t frequency;        // RF frequency in Hz
  uint8_t spreading_factor;  // LoRa spreading factor
  uint8_t bandwidth;         // LoRa bandwidth
  uint8_t coding_rate;       // LoRa coding rate
  uint8_t tx_power;          // TX power in dBm
  uint16_t preamble_length;  // Preamble length
} E22_Device;

/* ===== Function Prototypes ===== */
uint8_t E22_Init(E22_Device *dev);
uint8_t E22_SetLoraModulation(E22_Device *dev);
uint8_t E22_SetFrequency(E22_Device *dev, uint32_t frequency);
uint8_t E22_SendData(E22_Device *dev, const uint8_t *data, uint8_t length);
uint8_t E22_ReceiveData(E22_Device *dev, uint8_t *buffer, uint8_t max_length, uint32_t timeout_ms);
uint8_t E22_SetStandby(E22_Device *dev);
int16_t E22_GetRssi(E22_Device *dev);
int8_t E22_GetSnr(E22_Device *dev);

/* ===== Private prototypes ===== */
static void E22_WaitBusy(E22_Device *dev);
static void E22_SetRfSwitch(E22_Device *dev, uint8_t tx_enable);
static void E22_HardReset(E22_Device *dev);
static void E22_WriteCommand(E22_Device *dev, uint8_t opcode, const uint8_t *params, uint8_t param_len);
static void E22_ReadCommand(E22_Device *dev, uint8_t opcode, uint8_t *data, uint8_t data_len);
static uint16_t E22_GetIrqStatus(E22_Device *dev);
static void E22_ClearIrqStatus(E22_Device *dev, uint16_t irq_mask);
static void E22_WriteRegisterRaw(E22_Device *dev, uint16_t addr, const uint8_t *data, uint8_t len);
static void E22_ReadRegisterRaw(E22_Device *dev, uint16_t addr, uint8_t *data, uint8_t len);
static void E22_LogStatus(E22_Device *dev, const char *ctx);  // new
static void E22_PrintHexBuf(const uint8_t *buf, uint8_t len);
/* ===== Implementations ===== */

/**
 * @brief Wait until BUSY pin goes low
 */
static void E22_WaitBusy(E22_Device *dev) {
  uint32_t timeout = millis() + 100;
  while (digitalRead(dev->busy_pin) == HIGH) {
    if (millis() > timeout) {
#ifdef E22_DEBUG
      E22_PL("[E22] BUSY timeout");
#endif
      break;
    }
  }
  delayMicroseconds(10);
}

/**
 * @brief Control RF switch (E22 specific: TXEN/RXEN)
 */
static void E22_SetRfSwitch(E22_Device *dev, uint8_t tx_enable) {
  if (tx_enable) {
    digitalWrite(dev->txen_pin, HIGH);
    digitalWrite(dev->rxen_pin, LOW);
#ifdef E22_DEBUG
    E22_PL("[E22] RF->TX");
#endif
  } else {
    digitalWrite(dev->txen_pin, LOW);
    digitalWrite(dev->rxen_pin, HIGH);
#ifdef E22_DEBUG
    E22_PL("[E22] RF->RX");
#endif
  }
}

/**
 * @brief Hardware reset of the module
 */
static void E22_HardReset(E22_Device *dev) {
#ifdef E22_DEBUG
  E22_PL("[E22] HW reset");
#endif
  digitalWrite(dev->nrst_pin, LOW);
  delay(10);
  digitalWrite(dev->nrst_pin, HIGH);
  delay(10);
  E22_WaitBusy(dev);
#ifdef E22_DEBUG
  E22_LogStatus(dev, "POST_RESET");
#endif
}

/**
 * @brief Write command to SX1268
 */
static void E22_WriteCommand(E22_Device *dev, uint8_t opcode, const uint8_t *params, uint8_t param_len) {
  E22_WaitBusy(dev);
#ifdef E22_DEBUG
  E22_P("[E22] TX CMD 0x");
  E22_PHEX(opcode);
  E22_PSP();
  E22_PDEC(param_len);
  E22_PL(" bytes");
#endif
  dev->spi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dev->nss_pin, LOW);
  dev->spi->transfer(opcode);
#ifdef E22_DEBUG
  E22_P("[E22] TX CMD DATA: ");
  E22_PrintHexBuf(params, param_len);
#endif
  for (uint8_t i = 0; i < param_len; i++) {
    dev->spi->transfer(params[i]);
  }
  digitalWrite(dev->nss_pin, HIGH);
  dev->spi->endTransaction();
  E22_WaitBusy(dev);
#ifdef E22_DEBUG
  E22_PL("[E22] TX CMD done");
#endif
}

/**
 * @brief Read command response from SX1268
 */
static void E22_ReadCommand(E22_Device *dev, uint8_t opcode, uint8_t *data, uint8_t data_len) {
  E22_WaitBusy(dev);
#ifdef E22_DEBUG
  E22_P("[E22] RX CMD 0x");
  E22_PHEX(opcode);
  E22_PSP();
  E22_PDEC(data_len);
  E22_PL(" bytes");
#endif
  dev->spi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dev->nss_pin, LOW);
  dev->spi->transfer(opcode);
  dev->spi->transfer(0x00);  // NOP / status
  for (uint8_t i = 0; i < data_len; i++) {
    data[i] = dev->spi->transfer(0x00);
  }
#ifdef E22_DEBUG
  E22_P("[E22] RX CMD DATA: ");
  E22_PrintHexBuf(data, data_len);
#endif
  digitalWrite(dev->nss_pin, HIGH);
  dev->spi->endTransaction();
#ifdef E22_DEBUG
  E22_PL("[E22] RX CMD done");
#endif
}

/**
 * @brief Low-level write register via SPI
 */
static void E22_WriteRegisterRaw(E22_Device *dev, uint16_t addr, const uint8_t *data, uint8_t len) {
  E22_WaitBusy(dev);
#ifdef E22_DEBUG
  E22_P("[E22] WR_REG 0x");
  E22_PHEX(addr);
  E22_PSP();
  E22_PDEC(len);
  E22_PL("");
#endif
  dev->spi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dev->nss_pin, LOW);
  dev->spi->transfer(SX1268_CMD_WRITE_REGISTER);
  dev->spi->transfer((addr >> 8) & 0xFF);
  dev->spi->transfer(addr & 0xFF);
  for (uint8_t i = 0; i < len; i++) {
#ifdef E22_DEBUG
    E22_P("[E22]  W[");
    E22_PDEC(i);
    E22_P("]=0x");
    E22_PHEX(data[i]);
    E22_PL("");
#endif
    dev->spi->transfer(data[i]);
  }
  digitalWrite(dev->nss_pin, HIGH);
  dev->spi->endTransaction();
  E22_WaitBusy(dev);
#ifdef E22_DEBUG
  E22_PL("[E22] WR_REG done");
#endif
}

/**
 * @brief Low-level read register via SPI
 */
static void E22_ReadRegisterRaw(E22_Device *dev, uint16_t addr, uint8_t *data, uint8_t len) {
  E22_WaitBusy(dev);
#ifdef E22_DEBUG
  E22_P("[E22] RD_REG 0x");
  E22_PHEX(addr);
  E22_PSP();
  E22_PDEC(len);
  E22_PL("");
#endif
  dev->spi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dev->nss_pin, LOW);
  dev->spi->transfer(SX1268_CMD_READ_REGISTER);
  dev->spi->transfer((addr >> 8) & 0xFF);
  dev->spi->transfer(addr & 0xFF);
  dev->spi->transfer(0x00);
  for (uint8_t i = 0; i < len; i++) {
    data[i] = dev->spi->transfer(0x00);
#ifdef E22_DEBUG
    E22_P("[E22]  R[");
    E22_PDEC(i);
    E22_P("]=0x");
    E22_PHEX(data[i]);
    E22_PL("");
#endif
  }
  digitalWrite(dev->nss_pin, HIGH);
  dev->spi->endTransaction();
#ifdef E22_DEBUG
  E22_PL("[E22] RD_REG done");
#endif
}

/**
 * @brief Get IRQ status register
 */
static uint16_t E22_GetIrqStatus(E22_Device *dev) {
  uint8_t status[2] = { 0 };
  E22_ReadCommand(dev, SX1268_CMD_GET_IRQ_STATUS, status, 2);
  return ((uint16_t)status[0] << 8) | status[1];
}

/**
 * @brief Clear IRQ status flags
 */
static void E22_ClearIrqStatus(E22_Device *dev, uint16_t irq_mask) {
  uint8_t params[2];
  params[0] = (irq_mask >> 8) & 0xFF;
  params[1] = irq_mask & 0xFF;
  E22_WriteCommand(dev, SX1268_CMD_CLEAR_IRQ_STATUS, params, 2);
}

/**
 * @brief Read and pretty-print device status (mode + command status)
 *
 * Reads the status byte via GetStatus (opcode 0xC0) and logs human-readable mode and command status.
 * ctx — short context string (stored in RAM by caller; kept small).
 */
static void E22_LogStatus(E22_Device *dev, const char *ctx) {
#ifdef E22_DEBUG
  uint8_t st = 0;

  // Read Command but w/o logs
  E22_WaitBusy(dev);
  dev->spi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dev->nss_pin, LOW);
  dev->spi->transfer(SX1268_CMD_GET_STATUS);
  dev->spi->transfer(0x00);  // NOP / status
  st = dev->spi->transfer(0x00);
  digitalWrite(dev->nss_pin, HIGH);
  dev->spi->endTransaction();

  E22_P("[E22] STS ");
  if (ctx) {
    Serial.print(F("("));
    Serial.print(ctx);
    Serial.print(F(") "));
  }

  // mode = bits 6:4
  uint8_t mode = (st >> 4) & 0x07;
  // cmd status = bits 3:1
  uint8_t cmdst = (st >> 1) & 0x07;

  // print raw
  E22_P("raw=0x");
  E22_PHEX(st);
  E22_PSP();

  // print mode
  E22_P("mode=");
  switch (mode) {
    case 0x2: E22_P("STBY_RC"); break;
    case 0x3: E22_P("STBY_XOSC"); break;
    case 0x4: E22_P("FS"); break;
    case 0x5: E22_P("RX"); break;
    case 0x6: E22_P("TX"); break;
    default:
      E22_P("0x");
      E22_PHEX(mode);
      break;
  }
  E22_PSP();

  // print command status
  E22_P("cstat=");
  switch (cmdst) {
    case 0x2: E22_P("DATA_RDY"); break;  // data available to host
    case 0x3: E22_P("CMD_TIMEOUT"); break;
    case 0x4: E22_P("PROC_ERR"); break;
    case 0x5: E22_P("EXEC_FAIL"); break;
    case 0x6: E22_P("CMD_TX_DONE"); break;
    case 0x0: E22_P("OK"); break;
    default:
      E22_P("0x");
      E22_PHEX(cmdst);
      break;
  }
  E22_PL("");
#endif
}

static void E22_PrintHexBuf(const uint8_t *buf, uint8_t len) {
#ifdef E22_DEBUG
  E22_P("0x");
  for (uint8_t i = 0; i < len; ++i) {
    if (i) E22_PSP();
    if (buf[i] < 0x10) E22_P("0");
    E22_PHEX(buf[i]);
  }
  E22_PL("");
#endif
}

/* ===== Public implementations ===== */

/**
 * @brief Initialize E22 device with default configuration
 */
uint8_t E22_Init(E22_Device *dev) {
#ifdef E22_DEBUG
  E22_PL("[E22] Init");
#endif

  pinMode(dev->nss_pin, OUTPUT);
  pinMode(dev->busy_pin, INPUT);
  pinMode(dev->nrst_pin, OUTPUT);
  pinMode(dev->dio1_pin, INPUT);
  pinMode(dev->txen_pin, OUTPUT);
  pinMode(dev->rxen_pin, OUTPUT);

  digitalWrite(dev->nss_pin, HIGH);
  digitalWrite(dev->txen_pin, LOW);
  digitalWrite(dev->rxen_pin, LOW);

#ifdef E22_DEBUG
  E22_P("[E22] Pins N=");
  E22_PDEC(dev->nss_pin);
  E22_PSP();
  E22_P("B=");
  E22_PDEC(dev->busy_pin);
  E22_PSP();
  E22_P("R=");
  E22_PDEC(dev->nrst_pin);
  E22_PL("");
#endif

  dev->spi->begin();
#ifdef E22_DEBUG
  E22_PL("[E22] SPI started");
#endif

  E22_HardReset(dev);
  delay(10);

  E22_SetStandby(dev);

  uint8_t reg_mode = SX1268_REGULATOR_DC_DC;
#ifdef E22_DEBUG
  E22_P("[E22] RegMode: ");
  E22_PHEX(reg_mode);
  E22_PL("");
#endif
  E22_WriteCommand(dev, SX1268_CMD_SET_REGULATOR_MODE, &reg_mode, 1);

  uint8_t cal_freq[2] = { 0x6B, 0x6F };
#ifdef E22_DEBUG
  E22_P("[E22] CalFreq: ");
  E22_PrintHexBuf(cal_freq, 2);
#endif
  E22_WriteCommand(dev, SX1268_CMD_CALIBRATE_IMAGE, cal_freq, 2);

  uint8_t pa_config[4] = { 0x04, 0x06, 0x00, 0x01 };
#ifdef E22_DEBUG
  E22_P("[E22] PA: ");
  E22_PrintHexBuf(pa_config, 4);
#endif
  E22_WriteCommand(dev, SX1268_CMD_SET_PA_CONFIG, pa_config, 4);

  uint8_t packet_type = SX1268_PACKET_TYPE_LORA;
#ifdef E22_DEBUG
  E22_P("[E22] PacketType: ");
  E22_PHEX(packet_type);
  E22_PL("");
#endif
  E22_WriteCommand(dev, SX1268_CMD_SET_PACKET_TYPE, &packet_type, 1);

  E22_SetFrequency(dev, dev->frequency);
  E22_SetLoraModulation(dev);

  uint8_t tx_params[2];
  tx_params[0] = dev->tx_power;
  tx_params[1] = 0x04;
  E22_WriteCommand(dev, SX1268_CMD_SET_TX_PARAMS, tx_params, 2);

  uint8_t buffer_addr[2] = { 0x00, 0x00 };
  E22_WriteCommand(dev, SX1268_CMD_SET_BUFFER_BASE_ADDRESS, buffer_addr, 2);

  uint8_t irq_params[8];
  irq_params[0] = (SX1268_IRQ_ALL >> 8) & 0xFF;
  irq_params[1] = SX1268_IRQ_ALL & 0xFF;
  irq_params[2] = (SX1268_IRQ_ALL >> 8) & 0xFF;
  irq_params[3] = SX1268_IRQ_ALL & 0xFF;
  irq_params[4] = 0x00;
  irq_params[5] = 0x00;
  irq_params[6] = 0x00;
  irq_params[7] = 0x00;
  E22_WriteCommand(dev, SX1268_CMD_SET_DIO_IRQ_PARAMS, irq_params, 8);

  /* Verify init by writing and reading sync word */
#ifdef E22_DEBUG
  E22_PL("[E22] Verify syncword");
#endif
  uint8_t sync_write[2] = { REG_LR_SYNCWORD_DEFAULT_MSB, REG_LR_SYNCWORD_DEFAULT_LSB };
  E22_WriteRegisterRaw(dev, REG_LR_SYNCWORD, sync_write, 2);

  uint8_t sync_read[2] = { 0, 0 };
  E22_ReadRegisterRaw(dev, REG_LR_SYNCWORD, sync_read, 2);

#ifdef E22_DEBUG
  E22_P("[E22] SYNC R=");
  E22_PHEX(sync_read[0]);
  E22_PSP();
  E22_PHEX(sync_read[1]);
  E22_PL("");
#endif

  if (sync_read[0] != sync_write[0] || sync_read[1] != sync_write[1]) {
#ifdef E22_DEBUG
    E22_PL("[E22] SYNC FAIL");
#endif
    return 0;
  }

#ifdef E22_DEBUG
  E22_PL("[E22] Init OK");
#endif
  return 1;
}

/**
 * @brief Configure LoRa modulation parameters
 */
uint8_t E22_SetLoraModulation(E22_Device *dev) {
  uint8_t mod_params[4];
  mod_params[0] = dev->spreading_factor;
  mod_params[1] = dev->bandwidth;
  mod_params[2] = dev->coding_rate;
  mod_params[3] = 0x00;
  E22_WriteCommand(dev, SX1268_CMD_SET_MODULATION_PARAMS, mod_params, 4);

  uint8_t packet_params[6];
  packet_params[0] = (dev->preamble_length >> 8) & 0xFF;
  packet_params[1] = dev->preamble_length & 0xFF;
  packet_params[2] = 0x00;
  packet_params[3] = SX1268_MAX_PAYLOAD_LENGTH;
  packet_params[4] = 0x01;
  packet_params[5] = 0x00;
  E22_WriteCommand(dev, SX1268_CMD_SET_PACKET_PARAMS, packet_params, 6);

#ifdef E22_DEBUG
  E22_PL("[E22] Mod params set");
  E22_LogStatus(dev, "MOD_SET");
#endif
  return 1;
}

/**
 * @brief Set RF frequency
 */
uint8_t E22_SetFrequency(E22_Device *dev, uint32_t frequency) {
  uint32_t freq_reg = (uint32_t)(((uint64_t)frequency * 33554432ULL) / SX1268_XTAL_FREQ);
  uint8_t params[4];
  params[0] = (freq_reg >> 24) & 0xFF;
  params[1] = (freq_reg >> 16) & 0xFF;
  params[2] = (freq_reg >> 8) & 0xFF;
  params[3] = freq_reg & 0xFF;
#ifdef E22_DEBUG
  E22_P("[E22] Freq ");
  E22_PDEC(frequency);
  E22_P(" Hz reg=0x");
  E22_PHEX(freq_reg);
  E22_PL("");
#endif
  E22_WriteCommand(dev, SX1268_CMD_SET_RF_FREQUENCY, params, 4);
  dev->frequency = frequency;
#ifdef E22_DEBUG
  E22_LogStatus(dev, "SET_FREQ");
#endif
  return 1;
}

/**
 * @brief Send data packet (blocking mode)
 */
uint8_t E22_SendData(E22_Device *dev, const uint8_t *data, uint8_t length) {
  if (length > SX1268_MAX_PAYLOAD_LENGTH) return 0;

  E22_SetStandby(dev);
  E22_ClearIrqStatus(dev, SX1268_IRQ_ALL);

#ifdef E22_DEBUG
  E22_P("[E22] Send len=");
  E22_PDEC(length);
  E22_PL("");
#endif

  E22_WaitBusy(dev);
  dev->spi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dev->nss_pin, LOW);

  dev->spi->transfer(SX1268_CMD_WRITE_BUFFER);
  dev->spi->transfer(0x00);
#ifdef E22_DEBUG
  E22_P("[E22] TX DATA: ");
  E22_PrintHexBuf(data, length);
#endif
  for (uint8_t i = 0; i < length; i++) {
    dev->spi->transfer(data[i]);
  }

  digitalWrite(dev->nss_pin, HIGH);
  dev->spi->endTransaction();

  uint8_t packet_params[6];
  packet_params[0] = (dev->preamble_length >> 8) & 0xFF;
  packet_params[1] = dev->preamble_length & 0xFF;
  packet_params[2] = 0x00;
  packet_params[3] = length;
  packet_params[4] = 0x01;
  packet_params[5] = 0x00;
  E22_WriteCommand(dev, SX1268_CMD_SET_PACKET_PARAMS, packet_params, 6);

  E22_SetRfSwitch(dev, 1);

  uint8_t tx_params[3] = { 0x00, 0x00, 0x00 };
  E22_WriteCommand(dev, SX1268_CMD_SET_TX, tx_params, 3);

#ifdef E22_DEBUG
  E22_PL("[E22] TX started");
  E22_LogStatus(dev, "SET_TX");
#endif

  uint32_t timeout = millis() + SX1268_DEFAULT_TIMEOUT_MS;
  while (1) {
    uint16_t irq_status = E22_GetIrqStatus(dev);
    if (irq_status & SX1268_IRQ_TX_DONE) {
      E22_ClearIrqStatus(dev, SX1268_IRQ_TX_DONE);
      E22_SetStandby(dev);
      E22_SetRfSwitch(dev, 0);
#ifdef E22_DEBUG
      E22_PL("[E22] TX done");
      E22_LogStatus(dev, "POST_TX");
#endif
      return 1;
    }
    if (irq_status & SX1268_IRQ_TIMEOUT) {
      E22_ClearIrqStatus(dev, SX1268_IRQ_TIMEOUT);
      E22_SetStandby(dev);
      E22_SetRfSwitch(dev, 0);
#ifdef E22_DEBUG
      E22_PL("[E22] TX irq timeout");
      E22_LogStatus(dev, "TX_ERR");
#endif
      return 0;
    }
    if (millis() > timeout) {
      E22_SetStandby(dev);
      E22_SetRfSwitch(dev, 0);
#ifdef E22_DEBUG
      E22_PL("[E22] TX timeout");
      E22_LogStatus(dev, "TX_TO");
#endif
      return 0;
    }
    delay(1);
  }
}

/**
 * @brief Receive data packet (blocking mode)
 */
uint8_t E22_ReceiveData(E22_Device *dev, uint8_t *buffer, uint8_t max_length, uint32_t timeout_ms) {
  E22_SetStandby(dev);
  E22_ClearIrqStatus(dev, SX1268_IRQ_ALL);
  E22_SetRfSwitch(dev, 0);

  uint64_t timeout_val = ((uint64_t)timeout_ms * 1000 * 1000) / 15625;
  if (timeout_val > 0xFFFFFF) timeout_val = 0xFFFFFF;
  uint8_t rx_params[3];
  rx_params[0] = (timeout_val >> 16) & 0xFF;
  rx_params[1] = (timeout_val >> 8) & 0xFF;
  rx_params[2] = timeout_val & 0xFF;
  E22_WriteCommand(dev, SX1268_CMD_SET_RX, rx_params, 3);

#ifdef E22_DEBUG
  E22_P("[E22] RX start t=");
  E22_PDEC(timeout_ms);
  E22_PL(" ms");
  E22_LogStatus(dev, "SET_RX");
#endif

  uint32_t start_time = millis();
  while (1) {
    uint16_t irq_status = E22_GetIrqStatus(dev);
    if (irq_status & SX1268_IRQ_RX_DONE) {
      if (irq_status & SX1268_IRQ_CRC_ERR) {
        E22_ClearIrqStatus(dev, SX1268_IRQ_ALL);
        E22_SetStandby(dev);
#ifdef E22_DEBUG
        E22_PL("[E22] RX crc err");
        E22_LogStatus(dev, "RX_CRC");
#endif
        return 0;
      }

      uint8_t rx_status[2];
      E22_ReadCommand(dev, SX1268_CMD_GET_RX_BUFFER_STATUS, rx_status, 2);
      uint8_t payload_length = rx_status[0];
      uint8_t rx_start_ptr = rx_status[1];

#ifdef E22_DEBUG
      E22_P("[E22] RX len=");
      E22_PDEC(payload_length);
      E22_P(" ptr=0x");
      E22_PHEX(rx_start_ptr);
      E22_PL("");
#endif

      if (payload_length > max_length) payload_length = max_length;

      E22_WaitBusy(dev);
      dev->spi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
      digitalWrite(dev->nss_pin, LOW);

      dev->spi->transfer(SX1268_CMD_READ_BUFFER);
      dev->spi->transfer(rx_start_ptr);
      dev->spi->transfer(0x00);
      for (uint8_t i = 0; i < payload_length; i++) {
        buffer[i] = dev->spi->transfer(0x00);
      }
#ifdef E22_DEBUG
      E22_P("[E22] RX DATA: ");
      E22_PrintHexBuf(buffer, payload_length);
#endif


      digitalWrite(dev->nss_pin, HIGH);
      dev->spi->endTransaction();

      E22_ClearIrqStatus(dev, SX1268_IRQ_ALL);
      E22_SetStandby(dev);
#ifdef E22_DEBUG
      E22_LogStatus(dev, "POST_RX");
#endif
      return payload_length;
    }

    if (irq_status & SX1268_IRQ_TIMEOUT) {
      E22_ClearIrqStatus(dev, SX1268_IRQ_TIMEOUT);
      E22_SetStandby(dev);
#ifdef E22_DEBUG
      E22_PL("[E22] RX irq timeout");
      E22_LogStatus(dev, "RX_TO");
#endif
      return 0;
    }

    if (millis() - start_time > timeout_ms) {
      E22_SetStandby(dev);
#ifdef E22_DEBUG
      E22_PL("[E22] RX timeout");
      E22_LogStatus(dev, "RX_TO2");
#endif
      return 0;
    }
    delay(1);
  }
}

/**
 * @brief Enter standby mode
 */
uint8_t E22_SetStandby(E22_Device *dev) {
  uint8_t standby_mode = SX1268_STANDBY_RC;
  E22_WriteCommand(dev, SX1268_CMD_SET_STANDBY, &standby_mode, 1);
  E22_SetRfSwitch(dev, 0);
#ifdef E22_DEBUG
  E22_PL("[E22] Standby (RC)");
  E22_LogStatus(dev, "SET_STBY");
#endif
  return 1;
}

/**
 * @brief Get RSSI of last received packet
 */
int16_t E22_GetRssi(E22_Device *dev) {
  uint8_t packet_status[3];
  E22_ReadCommand(dev, SX1268_CMD_GET_PACKET_STATUS, packet_status, 3);
  int16_t rssi = -(int16_t)packet_status[0] / 2;
#ifdef E22_DEBUG
  E22_P("[E22] RSSI raw=0x");
  E22_PHEX(packet_status[0]);
  E22_P(" => ");
  E22_PDEC(rssi);
  E22_PL(" dBm");
#endif
  return rssi;
}

/**
 * @brief Get SNR of last received packet (LoRa only)
 */
int8_t E22_GetSnr(E22_Device *dev) {
  uint8_t packet_status[3];
  E22_ReadCommand(dev, SX1268_CMD_GET_PACKET_STATUS, packet_status, 3);
  int8_t snr = (int8_t)packet_status[1] / 4;
#ifdef E22_DEBUG
  E22_P("[E22] SNR raw=0x");
  E22_PHEX(packet_status[1]);
  E22_P(" => ");
  E22_PDEC(snr);
  E22_PL(" dB");
#endif
  return snr;
}

#endif  // E22_SX1268_H
