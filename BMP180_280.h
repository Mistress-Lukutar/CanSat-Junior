#pragma once
#include "IICFuncs.h"

/**
 * @brief Default I2C addresses for BMP sensors
 */
enum BMP_Address {
  BMP280_ADDR = 0x76,
  BMP180_ADDR = 0x77
};

/**
 * @brief Default chip IDs for BMP sensors
 */
enum BMP_ChipID {
  BMP280_ID = 0x58,
  BMP180_ID = 0x55
};

/**
 * @brief BMP280 operation modes
 */
enum BMP280_Mode {
  SLEEP = 0b00,
  FORCED = 0b01,
  NORMAL = 0b11
};

/**
 * @brief BMP280 oversampling settings
 */
enum BMP280_OS {
  OS_SKIP = 0b000,
  OS_x1 = 0b001,
  OS_x2 = 0b010,
  OS_x4 = 0b011,
  OS_x8 = 0b100,
  OS_x16 = 0b111
};

/**
 * @brief BMP280 IIR filter coefficients
 */
enum BMP280_Filter {
  IIR_OFF = 0b000,
  IIR_2 = 0b001,
  IIR_4 = 0b010,
  IIR_8 = 0b011,
  IIR_16 = 0b100
};

/**
 * @brief BMP280 standby time settings
 */
enum BMP280_Standby {
  STBY_0_5 = (0b000 << 5),
  STBY_62_5 = (0b001 << 5),
  STBY_125 = (0b010 << 5),
  STBY_250 = (0b011 << 5),
  STBY_500 = (0b100 << 5),
  STBY_1000 = (0b101 << 5),
  STBY_2000 = (0b110 << 5),
  STBY_4000 = (0b111 << 5)
};

/**
 * @brief BMP180 oversampling modes
 */
enum BMP180_OSS {
  OSS_ULP = (0 << 6),  // Ultra low power (4.5ms)
  OSS_STD = (1 << 6),  // Standard (7.5ms)
  OSS_HR = (2 << 6),   // High resolution (13.5ms)
  OSS_UHR = (3 << 6)   // Ultra high resolution (25.5ms)
};

/**
 * @brief BMP sensor type identifiers
 */
enum BMP_Type {
  BMP180,
  BMP280
};

/**
 * @brief BMP280 register map structure
 */
typedef struct {
  uint8_t calib00;      // 0x88
  uint8_t chipID;       // 0xD0
  uint8_t reset;        // 0xE0
  uint8_t status;       // 0xF3
  uint8_t ctrl_meas;    // 0xF4
  uint8_t config;       // 0xF5
  uint8_t pressureMSB;  // 0xF7
  uint8_t tempMSB;      // 0xFA
} BMP280_RegMap;

/**
 * @brief BMP180 register map structure
 */
typedef struct {
  uint8_t calib00;    // 0xAA
  uint8_t chipID;     // 0xD0
  uint8_t control;    // 0xF4
  uint8_t resultMSB;  // 0xF6
} BMP180_RegMap;

/**
 * @brief BMP280 calibration data structure
 */
typedef struct {
  uint16_t dig_T1;
  int16_t dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} BMP280_CalData;

/**
 * @brief BMP180 calibration data structure
 */
typedef struct {
  int16_t ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t b1, b2;
  int16_t mb, mc, md;
} BMP180_CalData;

/**
 * @brief BMP sensor raw data structure
 */
typedef struct {
  int32_t pressure;
  int32_t temperature;
} BMP_RawData;

/**
 * @brief Main BMP sensor device structure
 */
typedef struct {
  BMP_Type type;
  uint8_t address;
  uint8_t ctrl_meas;
  uint8_t config;
  BMP_RawData raw;
  union {
    BMP280_CalData cal280;
    BMP180_CalData cal180;
  } cal;
} BMP_Device;

// Register maps
const BMP280_RegMap bmp280Regs = { 0x88, 0xD0, 0xE0, 0xF3, 0xF4, 0xF5, 0xF7, 0xFA };
const BMP180_RegMap bmp180Regs = { 0xAA, 0xD0, 0xF4, 0xF6 };

/**
 * @brief Read calibration data for BMP280
 * @param dev Pointer to BMP device structure
 */
void BMP280_ReadCalData(BMP_Device* dev) {
  uint8_t* pt = (uint8_t*)&dev->cal.cal280;
  IIC_Request(dev->address, bmp280Regs.calib00, 24);
  for (uint8_t i = 0; i < sizeof(dev->cal.cal280); i++) {
    pt[i] = IIC_Read();
  }
}

/**
 * @brief Read calibration data for BMP180
 * @param dev Pointer to BMP device structure
 */
void BMP180_ReadCalData(BMP_Device* dev) {
  uint8_t* pt = (uint8_t*)&dev->cal.cal180;
  IIC_Request(dev->address, bmp180Regs.calib00, 22);
  for (uint8_t i = 0; i < sizeof(dev->cal.cal180); i += 2) {
    pt[i + 1] = IIC_Read();
    pt[i] = IIC_Read();
  }
}

/**
 * @brief Initialize BMP sensor
 * @param dev Pointer to BMP device structure
 * @return true if initialization succeeded, false otherwise
 */
bool BMP_Init(BMP_Device* dev) {
  IIC_Begin();

  // Try BMP280 first
  dev->address = BMP_Address::BMP280_ADDR;
  uint8_t id = IIC_ReadByte(dev->address, bmp280Regs.chipID);
  if (id == BMP_ChipID::BMP280_ID) {
    dev->type = BMP_Type::BMP280;
    BMP280_ReadCalData(dev);
    dev->ctrl_meas = (BMP280_Mode::NORMAL | (BMP280_OS::OS_x8 << 2) | (BMP280_OS::OS_x1 << 5));
    dev->config = BMP280_Filter::IIR_8 | BMP280_Standby::STBY_0_5;
    IIC_WriteByte(dev->address, bmp280Regs.ctrl_meas, dev->ctrl_meas);
    IIC_WriteByte(dev->address, bmp280Regs.config, dev->config);
    return true;
  }

  // Try BMP180 if BMP280 not found
  dev->address = BMP_Address::BMP180_ADDR;
  id = IIC_ReadByte(dev->address, bmp180Regs.chipID);
  if (id == BMP_ChipID::BMP180_ID) {
    dev->type = BMP_Type::BMP180;
    dev->ctrl_meas = BMP180_OSS::OSS_STD;
    IIC_WriteByte(dev->address, bmp180Regs.control, dev->config);
    BMP180_ReadCalData(dev);
    return true;
  }

  return false;
}

/**
 * @brief Read raw temperature from BMP180
 * @param dev Pointer to BMP device structure
 * @return Raw temperature value
 */
int32_t BMP180_ReadRawTemp(BMP_Device* dev) {
  IIC_WriteByte(dev->address, bmp180Regs.control, 0x2E);
  _delay_ms(5);  // Max conversion time 4.5ms
  IIC_Request(dev->address, bmp180Regs.resultMSB, 2);
  return (int32_t)(((uint16_t)IIC_Read() << 8) | IIC_Read());
}

/**
 * @brief Read raw pressure from BMP180
 * @param dev Pointer to BMP device structure
 * @return Raw pressure value
 */
int32_t BMP180_ReadRawPressure(BMP_Device* dev) {
  uint8_t cmd = 0x34 | dev->ctrl_meas;
  IIC_WriteByte(dev->address, bmp180Regs.control, cmd);

 // Wait according to oversampling setting
  switch (dev->ctrl_meas) {
    case BMP180_OSS::OSS_ULP: _delay_ms(5); break;
    case BMP180_OSS::OSS_STD: _delay_ms(8); break;
    case BMP180_OSS::OSS_HR: _delay_ms(14); break;
    case BMP180_OSS::OSS_UHR: _delay_ms(26); break;
  }

  IIC_Request(dev->address, bmp180Regs.resultMSB, 3);
  return (((int32_t)IIC_Read() << 16) | ((int32_t)IIC_Read() << 8) | IIC_Read()) >> (8 - (dev->ctrl_meas>>6));
}

/**
 * @brief Read and calculate compensated temperature and pressure
 * @param dev Pointer to BMP device structure
 * @param temperature Output for compensated temperature (centigrade * 100)
 * @param pressure Output for compensated pressure (Pa)
 * @return true if successful, false otherwise
 */
bool BMP_ReadData(BMP_Device* dev, int32_t* temperature, uint32_t* pressure) {
  int32_t var1, var2, t_fine;

  if (dev->type == BMP_Type::BMP280) {
    // BMP280 measurement logic
    if (dev->ctrl_meas & BMP280_Mode::SLEEP) {
      IIC_WriteByte(dev->address, bmp280Regs.ctrl_meas, dev->ctrl_meas | BMP280_Mode::FORCED);
    }

    // Wait for measurement completion
    while (IIC_ReadByte(dev->address, bmp280Regs.status) & (1 << 3)) {};

    // Read raw data
    IIC_Request(dev->address, bmp280Regs.pressureMSB, 6);
    dev->raw.pressure = (int32_t)IIC_Read() << 12 | (int32_t)IIC_Read() << 4 | (int32_t)IIC_Read() >> 4;
    dev->raw.temperature = (int32_t)IIC_Read() << 12 | (int32_t)IIC_Read() << 4 | (int32_t)IIC_Read() >> 4;

    // Temperature compensation
    var1 = (((dev->raw.temperature >> 3) - ((int32_t)dev->cal.cal280.dig_T1 << 1))) * ((int32_t)dev->cal.cal280.dig_T2) >> 11;
    var2 = (((((dev->raw.temperature >> 4) - ((int32_t)dev->cal.cal280.dig_T1)) * ((dev->raw.temperature >> 4) - ((int32_t)dev->cal.cal280.dig_T1))) >> 12) * ((int32_t)dev->cal.cal280.dig_T3)) >> 14;
    t_fine = var1 + var2;
    *temperature = (t_fine * 5 + 128) >> 8;

    // Pressure compensation
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dev->cal.cal280.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dev->cal.cal280.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dev->cal.cal280.dig_P4) << 16);
    var1 = (((dev->cal.cal280.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dev->cal.cal280.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)dev->cal.cal280.dig_P1)) >> 15);
    if (var1 == 0) return false;

    *pressure = (((uint32_t)(((int32_t)1048576) - dev->raw.pressure) - (var2 >> 12))) * 3125;
    *pressure = (*pressure < 0x80000000) ? (*pressure << 1) / ((uint32_t)var1) : (*pressure / (uint32_t)var1) * 2;

    var1 = (((int32_t)dev->cal.cal280.dig_P9) * ((int32_t)(((*pressure >> 3) * (*pressure >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(*pressure >> 2)) * ((int32_t)dev->cal.cal280.dig_P8)) >> 13;
    *pressure = (uint32_t)((int32_t)*pressure + ((var1 + var2 + dev->cal.cal280.dig_P7) >> 4));
  } else {
    // BMP180 measurement logic
    int32_t ut = BMP180_ReadRawTemp(dev);
    int32_t up = BMP180_ReadRawPressure(dev);

    // Temperature compensation
    var1 = ((ut - (int32_t)dev->cal.cal180.ac6) * (int32_t)dev->cal.cal180.ac5) >> 15;
    var2 = ((int32_t)dev->cal.cal180.mc << 11) / (var1 + (int32_t)dev->cal.cal180.md);
    t_fine = var1 + var2;
    *temperature = (int32_t)(((t_fine + 8) >> 4) * 10.0);
    // Pressure compensation
    int32_t b6 = t_fine - 4000;
    var1 = ((int32_t)dev->cal.cal180.b2 * ((b6 * b6) >> 12)) >> 11;
    var2 = ((int32_t)dev->cal.cal180.ac2 * b6) >> 11;
    int32_t x3 = var1 + var2;
    int32_t b3 = ((((int32_t)dev->cal.cal180.ac1 * 4 + x3) << (dev->ctrl_meas >> 6)) + 2) >> 2;
    var1 = ((int32_t)dev->cal.cal180.ac3 * b6) >> 13;
    var2 = ((int32_t)dev->cal.cal180.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((var1 + var2) + 2) >> 2;
    uint32_t b4 = ((uint32_t)dev->cal.cal180.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)up - b3) * (uint32_t)(50000UL >> (dev->ctrl_meas >> 6));
    int32_t p;
    if (b7 < 0x80000000) {
      p = (b7 << 1) / b4;
    } else {
      p = (b7 / b4) << 1;
    }
    var1 = (p >> 8) * (p >> 8);
    var1 = (var1 * 3038) >> 16;
    var2 = (-7357 * p) >> 16;
    p += (var1 + var2 + (int32_t)3791) >> 4;
    *pressure = (uint32_t)p;
  }

  return true;
}

/**
 * @brief Calculate altitude based on pressure and sea level pressure
 * @param pressure Current pressure in Pa
 * @param seaLevelPressure Sea level pressure in Pa
 * @return Altitude in cm
 */
int32_t BMP_GetAltitude(uint32_t* pressure, uint32_t* seaLevelPressure) {
  return 4433000 * (1.0f - pow((float)*pressure / *seaLevelPressure, 0.1903));
}