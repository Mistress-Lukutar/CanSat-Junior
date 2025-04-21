#pragma once
#include "IICFuncs.h"

/**
 * @def BMP280_DEFAULT_ADDR
 * @brief Default I2C address for BMP280 sensor
 */
#define BMP280_DEFAULT_ADDR 0x76
/**
 * @def BMP180_DEFAULT_ADDR
 * @brief Default I2C address for BMP180 sensor
 */
#define BMP180_DEFAULT_ADDR 0x77

/**
 * @def BMP280_DEFAULT_ID
 * @brief Default chip ID for BMP280 sensor
 */
#define BMP280_DEFAULT_ID 0x58
/**
 * @def BMP180_DEFAULT_ID
 * @brief Default chip ID for BMP180 sensor
 */
#define BMP180_DEFAULT_ID 0x55

// BMP280 specific defines
#define BMP280_MODE_SLEEP 0b00
#define BMP280_MODE_NORMAL 0b11
#define BMP280_MODE_FORCED 0b01

#define BMP280_PRESS_OSx0 0b000 << 2
#define BMP280_PRESS_OSx1 0b001 << 2
#define BMP280_PRESS_OSx2 0b010 << 2
#define BMP280_PRESS_OSx4 0b011 << 2
#define BMP280_PRESS_OSx8 0b100 << 2
#define BMP280_PRESS_OSx16 0b111 << 2

#define BMP280_TEMP_OSx0 0b000 << 5
#define BMP280_TEMP_OSx1 0b001 << 5
#define BMP280_TEMP_OSx2 0b010 << 5
#define BMP280_TEMP_OSx4 0b011 << 5
#define BMP280_TEMP_OSx8 0b100 << 5
#define BMP280_TEMP_OSx16 0b111 << 5

#define BMP280_IIR_0 0b000 << 2
#define BMP280_IIR_2 0b001 << 2
#define BMP280_IIR_4 0b010 << 2
#define BMP280_IIR_8 0b011 << 2
#define BMP280_IIR_16 0b100 << 2

#define BMP280_STBY_0_5 0b000 << 5
#define BMP280_STBY_62_5 0b001 << 5
#define BMP280_STBY_125 0b010 << 5
#define BMP280_STBY_250 0b011 << 5
#define BMP280_STBY_500 0b100 << 5
#define BMP280_STBY_1000 0b101 << 5
#define BMP280_STBY_2000 0b110 << 5
#define BMP280_STBY_4000 0b111 << 5

// BMP180 specific defines
#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6
#define BMP180_CMD_TEMP 0x2E
#define BMP180_CMD_PRESS 0x34

// Oversampling modes for BMP180
enum BMP180_OSS {
  BMP180_OSS_ULP = 0,  // Ultra low power
  BMP180_OSS_STD = 1,  // Standard
  BMP180_OSS_HR = 2,   // High resolution
  BMP180_OSS_UHR = 3   // Ultra high resolution
};

/**
 * @struct BMP280_RegMap
 * @brief Register map for BMP280 sensor
 */
struct BMP280_RegMap {
  const uint8_t calib00;      // Calibration Data 00 (0x88)
  const uint8_t chipID;       // Device ID (0xD0)
  const uint8_t reset;        // Soft Reset (0xE0)
  const uint8_t status;       // Sensor Status (0xF3)
  const uint8_t ctrl_meas;    // Control Measurement (0xF4)
  const uint8_t config;       // Configuration (0xF5)
  const uint8_t pressureMSB;  // Pressure Data (MSB) (0xF7)
  const uint8_t tempMSB;      // Temperature Data (MSB) (0xFA)
};

/**
 * @struct BMP180_RegMap
 * @brief Register map for BMP180 sensor
 */
struct BMP180_RegMap {
  const uint8_t calib00;    // Calibration Data 00 (0xAA)
  const uint8_t chipID;     // Device ID (0xD0)
  const uint8_t control;    // Control register (0xF4)
  const uint8_t resultMSB;  // Result Data (MSB) (0xF6)
};

/**
 * @struct BMP280_CalData
 * @brief Calibration data structure for BMP280
 */
struct BMP280_CalData {
  uint16_t dig_T1;
  int16_t dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} calData280;

/**
 * @struct BMP180_CalData
 * @brief Calibration data structure for BMP180
 */
struct BMP180_CalData {
  int16_t AC1, AC2, AC3;
  uint16_t AC4, AC5, AC6;
  int16_t b1, b2;
  int16_t MB, MC, MD;
} calData180;

/**
 * @struct BMP280_bmp280RawData
 * @brief Raw data structure for BMP280/BMP180
 */
struct BMP280_bmp280RawData {
  int32_t Pressure, Temperature;
} bmpRawData;

uint8_t bmp_addr;
uint8_t bmp_type;  // 0 = BMP180, 1 = BMP280
uint8_t bmp_mode;
uint8_t bmp_oss;  // Oversampling setting for BMP180
uint8_t bmp_ctrl_meas;

BMP280_RegMap bmp280Regs = { 0x88, 0xD0, 0xE0, 0xF3, 0xF4, 0xF5, 0xF7, 0xFA };
BMP180_RegMap bmp180Regs = { 0xAA, 0xD0, 0xF4, 0xF6 };

/**
 * @brief Read calibration data from BMP280
 */
inline void BMP280_ReadCalData() {
  uint8_t* pt = (uint8_t*)&calData280;
  IIC_Request(bmp_addr, bmp280Regs.calib00, 24);
  for (uint8_t i = 0; i < sizeof(calData280); i++) pt[i] = IIC_Read();
}

/**
 * @brief Read calibration data from BMP180
 */
inline void BMP180_ReadCalData() {
  uint8_t* pt = (uint8_t*)&calData180;
  IIC_Request(bmp_addr, bmp180Regs.calib00, 22);
  for (uint8_t i = 0; i < sizeof(calData180); i++) pt[i] = IIC_Read();
}

/**
 * @brief Initialize BMP sensor
 * @param ctrl_measByte Control measurement byte for BMP280
 * @param configByte Configuration byte for BMP280
 * @param oss Oversampling setting for BMP180
 * @param altAddr Alternative I2C address
 * @return true if initialization succeeded, false otherwise
 */
bool BMP_Init(uint8_t ctrl_measByte = BMP280_MODE_SLEEP | BMP280_PRESS_OSx8 | BMP280_TEMP_OSx1,
              uint8_t configByte = BMP280_IIR_8 | BMP280_STBY_0_5,
              uint8_t oss = BMP180_OSS_STD) {
  bmp_ctrl_meas = ctrl_measByte;
  bmp_oss = oss;
  IIC_Begin();

  // Try BMP280 first
  bmp_addr = BMP280_DEFAULT_ADDR;
  uint8_t id = IIC_ReadByte(bmp_addr, bmp280Regs.chipID);
  if (id == BMP280_DEFAULT_ID) {
    bmp_type = 1;
    BMP280_ReadCalData();
    IIC_WriteByte(bmp_addr, bmp280Regs.ctrl_meas, ctrl_measByte);
    IIC_WriteByte(bmp_addr, bmp280Regs.config, configByte);
    Serial.println("BMP280");
    return true;
  }

  // Try BMP180 if BMP280 not found
  bmp_addr = BMP180_DEFAULT_ADDR;
  id = IIC_ReadByte(bmp_addr, bmp180Regs.chipID);
  if (id == BMP180_DEFAULT_ID) {
    bmp_type = 0;
    BMP180_ReadCalData();
    Serial.println("BMP180");
    return true;
  }
  Serial.println("BMP_ERR: " + String(id));
  return false;
}

/**
 * @brief Read raw temperature from BMP180
 * @return Raw temperature value
 */
int32_t BMP180_ReadRawTemp() {
  IIC_WriteByte(bmp_addr, bmp180Regs.control, BMP180_CMD_TEMP);
  _delay_ms(5);  // Max conversion time 4.5ms
  IIC_Request(bmp_addr, bmp180Regs.resultMSB, 2);
  return ((int32_t)IIC_Read() << 8) | (int32_t)IIC_Read();
}

/**
 * @brief Read raw pressure from BMP180
 * @return Raw pressure value
 */
int32_t BMP180_ReadRawPressure() {
  uint8_t cmd = BMP180_CMD_PRESS | (bmp_oss << 6);
  IIC_WriteByte(bmp_addr, bmp180Regs.control, cmd);

  // Wait according to oversampling setting
  switch (bmp_oss) {
    case BMP180_OSS_ULP: _delay_ms(5); break;
    case BMP180_OSS_STD: _delay_ms(8); break;
    case BMP180_OSS_HR: _delay_ms(14); break;
    case BMP180_OSS_UHR: _delay_ms(26); break;
  }

  IIC_Request(bmp_addr, bmp180Regs.resultMSB, 3);
  return (((int32_t)IIC_Read() << 16) | ((int32_t)IIC_Read() << 8) | (int32_t)IIC_Read()) >> (8 - bmp_oss);
}

/**
 * @brief Read temperature and pressure data from sensor
 * @param temperature Pointer to store temperature (centigrade * 10^2)
 * @param pressure Pointer to store pressure (Pa)
 * @return true if read succeeded, false otherwise
 */
bool BMP_ReadData(int32_t* temperature, uint32_t* pressure) {
  static int32_t var1, var2, t_fine;
  static bool pr_measuring, measuring;

  if (bmp_type) {  // BMP280
    if (bmp_ctrl_meas | BMP280_MODE_SLEEP) {
      IIC_WriteByte(bmp_addr, bmp280Regs.ctrl_meas, bmp_ctrl_meas | BMP280_MODE_FORCED);
    }

    while (1) {
      measuring = IIC_ReadByte(bmp_addr, bmp280Regs.status) & (1 << 3);
      if (pr_measuring ^ measuring) {
        pr_measuring = measuring;
        if (!measuring) break;
      }
    }

    IIC_Request(bmp_addr, bmp280Regs.pressureMSB, 6);
    bmpRawData.Pressure = (int32_t)IIC_Read() << 12 | (int32_t)IIC_Read() << 4 | (int32_t)IIC_Read() >> 4;
    bmpRawData.Temperature = (int32_t)IIC_Read() << 12 | (int32_t)IIC_Read() << 4 | (int32_t)IIC_Read() >> 4;

    // Temperature calculation
    var1 = ((((bmpRawData.Temperature >> 3) - ((int32_t)calData280.dig_T1 << 1))) * ((int32_t)calData280.dig_T2)) >> 11;
    var2 = (((((bmpRawData.Temperature >> 4) - ((int32_t)calData280.dig_T1)) * ((bmpRawData.Temperature >> 4) - ((int32_t)calData280.dig_T1))) >> 12) * ((int32_t)calData280.dig_T3)) >> 14;
    t_fine = var1 + var2;

    // Pressure calculation for BMP280
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calData280.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calData280.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)calData280.dig_P4) << 16);
    var1 = (((calData280.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)calData280.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)calData280.dig_P1)) >> 15);
    if (var1 == 0) return false;

    *pressure = (((uint32_t)(((int32_t)1048576) - bmpRawData.Pressure) - (var2 >> 12))) * 3125;
    if (*pressure < 0x80000000) *pressure = (*pressure << 1) / ((uint32_t)var1);
    else *pressure = (*pressure / (uint32_t)var1) * 2;

    var1 = (((int32_t)calData280.dig_P9) * ((int32_t)(((*pressure >> 3) * (*pressure >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(*pressure >> 2)) * ((int32_t)calData280.dig_P8)) >> 13;
    *pressure = (uint32_t)((int32_t)*pressure + ((var1 + var2 + calData280.dig_P7) >> 4));

    *temperature = (t_fine * 5 + 128) >> 8;
  } else {  // BMP180
    // Read raw temperature
    int32_t UT = BMP180_ReadRawTemp();
    int32_t UP = BMP180_ReadRawPressure();

    // Calculate true temperature
    int32_t X1 = ((UT - (int32_t)calData180.AC6) * (int32_t)calData180.AC5) >> 15;
    int32_t X2 = ((int32_t)calData180.MC << 11) / (X1 + (int32_t)calData180.MD);
    t_fine = X1 + X2;
    *temperature = (t_fine + 8) >> 4;  // 0.1°C
    Serial.println(UT);
    Serial.println(X1);
    Serial.println(X2);
    Serial.println(t_fine);
    Serial.println(*temperature);
    // Calculate true pressure
    int32_t B6 = t_fine - 4000;
    X1 = ((int32_t)calData180.b2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)calData180.AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = ((((int32_t)calData180.AC1 * 4 + X3) << bmp_oss) + 2) >> 2;

    X1 = ((int32_t)calData180.AC3 * B6) >> 13;
    X2 = ((int32_t)calData180.b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = ((uint32_t)calData180.AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)UP - (uint32_t)B3) * (50000UL >> bmp_oss);

    int32_t p;
    if (B7 < 0x80000000)
      p = (B7 << 1) / B4;
    else
      p = (B7 / B4) << 1;

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    *pressure = p + ((X1 + X2 + 3791) >> 4);  // Pressure in Pa
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