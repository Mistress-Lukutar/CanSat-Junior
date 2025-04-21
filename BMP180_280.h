#pragma once
#include "IICFuncs.h"
#define BMP280_DEFAULT_ADDR 0x76
#define BMP280_DEFAULT_ID 0x58

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

struct BMP280_CalData {
  uint16_t dig_T1;
  int16_t dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} calData;

struct BMP280_bmp280RawData {
  int32_t Pressure, Temperature;
} bmp280RawData;

uint8_t bmp280_addr;
uint8_t bmp280_mode;
uint8_t bmp280_ctrl_meas;

BMP280_RegMap bmpRegs = { 0x88, 0xD0, 0xE0, 0xF3, 0xF4, 0xF5, 0xF7, 0xFA };

inline void BMP280_ReadCalData() {
  uint8_t* pt = (uint8_t*)&calData;
  IIC_Request(bmp280_addr, bmpRegs.calib00, 24);
  for (uint8_t i = 0; i < sizeof(calData); i++) pt[i] = IIC_Read();
}

bool BMP280_Init(uint8_t ctrl_measByte = BMP280_MODE_SLEEP | BMP280_PRESS_OSx8 | BMP280_TEMP_OSx1, uint8_t configByte = BMP280_IIR_8 | BMP280_STBY_0_5, uint8_t altAddr = BMP280_DEFAULT_ADDR) {
  bmp280_addr = altAddr;
  bmp280_ctrl_meas = ctrl_measByte;
  IIC_Begin();
  if (IIC_ReadByte(bmp280_addr, bmpRegs.chipID) != BMP280_DEFAULT_ID) return false;
  BMP280_ReadCalData();
  IIC_WriteByte(bmp280_addr, bmpRegs.ctrl_meas, ctrl_measByte);
  IIC_WriteByte(bmp280_addr, bmpRegs.config, configByte);
  return true;
}

bool BMP280_ReadData(int32_t* temperature, uint32_t* pressure) {  //Pressure in Pa, temperature in (centigrade * 10^2)
  static int32_t var1, var2, t_fine;
  static bool pr_measuring, measuring;
  if (bmp280_ctrl_meas | BMP280_MODE_SLEEP) IIC_WriteByte(bmp280_addr, bmpRegs.ctrl_meas, bmp280_ctrl_meas | BMP280_MODE_FORCED);
  while (1) {
    measuring = IIC_ReadByte(bmp280_addr, bmpRegs.status) & (1 << 3);
    if (pr_measuring ^ measuring) {
      pr_measuring = measuring;
      if (!measuring) break;
    }
  }
  IIC_Request(bmp280_addr, bmpRegs.pressureMSB, 6);
  bmp280RawData.Pressure = (int32_t)IIC_Read() << 12 | (int32_t)IIC_Read() << 4 | (int32_t)IIC_Read() >> 4;
  bmp280RawData.Temperature = (int32_t)IIC_Read() << 12 | (int32_t)IIC_Read() << 4 | (int32_t)IIC_Read() >> 4;

  var1 = ((((bmp280RawData.Temperature >> 3) - ((int32_t)calData.dig_T1 << 1))) * ((int32_t)calData.dig_T2)) >> 11;
  var2 = (((((bmp280RawData.Temperature >> 4) - ((int32_t)calData.dig_T1)) * ((bmp280RawData.Temperature >> 4) - ((int32_t)calData.dig_T1))) >> 12) * ((int32_t)calData.dig_T3)) >> 14;
  t_fine = var1 + var2;
  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calData.dig_P6);
  var2 = var2 + ((var1 * ((int32_t)calData.dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)calData.dig_P4) << 16);
  var1 = (((calData.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)calData.dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)calData.dig_P1)) >> 15);
  if (var1 == 0) return 0;
  *pressure = (((uint32_t)(((int32_t)1048576) - bmp280RawData.Pressure) - (var2 >> 12))) * 3125;
  if (*pressure < 0x80000000) *pressure = (*pressure << 1) / ((uint32_t)var1);
  else *pressure = (*pressure / (uint32_t)var1) * 2;
  var1 = (((int32_t)calData.dig_P9) * ((int32_t)(((*pressure >> 3) * (*pressure >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(*pressure >> 2)) * ((int32_t)calData.dig_P8)) >> 13;
  *temperature = (t_fine * 5 + 128) >> 8;
  *pressure = (uint32_t)((int32_t)*pressure + ((var1 + var2 + calData.dig_P7) >> 4));
  return true;
}
int32_t BMP280_GetAltitude(uint32_t* pressure, uint32_t* seaLevelPressure) {  //Pressure in Pa, height in cm
  return 4433000 * (1.0f - pow((float)*pressure / *seaLevelPressure, 0.1903));
}
