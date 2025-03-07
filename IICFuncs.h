#pragma once
#include <Wire.h>
uint8_t IIC_ReadByte(uint8_t addr, uint8_t reg) {
  uint8_t res = 1;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, res);
  res = Wire.read();
  return res;
}

void IIC_WriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void IIC_Request(uint8_t addr, uint8_t reg, uint8_t num) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, num);
}

inline uint8_t IIC_Read() {
  return Wire.read();
}

inline void IIC_Begin() {
  Wire.begin();
}