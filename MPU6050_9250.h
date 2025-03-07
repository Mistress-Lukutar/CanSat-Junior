#pragma once
#define MPU_DEFAULT_ADDR 0x68
#define MPU_DEFAULT_ID 0x75
#define MPU_DEFAULT_ALT_ID 0x71

#define CLOCK_INT 0b000
#define CLOCK_PLL_X 0b001
#define CLOCK_PLL_Y 0b010
#define CLOCK_PLL_Z 0b011
#define CLOCK_PLL_32 0b100
#define CLOCK_PLL_19 0b101
#define CLOCK_STOP 0b111
#define TEMP_DISABLE 1 << 3
#define MODE_CYCLE 1 << 5
#define MODE_SLEEP 1 << 6
#define DEVICE_RESET 1 << 7
#define AFS_SEL_2G 0b00 << 3
#define AFS_SEL_4G 0b01 << 3
#define AFS_SEL_8G 0b10 << 3
#define AFS_SEL_16G 0b11 << 3

#define GFS_SEL_250 0b00 << 3
#define GFS_SEL_500 0b01 << 3
#define GFS_SEL_1000 0b10 << 3
#define GFS_SEL_2000 0b11 << 3

struct MPU_RegMap {
  const uint8_t WHO_AM_I;
  const uint8_t GYRO_CONFIG;
  const uint8_t ACCEL_CONFIG;
  const uint8_t INT_STATUS;
  const uint8_t ACCEL_XOUT_H;
  const uint8_t PWR_MGMT_1;
};

struct MPU_RawData {
  int16_t accelX, accelY, accelZ,
    temp,
    gyroX, gyroY, gyroZ;
} mpuRawData;

MPU_RegMap mpuRegs{ 0x75, 0x1B, 0x1C, 0x3A, 0x3B, 0x6B };
uint8_t mpu_addr;
uint8_t _ctrlByte;
uint8_t _cfgByte;

MPU_SetScales(uint8_t accelScale, uint8_t gyroScale) {
  _cfgByte = accelScale >> 3 | gyroScale;
  IIC_WriteByte(mpu_addr, mpuRegs.ACCEL_CONFIG, accelScale);
  IIC_WriteByte(mpu_addr, mpuRegs.GYRO_CONFIG, gyroScale);
}

bool MPU_Init(uint8_t altAddr = MPU_DEFAULT_ADDR, uint8_t ctlrByte = CLOCK_INT) {
  mpu_addr = altAddr;
  uint8_t mpuStatus = 0;
  _ctrlByte = ctlrByte;
  IIC_Begin();
  if (IIC_ReadByte(mpu_addr, mpuRegs.WHO_AM_I) != MPU_DEFAULT_ID)
    if ((IIC_ReadByte(mpu_addr, mpuRegs.WHO_AM_I) != MPU_DEFAULT_ALT_ID)) return 0;
  IIC_WriteByte(mpu_addr, mpuRegs.PWR_MGMT_1, _ctrlByte);
  MPU_SetScales(AFS_SEL_16G, GFS_SEL_1000);
  return 1;
}

void MPU_ReadData(int32_t* accel, int32_t* gyro, int32_t* temperature = 0) {
  int16_t* pt = (int16_t*)&mpuRawData;
  uint8_t i;
  while (!(IIC_ReadByte(mpu_addr, mpuRegs.INT_STATUS) & 1)) {};
  IIC_Request(mpu_addr, mpuRegs.ACCEL_XOUT_H, 14);
  for (i = 0; i < sizeof(mpuRawData) / 2; i++) pt[i] = IIC_Read() << 8 | IIC_Read();
  if (temperature) *temperature = mpuRawData.temp * 100 / 334 + 2100;
  for (i = 0; i < 3; i++) accel[i] = (int32_t)pt[i] * 100 * (2 << (_cfgByte & 0b11)) / 32768;
  for (i = 0; i < 3; i++) gyro[i] = (int32_t)pt[i + 4] * 100 * 250 * (_cfgByte >> 3 & 0b11 ? 2 << (_cfgByte >> 3 & 0b11 - 1) : 1) / 32768;
}