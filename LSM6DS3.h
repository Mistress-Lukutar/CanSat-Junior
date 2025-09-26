/**
 * @file LSM6DS3.h
 * @brief LSM6DS3/LSM6DS3TR 6-axis IMU sensor library for Arduino
 * @author Nate Hunter
 * @date 2025-09-24
 * @version v1.0.0
 */

#ifndef LSM6DS3_H
#define LSM6DS3_H

#include <Arduino.h>
#include "IICFuncs.h"

// Device identification
/** @defgroup LSM6DS3_Device_ID Device Identification
 * @{
 */
#define LSM6DS3_WHO_AM_I_VALUE 0x69 /**< Device ID value */
/** @} */

// I2C addresses
/** @defgroup LSM6DS3_I2C_Addresses I2C Addresses
 * @{
 */
#define LSM6DS3_ADDR_LOW 0x6A  /**< I2C address when SA0 = 0 */
#define LSM6DS3_ADDR_HIGH 0x6B /**< I2C address when SA0 = 1 */
/** @} */

// Register addresses
/** @defgroup LSM6DS3_Registers Register Addresses
 * @{
 */
#define LSM6DS3_FUNC_CFG_ACCESS 0x01 /**< Embedded functions configuration register */
#define LSM6DS3_FIFO_CTRL1 0x06      /**< FIFO control register 1 */
#define LSM6DS3_FIFO_CTRL2 0x07      /**< FIFO control register 2 */
#define LSM6DS3_FIFO_CTRL3 0x08      /**< FIFO control register 3 */
#define LSM6DS3_FIFO_CTRL4 0x09      /**< FIFO control register 4 */
#define LSM6DS3_FIFO_CTRL5 0x0A      /**< FIFO control register 5 */
#define LSM6DS3_ORIENT_CFG_G 0x0B    /**< Angular rate sensor sign and orientation register */
#define LSM6DS3_INT1_CTRL 0x0D       /**< INT1 pad control register */
#define LSM6DS3_INT2_CTRL 0x0E       /**< INT2 pad control register */
#define LSM6DS3_WHO_AM_I 0x0F        /**< Who I am ID register */
#define LSM6DS3_CTRL1_XL 0x10        /**< Linear acceleration sensor control register 1 */
#define LSM6DS3_CTRL2_G 0x11         /**< Angular rate sensor control register 2 */
#define LSM6DS3_CTRL3_C 0x12         /**< Control register 3 */
#define LSM6DS3_CTRL4_C 0x13         /**< Control register 4 */
#define LSM6DS3_CTRL5_C 0x14         /**< Control register 5 */
#define LSM6DS3_CTRL6_C 0x15         /**< Angular rate sensor control register 6 */
#define LSM6DS3_CTRL7_G 0x16         /**< Angular rate sensor control register 7 */
#define LSM6DS3_CTRL8_XL 0x17        /**< Linear acceleration sensor control register 8 */
#define LSM6DS3_CTRL9_XL 0x18        /**< Linear acceleration sensor control register 9 */
#define LSM6DS3_CTRL10_C 0x19        /**< Control register 10 */
#define LSM6DS3_STATUS_REG 0x1E      /**< Status data register */
#define LSM6DS3_OUT_TEMP_L 0x20      /**< Temperature output data register low */
#define LSM6DS3_OUT_TEMP_H 0x21      /**< Temperature output data register high */
#define LSM6DS3_OUTX_L_G 0x22        /**< Gyroscope pitch axis (X) angular rate output register low */
#define LSM6DS3_OUTX_H_G 0x23        /**< Gyroscope pitch axis (X) angular rate output register high */
#define LSM6DS3_OUTY_L_G 0x24        /**< Gyroscope roll axis (Y) angular rate output register low */
#define LSM6DS3_OUTY_H_G 0x25        /**< Gyroscope roll axis (Y) angular rate output register high */
#define LSM6DS3_OUTZ_L_G 0x26        /**< Gyroscope yaw axis (Z) angular rate output register low */
#define LSM6DS3_OUTZ_H_G 0x27        /**< Gyroscope yaw axis (Z) angular rate output register high */
#define LSM6DS3_OUTX_L_XL 0x28       /**< Linear acceleration sensor X-axis output register low */
#define LSM6DS3_OUTX_H_XL 0x29       /**< Linear acceleration sensor X-axis output register high */
#define LSM6DS3_OUTY_L_XL 0x2A       /**< Linear acceleration sensor Y-axis output register low */
#define LSM6DS3_OUTY_H_XL 0x2B       /**< Linear acceleration sensor Y-axis output register high */
#define LSM6DS3_OUTZ_L_XL 0x2C       /**< Linear acceleration sensor Z-axis output register low */
#define LSM6DS3_OUTZ_H_XL 0x2D       /**< Linear acceleration sensor Z-axis output register high */
/** @} */

// Control register bit definitions
/** @defgroup LSM6DS3_Control_Bits Control Register Bit Definitions
 * @{
 */

// CTRL1_XL bits
#define LSM6DS3_CTRL1_XL_BW_XL_50Hz 0x03  /**< Anti-aliasing filter bandwidth 50 Hz */
#define LSM6DS3_CTRL1_XL_BW_XL_100Hz 0x02 /**< Anti-aliasing filter bandwidth 100 Hz */
#define LSM6DS3_CTRL1_XL_BW_XL_200Hz 0x01 /**< Anti-aliasing filter bandwidth 200 Hz */
#define LSM6DS3_CTRL1_XL_BW_XL_400Hz 0x00 /**< Anti-aliasing filter bandwidth 400 Hz */

#define LSM6DS3_CTRL1_XL_FS_XL_2G 0x00  /**< Accelerometer full-scale ±2g */
#define LSM6DS3_CTRL1_XL_FS_XL_16G 0x04 /**< Accelerometer full-scale ±16g */
#define LSM6DS3_CTRL1_XL_FS_XL_4G 0x08  /**< Accelerometer full-scale ±4g */
#define LSM6DS3_CTRL1_XL_FS_XL_8G 0x0C  /**< Accelerometer full-scale ±8g */

// CTRL2_G bits
#define LSM6DS3_CTRL2_G_FS_G_125DPS 0x02  /**< Gyroscope full-scale 125 dps */
#define LSM6DS3_CTRL2_G_FS_G_250DPS 0x00  /**< Gyroscope full-scale 250 dps */
#define LSM6DS3_CTRL2_G_FS_G_500DPS 0x04  /**< Gyroscope full-scale 500 dps */
#define LSM6DS3_CTRL2_G_FS_G_1000DPS 0x08 /**< Gyroscope full-scale 1000 dps */
#define LSM6DS3_CTRL2_G_FS_G_2000DPS 0x0C /**< Gyroscope full-scale 2000 dps */

// CTRL3_C bits
#define LSM6DS3_CTRL3_C_BOOT 0x80      /**< Reboot memory content */
#define LSM6DS3_CTRL3_C_BDU 0x40       /**< Block Data Update */
#define LSM6DS3_CTRL3_C_H_LACTIVE 0x20 /**< Interrupt activation level */
#define LSM6DS3_CTRL3_C_PP_OD 0x10     /**< Push-pull/open-drain selection */
#define LSM6DS3_CTRL3_C_SIM 0x08       /**< SPI Serial Interface Mode selection */
#define LSM6DS3_CTRL3_C_IF_INC 0x04    /**< Register address automatically incremented */
#define LSM6DS3_CTRL3_C_BLE 0x02       /**< Big/Little Endian Data selection */
#define LSM6DS3_CTRL3_C_SW_RESET 0x01  /**< Software reset */

// STATUS_REG bits
#define LSM6DS3_STATUS_TDA 0x04  /**< Temperature new data available */
#define LSM6DS3_STATUS_GDA 0x02  /**< Gyroscope new data available */
#define LSM6DS3_STATUS_XLDA 0x01 /**< Accelerometer new data available */
/** @} */

// ODR (Output Data Rate) definitions
/** @defgroup LSM6DS3_ODR Output Data Rate Definitions
 * @{
 */
typedef enum {
  LSM6DS3_ODR_POWER_DOWN = 0x00, /**< Power down mode */
  LSM6DS3_ODR_12_5_HZ = 0x10,    /**< 12.5 Hz */
  LSM6DS3_ODR_26_HZ = 0x20,      /**< 26 Hz */
  LSM6DS3_ODR_52_HZ = 0x30,      /**< 52 Hz */
  LSM6DS3_ODR_104_HZ = 0x40,     /**< 104 Hz */
  LSM6DS3_ODR_208_HZ = 0x50,     /**< 208 Hz */
  LSM6DS3_ODR_416_HZ = 0x60,     /**< 416 Hz */
  LSM6DS3_ODR_833_HZ = 0x70,     /**< 833 Hz */
  LSM6DS3_ODR_1666_HZ = 0x80,    /**< 1.66 kHz */
  LSM6DS3_ODR_3332_HZ = 0x90,    /**< 3.33 kHz */
  LSM6DS3_ODR_6664_HZ = 0xA0     /**< 6.66 kHz */
} LSM6DS3_ODR;
/** @} */

// Full-scale definitions
/** @defgroup LSM6DS3_FullScale Full Scale Definitions
 * @{
 */
typedef enum {
  LSM6DS3_ACCEL_FS_2G = 0x00,  /**< ±2g */
  LSM6DS3_ACCEL_FS_16G = 0x04, /**< ±16g */
  LSM6DS3_ACCEL_FS_4G = 0x08,  /**< ±4g */
  LSM6DS3_ACCEL_FS_8G = 0x0C   /**< ±8g */
} LSM6DS3_AccelFS;

typedef enum {
  LSM6DS3_GYRO_FS_125DPS = 0x02,  /**< ±125 dps */
  LSM6DS3_GYRO_FS_250DPS = 0x00,  /**< ±250 dps */
  LSM6DS3_GYRO_FS_500DPS = 0x04,  /**< ±500 dps */
  LSM6DS3_GYRO_FS_1000DPS = 0x08, /**< ±1000 dps */
  LSM6DS3_GYRO_FS_2000DPS = 0x0C  /**< ±2000 dps */
} LSM6DS3_GyroFS;
/** @} */

// Device structure
/** @defgroup LSM6DS3_Structures Data Structures
 * @{
 */
typedef struct {
  uint8_t i2c_address;      /**< I2C device address */
  LSM6DS3_AccelFS accel_fs; /**< Accelerometer full scale */
  LSM6DS3_GyroFS gyro_fs;   /**< Gyroscope full scale */
  float accel_sensitivity;  /**< Accelerometer sensitivity (mg/LSB) */
  float gyro_sensitivity;   /**< Gyroscope sensitivity (mdps/LSB) */
} LSM6DS3_Device;

typedef struct {
  int16_t x; /**< X-axis raw value */
  int16_t y; /**< Y-axis raw value */
  int16_t z; /**< Z-axis raw value */
} LSM6DS3_RawData;

typedef struct {
  float x; /**< X-axis value in g or dps */
  float y; /**< Y-axis value in g or dps */
  float z; /**< Z-axis value in g or dps */
} LSM6DS3_Data;
/** @} */

static void LSM6DS3_UpdateSensitivity(LSM6DS3_Device *dev);

// Function prototypes and implementations

/**
 * @brief Initialize the LSM6DS3 device
 * @param dev Pointer to the device structure
 * @param i2c_address I2C address of the device
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_Init(LSM6DS3_Device *dev, uint8_t i2c_address) {
  if (!dev) return 0;

  // Initialize I2C
  IIC_Begin();

  // Set device address
  dev->i2c_address = i2c_address;

  // Check device ID
  uint8_t who_am_i = IIC_ReadByte(dev->i2c_address, LSM6DS3_WHO_AM_I);
  if (who_am_i != LSM6DS3_WHO_AM_I_VALUE) {
    return 0;
  }

  // Software reset
  IIC_WriteByte(dev->i2c_address, LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_SW_RESET);
  delay(10);  // Wait for reset to complete

  // Enable Block Data Update and auto-increment
  IIC_WriteByte(dev->i2c_address, LSM6DS3_CTRL3_C,
                LSM6DS3_CTRL3_C_BDU | LSM6DS3_CTRL3_C_IF_INC);

  // Set default full scales
  dev->accel_fs = LSM6DS3_ACCEL_FS_2G;
  dev->gyro_fs = LSM6DS3_GYRO_FS_250DPS;

  // Update sensitivity values
  LSM6DS3_UpdateSensitivity(dev);

  return 1;
}

/**
 * @brief Update sensitivity values based on current full scale settings
 * @param dev Pointer to the device structure
 */
void LSM6DS3_UpdateSensitivity(LSM6DS3_Device *dev) {
  if (!dev) return;

  // Set accelerometer sensitivity based on full scale
  switch (dev->accel_fs) {
    case LSM6DS3_ACCEL_FS_2G:
      dev->accel_sensitivity = 0.061f;  // mg/LSB
      break;
    case LSM6DS3_ACCEL_FS_4G:
      dev->accel_sensitivity = 0.122f;  // mg/LSB
      break;
    case LSM6DS3_ACCEL_FS_8G:
      dev->accel_sensitivity = 0.244f;  // mg/LSB
      break;
    case LSM6DS3_ACCEL_FS_16G:
      dev->accel_sensitivity = 0.488f;  // mg/LSB
      break;
  }

  // Set gyroscope sensitivity based on full scale
  switch (dev->gyro_fs) {
    case LSM6DS3_GYRO_FS_125DPS:
      dev->gyro_sensitivity = 4.375f;  // mdps/LSB
      break;
    case LSM6DS3_GYRO_FS_250DPS:
      dev->gyro_sensitivity = 8.75f;  // mdps/LSB
      break;
    case LSM6DS3_GYRO_FS_500DPS:
      dev->gyro_sensitivity = 17.50f;  // mdps/LSB
      break;
    case LSM6DS3_GYRO_FS_1000DPS:
      dev->gyro_sensitivity = 35.0f;  // mdps/LSB
      break;
    case LSM6DS3_GYRO_FS_2000DPS:
      dev->gyro_sensitivity = 70.0f;  // mdps/LSB
      break;
  }
}

/**
 * @brief Configure accelerometer settings
 * @param dev Pointer to the device structure
 * @param odr Output data rate
 * @param fs Full scale range
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_ConfigAccel(LSM6DS3_Device *dev, LSM6DS3_ODR odr, LSM6DS3_AccelFS fs) {
  if (!dev) return 0;

  // Configure CTRL1_XL register
  uint8_t ctrl1_xl = odr | fs | LSM6DS3_CTRL1_XL_BW_XL_400Hz;
  IIC_WriteByte(dev->i2c_address, LSM6DS3_CTRL1_XL, ctrl1_xl);

  // Update device structure
  dev->accel_fs = fs;
  LSM6DS3_UpdateSensitivity(dev);

  return 1;
}

/**
 * @brief Configure gyroscope settings
 * @param dev Pointer to the device structure
 * @param odr Output data rate
 * @param fs Full scale range
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_ConfigGyro(LSM6DS3_Device *dev, LSM6DS3_ODR odr, LSM6DS3_GyroFS fs) {
  if (!dev) return 0;

  // Configure CTRL2_G register
  uint8_t ctrl2_g = odr | fs;
  IIC_WriteByte(dev->i2c_address, LSM6DS3_CTRL2_G, ctrl2_g);

  // Update device structure
  dev->gyro_fs = fs;
  LSM6DS3_UpdateSensitivity(dev);

  return 1;
}

/**
 * @brief Read raw accelerometer data
 * @param dev Pointer to the device structure
 * @param data Pointer to store raw data
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_ReadAccelRaw(LSM6DS3_Device *dev, LSM6DS3_RawData *data) {
  if (!dev || !data) return 0;

  uint8_t buffer[6];

  // Read 6 bytes starting from OUTX_L_XL
  IIC_Request(dev->i2c_address, LSM6DS3_OUTX_L_XL, 6);

  for (int i = 0; i < 6; i++) {
    buffer[i] = IIC_Read();
  }

  // Combine low and high bytes
  data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
  data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
  data->z = (int16_t)((buffer[5] << 8) | buffer[4]);

  return 1;
}

/**
 * @brief Read raw gyroscope data
 * @param dev Pointer to the device structure
 * @param data Pointer to store raw data
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_ReadGyroRaw(LSM6DS3_Device *dev, LSM6DS3_RawData *data) {
  if (!dev || !data) return 0;

  uint8_t buffer[6];

  // Read 6 bytes starting from OUTX_L_G
  IIC_Request(dev->i2c_address, LSM6DS3_OUTX_L_G, 6);

  for (int i = 0; i < 6; i++) {
    buffer[i] = IIC_Read();
  }

  // Combine low and high bytes
  data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
  data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
  data->z = (int16_t)((buffer[5] << 8) | buffer[4]);

  return 1;
}

/**
 * @brief Read scaled accelerometer data in g
 * @param dev Pointer to the device structure
 * @param data Pointer to store scaled data
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_ReadAccel(LSM6DS3_Device *dev, LSM6DS3_Data *data) {
  if (!dev || !data) return 0;

  LSM6DS3_RawData raw_data;

  if (!LSM6DS3_ReadAccelRaw(dev, &raw_data)) {
    return 0;
  }

  // Convert to g (sensitivity is in mg/LSB)
  data->x = (float)raw_data.x * dev->accel_sensitivity / 1000.0f;
  data->y = (float)raw_data.y * dev->accel_sensitivity / 1000.0f;
  data->z = (float)raw_data.z * dev->accel_sensitivity / 1000.0f;

  return 1;
}

/**
 * @brief Read scaled gyroscope data in dps
 * @param dev Pointer to the device structure
 * @param data Pointer to store scaled data
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_ReadGyro(LSM6DS3_Device *dev, LSM6DS3_Data *data) {
  if (!dev || !data) return 0;

  LSM6DS3_RawData raw_data;

  if (!LSM6DS3_ReadGyroRaw(dev, &raw_data)) {
    return 0;
  }

  // Convert to dps (sensitivity is in mdps/LSB)
  data->x = (float)raw_data.x * dev->gyro_sensitivity / 1000.0f;
  data->y = (float)raw_data.y * dev->gyro_sensitivity / 1000.0f;
  data->z = (float)raw_data.z * dev->gyro_sensitivity / 1000.0f;

  return 1;
}

/**
 * @brief Read temperature data
 * @param dev Pointer to the device structure
 * @return Temperature in degrees Celsius
 */
float LSM6DS3_ReadTemperature(LSM6DS3_Device *dev) {
  if (!dev) return 0.0f;

  uint8_t temp_l = IIC_ReadByte(dev->i2c_address, LSM6DS3_OUT_TEMP_L);
  uint8_t temp_h = IIC_ReadByte(dev->i2c_address, LSM6DS3_OUT_TEMP_H);

  int16_t temp_raw = (int16_t)((temp_h << 8) | temp_l);

  // Convert to Celsius (datasheet: 16 LSB/°C, 0 LSB at 25°C)
  float temperature = 25.0f + (float)temp_raw / 16.0f;

  return temperature;
}

/**
 * @brief Check data ready status
 * @param dev Pointer to the device structure
 * @return Status register value
 */
uint8_t LSM6DS3_GetStatus(LSM6DS3_Device *dev) {
  if (!dev) return 0;

  return IIC_ReadByte(dev->i2c_address, LSM6DS3_STATUS_REG);
}

/**
 * @brief Check if accelerometer data is ready
 * @param dev Pointer to the device structure
 * @return 1 if data ready, 0 otherwise
 */
uint8_t LSM6DS3_IsAccelDataReady(LSM6DS3_Device *dev) {
  return (LSM6DS3_GetStatus(dev) & LSM6DS3_STATUS_XLDA) ? 1 : 0;
}

/**
 * @brief Check if gyroscope data is ready
 * @param dev Pointer to the device structure
 * @return 1 if data ready, 0 otherwise
 */
uint8_t LSM6DS3_IsGyroDataReady(LSM6DS3_Device *dev) {
  return (LSM6DS3_GetStatus(dev) & LSM6DS3_STATUS_GDA) ? 1 : 0;
}

/**
 * @brief Check if temperature data is ready
 * @param dev Pointer to the device structure
 * @return 1 if data ready, 0 otherwise
 */
uint8_t LSM6DS3_IsTempDataReady(LSM6DS3_Device *dev) {
  return (LSM6DS3_GetStatus(dev) & LSM6DS3_STATUS_TDA) ? 1 : 0;
}

/**
 * @brief Read device ID
 * @param dev Pointer to the device structure
 * @return Device ID value
 */
uint8_t LSM6DS3_ReadDeviceID(LSM6DS3_Device *dev) {
  if (!dev) return 0;

  return IIC_ReadByte(dev->i2c_address, LSM6DS3_WHO_AM_I);
}

/**
 * @brief Enable accelerometer axes
 * @param dev Pointer to the device structure
 * @param x_en Enable X axis
 * @param y_en Enable Y axis
 * @param z_en Enable Z axis
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_EnableAccelAxes(LSM6DS3_Device *dev, uint8_t x_en, uint8_t y_en, uint8_t z_en) {
  if (!dev) return 0;

  uint8_t ctrl9_xl = 0;
  if (z_en) ctrl9_xl |= 0x20;  // Zen_XL
  if (y_en) ctrl9_xl |= 0x10;  // Yen_XL
  if (x_en) ctrl9_xl |= 0x08;  // Xen_XL

  IIC_WriteByte(dev->i2c_address, LSM6DS3_CTRL9_XL, ctrl9_xl);

  return 1;
}

/**
 * @brief Enable gyroscope axes
 * @param dev Pointer to the device structure
 * @param x_en Enable X axis
 * @param y_en Enable Y axis
 * @param z_en Enable Z axis
 * @return 1 on success, 0 on failure
 */
uint8_t LSM6DS3_EnableGyroAxes(LSM6DS3_Device *dev, uint8_t x_en, uint8_t y_en, uint8_t z_en) {
  if (!dev) return 0;

  uint8_t ctrl10_c = 0;
  if (z_en) ctrl10_c |= 0x20;  // Zen_G
  if (y_en) ctrl10_c |= 0x10;  // Yen_G
  if (x_en) ctrl10_c |= 0x08;  // Xen_G

  IIC_WriteByte(dev->i2c_address, LSM6DS3_CTRL10_C, ctrl10_c);

  return 1;
}

#endif  // LSM6DS3_H