#ifndef IMU_H
#define IMU_H

#include <M5Unified.h>
#include <MadgwickAHRS.h>
#include <math.h>

// I2C関連マクロ
#define writeRegB(i2c_addr, reg_addr, data) M5.Ex_I2C.writeRegister8(i2c_addr, reg_addr, data, I2C_CLK_FREQ)
#define writeReg(i2c_addr, reg_addr, data, len) M5.Ex_I2C.writeRegister(i2c_addr, reg_addr, data, len, I2C_CLK_FREQ)
#define readRegB(i2c_addr, reg_addr) M5.Ex_I2C.readRegister8(i2c_addr, reg_addr, I2C_CLK_FREQ)
#define readReg(i2c_addr, reg_addr, data, len) M5.Ex_I2C.readRegister(i2c_addr, reg_addr, data, len, I2C_CLK_FREQ)

#define I2C_CLK_FREQ 400000 // 400kHz

// BMI270 Register Addresses
#define BMI270_REG_CHIP_ID 0x00
#define BMI270_REG_STATUS 0x03
#define BMI270_REG_AUX_DATA 0x04
#define BMI270_REG_INTERNAL_STATUS 0x21
#define BMI270_REG_GYRO_RANGE 0x43
#define BMI270_REG_GYRO_CONFIG 0x42
#define BMI270_REG_ACC_CONFIG 0x40
#define BMI270_REG_ACC_RANGE 0x41
#define BMI270_REG_AUX_DEV_ID 0x4B
#define BMI270_REG_AUX_IF_CONF 0x4C
#define BMI270_REG_AUX_RD_ADDR 0x4D
#define BMI270_REG_AUX_WR_ADDR 0x4E
#define BMI270_REG_AUX_WR_DATA 0x4F
#define BMI270_REG_CMD 0x7E
#define BMI270_REG_PWR_CONF 0x7C
#define BMI270_REG_PWR_CTRL 0x7D
#define BMI270_REG_INIT_CTRL 0x59
#define BMI270_REG_INIT_ADDR_0 0x5B
#define BMI270_REG_INIT_ADDR_1 0x5C
#define BMI270_REG_INIT_DATA 0x5E

// BMI270 Configuration Values
#define BMI270_PWR_CONF_ADV_OFF 0x00
#define BMI270_PWR_CONF_FIFO_WU 0x02
#define BMI270_PWR_CTRL_ALL_ON 0x07
#define BMI270_INIT_START 0x00
#define BMI270_INIT_COMPLETE 0x01
#define BMI270_ACC_ODR_400HZ 0xAA
#define BMI270_ACC_RANGE_2G 0x00
#define BMI270_GYRO_ODR_400HZ 0xEA

// AUX (BMM150) related
#define BMM150_AUX_RD_BURST_LEN1 0x80

#define MAX_RETRY_COUNT 10
#define RETRY_DELAY_MS 1

// Error codes
#define BMI270_OK 0
#define BMI270_ERR_WRITE_FAILED 1
#define BMI270_ERR_READ_FAILED 2
#define BMI270_ERR_TIMEOUT 3
#define BMI270_ERR_WRONG_CHIP_ID 4
#define BMI270_ERR_AUX_WRITE_FAILED 5

// Physical constants
#define GRAVITY 9.80665f // Standard gravity in m/s^2

// Sensor data validity checks
#define ACC_MAX_MS2 2.0f     // Maximum acceleration in [g]
#define GYRO_MAX_DPS 2000.0f // Maximum angular rate in degrees/s
#define MAG_MAX_UT 1000.0f	 // Maximum magnetic field in μT

// IMU関連変数
extern Madgwick madgwick;
extern float ax, ay, az, gx, gy, gz;
extern float roll, pitch, yaw;
#define I2C_ADDR_IMU 0x68 // ID=0

// 関数宣言
int conv_value(uint8_t dh, uint8_t dl);
bool auxWriteRegB(uint8_t i2c_addr, uint8_t reg, uint8_t data);
int auxReadRegB(uint8_t i2c_addr, uint8_t reg);
int IMUinit(uint8_t i2c_addr);
bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
#define readReg(i2c_addr, reg_addr, data, len) M5.Ex_I2C.readRegister(i2c_addr, reg_addr, data, len, I2C_CLK_FREQ)

#endif // IMU_H