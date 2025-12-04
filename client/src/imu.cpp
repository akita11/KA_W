#include "imu.h"
#include "bmi270_config.h"

// IMU関連変数定義
Madgwick madgwick;
float ax, ay, az, gx, gy, gz;
float roll, pitch, yaw;

int conv_value(uint8_t dh, uint8_t dl)
{
	uint16_t d = dh << 8 | dl;
	int ret;
	if (d & 0x8000)
		ret = -(~d & 0x7fff);
	else
		ret = d & 0x7fff;
	return (ret);
}

bool auxWriteRegB(uint8_t i2c_addr, uint8_t reg, uint8_t data)
{
	// AUXにアドレスを書き込み
	if (!writeRegB(i2c_addr, BMI270_REG_AUX_WR_ADDR, reg))
	{
		return false;
	}

	// AUXにデータを書き込み
	if (!writeRegB(i2c_addr, BMI270_REG_AUX_WR_DATA, data))
	{
		return false;
	}

	// 操作完了を待機（タイムアウト付き）
	int retry = MAX_RETRY_COUNT;
	int status;
	do
	{
		status = readRegB(i2c_addr, BMI270_REG_STATUS);
		if (status < 0)
		{ // 読み取りエラー
			return false;
		}
		if (--retry <= 0)
		{ // タイムアウト
			return false;
		}
		vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
	} while (status & 0b100);

	return true;
}

int auxReadRegB(uint8_t i2c_addr, uint8_t reg)
{
	// Enable read with burst length 1
	if (!writeRegB(i2c_addr, BMI270_REG_AUX_IF_CONF, BMM150_AUX_RD_BURST_LEN1))
	{
		return BMI270_ERR_WRITE_FAILED;
	}

	// Set address to read from AUX
	if (!writeRegB(i2c_addr, BMI270_REG_AUX_RD_ADDR, reg))
	{
		return BMI270_ERR_WRITE_FAILED;
	}

	// Wait for operation to complete with timeout
	int retry = MAX_RETRY_COUNT;
	while ((readRegB(i2c_addr, BMI270_REG_STATUS) & 0b100) && --retry)
	{
		vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
	}

	if (retry <= 0)
	{
		return BMI270_ERR_TIMEOUT;
	}

	// Read the data
	return(readRegB(i2c_addr, BMI270_REG_AUX_DATA));
}

// for BMI270&BMM150
int IMUinit(uint8_t i2c_addr)
{
	uint8_t index = 0;
	uint8_t addr_array[2] = {(uint8_t)((index >> 1) & 0x0F), (uint8_t)(index >> 5)};

	// IMU init sequence
	printf("CHIP_ID : %02x\n", readRegB(i2c_addr, 0x00)); // CHIP_ID = 0x24
	if (!writeRegB(i2c_addr, BMI270_REG_PWR_CONF, BMI270_PWR_CONF_ADV_OFF))
	{ // disable adv.power save
		return BMI270_ERR_WRITE_FAILED;
	}
	delayMicroseconds(450);

	if (!writeRegB(i2c_addr, BMI270_REG_INIT_CTRL, BMI270_INIT_START))
	{ // prepare init
		return BMI270_ERR_WRITE_FAILED;
	}
	if (!writeReg(i2c_addr, BMI270_REG_INIT_ADDR_0, addr_array, 2))
	{
		return BMI270_ERR_WRITE_FAILED;
	}
	if (!writeReg(i2c_addr, BMI270_REG_INIT_DATA, (uint8_t *)bmi270_config_file, sizeof(bmi270_config_file)))
	{
		return BMI270_ERR_WRITE_FAILED;
	}
	if (!writeRegB(i2c_addr, BMI270_REG_INIT_CTRL, BMI270_INIT_COMPLETE))
	{
		return BMI270_ERR_WRITE_FAILED;
	}

	// 初期化完了待ち
	uint8_t status;
	uint8_t retry = 0;
	do
	{
		status = readRegB(i2c_addr, BMI270_REG_INTERNAL_STATUS);
		if (status == 0xFF)
		{ // 読み取りエラー
			return BMI270_ERR_READ_FAILED;
		}
		if (++retry > 100)
		{ // タイムアウト
			return BMI270_ERR_TIMEOUT;
		}
		delay(1);
	} while (status != BMI270_INIT_COMPLETE);

	// センサーの設定
	writeRegB(i2c_addr, BMI270_REG_PWR_CTRL, BMI270_PWR_CTRL_ALL_ON);	 // enable acc/gyro/aux
	writeRegB(i2c_addr, BMI270_REG_ACC_CONFIG, BMI270_ACC_ODR_400HZ);	 // Acc ODR=400Hz
	writeRegB(i2c_addr, BMI270_REG_PWR_CONF, BMI270_PWR_CONF_FIFO_WU);	 // disable adv. power save
	writeRegB(i2c_addr, BMI270_REG_ACC_RANGE, BMI270_ACC_RANGE_2G);		 // Acc range : +-2g
	writeRegB(i2c_addr, BMI270_REG_GYRO_CONFIG, BMI270_GYRO_ODR_400HZ); // Gyro config
	writeRegB(i2c_addr, BMI270_REG_GYRO_RANGE,  0x00); // Gyro range = +-2000dps

	// AUX (BMM150) の初期化
	writeRegB(i2c_addr, 0x6b, 0x20); // AUX I2C enable
	writeRegB(i2c_addr, 0x7c, 0x00); // Power save disabled
	writeRegB(i2c_addr, 0x7d, 0x0e); // AUX sensor disable
	writeRegB(i2c_addr, 0x4c, 0x80); // enable manual AUX
	writeRegB(i2c_addr, 0x4b, 0x10 << 1);

	if (!auxWriteRegB(i2c_addr, 0x4b, 0x83))
	{ // software reset + power on
		return BMI270_ERR_AUX_WRITE_FAILED;
	}

	auto who_am_i = auxReadRegB(i2c_addr, 0x40); // 0x40 = WhoAmI
	if (who_am_i != 0x32)
	{
		return BMI270_ERR_WRONG_CHIP_ID;
	}

	auxWriteRegB(i2c_addr, 0x4C, 0x38);  // normal mode / ODR 30Hz
	writeRegB(i2c_addr, 0x4c, 0x4f); 	 // FCU_WRITE_EN + Manual BurstLength 8
	writeRegB(i2c_addr, 0x4d, 0x42); 	 // 0x42 = BMM150 I2C Data X LSB reg
	writeRegB(i2c_addr, 0x7d, 0x0f);  // temp en | ACC en | GYR en | AUX en

	return BMI270_OK;
}

bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
	uint8_t buf[20];
	// BMI270 acc/gyroデータレジスタ: 0x0C～0x1F
	if (!readReg(I2C_ADDR_IMU, BMI270_REG_AUX_DATA, buf, 20)) return(false);
	//mx = (float)(conv_value(buf[1], buf[0]) >> 3); // 13bit (+-4096)
	//my = (float)(conv_value(buf[3], buf[2]) >> 3); // 13bit (+-4096)
	//mz = (float)(conv_value(buf[5], buf[4]) >> 1); // 15bit (+-16384)
	ax = (float)conv_value(buf[ 9], buf[ 8]) / 16384.0f; // [g]
	ay = (float)conv_value(buf[11], buf[10]) / 16384.0f;
	az = (float)conv_value(buf[13], buf[12]) / 16384.0f;
	gx = (float)conv_value(buf[15], buf[14]) / 32768.0f * 2000.0f; // [dps]
	gy = (float)conv_value(buf[17], buf[16]) / 32768.0f * 2000.0f;
	gz = (float)conv_value(buf[19], buf[18]) / 32768.0f * 2000.0f;
	return true;
}