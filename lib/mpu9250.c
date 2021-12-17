/*
 ************ https://github.com/sin1111yi ************
 ******************************************************
 *        .__       ____ ____ ____ ____        .__
 *   _____|__| ____/_   /_   /_   /_   |___.__.|__|
 *  /  ___/  |/    \|   ||   ||   ||   <   |  ||  |
 *  \___ \|  |   |  \   ||   ||   ||   |\___  ||  |
 *  /___  >__|___|  /___||___||___||___|/ ____||__|
 *      \/        \/                    \/
 ******************************************************
 * @beginning
 * I don't like over encapsulation, low freedom and less api.
 * After read several projects on Github, I decided to write it by myself.
 *
 * @suggestions
 * If someone want to use SPI to drive MPU9250 to read 9DOF data,
 * I'll suggest him/her to use MPU6500 and another independent magnetometer chip.
 * Because MPU9250 is MPU6500 & AK8963, and AK8963 doesn't support SPI serial.
 * AK8963 must drive by I2C serial, which is put in MPU9250 connect MPU6500 and AK8963.
 * That means to drive AK8963, I need use SPI to drive MPU6500's I2C to drive AK8963.
 * That's troublesome. But it can be much more easier when using I2C.
 *
 * @my own view
 * I think it is stupid to use MPU9250, if PCB space and capital are enough.
 * And I think the designers of MPU9250 are really lazy, cause they name their registers in many different ways.
 *
 * @other
 * Those words above only represent myself. If someone have different views, please ignore my views.
 *
 *
 * @author  sin1111yi
 * @date    2021/12/9   restructure version 1
 *          2021/12/10  restructure version 2
 *
 * @refer   https://github.com/desertkun/MPU9250
 *          https://blog.csdn.net/liuyifanliu/article/details/99309839
 *
 */

#include "mpu9250.h"

#define mpu_select()    HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET)
#define mpu_deselect()  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET)

#define DATABUF_SIZ     16

static uint8_t dataBuf[DATABUF_SIZ] = { 0 }; // all read data will store in this array
static uint16_t i = 0; // loop control

/*** basic spi operate ***/
/*** I want to make these functions can be reused ***/
/*
 * @brief   write a byte through SPI and read feedback
 * @param   byte: byte to write
 * @return  received byte
 * */
static uint8_t spi_wr_byte(SPI_HandleTypeDef *hspi, uint8_t byte) {
	uint8_t feedback = 0;

	// wait SPI serial free
	while (HAL_SPI_GetState(hspi) == HAL_SPI_STATE_BUSY_TX_RX)
		;

	if (HAL_SPI_TransmitReceive(hspi, &byte, &feedback, 1, 0x01f4) != HAL_OK) {
		return 0xff;
	}

	return feedback;
}
/*
 * @brief   write several bytes  spi
 * @param   address: address of the first reg
 * @param   bytes: number of bytes to write
 * @param   num: number of bytes
 * */
static void spi_w_bytes(SPI_HandleTypeDef *hspi, uint8_t address,
		uint8_t *bytes, uint16_t num) {
	mpu_select();

	spi_wr_byte(hspi, address);
	for (i = 0; i < num; i++)
		spi_wr_byte(hspi, bytes[i]);

	mpu_deselect();
}

/*
 * @brief   read several bytes through spi
 * @param   address: address of the first reg
 * @param   num: number of bytes to read, number < DATABUF_SIZ
 * @return  data read array
 * */
static void spi_r_bytes(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t num) {
	uint8_t _address = address | 0x80;

	mpu_select();

	// may be can use HAL_SPI_TransmitReceive()
	HAL_SPI_Transmit(hspi, &_address, 1, 0x01f4);
	HAL_SPI_Receive(hspi, dataBuf, num, 0x01f4); // store read data to dataBuf

	mpu_deselect();

}
/*** basic mpu9250 operate ***/
/*
 * @brief   write mpu9250 reg through spi
 * @param   address: address of reg to write
 * @param   byte: byte to write
 * */
static void mpu_w_reg(uint8_t address, uint8_t byte) {
	spi_w_bytes(&MPU9250_SPI, address, &byte, 1);
}

/*
 * @brief   read mpu9250 regs through spi
 * @param   address: address of reg to write
 * @param   num: number of byte to read
 * @return  read bytes array
 * */
static void mpu_r_regs(uint8_t address, uint8_t num) {
	spi_r_bytes(&MPU9250_SPI, address, num);
}
/******* basic ak8963 operate *******/
/*
 * @brief   write AK8963 regs through I2C in MPU9250
 * @param   address: address of AK8963 reg to write
 * @param   byte: byte to write
 * */
static void mpu_w_ak8963_reg(uint8_t address, uint8_t byte) {
	mpu_w_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR);
	mpu_w_reg(I2C_SLV0_REG, address);
	mpu_w_reg(I2C_SLV0_DO, byte);
	mpu_w_reg(I2C_SLV0_CTRL, 0x81);
}
/*
 * @brief read AK8963 regs through I2C in MPU9250
 * @param   address: first address of AK8963 regs to read
 * @param   num: number of byte to read
 * @return  read bytes array
 * */
static void mpu_r_ak8963_regs(uint8_t address, uint8_t num) {
	mpu_w_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
	mpu_w_reg(I2C_SLV0_REG, address);
	mpu_w_reg(I2C_SLV0_CTRL, 0x80 | num);
	HAL_Delay(1);
	mpu_r_regs(EXT_SENS_DATA_00, num);
}
/*
 * @brief   read ak8963 WHO_AM_I reg
 * @return  AK8963 WHO_AM_I value, expected to be 0x48
 * */
uint8_t mpu_r_ak8963_WhoAmI(MPU9250 *mpu) {
	mpu_r_ak8963_regs(AK8963_WHOAMI_REG, 1);
	return dataBuf[0];
}
/*
 * @brief   read mpu9250(mpu6500) WHO_AM_I reg
 * @return  mpu9250(mpu6500) WHO_AM_I value, expected to be 0x48
 * */
uint8_t mpu_r_WhoAmI(MPU9250 *mpu) {
	mpu_r_regs(WHO_AM_I, 1);
	return dataBuf[0];
}
/*
 * @brief   init origin data
 * */
static void MPU9250_StructInit(MPU9250 *mpu) {

	for (uint8_t i = 0; i < 3; i++) {
		mpu->mpu_data.Accel[i] = 0;
		mpu->mpu_data.Gyro[i] = 0;
		mpu->mpu_data.Magn[i] = 0;

		mpu->mpu_data.Accel_row[i] = 0;
		mpu->mpu_data.Gyro_row[i] = 0;
		mpu->mpu_data.Magn_row[i] = 0.0;
	}
}
/*
 * @brief   init mpu9250
 * */
uint8_t MPU9250_Init(MPU9250 *mpu) {
	MPU9250_StructInit(mpu);

	mpu_w_reg(PWR_MGMT_1, (uint8_t) 0x80); // reset MPU9250, reg107
	HAL_Delay(10);
	mpu_w_reg(USER_CTRL, (uint8_t) 0x20); // enable I2C master mode, reg106
	mpu_w_reg(I2C_MST_CTRL, (uint8_t) 0x0D); // set I2C clock speed to 400kHz, reg36
	mpu_w_ak8963_reg(AK8963_CNTL1_REG, (uint8_t) 0x00); // set AK8963 to power down
	mpu_w_reg(PWR_MGMT_1, (uint8_t) 0x80); // reset MPU9250, Bit[7] will auto clear
	HAL_Delay(10);
	mpu_w_ak8963_reg(AK8963_CNTL2_REG, AK8963_CNTL2_SRST); // reset AK8963
	mpu_w_reg(PWR_MGMT_1, (uint8_t) 0x01); // select clock source
	mpu_w_reg(PWR_MGMT_2, (uint8_t) 0x00); // enable accel and gyro

	/* init GYRO and ACCEL */
	mpu_w_reg(SMPLRT_DIV, (uint8_t) 0x00); // SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV), Internal_Sample_Rate==8K
	mpu_w_reg(GYRO_CONFIG, (uint8_t) MPU9250_Gyro_Range_2000dps); // gyro full scale select
	mpu_w_reg(ACCEL_CONFIG, (uint8_t) MPU9250_Accel_Range_16G); // accel full scale select
	mpu_w_reg(ACCEL_CONFIG_2, (uint8_t) MPU9250_Accel_DLPFBandwidth_460);
	mpu_w_reg(CONFIG, (uint8_t) MPU9250_Gyro_DLPFBandwidth_250);
	/* init MAG */
	mpu_w_reg(USER_CTRL, (uint8_t) 0x20); // enable I2C master mode
	mpu_w_reg(I2C_MST_CTRL, (uint8_t) 0x0D); // set I2C clock speed to 400kHz, reg36
	mpu_w_ak8963_reg(AK8963_CNTL1_REG, (uint8_t) 0x00); // set AK8963 to power down
	HAL_Delay(100);
	mpu_w_ak8963_reg(AK8963_CNTL1_REG, (uint8_t) 0x0f); // set AK8963 to Fuse ROM access mode
	HAL_Delay(100);
	mpu_w_ak8963_reg(AK8963_CNTL1_REG, (uint8_t) 0x00); // set AK8963 to power down
	HAL_Delay(100);
	mpu_w_ak8963_reg(AK8963_CNTL1_REG, (uint8_t) 0x16); // AK8963 working on Continuous measurement mode 2 & 16-bit output
	HAL_Delay(100);
	mpu_w_reg(PWR_MGMT_1, (uint8_t) 0x01); // select clock source
	mpu_r_ak8963_regs(MAG_XOUT_L, 7);

	return 0x00;
}
/*
 * @brief   read accel origin value and calculate real value
 *          data will be stored in mpu
 * */
void MPU9250_ReadAccel(MPU9250 *mpu) {
	// m/s
	mpu_r_regs(ACCEL_XOUT_H, 6);
	// calculate x axis
	mpu->mpu_data.Accel_row[0] = ((int16_t)dataBuf[0] << 8) | dataBuf[1];
	mpu->mpu_data.Accel[0] = (float) mpu->mpu_data.Accel_row[0] / 208.980;

	// calculate y axis
	mpu->mpu_data.Accel_row[1] = ((int16_t)dataBuf[2] << 8) | dataBuf[3];
	mpu->mpu_data.Accel[1] = (float) mpu->mpu_data.Accel_row[1] / 208.980;

	// calculate z axis
	mpu->mpu_data.Accel_row[2] = ((int16_t)dataBuf[4] << 8) | dataBuf[5];
	mpu->mpu_data.Accel[2] = (float) mpu->mpu_data.Accel_row[2] / 208.980;
}
/*
 * @brief   read gyro origin value and calculate real value
 *          data will be stored in mpu
 * */
void MPU9250_ReadGyro(MPU9250 *mpu) {
	// d/s
	mpu_r_regs(GYRO_XOUT_H, 6);
	// calculate x axis
	mpu->mpu_data.Gyro_row[0] = ((int16_t)dataBuf[0] << 8) | dataBuf[1];
	mpu->mpu_data.Gyro[0] = mpu->mpu_data.Gyro_row[0] / 16.384;

	// calculate y axis
	mpu->mpu_data.Gyro_row[1] = ((int16_t)dataBuf[2] << 8) | dataBuf[3];
	mpu->mpu_data.Gyro[1] = mpu->mpu_data.Gyro_row[1] / 16.384;

	// calculate z axis
	mpu->mpu_data.Gyro_row[2] = ((int16_t)dataBuf[4] << 8) | dataBuf[5];
	mpu->mpu_data.Gyro[2] = mpu->mpu_data.Gyro_row[2] / 16.384;
}
/*
 * @brief   read mag origin value and calculate real value
 *          data will be stored in mpu
 * */
void MPU9250_ReadMag(MPU9250 *mpu) {
	uint8_t mag_adjust[3] = { 0 };
	uint8_t mag_buffer[6] = { 0 };

	mpu_r_ak8963_regs(AK8963_ASAX, 3);
	mag_adjust[0] = dataBuf[0];
	mag_adjust[1] = dataBuf[1];
	mag_adjust[2] = dataBuf[2];

	// read AK8963_ST2_REG is necessary
	// ST2 register has a role as data reading end register(on page 51)

	mpu_r_ak8963_regs(MAG_XOUT_L, 1);
	mag_buffer[0] = dataBuf[0];
	mpu_r_ak8963_regs(AK8963_ST2_REG, 1); // data read finish reg
	mpu_r_ak8963_regs(MAG_XOUT_H, 1);
	mag_buffer[1] = dataBuf[0];
	mpu_r_ak8963_regs(AK8963_ST2_REG, 1);

	mpu_r_ak8963_regs(MAG_YOUT_L, 1);
	mag_buffer[2] = dataBuf[0];
	mpu_r_ak8963_regs(AK8963_ST2_REG, 1);
	mpu_r_ak8963_regs(MAG_YOUT_H, 1);
	mag_buffer[3] = dataBuf[0];
	mpu_r_ak8963_regs(AK8963_ST2_REG, 1);

	mpu_r_ak8963_regs(MAG_ZOUT_L, 1);
	mag_buffer[4] = dataBuf[0];
	mpu_r_ak8963_regs(AK8963_ST2_REG, 1);
	mpu_r_ak8963_regs(MAG_ZOUT_H, 1);
	mag_buffer[5] = dataBuf[0];
	mpu_r_ak8963_regs(AK8963_ST2_REG, 1);


	mpu->mpu_data.Magn_row[0] = ((int16_t)mag_buffer[1] << 8) | mag_buffer[0];
	mpu->mpu_data.Magn_row[1] = ((int16_t)mag_buffer[3] << 8) | mag_buffer[2];
	mpu->mpu_data.Magn_row[2] = ((int16_t)mag_buffer[5] << 8) | mag_buffer[4];

	// calculate real value, check page53
	mpu->mpu_data.Magn[0] = (float) mpu->mpu_data.Magn_row[0]
			* (((mag_adjust[0] - 128) / 256.0) + 1);
	mpu->mpu_data.Magn[0] = mpu->mpu_data.Magn_row[0] * 0.15;

	mpu->mpu_data.Magn[1] = (float) mpu->mpu_data.Magn_row[1]
			* (((mag_adjust[1] - 128) / 256.0) + 1);
	mpu->mpu_data.Magn[1] = mpu->mpu_data.Magn_row[1] * 0.15;

	mpu->mpu_data.Magn[2] = (float) mpu->mpu_data.Magn_row[2]
			* (((mag_adjust[2] - 128) / 256.0) + 1);
	mpu->mpu_data.Magn[2] = mpu->mpu_data.Magn_row[2] * 0.15;
}
/*
 * @brief   read all 9 DOF data
 *          data will be stored in mpu
 * */
void MPU9250_ReadData(MPU9250 *mpu) {
	MPU9250_ReadAccel(mpu);
	MPU9250_ReadGyro(mpu);
	MPU9250_ReadMag(mpu);
}
