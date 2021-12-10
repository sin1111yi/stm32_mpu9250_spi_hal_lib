#ifndef __MPU9250_H
#define __MPU9250_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"

#include <stdlib.h>

#define MPU9250_CS_GPIO    GY_CS_GPIO_Port
#define MPU9250_CS_PIN     GY_CS_Pin
#define MPU9250_SPI        hspi1
#define MPU9250_TIMEOUT_S  0x0100
#define MPU9250_TIMEOUT_L  0x1000

// MPU9250 registers

#define	SMPLRT_DIV		                    (uint8_t)0x19
#define	CONFIG			                    (uint8_t)0x1A
#define	GYRO_CONFIG		                    (uint8_t)0x1B
#define	ACCEL_CONFIG	                    (uint8_t)0x1C
#define	ACCEL_CONFIG_2                      (uint8_t)0x1D

#define INT_PIN_CFG                         (uint8_t)0x37
#define USER_CTRL                           (uint8_t)0x6a
#define I2C_MST_CTRL                        (uint8_t)0x24
#define I2C_MST_DELAY_CTRL                  (uint8_t)0x67
//--------------------i2c slv0-------------------------------//
#define I2C_SLV0_ADDR                       (uint8_t)0x25
#define I2C_SLV0_REG                        (uint8_t)0x26
#define I2C_SLV0_CTRL                       (uint8_t)0x27
#define I2C_SLV0_DO                         (uint8_t)0x63 //output reg
//--------------------AK8963 reg addr------------------------//
#define AK8963_I2C_ADDR                     (uint8_t)0x0C  //AKM addr
#define AK8963_WHOAMI_REG                   (uint8_t)0x00  //AKM ID addr
#define AK8963_WHOAMI_ID                    (uint8_t)0x48  //ID
#define AK8963_ST1_REG                      (uint8_t)0x02  //Data Status1
#define AK8963_ST2_REG                      (uint8_t)0x09  //Data reading end register & check Magnetic sensor overflow occurred
#define AK8963_ST1_DOR                      (uint8_t)0x02
#define AK8963_ST1_DRDY                     (uint8_t)0x01  //Data Ready
#define AK8963_ST2_BITM                     (uint8_t)0x10
#define AK8963_ST2_HOFL                     (uint8_t)0x08  // Magnetic sensor overflow
#define AK8963_CNTL1_REG                    (uint8_t)0x0A
#define AK8963_CNTL2_REG                    (uint8_t)0x0B
#define AK8963_CNTL2_SRST                   (uint8_t)0x01  //soft Reset
#define AK8963_ASAX                         (uint8_t)0x10  //X-axis sensitivity adjustment value
#define AK8963_ASAY                         (uint8_t)0x11  //Y-axis sensitivity adjustment value
#define AK8963_ASAZ                         (uint8_t)0x12  //Z-axis sensitivity adjustment value
//--------------------9axis  reg addr-----------------------//
#define	ACCEL_XOUT_H	(uint8_t)0x3B
#define	ACCEL_XOUT_L	(uint8_t)0x3C
#define	ACCEL_YOUT_H	(uint8_t)0x3D
#define	ACCEL_YOUT_L	(uint8_t)0x3E
#define	ACCEL_ZOUT_H	(uint8_t)0x3F
#define	ACCEL_ZOUT_L	(uint8_t)0x40

#define	TEMP_OUT_H		(uint8_t)0x41   //temperture
#define	TEMP_OUT_L		(uint8_t)0x42

#define	GYRO_XOUT_H		(uint8_t)0x43
#define	GYRO_XOUT_L		(uint8_t)0x44
#define	GYRO_YOUT_H		(uint8_t)0x45
#define	GYRO_YOUT_L		(uint8_t)0x46
#define	GYRO_ZOUT_H		(uint8_t)0x47
#define	GYRO_ZOUT_L		(uint8_t)0x48

#define MAG_XOUT_L		(uint8_t)0x03
#define MAG_XOUT_H		(uint8_t)0x04
#define MAG_YOUT_L		(uint8_t)0x05
#define MAG_YOUT_H		(uint8_t)0x06
#define MAG_ZOUT_L		(uint8_t)0x07
#define MAG_ZOUT_H		(uint8_t)0x08
//--------------------other reg addr-----------------------//
#define	PWR_MGMT_1		    (uint8_t)0x6B
#define PWR_MGMT_2          (uint8_t)0x6C
#define	WHO_AM_I		    (uint8_t)0x75

#define EXT_SENS_DATA_00    (uint8_t)0x49
#define EXT_SENS_DATA_01    (uint8_t)0x4a
#define EXT_SENS_DATA_02    (uint8_t)0x4b
#define EXT_SENS_DATA_03    (uint8_t)0x4c

typedef enum __MPU9250_AccelRange {
	MPU9250_Accel_Range_2G = 0x00,
	MPU9250_Accel_Range_4G = 0x08,
	MPU9250_Accel_Range_8G = 0x10,
	MPU9250_Accel_Range_16G = 0x18
/*
 * 0b00000000 for +-2G
 * 0b00001000 for +-4G
 * 0b00010000 for +-8G
 * 0b00011000 for +-16G
 * */
} MPU9250_AccelRange;

typedef enum __MPU9250_GyroRange {
	MPU9250_Gyro_Range_250dps = 0x00,
	MPU9250_Gyro_Range_500dps = 0x08,
	MPU9250_Gyro_Range_1000dps = 0x10,
	MPU9250_Gyro_Range_2000dps = 0x18
/*
 * 0b00000000 for 250dps
 * 0b00001000 for 500dps
 * 0b00010000 for 1000dps
 * 0b00011000 for 2000dps
 * */
} MPU9250_GyroRange;

typedef enum __MPU9250_Accel_DLPFBandwidth {
	// set reg29 Bit[3] & Bit[2:0]
	// Hz
	MPU9250_Accel_DLPFBandwidth_460 = 0x00,    // delay 1.94ms
	MPU9250_Accel_DLPFBandwidth_184,           // delay 5.80ms
	MPU9250_Accel_DLPFBandwidth_92,            // delay 7.80ms
	MPU9250_Accel_DLPFBandwidth_41,            // delay 11.80ms
	MPU9250_Accel_DLPFBandwidth_20,            // delay 19.80ms
	MPU9250_Accel_DLPFBandwidth_10,            // delay 35.70ms
	MPU9250_Accel_DLPFBandwidth_5,             // delay 66.96ms
	MPU9250_Accel_DLPFBandwidth_460_2,         // delay 1.94ms
	MPU9250_Accel_DLPFBandwidth_1_13k = 0x08,  // delay 0.75ms

} MPU9250_Accel_DLPFBandwidth;

typedef enum __MPU9250_Accel_SampleRateDivider {
	// check reg30, when use lower power mode
	// Hz
	// Bandwidth 1.1kHz, Delay 1ms
	MPU9250_LP_ACCEL_ODR_0_24HZ = 0x00,
	MPU9250_LP_ACCEL_ODR_0_49HZ,
	MPU9250_LP_ACCEL_ODR_0_98HZ,
	MPU9250_LP_ACCEL_ODR_1_95HZ,
	MPU9250_LP_ACCEL_ODR_3_91HZ,
	MPU9250_LP_ACCEL_ODR_7_81HZ,
	MPU9250_LP_ACCEL_ODR_15_63HZ,
	MPU9250_LP_ACCEL_ODR_31_25HZ,
	MPU9250_LP_ACCEL_ODR_62_50HZ,
	MPU9250_LP_ACCEL_ODR_125HZ,
	MPU9250_LP_ACCEL_ODR_250HZ,
	MPU9250_LP_ACCEL_ODR_500HZ
} MPU9250_Accel_SampleRateDivider;

typedef enum __MPU9250_Gyro_DLPFBandwidth {
	// to use following 2 options, reg27 Bit[1:0](fchoice_b) must be set
	// these to options are not related to reg26 Bit[2:0](DLPF_CFG), but reg27 Bit[1:0](fchoice_b)
	// notice: this options can also affect temperature sensor
	// Hz
	MPU9250_Gyro_DLPFBandwidth_8800_x = 0x00,    // delay 0.064ms
	MPU9250_Gyro_DLPFBandwidth_3600_x = 0x00,    // delay 0.11ms

	// follow options can be set to reg26 Bit[2:0](DLPF_CFG) to set DLPFBandwidth
	MPU9250_Gyro_DLPFBandwidth_250 = 0x00,     // delay 0.97ms
	MPU9250_Gyro_DLPFBandwidth_184,            // delay 2.9ms
	MPU9250_Gyro_DLPFBandwidth_92,             // delay 3.9ms
	MPU9250_Gyro_DLPFBandwidth_41,             // delay 5.9ms
	MPU9250_Gyro_DLPFBandwidth_20,             // delay 9.9ms
	MPU9250_Gyro_DLPFBandwidth_10,             // delay 17.85ms
	MPU9250_Gyro_DLPFBandwidth_5,              // delay 33.48ms
	MPU9250_Gyro_DLPFBandwidth_3600,           // delay 0.17ms

} MPU9250_Gyro_DLPFBandwidth;

typedef struct __MPU9250_PropTypDef {

// reserved, maybe useless

} MPU9250_PropTypeDef;

typedef struct __MPU9250_DataTypeDef {
	// origin data from mpu9250
	volatile int16_t Accel_row[3];
	volatile int16_t Gyro_row[3];
	volatile int16_t Magn_row[3];
	// store real data in float
	float Accel[3];
	float Gyro[3];
	float Magn[3];

} MPU9250_DataTypeDef;

typedef struct __MPU9250 {
	MPU9250_PropTypeDef mpu_prop;
	MPU9250_DataTypeDef mpu_data;
} MPU9250;

uint8_t mpu_r_ak8963_WhoAmI(MPU9250 *mpu);
uint8_t mpu_r_WhoAmI(MPU9250 *mpu);
uint8_t MPU9250_Init(MPU9250 *mpu);
void MPU9250_ReadAccel(MPU9250 *mpu);
void MPU9250_ReadGyro(MPU9250 *mpu);
void MPU9250_ReadMag(MPU9250 *mpu);

#ifdef __cplusplus
}
#endif

#endif
