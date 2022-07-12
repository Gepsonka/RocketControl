/*
 * IMU.c
 *
 *  Created on: Jul 10, 2022
 *      Author: expel
 */


#include "IMU.h"


uint16_t _accel_scale_factor;

float _gyro_scale_factor;

/*
 * Checking if the device is an MPU9250 on the
 */
IdentificationStatus Identify_GY91(I2C_HandleTypeDef* i2c_interface) {
	uint8_t check;

	HAL_StatusTypeDef hal_res = HAL_I2C_Mem_Read(i2c_interface, MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, &check, 1, HAL_MAX_DELAY);
	if (hal_res == HAL_ERROR){
		return IDENTIFY_I2C_ERROR;
	}

	if (check  == WHO_AM_I_MPU9250_SUPPOSED_TO){
		return IDENTIFY_OK;
	} else {
		return IDENTIFY_BAD_DEVICE;
	}
}



HAL_StatusTypeDef MPU9250_Write_Gyro_Full_Scale_Range(I2C_HandleTypeDef* i2c_interface, Gscale gscale){

	HAL_StatusTypeDef res;
	uint8_t mode = gscale;
	res = HAL_I2C_Mem_Write(i2c_interface, MPU9250_ADDRESS, GYRO_CONFIG, 1, &mode, 1, HAL_MAX_DELAY);

	switch (gscale){
		case GFS_250DPS:
			_gyro_scale_factor = GYRO_SF_250DPS;
			break;

		case GFS_500DPS:
			_gyro_scale_factor = GYRO_SF_500DPS;
			break;

		case GFS_1000DPS:
			_gyro_scale_factor = GYRO_SF_1000DPS;
			break;

		case GFS_2000DPS:
			_gyro_scale_factor = GYRO_SF_2000DPS;
			break;

		default:
			_gyro_scale_factor = GYRO_SF_500DPS;
			break;
	}

	return res;
}


//HAL_StatusTypeDef MPU9250_Write_Accel_Full_Scale_Range(Ascale ascale){
//
//}
