/*
 * IMU.c
 *
 *  Created on: Jul 10, 2022
 *      Author: expel
 */


#include "IMU.h"

/*
 * For more info on self-testing:
 * https://github.com/kriswiner/MPU9250/blob/master/Documents/AN-MPU-9250A-03%20MPU-9250%20Accel%20Gyro%20and%20Compass%20Self-Test%20Implementation%20v1%200_062813.pdf
 */
void Self_Test(){

	uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};
	uint8_t self_test[6];
	int32_t gyroAVG[3] = {0};
	int32_t gyro_self_testAVG[3] = {0};
	int32_t accelAVG[3] = {0};
	int32_t accel_self_testAVG[3] = {0};
	float factory_trim[6];
	uint8_t FS;

	// Set gyro sample rate to 1kHz
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, SMPLRT_DIV, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Set DLFP to 92Hz
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, CONFIG, 1, (uint8_t*)(0x02), 1, HAL_MAX_DELAY);
	// Set full scale range of the gyro to 250 dps
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, (uint8_t*)(1<<FS), 1, HAL_MAX_DELAY);
	// Set accel rate to 1kHz and bandwidth to 92Hz
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, (uint8_t*)(0x02), 1, HAL_MAX_DELAY);
	// Set accel full scale range to 2G
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, (uint8_t*)(1<<FS), 1, HAL_MAX_DELAY);


	// Calculate data needed to get the average
	for(int i=0; i<200; i++){
		// Reading accel data registers
		HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &raw_data[0], 6, HAL_MAX_DELAY);
		accelAVG[0] += (int16_t) (((int16_t)raw_data[0] << 8) | raw_data[1]);
		accelAVG[1] += (int16_t) (((int16_t)raw_data[2] << 8) | raw_data[3]);
		accelAVG[2] += (int16_t) (((int16_t)raw_data[4] << 8) | raw_data[5]);

		HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, 6, &raw_data[0], 6, HAL_MAX_DELAY);
		gyroAVG[0] += (int16_t) (((int16_t)raw_data[0] << 8) | raw_data[1]);
		gyroAVG[1] += (int16_t) (((int16_t)raw_data[2] << 8) | raw_data[3]);
		gyroAVG[2] += (int16_t) (((int16_t)raw_data[4] << 8) | raw_data[5]);

	}

	// Get the average of the 200 readings
	for(int i=0; i<3; i++){
		accelAVG[i] /= 200;
		gyroAVG[i] /= 200;
	}

	// Configure accel and gyro for self test
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, (uint8_t*)(0xE0), 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, (uint8_t*)(0xE0), 1, HAL_MAX_DELAY);
	HAL_Delay(100); // Let the device to stabilize

	// Calculate data needed to get the average (self-test)
	for(int i=0; i<200; i++){
		// Reading accel data registers
		HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &raw_data[0], 6, HAL_MAX_DELAY);
		accel_self_testAVG[0] += (int16_t) (((int16_t)raw_data[0] << 8) | raw_data[1]);
		accel_self_testAVG[1] += (int16_t) (((int16_t)raw_data[2] << 8) | raw_data[3]);
		accel_self_testAVG[2] += (int16_t) (((int16_t)raw_data[4] << 8) | raw_data[5]);

		HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, 6, &raw_data[0], 6, HAL_MAX_DELAY);
		gyro_self_testAVG[0] += (int16_t) (((int16_t)raw_data[0] << 8) | raw_data[1]);
		gyro_self_testAVG[1] += (int16_t) (((int16_t)raw_data[2] << 8) | raw_data[3]);
		gyro_self_testAVG[2] += (int16_t) (((int16_t)raw_data[4] << 8) | raw_data[5]);
	}


	// Get the average of the 200 readings for self test
	for(int i=0; i<3; i++){
		accel_self_testAVG[i] /= 200;
		gyro_self_testAVG[i] /= 200;
	}

	// Configure accel and gyro for normal operation
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_X_ACCEL, 1, &self_test[0], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_Y_ACCEL, 1, &self_test[1], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_Z_ACCEL, 1, &self_test[2], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_X_GYRO, 1, &self_test[3], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_Y_GYRO, 1, &self_test[4], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_Z_GYRO, 1, &self_test[5], 1, HAL_MAX_DELAY);


	// Retrieve factory self-test value from self-test code reads
	factory_trim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test[0] - 1.0) )); // FT[Xa] factory trim calculation
	factory_trim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test[1] - 1.0) )); // FT[Ya] factory trim calculation
	factory_trim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test[2] - 1.0) )); // FT[Za] factory trim calculation
	factory_trim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test[3] - 1.0) )); // FT[Xg] factory trim calculation
	factory_trim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test[4] - 1.0) )); // FT[Yg] factory trim calculation
	factory_trim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)self_test[5] - 1.0) )); // FT[Zg] factory trim calculation


	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		_self_test_result[i]   = 100.0f*((float)(accel_self_testAVG[i] - accelAVG[i]))/factory_trim[i] - 100.0f;   // Report percent differences
		_self_test_result[i+3] = 100.0f*((float)(gyro_self_testAVG[i] - gyroAVG[i]))/factory_trim[i+3] - 100.0f; // Report percent differences
	}
}

void Calibrate_MPU9250(){
	uint8_t raw_data[12]; // holds gyro and accel x,y,z data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0};
	int32_t accel_bias[3] = {0, 0, 0};

	// reset device
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, (uint8_t*)(0x80), 1, HAL_MAX_DELAY);
	HAL_Delay(100); // Wait till the the device resets


	// Get stable time source
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, (uint8_t*)(0x01), 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_2, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	HAL_Delay(200);

	// Configure for bias
	// Disable all interrupts
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_ENABLE, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Disable FIFO
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, FIFO_EN, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Turn on internal clock
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Disable i2c master
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_MST_CTRL, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Disable FIFO and i2c master modes
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Reset FIFO and DMP
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, (uint8_t*)(0x0C), 1, HAL_MAX_DELAY);
	HAL_Delay(100);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	// Set low-pass filter to 188Hz
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, CONFIG, 1, (uint8_t*)(0x01), 1, HAL_MAX_DELAY);
	// Disable FIFO
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, FIFO_EN, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Turn on internal clock
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);



}


void Init_MPU9250(){

}


uint8_t get_MPU9250_ID(){
	uint8_t id;
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, &id, 1, HAL_MAX_DELAY);
	return id;
}

uint8_t get_AK8963_ID(){
	// enable i2c master mode
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, (uint8_t*)(0x20), 1, HAL_MAX_DELAY);
	// set i2c multi-master to 400KHz
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_MST_CTRL, 1, (uint8_t*)(0x0D), 1, HAL_MAX_DELAY);
	// Set the magnetometer the slave and set to read
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_ADDR, 1, (uint8_t*)(AK8963_ADDRESS | 0x80), 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_REG, 1, AK8963_WHO_AM_I, 1, HAL_MAX_DELAY);
	// Enable i2c and transfer 1 byte into the MPU9250
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_CTRL, 1, (uint8_t*)(0x81), 1, HAL_MAX_DELAY);
	HAL_Delay(10); // Wait till data transfer happens
	uint8_t id = 0;
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, EXT_SENS_DATA_00, 1, &id, 1,HAL_MAX_DELAY);

	return id;

}


HAL_StatusTypeDef MPU9250_Write_Gyro_Full_Scale_Range(Gscale gscale){

	HAL_StatusTypeDef res;
	uint8_t mode = gscale;
	res = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, &mode, 1, HAL_MAX_DELAY);

	// Storing the value set for range
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
