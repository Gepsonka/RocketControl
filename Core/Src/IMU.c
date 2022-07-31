/*
 * IMU.c
 *
 *  Created on: Jul 10, 2022
 *      Author: expel
 */


#include "IMU.h"

uint16_t _accel_scale_factor;

float _gyro_scale_factor;

float _self_test_result[6];

float _gyro_bias[3];

float _accel_bias[3];


/*
 * For more info on self-testing:
 * https://github.com/kriswiner/MPU9250/blob/master/Documents/AN-MPU-9250A-03%20MPU-9250%20Accel%20Gyro%20and%20Compass%20Self-Test%20Implementation%20v1%200_062813.pdf
 */
void Self_Test(){
	uint8_t i;
	uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};
	uint8_t self_test[6];
	int32_t gyroAVG[3] = {0};
	int32_t gyro_self_testAVG[3] = {0};
	int32_t accelAVG[3] = {0};
	int32_t accel_self_testAVG[3] = {0};
	float factory_trim[6];
	uint8_t FS = 0;
	HAL_StatusTypeDef status;

	// Set gyro sample rate to 1kHz
	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, SMPLRT_DIV, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Set DLFP to 92Hz
	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, CONFIG, 1, (uint8_t*)(0x02), 1, HAL_MAX_DELAY);
	// Set full scale range of the gyro to 250 dps
	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, (uint8_t*)(1<<FS), 1, HAL_MAX_DELAY);
	// Set accel rate to 1kHz and bandwidth to 92Hz
	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, (uint8_t*)(0x02), 1, HAL_MAX_DELAY);
	// Set accel full scale range to 2G
	status = HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, (uint8_t*)(1<<FS), 1, HAL_MAX_DELAY);


	// Calculate data needed to get the average
	for(i=0; i<200; i++){
		// Reading accel data registers
		HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, 1, &raw_data[0], 6, HAL_MAX_DELAY);
		accelAVG[0] += (int16_t) (((int16_t)raw_data[0] << 8) | raw_data[1]);
		accelAVG[1] += (int16_t) (((int16_t)raw_data[2] << 8) | raw_data[3]);
		accelAVG[2] += (int16_t) (((int16_t)raw_data[4] << 8) | raw_data[5]);

		HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, 1, &raw_data[0], 6, HAL_MAX_DELAY);
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
		HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, 1, &raw_data[0], 6, HAL_MAX_DELAY);
		accel_self_testAVG[0] += (int16_t) (((int16_t)raw_data[0] << 8) | raw_data[1]);
		accel_self_testAVG[1] += (int16_t) (((int16_t)raw_data[2] << 8) | raw_data[3]);
		accel_self_testAVG[2] += (int16_t) (((int16_t)raw_data[4] << 8) | raw_data[5]);

		HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, 1, &raw_data[0], 6, HAL_MAX_DELAY);
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
	// Set sample rate to 1kHz
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, SMPLRT_DIV, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Set gyro full-scale to 250 degrees per second
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Set accelerometer full-scale to 2 g
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);

	uint16_t gyro_sensitivity = 131; // 131 LSB/degrees/sec
	uint16_t accel_sensitivity = 16384; // 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	// Enable FIFO
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, (uint8_t*)(0x40), 1, HAL_MAX_DELAY);
	// Enable gyro and accelerometer sensors for FIFO
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, FIFO_EN, 1, (uint8_t*)(0x78), 1, HAL_MAX_DELAY);
	HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes


	// At end of sample accumulation, turn off FIFO sensor read
	// Disable gyro and accel sensors for FIFO
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, FIFO_EN, 1, (uint8_t*)(0x00), 1, HAL_MAX_DELAY);
	// Read FIFO sample count
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, FIFO_COUNTH, 1, &raw_data[0], 6, HAL_MAX_DELAY);
	fifo_count = ((uint16_t)raw_data[0] << 8) | raw_data[1];
	packet_count = (uint16_t)(fifo_count / 12); // How many sets of full gyro and accelerometer data for averaging

	for (int i=0; i<packet_count; i++){
		int16_t accel_temp[3] = {0, 0, 0};
		int16_t gyro_temp[3] = {0, 0, 0};

		HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, FIFO_R_W, 1, &raw_data[0], 12, HAL_MAX_DELAY);
		accel_temp[0] = (int16_t) (((int16_t)raw_data[0] << 8) | raw_data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)raw_data[2] << 8) | raw_data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)raw_data[4] << 8) | raw_data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)raw_data[6] << 8) | raw_data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)raw_data[8] << 8) | raw_data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)raw_data[10] << 8) | raw_data[11]) ;

		// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[0] += (int32_t) accel_temp[0];
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	// Normalize sums to get average count biases
	accel_bias[0] /= (int32_t) packet_count;
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	// Remove gravity from the z-axis accelerometer bias calculation
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accel_sensitivity;}
	else {accel_bias[2] += (int32_t) accel_sensitivity;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	raw_data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	raw_data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	raw_data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	raw_data[3] = (-gyro_bias[1]/4)       & 0xFF;
	raw_data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	raw_data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, XG_OFFSET_H, 1, &raw_data[0], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, XG_OFFSET_L, 1, &raw_data[1], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, YG_OFFSET_H, 1, &raw_data[2], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, YG_OFFSET_L, 1, &raw_data[3], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ZG_OFFSET_H, 1, &raw_data[4], 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ZG_OFFSET_L, 1, &raw_data[5], 1, HAL_MAX_DELAY);


	// Store scaled gyro bias
	_gyro_bias[0] = (float) gyro_bias[0]/(float) gyro_sensitivity;
	_gyro_bias[1] = (float) gyro_bias[1]/(float) gyro_sensitivity;
	_gyro_bias[2] = (float) gyro_bias[2]/(float) gyro_sensitivity;


	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.
	int32_t accel_bias_reg[3] = {0, 0, 0};
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, XA_OFFSET_H, 1, &raw_data[0], 2, HAL_MAX_DELAY);
	accel_bias_reg[0] = (int32_t) (((int16_t)raw_data[0] << 8) | raw_data[1]);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, YA_OFFSET_H, 1, &raw_data[0], 2, HAL_MAX_DELAY);
	accel_bias_reg[1] = (int32_t) (((int16_t)raw_data[0] << 8) | raw_data[1]);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ZA_OFFSET_H, 1, &raw_data[0], 2, HAL_MAX_DELAY);
	accel_bias_reg[2] = (int32_t) (((int16_t)raw_data[0] << 8) | raw_data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis


	for(ii = 0; ii < 3; ii++) {
		// If temperature compensation bit is set, record that fact in mask_bit
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01;
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	raw_data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	raw_data[1] = (accel_bias_reg[0])      & 0xFF;
	raw_data[1] = raw_data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	raw_data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	raw_data[3] = (accel_bias_reg[1])      & 0xFF;
	raw_data[3] = raw_data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	raw_data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	raw_data[5] = (accel_bias_reg[2])      & 0xFF;
	raw_data[5] = raw_data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Output scaled accelerometer biases for display in the main program
	_accel_bias[0] = (float)accel_bias[0]/(float) accel_sensitivity;
	_accel_bias[1] = (float)accel_bias[1]/(float) accel_sensitivity;
	_accel_bias[2] = (float)accel_bias[2]/(float) accel_sensitivity;


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
