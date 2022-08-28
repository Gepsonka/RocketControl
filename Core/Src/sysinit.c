/*
 * sysinit.h
 *
 *  Created on: Aug 18, 2022
 *      Author: gepsonka
 */


#include "sysinit.h"


extern lora_sx1276 LoRa;
extern BMP280_HandleTypedef bmp280;
extern float gyroBias[3], accelBias[3], magCalibration[3];

/*
 * Check peripherals, if an essential peripherals
 * does not work, got to error state
 * Essentials:
 * 		- IMU
 * 		- LoRa
 * 		- Altimeter
 * 		- Servos
 * 	(all of the peripherals are essential pretty much)
 */
void Check_Peripherals(){
	if (!Is_BMP280_Available()){ // not putting all check into one if
		Error_Handler();		 // bc this way it is easier to debug
	}

	if (!Is_IMU_Available()){
		Error_Handler();
	}

	if (!Is_LoRa_Available()){
		Error_Handler();
	}

	// Manual testing (look at the rocket canards)
	Are_Servos_Available();
	HAL_Delay(2000);
}

void Init_Peripherals(){
	Init_LoRa();
	Init_BMP280();
	Init_IMU();
}



/**
 * Checking if altimeter is available.
 */
bool Is_BMP280_Available(){
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c3, BMP280_I2C_ADDRESS_0 << 1, 10, 10);
	if (status == HAL_OK){
		return 1;
	}

	return 0;
}


bool Is_IMU_Available(){
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, MPU9250_ADDRESS, 10, 10);
	if (status == HAL_OK){
		return 1;
	}

	return 0;
}


bool Is_LoRa_Available(){
	uint32_t res;
	uint8_t address = 0x0D;
	for (int i=0;i<50;i++){
		HAL_GPIO_WritePin(GPIOA, LoRa_NSS_Pin, GPIO_PIN_RESET);
		res = HAL_SPI_Transmit(&hspi1, &address, 1, 500);
		HAL_GPIO_WritePin(GPIOA, LoRa_NSS_Pin, GPIO_PIN_SET);

		if (res == HAL_OK){
			return 1;
		}
	}

	return 0;
}


void Init_LoRa(){
	lora_init(&LoRa, &hspi1, LoRa_NSS_GPIO_Port, LoRa_NSS_Pin, LoRa_Reset_GPIO_Port, LoRa_Reset_Pin, LORA_BASE_FREQUENCY_EU, 0x10);
	lora_set_tx_power(&LoRa, 17);
	lora_set_signal_bandwidth(&LoRa, LORA_BANDWIDTH_7_8_KHZ);
	lora_set_spreading_factor(&LoRa, 12);
	lora_set_coding_rate(&LoRa, LORA_CODING_RATE_4_8);
	lora_set_crc(&LoRa, 3);
	lora_set_preamble_length(&LoRa, 12);
	lora_set_explicit_header_mode(&LoRa);
	lora_enable_interrupt_rx_done(&LoRa);
	lora_enable_interrupt_tx_done(&LoRa);
}

void Init_BMP280(){
	  bmp280_init_default_params(&bmp280.params);
	  bmp280.addr = BMP280_I2C_ADDRESS_0;
	  bmp280.i2c = &hi2c3;
	  bool bme280p = bmp280.id == BME280_CHIP_ID;
	  bmp280_init(&bmp280, &bmp280.params);
}


void Init_IMU(){
	if (IMUreadByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) != 0x70){
		uint8_t res = IMUreadByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
		Error_Handler();
	}

	initMPU9250(AFS_8G, GFS_250DPS, 7); // sample rate  SAMPLE_RATE = Internal_Sample_Rate / (1 + 7) = 1kHz
	MinitAK8963Slave(MFS_16BITS, 6, magCalibration);

}


void Calibrate_IMU(){
	resetMPU9250();
	calibrateMPU9250(gyroBias, accelBias);

}
