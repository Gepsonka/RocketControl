/*
 * sysinit.h
 *
 *  Created on: Aug 18, 2022
 *      Author: gepsonka
 */


#include "sysinit.h"

/*
 * Check peripherals, if an essential peripherals
 * does not work, got to error state
 * Essentials:
 * 		- IMU
 * 		- LoRa
 * 		- Altimeter
 * 		- Servos
 * 	(all of the periphery are essential pretty much)
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

	// Manual testing (look at the rocket fins)
	Are_Servos_Available();
	HAL_Delay(2000);
}

/**
 * Checking if altimeter is available.
 */
bool Is_BMP280_Available(){
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c3, BMP280_I2C_ADDRESS_0 << 1, 50, 50);
	if (status == HAL_OK){
		return 1;
	}

	return 0;
}


bool Is_IMU_Available(){
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, MPU9250_ADDRESS, 50, 50);
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
		// Transmit reg address, then receive it value
		res = HAL_SPI_Transmit(&hspi1, &address, 1, 500);
		// End SPI transaction
		HAL_GPIO_WritePin(GPIOA, LoRa_NSS_Pin, GPIO_PIN_SET);
	}

	if (res == HAL_OK){
		return 1;
	}

	return 0;
}

