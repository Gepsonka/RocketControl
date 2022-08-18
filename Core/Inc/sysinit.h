/*
 * sysinit.h
 *
 *  Created on: Aug 18, 2022
 *      Author: gepsonka
 */

#ifndef INC_SYSINIT_H_
#define INC_SYSINIT_H_

#include <stdbool.h>
#include "i2c.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "bmp280.h"
#include "LoRa.h"
#include "IMU.h"
#include "servo.h"

#define Are_Servos_Available() __Test_Servos()

void Check_Pheriperials();

bool Is_BMP280_Available();

bool Is_IMU_Available();

bool Is_LoRa_Available();




#endif /* INC_SYSINIT_H_ */
