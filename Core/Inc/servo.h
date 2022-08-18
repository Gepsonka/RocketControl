/*
 * servo.h
 *
 *  Created on: Jul 10, 2022
 *      Author: expel
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>

#include "tim.h"

#define NUM_OF_SERVO_MOTORS 4
#define Wait_Till_Servo_Processes() Hal_Delay(21); // 21ms if the signal output drifts accidentally

typedef enum {
	INLINE_DUTY_CYCLE = 150, // Fins are parallel with the rocket, default position
	LOWEST_ENABLED_PITCH_DUTY_CYCLE = 125,  // -45 DEG
	HIGHEST_ENABLED_PICH_DUTY_CYCLE = 175,   // +45 DEG
	LOWEST_PITCH_DUTY_CYCLE = 100, // -90 DEG
	HIGHEST_PITCH_DUTY_CYCLE = 200 // +90 DEG
} PitchesDutyCycle;

typedef enum {
	INLINE_DEG = 0, // Fins are parallel with the rocket, default position
	LOWEST_ENABLED_PITCH_DEG = -45,
	HIGHEST_ENABLED_PICH_DEG = 45,
	LOWEST_PITCH_DEG = -90,
	HIGHEST_PITCH_DEG = 90
} PitchesDegree;

typedef enum {
	NORTH_SERVO = 1,
	EAST_SERVO,
	SOUTH_SERVO,
	WEST_SERVO
} ServoId;


typedef enum {
	LONGITUDINAL, // NORTH_SERVO and SOUTH_SERVO
	HORIZONTAL // EAST_SERVO and WEST_SERVO
} ServoPair; // Position is decided by the state of the IMU

/*
 * Servo PWMs controlling the servos.
 * The duty cycle varies between 1 and 2ms each pulse 50hz long.
 * 1ms duty cycle is -90deg, 2ms is +90deg, 1.5 is 0deg.
 */
typedef struct {
	volatile uint32_t* Servo1DutyCycle;
	volatile uint32_t* Servo2DutyCycle;
	volatile uint32_t* Servo3DutyCycle;
	volatile uint32_t* Servo4DutyCycle;
} ServoDutyCycles;




void Servo_Init();

void Set_Servos_Into_Default_Position();

void Check_Servos_Manually();

void Move_All_Servos(uint8_t num_of_degrees);

void Move_Servo_Pair(uint8_t num_of_degrees, ServoPair servo_pair);

void _Move_Servo(int8_t num_of_servo, int8_t degree);

void _Check_And_Switch_Allowed_Degree(uint8_t* deg);

void __Test_Servos();




#endif /* INC_SERVO_H_ */
