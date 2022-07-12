/*
 * servo.c
 *
 *  Created on: Jul 10, 2022
 *      Author: expel
 */


#include "servo.h"

ServoDutyCycles servoDutyCycles;

/*
 * Initialising the servos.
 * Starting the timer of each channel.
 * Storing Capture Compare Registers in the servo module.
 * Set the servos to their default (0deg) position.
 * All the funtions below require to call this before using them.
 */
void Servo_Init(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	servoDutyCycles.Servo1DutyCycle = &TIM1->CCR1;
	servoDutyCycles.Servo2DutyCycle = &TIM1->CCR2;
	servoDutyCycles.Servo3DutyCycle = &TIM1->CCR3;
	servoDutyCycles.Servo4DutyCycle = &TIM1->CCR4;

	Set_Servos_Into_Default_Position();
}


/*
 * Always runs before system start.
 * Checking if the servos work properly.
 * Starts to move the servos, checking every possible position. User needs to confirm.
 * If something is not right power off the device! The rocket will probably crash if even one of
 * the servos is not working properly.
 */
void Check_Servos_Manually(){
	int8_t degree = LOWEST_PITCH_DEG;

	for (int servo = 1; servo <= 4; servo++){
		for (int i = 0; i < 10; i++){
			_Move_Servo(servo, degree);
			degree += 20;
			HAL_Delay(1000);
		}
		degree = LOWEST_PITCH_DEG;
	}

	Set_Servos_Into_Default_Position();

}


void Set_Servos_Into_Default_Position(){
	TIM1->CCR1 = INLINE;
	TIM1->CCR2 = INLINE;
	TIM1->CCR3 = INLINE;
	TIM1->CCR4 = INLINE;
}

/*
 * Its purpose will be to rotate the rocket.
 */
void Move_All_Servos(uint8_t num_of_degrees){
	_Check_And_Switch_Allowed_Degree(&num_of_degrees);

	_Move_Servo(NORTH_SERVO, num_of_degrees);
	_Move_Servo(EAST_SERVO, num_of_degrees);
	_Move_Servo(SOUTH_SERVO, num_of_degrees);
	_Move_Servo(WEST_SERVO, num_of_degrees);
}


/*
 * Moving opposite servos at the same deg.
 * Helps guide the rocket.
 */
void Move_Servo_Pair(uint8_t num_of_degrees, ServoPair servo_pair){
	_Check_And_Switch_Allowed_Degree(&num_of_degrees);

	switch (servo_pair){
		case LONGITUDINAL:
			_Move_Servo(NORTH_SERVO, num_of_degrees);
			_Move_Servo(SOUTH_SERVO, num_of_degrees);
			break;
		case HORIZONTAL:
			_Move_Servo(EAST_SERVO, num_of_degrees);
			_Move_Servo(WEST_SERVO, num_of_degrees);
	}
}


/*
 * @param num_of_servo choose servo to move
 * @param degree to rotate the servo into (from -90 to +90)
 */
void _Move_Servo(int8_t num_of_servo, int8_t degree){
	uint8_t duty_cycle;

	if (degree < LOWEST_PITCH_DEG){
		duty_cycle = 100; // -90deg

	} else if (degree > HIGHEST_ENABLED_PICH_DEG){
		duty_cycle = 200; // +90deg

	} else {
		/* To get the duty cycle to the according degree we need this equation*/
		duty_cycle = (uint8_t) ( ( degree - (-90) ) / 1.8 ) + 100;
	}

	switch (num_of_servo){
		case NORTH_SERVO:
			*servoDutyCycles.Servo1DutyCycle = duty_cycle;
			break;

		case EAST_SERVO:
			*servoDutyCycles.Servo2DutyCycle = duty_cycle;
			break;

		case SOUTH_SERVO:
			*servoDutyCycles.Servo3DutyCycle = duty_cycle;
			break;

		case WEST_SERVO:
			*servoDutyCycles.Servo4DutyCycle = duty_cycle;
			break;
	}
}


/*
 * Must check at the beginning of each servo move function which changes the
 * direction of the rocket mid-flight.
 */
void _Check_And_Switch_Allowed_Degree(uint8_t* deg){
	if (*deg < LOWEST_ENABLED_PITCH_DEG){
		*deg = LOWEST_ENABLED_PITCH_DEG;
		return;
	}

	if (*deg > HIGHEST_ENABLED_PICH_DEG){
		*deg = HIGHEST_ENABLED_PICH_DEG;
		return;
	}
}


/*
 * For testing purposes!
 */
void __Test_Servos(){
	TIM1->CCR1 = INLINE;
	TIM1->CCR2 = INLINE;
	TIM1->CCR3 = INLINE;
	TIM1->CCR4 = INLINE;

	HAL_Delay(2000);


	TIM1->CCR1 = LOWEST_ENABLED_PITCH;
	TIM1->CCR2 = LOWEST_ENABLED_PITCH;
	TIM1->CCR3 = LOWEST_ENABLED_PITCH;
	TIM1->CCR4 = LOWEST_ENABLED_PITCH;

	HAL_Delay(2000);


	TIM1->CCR1 = HIGHEST_ENABLED_PICH;
	TIM1->CCR2 = HIGHEST_ENABLED_PICH;
	TIM1->CCR3 = HIGHEST_ENABLED_PICH;
	TIM1->CCR4 = HIGHEST_ENABLED_PICH;


	HAL_Delay(2000);

	TIM1->CCR1 = INLINE;
	TIM1->CCR2 = INLINE;
	TIM1->CCR3 = INLINE;
	TIM1->CCR4 = INLINE;
}
