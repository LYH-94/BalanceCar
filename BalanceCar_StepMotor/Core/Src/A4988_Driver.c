/*
 * A4988_Driver.c
 *
 *  Created on: Oct 6, 2021
 *      Author: LYH
 */

#include "A4988_Driver.h"

void StepMotorDriver_Iint(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_WritePin(GPIOA, PA8_DIR_L_Pin, GPIO_PIN_RESET);  // StepMotor direction
	HAL_GPIO_WritePin(GPIOA, PA9_DIR_R_Pin, GPIO_PIN_SET);  // StepMotor direction
	HAL_GPIO_WritePin(GPIOB, PB12_ENA_Pin, GPIO_PIN_SET);  // StepMotor output DISABLE

	switch(MICRO_STEP_RESOLUTION)
	{
	case 16:
		HAL_GPIO_WritePin(GPIOB, PB13_MS1_Pin, GPIO_PIN_SET);  // StepMotor microstep MS1 , HIGH
		HAL_GPIO_WritePin(GPIOB, PB14_MS2_Pin, GPIO_PIN_SET);  // StepMotor microstep MS2 , HIGH
		HAL_GPIO_WritePin(GPIOB, PB15_MS3_Pin, GPIO_PIN_SET);  // StepMotor microstep MS3 , HIGH
		//printf("microstep = 16\n");
		break;
	case 8:
		HAL_GPIO_WritePin(GPIOB, PB13_MS1_Pin, GPIO_PIN_SET);  // StepMotor microstep MS1 , HIGH
		HAL_GPIO_WritePin(GPIOB, PB14_MS2_Pin, GPIO_PIN_SET);  // StepMotor microstep MS2 , HIGH
		HAL_GPIO_WritePin(GPIOB, PB15_MS3_Pin, GPIO_PIN_RESET);  // StepMotor microstep MS3 , LOW
		//printf("microstep = 8\n");
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOB, PB13_MS1_Pin, GPIO_PIN_RESET);  // StepMotor microstep MS1 , LOW
		HAL_GPIO_WritePin(GPIOB, PB14_MS2_Pin, GPIO_PIN_SET);  // StepMotor microstep MS2 , HIGH
		HAL_GPIO_WritePin(GPIOB, PB15_MS3_Pin, GPIO_PIN_RESET);  // StepMotor microstep MS3 , LOW
		//printf("microstep = 4\n");
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, PB13_MS1_Pin, GPIO_PIN_SET);  // StepMotor microstep MS1 , HIGH
		HAL_GPIO_WritePin(GPIOB, PB14_MS2_Pin, GPIO_PIN_RESET);  // StepMotor microstep MS2 , LOW
		HAL_GPIO_WritePin(GPIOB, PB15_MS3_Pin, GPIO_PIN_RESET);  // StepMotor microstep MS3 , LOW
		//printf("microstep = 2\n");
		break;
	default:
		HAL_GPIO_WritePin(GPIOB, PB13_MS1_Pin, GPIO_PIN_RESET);  // StepMotor microstep MS1 , LOW
		HAL_GPIO_WritePin(GPIOB, PB14_MS2_Pin, GPIO_PIN_RESET);  // StepMotor microstep MS2 , LOW
		HAL_GPIO_WritePin(GPIOB, PB15_MS3_Pin, GPIO_PIN_RESET);  // StepMotor microstep MS3 , LOW
		//printf("microstep = 1\n");
		break;
	}

	//printf("PWM_PSC = %d\n", PWM_PSC);
	__HAL_TIM_SET_PRESCALER(htim, PWM_PSC);
}

/**
  * brief
  *
  * note
  *
  * param - htim ： Is the timer
  * param - angular_velocity ： The angular velocity of the motor (unit:degree/sec)
  * return - step
  */
void StepMotor_Control(TIM_HandleTypeDef *htim, double angular_velocity, uint8_t motor)
{
	if(motor == 1)  // 1=left motor; 0=right motor
	{
		if(angular_velocity < 0)
		{
			HAL_GPIO_WritePin(GPIOA, PA8_DIR_L_Pin, GPIO_PIN_SET);
			angular_velocity = -angular_velocity;
		}
		else
			HAL_GPIO_WritePin(GPIOA, PA8_DIR_L_Pin, GPIO_PIN_RESET);
	}
	else if(motor == 0)
	{
		if(angular_velocity < 0)
		{
			HAL_GPIO_WritePin(GPIOA, PA9_DIR_R_Pin, GPIO_PIN_RESET);
			angular_velocity = -angular_velocity;
		}
		else
			HAL_GPIO_WritePin(GPIOA, PA9_DIR_R_Pin, GPIO_PIN_SET);
	}

	if(angular_velocity < 1.502) angular_velocity = 1.502;
	if(angular_velocity > 1081.73) angular_velocity = 1081.73;
	uint16_t PWM_ARR = lroundf((PWM_TIMER_CLOCKS/(PWM_PSC+1))/(angular_velocity/(1.8/MICRO_STEP_RESOLUTION)))-1;
	if(PWM_ARR < 90) PWM_ARR = 90;
	if(PWM_ARR > 65535) PWM_ARR = 65535;

	//printf("PWM_ARR = %d\n", PWM_ARR);
	__HAL_TIM_SET_AUTORELOAD(htim, PWM_ARR);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (__HAL_TIM_GET_AUTORELOAD(htim) + 1)/2);

	HAL_GPIO_WritePin(GPIOB, PB12_ENA_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_1);
}

void StepMotor_Stop(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
}
