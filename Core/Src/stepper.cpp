/*
 * stepper.cpp
 *
 *  Created on: Mar 7, 2023
 *      Author: Merthan
 */
#include "main.h"
#include "stepper.h"
#include "pid.h"
#include "math.h"
#include <stdbool.h>
#include <stdio.h>


#define stepper_timer htim4
extern TIM_HandleTypeDef stepper_timer;

const double stp_kp = 0.4, stp_ki = 0.2, stp_kd = 2.0;
double stp_pid_input, stp_pid_output, stp_pid_setpoint;
PID stp_PID(&stp_pid_input, &stp_pid_output, &stp_pid_setpoint, stp_kp, stp_ki, stp_kd, _PID_CD_DIRECT);


void stepper_Init(){
	//HAL_GPIO_WritePin(STP_EN_GPIO_Port, STP_EN_Pin, GPIO_PIN_SET);

	stepper_timer.Init.Prescaler = 72 - 1;
	stepper_timer.Init.Period = 60;
	stepper_timer.Instance->CCR1 = 0;
	HAL_TIM_PWM_Start(&stepper_timer,TIM_CHANNEL_1);

	stp_PID.SetMode(_PID_MODE_AUTOMATIC);
	stp_PID.SetSampleTime(1);
	stp_PID.SetOutputLimits(-60, 60);

}

void stepper_set_pwm(int16_t pwm){
	if (pwm != 0){
		HAL_GPIO_WritePin(STP_EN_GPIO_Port, STP_EN_Pin, GPIO_PIN_SET);
		if(pwm>0){
			HAL_GPIO_WritePin(STP_DIR_GPIO_Port, STP_DIR_Pin, GPIO_PIN_SET);
			stepper_timer.Instance->CCR1 = pwm;
		}
		else{
			pwm *= -1;
			HAL_GPIO_WritePin(STP_DIR_GPIO_Port, STP_DIR_Pin, GPIO_PIN_RESET);
			stepper_timer.Instance->CCR1 = pwm;
		}
	}
	else{
		HAL_GPIO_WritePin(STP_EN_GPIO_Port, STP_EN_Pin, GPIO_PIN_RESET);;
	}
}
