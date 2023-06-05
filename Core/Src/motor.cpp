/*
 * encoder_v2.cpp
 *
 *  Created on: 11 Kas 2022
 *      Author: Merthan
 */
#include "motor.h"
#include "pid.h"
#include "math.h"

const double drive_gear_ratio = 1.0, steer_gear_ratio= 1.0;

#define encoder_timer htim2
extern TIM_HandleTypeDef encoder_timer;

#define pwm_timer htim3
extern TIM_HandleTypeDef pwm_timer;

#define aar 900

int32_t encoder_pulses, encoder_pulses_prev;
int16_t encoder_temp = 0, encoder_temp_shifting = 0;
uint32_t encoder_temp_timer = 0;

double pulses_diff, speed_act, speed_wanted, speed_diff, angle_act;

#define PID_TIME 50
const double kp = 0.4, ki = 0.2, kd = 2.0;
double pid_input, pid_output, pid_setpoint;
double pwm_duty = 0.0;

PID motor_PID(&pid_input, &pid_output, &pid_setpoint, kp, ki, kd, _PID_CD_DIRECT);

static int32_t nowTime = 0, dTime = 0;

void motor_pins_setup(){
	/*
	pwm_timer.Init.Prescaler = timer_prescale - 1;
	pwm_timer.Init.Period = aar - 1;
	pwm_timer.Instance->CCR1 = 0;
	pwm_timer.Instance->CCR2 = 0;
	*/
	HAL_TIM_PWM_Start(&pwm_timer,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&pwm_timer,TIM_CHANNEL_2);
}


void encoder_setup(){
	HAL_TIM_Encoder_Start(&encoder_timer, TIM_CHANNEL_ALL);

	HAL_TIM_Base_Start_IT(&encoder_timer);
	encoder_temp = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &encoder_timer){
		encoder_temp_timer = (uint32_t) __HAL_TIM_GET_COUNTER(&encoder_timer);
		if (encoder_temp_timer == 0){
			encoder_temp++;
		}
		else if (encoder_temp_timer == 65535){
			encoder_temp--;
		}
	}
	if (htim->Instance == TIM1) {
	    HAL_IncTick();
	}
}

int32_t encoder_counter(){
	int32_t pulses = (int32_t)__HAL_TIM_GET_COUNTER(&encoder_timer) + (encoder_temp << 16);
	return pulses/4;
}

void pid_setup(){
	motor_PID.SetMode(_PID_MODE_AUTOMATIC);
	motor_PID.SetSampleTime(PID_TIME);
	motor_PID.SetOutputLimits(-50, 50);
}

void motor_Init(){
	pid_setup();
	encoder_setup();
	motor_pins_setup();
}

void motor_set_pwm(int16_t pwm){
	(pwm > aar) ? (pwm = aar) : ((pwm < -aar) ? (pwm = -aar) : (pwm = pwm));
	if (pwm != 0){
		HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
		if(pwm>0){
			pwm_timer.Instance->CCR1 = pwm;  //PA6
			pwm_timer.Instance->CCR2 = 0;    //PA7
		}
		else{
			pwm_timer.Instance->CCR1 = 0;
			pwm_timer.Instance->CCR2 = -pwm;
		}
	}
	else{
		pwm_timer.Instance->CCR1 = 0;        //PA6
		pwm_timer.Instance->CCR2 = 0;        //PA7
		HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);;
	}
}
double motor_get_speed(){
	return speed_act;
}
double motor_get_angle(){
	return angle_act;
}

void motor_set_speed(double speed){
	nowTime = HAL_GetTick();
	if (nowTime - dTime >= PID_TIME){
		pid_setpoint=speed;

		encoder_pulses = encoder_counter();
		encoder_pulses_prev = encoder_pulses - encoder_pulses_prev;

		speed_act = ((1000.00 * (double) encoder_pulses_prev) / ((double) (nowTime - dTime) * 600)) / drive_gear_ratio;

		pid_input = speed_act;

		motor_PID.Compute();

		pwm_duty += pid_output;
		if(pwm_duty > aar){
			pwm_duty = aar;
		}
		else if(pwm_duty < -aar){
			pwm_duty = -aar;
		}

		if(pid_setpoint == 0){
			pwm_duty=0;
			motor_set_pwm(pwm_duty);
		}
		else{
			motor_set_pwm((int16_t)pwm_duty);

		}
		encoder_pulses_prev = encoder_counter();

		dTime = HAL_GetTick();
	}
}

void motor_set_angle(double angle){
	nowTime = HAL_GetTick();
	if (nowTime - dTime >= PID_TIME){
		pid_setpoint=angle;

		encoder_pulses = encoder_counter();
		encoder_pulses_prev = encoder_pulses - encoder_pulses_prev;

		angle_act = (((1000.00 * (double) encoder_pulses_prev) / ((double) (nowTime - dTime) * 600)) * M_PI) / steer_gear_ratio;

		pid_input = angle_act;

		motor_PID.Compute();

		pwm_duty += pid_output;
		if(pwm_duty > aar){
			pwm_duty = aar;
		}
		else if(pwm_duty < -aar){
			pwm_duty = -aar;
		}

		if(pid_setpoint == 0){
			pwm_duty=0;
			motor_set_pwm(pwm_duty);
		}
		else{
			motor_set_pwm((int16_t)pwm_duty);

		}
		encoder_pulses_prev = encoder_counter();

		dTime = HAL_GetTick();
	}
}
