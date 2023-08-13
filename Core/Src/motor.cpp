/*
 * encoder_v2.cpp
 *
 *  Created on: 11 Kas 2022
 *      Author: Merthan
 */
#include "motor.h"
#include "pid.h"
#include "math.h"

const double drive_motor2encoder = 12.0/36.0, drive_pinion= 12.0, drive_pinion2 = 70.0, drive_middle = 30.0, drive_middle2 = 26.0, drive_bevel = 20.0, drive_bevel2 = 68.0;
const double drive_gear_ratio = (1 / ((drive_pinion / drive_pinion2) * (drive_middle / drive_middle2) * (drive_bevel / drive_bevel2) )) * drive_motor2encoder;

const double steer_motor2encoder = 12.0/36.0, steer_pinion = 12.0, steer_pinion2 = 60.0, steer_middle = 16.32, steer_middle2 = 98;
const double steer_gear_ratio = (1 / ((steer_pinion / steer_pinion2) * (steer_middle / steer_middle2) )) * steer_motor2encoder;

const double print_gear_ratio(int a){
	if (a == 1)
		return steer_gear_ratio;
	else if (a == 2)
		return drive_gear_ratio;
}

#define drive_encoder_timer htim2
extern TIM_HandleTypeDef drive_encoder_timer;

#define steer_encoder_timer htim3
extern TIM_HandleTypeDef steer_encoder_timer;

#define pwm_timer htim1
extern TIM_HandleTypeDef pwm_timer;

#define aar 900
#define min_aar 120

int32_t steer_encoder_pulses, steer_encoder_pulses_prev;
int32_t drive_encoder_pulses, drive_encoder_pulses_prev;

int16_t steer_encoder_temp = 0;
int16_t drive_encoder_temp = 0;

uint32_t steer_encoder_temp_timer = 0;
uint32_t drive_encoder_temp_timer = 0;

const double encoder_resolution = 600.0;

double drive_pulses_diff, steer_pulses_diff, angular_speed_act, angle_act;

#define PID_TIME 50
const double steer_kp = 20.0, steer_ki = 10.0, steer_kd = 10.0;
double steer_pid_input, steer_pid_output, steer_pid_setpoint;

const double drive_kp = 80, drive_ki = 20.0, drive_kd = 10.0;
double drive_pid_input, drive_pid_output, drive_pid_setpoint;

double drive_pwm_duty = 0.0;
double steer_pwm_duty = 0.0;

PID steer_PID(&steer_pid_input, &steer_pid_output, &steer_pid_setpoint, steer_kp, steer_ki, steer_kd, _PID_CD_DIRECT);
PID drive_PID(&drive_pid_input, &drive_pid_output, &drive_pid_setpoint, drive_kp, drive_ki, drive_kd, _PID_CD_DIRECT);

static int32_t drive_nowTime = 0, drive_dTime = 0;
static int32_t steer_nowTime = 0, steer_dTime = 0;

void motor_pins_setup(){
	/*
	pwm_timer.Init.Prescaler = timer_prescale - 1;
	pwm_timer.Init.Period = aar - 1;
	pwm_timer.Instance->CCR1 = 0;
	pwm_timer.Instance->CCR2 = 0;
	*/
	HAL_TIM_PWM_Start(&pwm_timer,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&pwm_timer,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&pwm_timer,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&pwm_timer,TIM_CHANNEL_4);
	pwm_timer.Instance->CCR1 = 0;
	pwm_timer.Instance->CCR2 = 0;
	pwm_timer.Instance->CCR3 = 0;
	pwm_timer.Instance->CCR4 = 0;
}


void encoder_setup(){
	HAL_TIM_Encoder_Start(&drive_encoder_timer, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&drive_encoder_timer, 0);
	HAL_TIM_Base_Start_IT(&drive_encoder_timer);

	HAL_TIM_Encoder_Start(&steer_encoder_timer, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&steer_encoder_timer, 0);
	HAL_TIM_Base_Start_IT(&steer_encoder_timer);

	steer_encoder_temp = 0;
	drive_encoder_temp = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &steer_encoder_timer){
		steer_encoder_temp_timer = (uint32_t) __HAL_TIM_GET_COUNTER(&steer_encoder_timer);
		if (steer_encoder_temp_timer == 0){
			steer_encoder_temp++;
		}
		else if (steer_encoder_temp_timer == 65535){
			steer_encoder_temp--;
		}
	}
	if (htim == &drive_encoder_timer){
		drive_encoder_temp_timer = (uint32_t) __HAL_TIM_GET_COUNTER(&drive_encoder_timer);
			if (drive_encoder_temp_timer == 0){
				drive_encoder_temp++;
			}
			else if (drive_encoder_temp_timer == 65535){
				drive_encoder_temp--;
			}
		}
	if (htim->Instance == TIM1) {
	    HAL_IncTick();
	}
}
double pwm_debug(int16_t select){
	if (select == 1) return drive_pwm_duty;
	else if (select == 2) return drive_pid_input;
	else if (select == 3) return drive_pid_output;
	else if (select == 4) return steer_pwm_duty;
	else if (select == 5) return steer_pid_input;
	else if (select == 6) return steer_pid_output;
}


int32_t steer_encoder_counter(){
	int32_t pulses = (uint32_t)__HAL_TIM_GET_COUNTER(&steer_encoder_timer) + (steer_encoder_temp << 16);
	return pulses/4;
}

int32_t drive_encoder_counter(){
	int32_t pulses = (uint32_t)__HAL_TIM_GET_COUNTER(&drive_encoder_timer) + (drive_encoder_temp << 16);
	return pulses/4;
}

void pid_setup(){
	steer_PID.SetMode(_PID_MODE_AUTOMATIC);
	steer_PID.SetSampleTime(PID_TIME);
	steer_PID.SetOutputLimits(-30, 30);

	drive_PID.SetMode(_PID_MODE_AUTOMATIC);
	drive_PID.SetSampleTime(PID_TIME);
	drive_PID.SetOutputLimits(-200, 200);
}

void motor_Init(){
	pid_setup();
	encoder_setup();
	motor_pins_setup();
}

void motor_set_pwm(int16_t pwm, short motor){
	(pwm > aar) ? (pwm = aar) : ((pwm < -aar) ? (pwm = -aar) : (pwm = pwm));
	if (motor == 1){
		if (pwm != 0){
			HAL_GPIO_WritePin(drive_m_en_GPIO_Port, drive_m_en_Pin, GPIO_PIN_SET);
			if(pwm>0){
				pwm_timer.Instance->CCR1 = pwm;  //PA8
				pwm_timer.Instance->CCR2 = 0;    //PA9
			}
			else{
				pwm_timer.Instance->CCR1 = 0;
				pwm_timer.Instance->CCR2 = -pwm;
			}
		}
		else{
			pwm_timer.Instance->CCR1 = 0;        //PA8
			pwm_timer.Instance->CCR2 = 0;        //PA9
			HAL_GPIO_WritePin(drive_m_en_GPIO_Port, drive_m_en_Pin, GPIO_PIN_RESET);;
		}
	}
	else if (motor == 2){
		if (pwm != 0){
			HAL_GPIO_WritePin(steer_m_en_GPIO_Port, steer_m_en_Pin, GPIO_PIN_SET);
			if(pwm>0){
				pwm_timer.Instance->CCR3 = pwm;  //PA10
				pwm_timer.Instance->CCR4 = 0;    //PA11
			}
			else{
				pwm_timer.Instance->CCR3 = 0;
				pwm_timer.Instance->CCR4 = -pwm;
			}
		}
		else{
			pwm_timer.Instance->CCR3 = 0;        //PA10
			pwm_timer.Instance->CCR4 = 0;        //PA11
			HAL_GPIO_WritePin(steer_m_en_GPIO_Port, steer_m_en_Pin, GPIO_PIN_RESET);;
		}
	}
}
double motor_get_angular_speed(){
	return angular_speed_act;
}
double motor_get_angle(){
	return angle_act;
}

void motor_set_angular_speed(double angular_speed){
	drive_nowTime = HAL_GetTick();
	if (drive_nowTime - drive_dTime >= PID_TIME){
		drive_pid_setpoint=angular_speed;

		drive_encoder_pulses = drive_encoder_counter();
		drive_encoder_pulses_prev = drive_encoder_pulses - drive_encoder_pulses_prev;

		angular_speed_act = ((1000.00 * (double) drive_encoder_pulses_prev) / ((double) (drive_nowTime - drive_dTime) * encoder_resolution)) / drive_gear_ratio;

		//speed_act = ((double) encoder_pulses / encoder_resolution) / drive_gear_ratio;

		drive_pid_input = angular_speed_act;// * 50.0;

		drive_PID.Compute();

		if (drive_pid_input != 0.0)
			drive_pwm_duty += drive_pid_output;
		else
			drive_pwm_duty = 0;
/*
		if ((drive_pid_setpoint > 0) and (drive_pwm_duty < min_aar)){
			drive_pwm_duty = min_aar;
		}
		else if ((drive_pid_setpoint < 0) and (drive_pwm_duty > -min_aar)){
			drive_pwm_duty = -min_aar;
		}*/

		if(drive_pwm_duty > aar){
			drive_pwm_duty = aar;
		}
		else if(drive_pwm_duty < -aar){
			drive_pwm_duty = -aar;
		}

		if(drive_pid_setpoint == 0.0){
			//drive_pwm_duty=0;
			motor_set_pwm(drive_pwm_duty, 1);
		}
		else{
			motor_set_pwm((int16_t)drive_pwm_duty, 1);

		}
		drive_encoder_pulses_prev = drive_encoder_counter();

		drive_dTime = HAL_GetTick();
	}
}

void motor_set_angle(double angle){
	steer_nowTime = HAL_GetTick();
	if (steer_nowTime - steer_dTime >= PID_TIME){
		steer_pid_setpoint=angle * (180.0 / M_PI);

		steer_encoder_pulses = steer_encoder_counter();

	    angle_act = (((double) steer_encoder_pulses / encoder_resolution) * M_PI * 2.0) / steer_gear_ratio;

	    steer_pid_input = angle_act * (180.0 / M_PI);

	    steer_PID.Compute();

	    if (abs(steer_pid_setpoint - steer_pid_input) != 0.0)
	    	steer_pwm_duty += steer_pid_output;
		else
			steer_pwm_duty = 0;

/*
	    if ((steer_pid_setpoint > 0) and (steer_pwm_duty < min_aar)){
	    	steer_pwm_duty = min_aar;
		}
	    else if ((steer_pid_setpoint < 0) and (steer_pwm_duty > -min_aar)){
	    	steer_pwm_duty = -min_aar;
		}*/


		if(steer_pwm_duty > aar){
			steer_pwm_duty = aar;
		}
		else if(steer_pwm_duty < -aar){
			steer_pwm_duty = -aar;
		}

		if(steer_pid_setpoint == 0.0){
			//steer_pwm_duty=0;
			motor_set_pwm(steer_pwm_duty, 2);
		}
		else{
			motor_set_pwm((int16_t)steer_pwm_duty, 2);

		}
		steer_encoder_pulses_prev = steer_encoder_counter();

		steer_dTime = HAL_GetTick();
	}
}
