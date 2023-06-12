/*
 * motor.h
 *
 *  Created on: 12 Kas 2022
 *      Author: Merthan
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif


/*pwm 20khz timer 72mhz
16 prescale 4.5mhz 225aar
8  prescale 9mhz 450aar
4  prescale 18mhz 900aar
2  prescale 36mhz 1800aar
1  prescale 72mhz 3600aar
*/
void motor_Init();
//Init Motor settings

int32_t encoder_counter();
double ba(int a);
int32_t aaa();
//Encoder pins
//Enc+ A0
//Enc- A1

void motor_set_pwm(int16_t pwm);
//Max pwm 900
//Motor pins
//Enable A5
//Pwm+ A6
//PWM- A7

void motor_set_speed(double speed);

void motor_set_angle(double angle);

double motor_get_speed();

double motor_get_angle();

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_H_ */
