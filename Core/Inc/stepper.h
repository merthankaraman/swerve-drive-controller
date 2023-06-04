/*
 * stepper.h
 *
 *  Created on: Mar 7, 2023
 *      Author: Merthan
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_
#include <stdint.h>
#include "as5600.h"
#include "main.h"

#ifdef __cplusplus
 extern "C" {
#endif

void stepper_Init();
void stepper_set_pwm(int16_t pwm);
int16_t angle_loop();


#ifdef __cplusplus
}
#endif

#endif /* INC_STEPPER_H_ */
