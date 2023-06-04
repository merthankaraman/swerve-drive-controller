/*
 * can.h
 *
 *  Created on: 28 May 2023
 *      Author: merth
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define CAN_DEADTIME 60000 //millisecond

#define MYSTDID 0x010
//10
#define OTHERSTDID 0x017
uint16_t get_can_numbers(short data);
extern CAN_HandleTypeDef hcan;

extern void Error_Handler(void);

struct MOTOR{
	int8_t temp;
	uint8_t voltage;
	uint8_t current;

};

void Steer_CAN_Init();

void Steer_CAN_Transmit(struct MOTOR motorx);

void Steer_CAN_Receive_motor(float *p, float *p2);

#ifdef __cplusplus
}
#endif

#endif /* INC_CAN_H_ */
