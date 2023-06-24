/*
 * can.cpp
 *
 *  Created on: 28 May 2023
 *      Author: merth
 */

#include <can.h>
#include <cstring>


//extern UART_HandleTypeDef huart1;

uint8_t uartData[8]={0};

uint8_t canData[8]={0};


CAN_TxHeaderTypeDef myTxHeader;
CAN_RxHeaderTypeDef myRxHeader;
uint8_t rxData[8] = { 0 };
uint8_t rxData_1[8] = {0};
uint8_t rxData_2[8] = {0};
int32_t data1, data2;

union byte_to_float {
	float f;
	unsigned char bytes[4];
};


static int32_t nowTime_1=0, dTime_1=0;
uint16_t recive_numbers = 0, send_numbers = 0;


void CAN_Fonk_Init() {

	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 10;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	// değişken 16, id 11 bit olduğu için kaydırıyoruz.
	canfilterconfig.FilterIdHigh = 0x0000; //OTHERSTDID << 5
	canfilterconfig.FilterIdLow = 0x0000; // for ext id //0x0000
	canfilterconfig.FilterMaskIdHigh = 0x0000;//0xFFFF << 5
	canfilterconfig.FilterMaskIdLow = 0x0000; // for ext id

	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 10;


	if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}

	//CAN hata durumunda bir kere interrupt girmesi için CAN_IT_ERROR_WARNING
	//hata boyunca interrupta kalması için CAN_IT_LAST_ERROR_CODE
	if (HAL_CAN_ActivateNotification(&hcan,
			CAN_IT_ERROR | CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}

	myTxHeader.DLC = 8; // max byte
	myTxHeader.StdId = MYSTDID;
	myTxHeader.RTR = CAN_RTR_DATA;
	myTxHeader.IDE = CAN_ID_STD;

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {


	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &myRxHeader, canData)
			!= HAL_OK) {
		Error_Handler();
	}
	if(myRxHeader.StdId == 0x16){
		memcpy(uartData,canData,8);
		//HAL_UART_Transmit(&huart1, uartData, 8, 50);
	}
	else{
		memcpy(rxData,canData,8);
	}
	nowTime_1 = HAL_GetTick();

	recive_numbers = recive_numbers + 1;
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

}

uint16_t get_can_numbers(short data){
	if (data == 1)
		return recive_numbers;
	else if (data == 2)
		return send_numbers;
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {

	//hata logları

}

void CAN_Receive_motors(float *p, float *p2){
	dTime_1 = HAL_GetTick();
	if(dTime_1 - nowTime_1 > CAN_DEADTIME){
		*p =0;
		*p2=0;
	}
	else{
		memcpy(p, rxData,4);
		memcpy(p2, &rxData[4],4);
	}
}

void CAN_Transmit_Datas(struct MOTOR motorx) {
	uint32_t txmailbox;
	uint8_t txdata[8];


	txdata[0] = 0xFF;//motorx.temp;
	txdata[1] = 0xFF;//motorx.voltage;
	txdata[2] = 0xF0;//motorx.current;
	myTxHeader.DLC = 3;
	HAL_StatusTypeDef a;
	a = HAL_CAN_AddTxMessage(&hcan, &myTxHeader, txdata, &txmailbox);
	if (a == HAL_ERROR) send_numbers = send_numbers - 1;
	else if (a == HAL_OK) send_numbers = send_numbers + 1;

}
