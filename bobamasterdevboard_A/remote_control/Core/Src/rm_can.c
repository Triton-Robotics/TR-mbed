/*
 * rm_can.c
 *
 *  Created on: Feb 8, 2020
 *      Author: WYT
 *
 *  Sources:
 *  https://github.com/RoboMaster/Development-Board-C-Examples/tree/master/14.CAN
 */

#include "rm_can.h"

motor_measure_t motors[NUM_MOTORS];

void can_filter_init(void)
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){}
	HAL_CAN_Start(&hcan1);

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){}
	HAL_CAN_Start(&hcan2);
}

void can_transmit(CAN_HandleTypeDef* hcan, uint16_t id, int16_t msg1, int16_t msg2, int16_t msg3, int16_t msg4) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             data[8];
    uint32_t            pTxMailbox;

    tx_header.StdId = id;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 0x08;
    tx_header.TransmitGlobalTime = DISABLE;
    data[0] = msg1 >> 8; 	//Higher 8 bits of ESC 1
    data[1] = msg1;		//Lower 8 bits of ESC 1
    data[2] = msg2 >> 8;
    data[3] = msg2;
    data[4] = msg3 >> 8;
    data[5] = msg3;
    data[6] = msg4 >> 8;
    data[7] = msg4;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &pTxMailbox) != HAL_OK){}
        //bsp_error_handler(__FUNCTION__, __LINE__, "can transmit fail");
    else
        while (HAL_CAN_IsTxMessagePending(hcan, pTxMailbox));
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[CAN_DATA_SIZE];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    uint8_t idx = rx_header.StdId - CAN_RX_ID_START;
    get_motor_measure(&motors[idx], rx_data);

}

void get_motor_measure(motor_measure_t* m, uint8_t data[CAN_DATA_SIZE]){
    (m)->last_ecd = (m)->ecd;
    (m)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    (m)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
    (m)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
    (m)->temperature = (data)[6];
}


