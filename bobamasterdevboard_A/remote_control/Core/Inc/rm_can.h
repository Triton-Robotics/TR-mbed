/*
 * rm_can.h
 *
 *  Created on: Feb 8, 2020
 *      Author: WYT
 *
 *  Sources:
 *  https://github.com/RoboMaster/Development-Board-C-Examples/tree/master/14.CAN
 */

#ifndef INC_RM_CAN_H_
#define INC_RM_CAN_H_

#include "main.h"

#define NUM_MOTORS		7
#define CAN_DATA_SIZE       8
#define CAN_RX_ID_START    0x201

typedef struct
{
    uint16_t ecd; // Encoder value from 0 - 8191 (0 - 360)
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    int16_t last_ecd;
    int16_t total_ecd;
} motor_measure_t;


typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,
}CAN_Message_ID;

extern motor_measure_t motors[NUM_MOTORS];

void get_motor_measure(motor_measure_t* m, uint8_t data[CAN_DATA_SIZE]);
void can_filter_init(void);
void can_transmit(CAN_HandleTypeDef* hcan, uint16_t id, int16_t msg1, int16_t msg2, int16_t msg3, int16_t msg4);

#endif /* INC_RM_CAN_H_ */
