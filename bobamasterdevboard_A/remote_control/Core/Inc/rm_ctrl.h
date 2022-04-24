/*
 * rm_ctrl.h
 *
 *  Created on: Feb 18, 2020
 *      Author: Yitian Wang
 */

#ifndef INC_RM_CTRL_H_
#define INC_RM_CTRL_H_

#include "main.h"
#include "pid.h"



#define TEMPERATURE_PID_KP 1600.0f //kp of temperature control PID
#define TEMPERATURE_PID_KI 0.1f    //ki of temperature control PID
#define TEMPERATURE_PID_KD 0.0f    //kd of temperature control PID

#define TEMPERATURE_PID_MAX_OUT 4500.0f  //max out of temperature control PID
#define TEMPERATURE_PID_MAX_IOUT 4400.0f //max iout of temperature control PID


extern void grand_pid_init();

extern void imu_calibration();

extern void imu_temp_pid_ctrl(float feedback, float target);

extern float yaw_imu_pid_ctrl(float feedback, float target);

extern float pit_imu_pid_ctrl(float feedback, float target);

extern float yaw_ecd_cascade_ctrl(float target);

extern float pit_ecd_cascade_ctrl(float target);

extern float yaw_rpm_pid_ctrl(float target);

extern float pit_rpm_pid_ctrl(float target);

extern float yaw_ecd_direct_ctrl(float target);

extern float pit_ecd_direct_ctrl(float target);

extern void gimbal_pid_clear();

extern void wheels_rpm_ctrl_calc(float LF_speed, float RF_speed, float LB_speed, float RB_speed, float output[]);

extern float chassis_follow_ctrl(float target, float deadzone);

extern float indexer_rpm_ctrl_calc(float target);

#endif /* INC_RM_CTRL_H_ */
