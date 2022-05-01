/*
 * rm_ctrl.c
 *
 *  Created on: Feb 18, 2020
 *      Author: Yitian Wang
 */

#include "stdlib.h"
#include "math.h"
#include "rm_ctrl.h"
#include "rm_pwm.h"
#include "rm_can.h"
#include "rm_imu.h"

const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
pid_type_def imu_temp_pid;

const float wheels_rpm_PID[3] = {5, 0, 1};
pid_type_def wheels_rpm_pid[4];

const float yaw_deg_imu_PID[3] = {20, 0.1, 3};
pid_type_def yaw_deg_imu_pid;
const float yaw_rpm_imu_PID[3] = {50, 0.1, 0};
pid_type_def yaw_rpm_imu_pid;

const float pit_deg_imu_PID[3] = {20, 0.1, 3};
pid_type_def pit_deg_imu_pid;
const float pit_rpm_imu_PID[3] = {50, 0.1, 0};
pid_type_def pit_rpm_imu_pid;

const float yaw_ecd_rpm_PID[3] = {1, 0, 0};
pid_type_def yaw_ecd_rpm_pid;
const float yaw_rpm_cur_PID[3] = {100, 0, 0};
pid_type_def yaw_rpm_cur_pid;

const float pit_ecd_rpm_PID[3] = {1, 0, 0};
pid_type_def pit_ecd_rpm_pid;
const float pit_rpm_cur_PID[3] = {100, 0, 0};
pid_type_def pit_rpm_cur_pid;

const float yaw_ecd_cur_PID[3] = {0.5 * 350, 0, 0.5 * 350 * 0.3}; //Oscillation period 0.3 seconds
pid_type_def yaw_ecd_cur_pid;

const float pit_ecd_cur_PID[3] = {20, 0, 0};
pid_type_def pit_ecd_cur_pid;

const float idx_rpm_cur_PID[3] = {10, 0.1, 0}; // indexer
pid_type_def idx_rpm_cur_pid;

const float chassis_follow_PID[3] = {150, 0.01, 0}; // chassis follow
pid_type_def chassis_follow_pid;

void grand_pid_init(){
	PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

	for (int i = 0; i < 4; i++){
		PID_init(&wheels_rpm_pid[i], PID_POSITION, wheels_rpm_PID, 16384, 1000); // M3508 output limit: 16384, rpm limit 450
	}
	PID_init(&chassis_follow_pid, PID_POSITION, chassis_follow_PID, 16384, 1000);

	PID_init(&yaw_deg_imu_pid, PID_POSITION, yaw_deg_imu_PID, 3200, 100);
	PID_init(&yaw_rpm_imu_pid, PID_POSITION, yaw_rpm_imu_PID, 30000, 5000); // GM6020 output limit: 30000

	PID_init(&yaw_ecd_rpm_pid, PID_POSITION, yaw_ecd_rpm_PID, 3200, 100); // GM6020 max rpm: 320
	PID_init(&yaw_rpm_cur_pid, PID_POSITION, yaw_rpm_cur_PID, 30000, 5000);

	PID_init(&pit_deg_imu_pid, PID_POSITION, pit_deg_imu_PID, 3200, 100);
	PID_init(&pit_rpm_imu_pid, PID_POSITION, pit_rpm_imu_PID, 30000, 5000);

	PID_init(&pit_ecd_rpm_pid, PID_POSITION, pit_ecd_rpm_PID, 3200, 100);
	PID_init(&pit_rpm_cur_pid, PID_POSITION, pit_rpm_cur_PID, 30000, 5000);

	PID_init(&yaw_ecd_cur_pid, PID_POSITION, yaw_ecd_cur_PID, 30000, 5000);
	PID_init(&pit_ecd_cur_pid, PID_POSITION, pit_ecd_cur_PID, 30000, 5000);

	PID_init(&idx_rpm_cur_pid, PID_POSITION, idx_rpm_cur_PID, 10000, 100); // GM2006 output limit: 10000, max rpm 500


}

void gimbal_pid_clear(){
	PID_clear(&yaw_ecd_rpm_pid);
	PID_clear(&pit_ecd_rpm_pid);
	PID_clear(&yaw_rpm_cur_pid);
	PID_clear(&pit_rpm_cur_pid);
	PID_clear(&yaw_ecd_cur_pid);
	PID_clear(&pit_ecd_cur_pid);
	PID_clear(&chassis_follow_pid);
}

void imu_calibration(){
	  mpu_data.gx_offset=0;
	  mpu_data.gy_offset=0;
	  mpu_data.gz_offset=0;
	  long gx_off_int = 0;
	  long gy_off_int = 0;
	  long gz_off_int = 0;

	  mpu_data.ax_offset=0;
	  mpu_data.ay_offset=0;
	  mpu_data.az_offset=0;
	  long ax_off_int = 0;
	  long ay_off_int = 0;
	  long az_off_int = 0;

	  int offset_counter = 0;
	  int wait_for_stable = 500;
	  set_pwm_buzzer(2000);
	  HAL_Delay(100);
	  set_pwm_buzzer(0);
	  while (offset_counter < 500)
	  {
		  mpu_get_data();
		  if (imu.temp < 48.0f){
			  set_pwm_imu((unsigned short int)TEMPERATURE_PID_MAX_OUT);
		  } else {
			  imu_temp_pid_ctrl(imu.temp, 50.0f);
		  }
		  /* Wait until the temperature is stable */
		  if (abs(imu.temp - 50.0f) < 1.0f)
		  {
			  if (wait_for_stable == 0){
				  ax_off_int += mpu_data.ax;
				  ay_off_int += mpu_data.ay;
				  az_off_int += mpu_data.az;
				  gx_off_int += mpu_data.gx;
				  gy_off_int += mpu_data.gy;
				  gz_off_int += mpu_data.gz;
				  offset_counter++;
			  } else {
				  wait_for_stable--;
			  }
		  }
		  HAL_Delay(5);
	  }
	  mpu_data.ax_offset= ax_off_int / 500;
	  mpu_data.ay_offset= ay_off_int / 500;
	  mpu_data.az_offset= az_off_int / 500 - 4096; // gravity
	  mpu_data.gx_offset= gx_off_int / 500;
	  mpu_data.gy_offset= gy_off_int / 500;
	  mpu_data.gz_offset= gz_off_int / 500;
	  set_pwm_buzzer(2000);
	  HAL_Delay(100);
	  set_pwm_buzzer(0);

	  for(int i = 0; i < 400; i++){
		  mpu_get_data();
		  imu_ahrs_update();
		  imu_attitude_update();
		  HAL_Delay(5);
	  }
}

void imu_temp_pid_ctrl(float feedback, float target){
	PID_calc(&imu_temp_pid, feedback, target);
	if (imu_temp_pid.out < 0.0f)
	{
		imu_temp_pid.out = 0.0f;
	}
	set_pwm_imu((unsigned short int) imu_temp_pid.out);
}

float yaw_imu_pid_ctrl(float feedback, float target){
	float error = get_rotation_actual_error(feedback, target, ANGLE_PERIOD);
	if (abs(error) < 1.0f){
		error = 0;
	}
	PID_calc(&yaw_deg_imu_pid, 0, error);
	PID_calc(&yaw_rpm_imu_pid, motors[4].speed_rpm, yaw_deg_imu_pid.out);
#ifdef BOARD_DOWN
	return yaw_rpm_imu_pid.out;
#else
	return -yaw_rpm_imu_pid.out;
#endif
}

float pit_imu_pid_ctrl(float feedback, float target){
	float error = get_rotation_actual_error(feedback, target, ANGLE_PERIOD);
	PID_calc(&pit_deg_imu_pid, 0, error);
	PID_calc(&pit_rpm_imu_pid, motors[5].speed_rpm, pit_deg_imu_pid.out);
#ifdef BOARD_DOWN
	return pit_rpm_imu_pid.out;
#else
	return -pit_rpm_imu_pid.out;
#endif
}


float yaw_ecd_cascade_ctrl(float target){

	float feedback = motors[4].ecd;
	float error = get_rotation_actual_error(feedback, target, ECD_PERIOD);
	PID_calc(&yaw_ecd_rpm_pid, 0, error);
	PID_calc(&yaw_rpm_cur_pid, motors[4].speed_rpm, yaw_ecd_rpm_pid.out);
	return yaw_rpm_cur_pid.out;
}


float pit_ecd_cascade_ctrl(float target){

	float feedback = motors[5].ecd;
	float error = get_rotation_actual_error(feedback, target, ECD_PERIOD);
	PID_calc(&pit_ecd_rpm_pid, 0, error);
	PID_calc(&pit_rpm_cur_pid, motors[5].speed_rpm, pit_ecd_rpm_pid.out);
	return pit_rpm_cur_pid.out;
}

float yaw_ecd_direct_ctrl(float target){
	float feedback = motors[4].ecd;
	float error = get_rotation_actual_error(feedback, target, ECD_PERIOD);
	PID_calc(&yaw_ecd_cur_pid, 0, error);
	return yaw_ecd_cur_pid.out;
}


float pit_ecd_direct_ctrl(float target){
	float feedback = motors[5].ecd;
	float error = get_rotation_actual_error(feedback, target, ECD_PERIOD);
	PID_calc(&pit_ecd_cur_pid, 0, error);
	return pit_ecd_cur_pid.out;
}



float yaw_rpm_pid_ctrl(float target){
	PID_calc(&yaw_rpm_cur_pid, motors[4].speed_rpm, target);
	return yaw_rpm_cur_pid.out;
}

float pit_rpm_pid_ctrl(float target){
	PID_calc(&pit_rpm_cur_pid, motors[5].speed_rpm, target);
	return pit_rpm_cur_pid.out;
}

float chassis_follow_ctrl(float target, float deadzone) {
	deadzone = deadzone > 0 ? deadzone : -deadzone;
	float feedback = motors[4].ecd;
	float error = get_rotation_actual_error(feedback, target, ECD_PERIOD);
	PID_calc(&chassis_follow_pid, 0, error);
	return ((chassis_follow_pid.out < deadzone) && (chassis_follow_pid.out > -deadzone)) ? 0 : chassis_follow_pid.out;
}

void wheels_rpm_ctrl_calc(float LF_speed, float RF_speed, float LB_speed, float RB_speed, float output[]){
	PID_calc(&wheels_rpm_pid[0], motors[0].speed_rpm, -LF_speed);
	PID_calc(&wheels_rpm_pid[1], motors[1].speed_rpm, RF_speed);
	PID_calc(&wheels_rpm_pid[2], motors[2].speed_rpm, -LB_speed);
	PID_calc(&wheels_rpm_pid[3], motors[3].speed_rpm, RB_speed);
	output[0] = wheels_rpm_pid[0].out;
	output[1] = wheels_rpm_pid[1].out;
	output[2] = wheels_rpm_pid[2].out;
	output[3] = wheels_rpm_pid[3].out;
}

float indexer_rpm_ctrl_calc(float target) {
	PID_calc(&idx_rpm_cur_pid, motors[6].speed_rpm, target);
	return idx_rpm_cur_pid.out;
}
