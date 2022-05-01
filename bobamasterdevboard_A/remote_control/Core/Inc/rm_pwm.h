/*
 * rm_pwm.h
 *
 *  Created on: Feb 12, 2020
 *      Author: WYT
 */

#ifndef INC_RM_PWM_H_
#define INC_RM_PWM_H_

#include "main.h"

extern void pwm_imu_start();
extern void pwm_buzzer_start();
extern void set_pwm_imu(unsigned short int pwm);
extern void set_pwm_buzzer(unsigned short int pwm);
void pwm_flywheel_start();
void set_pwm_flywheel(unsigned short int pwm);

#endif /* INC_RM_PWM_H_ */
