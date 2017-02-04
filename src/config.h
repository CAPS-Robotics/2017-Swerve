/*
 * config.h
 *
 *  Created on: Jan 25, 2017
 *      Author: robotics
 */

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

#define BL_CAN		0
#define FL_CAN		1
#define CLIMB2_CAN	2
#define BR_CAN		3
#define FR_CAN		4
#define CLIMB_CAN	5

#define FR_PWM	0
#define FL_PWM	1
#define BL_PWM	2
#define BR_PWM	3

// Joystick buttons
#define JOY_BTN_X					1
#define JOY_BTN_A					2
#define JOY_BTN_B					3
#define JOY_BTN_Y					4

#define JOY_BTN_LBM					5
#define JOY_BTN_RBM					6
#define JOY_BTN_LTG					7
#define JOY_BTN_RTG					8

#define JOY_SPC_BCK					9  // Back button
#define JOY_SPC_STR					10 // Start button
#define JOY_SPC_LST					11 // Push the left stick in
#define JOY_SPC_RST					12 // Push the right stick in

#define ANALOG_GYRO					0

#define PI		3.1415926536
#define GR		1/1.2

#endif /* SRC_CONFIG_H_ */
