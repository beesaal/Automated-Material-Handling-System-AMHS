#ifndef __ROBOT_SERVOS_H
#define __ROBOT_SERVOS_H


#include "stm32f4xx_hal.h" 
#include <math.h>
#include "stdio.h"

// Constant for the servos (to generate pwm from provided angle)
#define MIN_ANGLE	0
#define MAX_ANGLE	180
#define PWM_MIN		200
#define PWM_MAX 	1150

// Segment lengths in cm (For whole arm movement)
#define L1 12.0  // Length from Motor 2 to Motor 3
#define L2 15.0  // Length from Motor 3 to Motor 4
#define L3 13.0  // Length from Motor 4 to Gripper

// Global variables to store calculated angles
static int motor_1, motor_2, motor_3, motor_4;
static TIM_HandleTypeDef *htim_servo;
static uint16_t tim_channel;



// Function to map an angle to PWM and control the servo motor
void pwm_init(TIM_HandleTypeDef *htim, uint32_t channel);				 										// Initialization function for PWM
uint16_t angle_to_pwm(uint8_t target_angle);                     										// Set servo angle function
void servos_movement_cal(float horizontal_degree, float horizontal_length, float vertical_length);	//Calculate angle of all servos to move whole arm
void robot_run(float x, float y, float z);




#endif // ROBOT_SERVOS_H