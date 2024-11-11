#ifndef __AGV_H
#define __AGV_H

#include "main.h"
#include <stdbool.h>
#include <stdio.h>

//Ultrasonic Settings
#define LINE_DETECTED 	1  // Adjust based on your line sensor logic
#define OBSTACLE_THRESHOLD 5        // 5 cm threshold for obstacle
#define START_DELAY_MS 5000         // 5-second delay for FSR

// Motor speed control (PWM) for smooth start/stop  //Do not change
#define MAX_SPEED 9999								// max pwm power
#define MIN_SPEED 9999								//least pwm power required to rotate the motor
#define SPEED_STEP 10

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;

extern uint16_t current_speed;

// Global variables
extern uint8_t car_moving;

void Obstacle_CheckAndHandle();
void LineFollowing();
uint16_t Ultrasonic_GetDistance();
void SmoothStart();
void SmoothStop();

// Motor control functions (implement as per your motor driver)
void Motor_Stop();
void Motor_MoveForward();
void Motor_MoveBackward();
void Motor_TurnLeft();
void Motor_TurnRight();
void SetMotorSpeed(uint16_t speed);

#endif
