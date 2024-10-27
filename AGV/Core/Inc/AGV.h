#ifndef __AGV_H
#define __AGV_H

#include "main.h"
#include <stdbool.h>

//Ultrasonic Settings
#define LINE_DETECTED GPIO_PIN_SET  // Adjust based on your line sensor logic
#define OBSTACLE_THRESHOLD 5        // 5 cm threshold for obstacle
#define START_DELAY_MS 5000         // 5-second delay for FSR

// Motor speed control (PWM) for smooth start/stop
#define MAX_SPEED 100
#define MIN_SPEED 0
#define SPEED_STEP 10


extern uint8_t current_speed;

// Global variables
 extern uint8_t car_moving;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Obstacle_CheckAndHandle();
void LineFollowing();
void StopAtMarkerAndPause();
void SmoothStart();
void SmoothStop();
uint16_t Ultrasonic_GetDistance();

// Motor control functions (implement as per your motor driver)
void Motor_Stop();
void Motor_MoveForward();
void Motor_TurnLeft();
void Motor_TurnRight();
void SetMotorSpeed(uint8_t speed);




#endif