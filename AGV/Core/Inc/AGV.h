#ifndef __AGV_H
#define __AGV_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>

#define LINE_DETECTED GPIO_PIN_SET  // Adjust based on your line sensor logic
#define OBSTACLE_THRESHOLD 5        // 5 cm threshold for obstacle
#define START_DELAY_MS 5000         // 5-second delay for FSR

// Define pins 
#define FSR_Pin GPIO_PIN_3
#define FSR_Pin_GPIO_Port GPIOA

#define MOTOR1_Pin GPIO_PIN_6
#define MOTOR1_GPIO_Port GPIOA
#define MOTOR2_Pin GPIO_PIN_0
#define MOTOR2_GPIO_Port GPIOB
#define MOTOR3_Pin GPIO_PIN_8
#define MOTOR3_GPIO_Port GPIOA
#define MOTOR4_Pin GPIO_PIN_7
#define MOTOR4_GPIO_Port GPIOB

#define BUZZER_Pin GPIO_PIN_1
#define BUZZER_Pin_GPIO_Port GPIOB

#define LEFT_IR_Pin GPIO_PIN_4
#define LEFT_IR_Pin_GPIO_Port GPIOB
#define RIGHT_IR_Pin GPIO_PIN_12
#define RIGHT_IR_Pin_GPIO_Port GPIOA
#define IR_Read3_Pin GPIO_PIN_11
#define IR_Read3_GPIO_Port GPIOA

#define ULTRASONIC_TRIG_Pin GPIO_PIN_5
#define ULTRASONIC_TRIG_Pin_GPIO_Port GPIOB
#define ULTRASONIC_ECHO_Pin GPIO_PIN_6
#define ULTRASONIC_ECHO_Pin_GPIO_Port GPIOB


// Global variables
 extern uint8_t car_moving;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Obstacle_CheckAndHandle();
void LineFollowing();
void StopAtMarkerAndPause();
uint16_t Ultrasonic_GetDistance();

// Motor control functions (implement as per your motor driver)
void Motor_Stop();
void Motor_MoveForward();
void Motor_TurnLeft();
void Motor_TurnRight();




#endif