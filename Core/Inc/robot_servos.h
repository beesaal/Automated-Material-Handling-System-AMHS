#ifndef __ROBOT_SERVOS_H
#define __ROBOT_SERVOS_H


#include "stm32f4xx_hal.h"  // For HAL functions
#include "FreeRTOS.h"       // For FreeRTOS support
#include "task.h"           // For task functions
#include <math.h>           // For math operations
#include <stdio.h>          // For printf
#include <stdint.h>         // Standard integer types
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

extern int motor_1;
extern int motor_2;
extern int motor_3;
extern int motor_4;


/** SMALL NOTE ON WHY TO USE EXTERN INSTEAD OF STATIC in case above.
 * Static Variables
Scope: When you declare a variable as static in a header file, it has internal linkage. This means it is only visible within the translation unit (source file) where it is defined. Other source files that include the header will have their own separate copies of the variable, and they cannot access each other's versions.
Use Case: This is useful when you want to limit the visibility of a variable to a single source file for encapsulation.
Extern Variables
Scope: Declaring a variable as extern allows it to be accessible across multiple source files. When you define the variable in one source file and declare it as extern in a header file, any other source file that includes this header can access the same variable.
Use Case: This is suitable when you need to share the same variable across different parts of your program (e.g., in multiple source files).
**/



extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


// Function to map an angle to PWM and control the servo motor

uint8_t TaskAdd(void (*FunctionName)(void *argument), char *TaskName,
                uint16_t StackSize, void *pvParameters,
                UBaseType_t uxPriority, TaskHandle_t *TaskHandle);

void KillTask(TaskHandle_t *TaskHandle);
void Robot_Arm_Movement(void *argument);
void ButtonPressed_Fn(void *argument);
void ClearScreen();


uint16_t angle_to_pwm(uint8_t target_angle);                     										// Set servo angle function
void servos_movement_cal(float horizontal_degree, float horizontal_length, float vertical_length);	//Calculate angle of all servos to move whole arm
void robot_PickItem(float x_axis, float y_axis, float z_axis);
void robot_DropItem(float x_axis, float y_axis, float z_axis);
void pickItem(void *argument);
void dropItem(void *argument);


#endif // ROBOT_SERVOS_H