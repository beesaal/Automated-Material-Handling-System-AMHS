#include "robot_servos.h"


#include "FreeRTOS.h"  // Correct way to include FreeRTOS header
#include "task.h"      // Include task header for task-related functions


int motor_1 = 0;
int motor_2 = 0;
int motor_3 = 0;
int motor_4 = 0;

uint32_t pulse_value;


TaskHandle_t pickItem_TaskHandle = NULL;
TaskHandle_t dropItem_TaskHandle = NULL;


//Function to covert angle(degree) into respective pwm value.
uint16_t angle_to_pwm(uint8_t target_angle) {
	if (target_angle < MIN_ANGLE || target_angle > MAX_ANGLE) {
		printf("Error!!!\r\nTarget angle out of range.\r\n");
		return 0;
	}

	// Calculate the new PWM value based on the target angle
	int pwm_value = (int) (((float) (PWM_MAX - PWM_MIN)
			/ (MAX_ANGLE - MIN_ANGLE)) * target_angle + PWM_MIN);

	// Check if the calculated value is within limits
	if (pwm_value < PWM_MIN || pwm_value > PWM_MAX) {
		printf("Error!!!\r\nPWM value out of range.\r\n");
		return 0;
	}


	// Debugging info
	printf("Setting PWM for angle %d to %d\r\n", target_angle, pwm_value);

	return pwm_value;
}

//Function to generate angle for each servos to rotate to move the whole arm to corresponding co-ordinates.
void servos_movement_cal(float horizontal_degree, float horizontal_length, float vertical_length) {
	float target_length = sqrt(
			(vertical_length * vertical_length)
					+ (horizontal_length * horizontal_length));

	// Motor 3 angle calculation
	motor_3 = (int) (acos(
			(L1 * L1 + L2 * L2 - target_length * target_length) / (2 * L1 * L2))
			* (180.0 / M_PI));

	// Motor 4 angle calculation
	motor_4 = (int) acos(
			(L2 * L2 + L3 * L3 - target_length * target_length) / (2 * L2 * L3))
			* (180.0 / M_PI);

	// Motor 2 angle calculation
	motor_2 = (int) acos(
			(L1 * L1 + target_length * target_length - L2 * L2)
					/ (2 * L1 * target_length)) * (180.0 / M_PI);

	//Motor 1 angle calculation
	motor_1 = (int) horizontal_degree;

}



void motor_run(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t startPulse){
	uint32_t steps = 50;
	float increment = startPulse / (float) steps;
	for (uint32_t i = 0; i <= steps; i++){
		pulse_value = startPulse + (uint32_t) (increment * i);
		printf("New PWM value = %ld\r\n", pulse_value);
		__HAL_TIM_SET_COMPARE(htim, channel, pulse_value);
		HAL_Delay(20);
	}
}

void pickItem(void *argument){

	uint16_t motor5_pwm = 0;
	uint16_t motor6_pwm = 0;

	//inittializze 0
    motor_run(&htim4, TIM_CHANNEL_1, 200);
    motor_run(&htim4, TIM_CHANNEL_2, 200);

	//Make Grip Horizontal (Default vertical position!)
	motor5_pwm = angle_to_pwm(90);		
    motor_run(&htim4, TIM_CHANNEL_1, motor5_pwm);
	HAL_Delay(1000);

	//Extend the Grip
	motor6_pwm = angle_to_pwm(25);						
	motor_run(&htim4, TIM_CHANNEL_1, motor6_pwm);
	HAL_Delay(1000);


	//Wait till arm reach it's position
	vTaskDelay(5000);


	//Close the Grip
	motor6_pwm = angle_to_pwm(0);						
	motor_run(&htim4, TIM_CHANNEL_1, motor6_pwm);
	HAL_Delay(1000);

	KillTask(&pickItem_TaskHandle);

}

void dropItem(void *argument){

	uint16_t motor5_pwm = 0;
	uint16_t motor6_pwm = 0;

	//Make Grip Horizontal (Default vertical position!)
	motor5_pwm = angle_to_pwm(90);		
    motor_run(&htim4, TIM_CHANNEL_1, motor5_pwm);
	HAL_Delay(1000);

	//closed  Grip variable value
	motor6_pwm = angle_to_pwm(0);						
	motor_run(&htim4, TIM_CHANNEL_1, motor6_pwm);
	HAL_Delay(1000);


	//Wait till arm reach it's position
	vTaskDelay(5000);


	//Extend the Grip
	motor6_pwm = angle_to_pwm(30);						
	motor_run(&htim4, TIM_CHANNEL_1, motor6_pwm);
	HAL_Delay(1000);

	KillTask(&dropItem_TaskHandle);

}

void robot_PickItem(float x_axis, float y_axis, float z_axis){
	// Call the function to calculate angles
    servos_movement_cal(x_axis, y_axis, z_axis);


    // Set the PWM value to move the servo motor
    uint16_t motor1Pwm = angle_to_pwm(motor_1);
    motor_run(&htim3, TIM_CHANNEL_1, motor1Pwm);
	HAL_Delay(1000);

	TaskAdd(pickItem, "Task03", 512, NULL, 8, &pickItem_TaskHandle);
	vTaskDelay(1000);


    uint16_t motor2_pwm = angle_to_pwm(motor_2);
    motor_run(&htim3, TIM_CHANNEL_2, motor2_pwm);
	HAL_Delay(1000);

	uint16_t motor3_pwm = angle_to_pwm(motor_3);
	motor_run(&htim3, TIM_CHANNEL_3, motor3_pwm);
	HAL_Delay(1000);

	uint16_t motor4_pwm = angle_to_pwm(motor_4);
	motor_run(&htim3, TIM_CHANNEL_4, motor4_pwm);
	HAL_Delay(1000);

}


void robot_DropItem(float x_axis, float y_axis, float z_axis){
	// Call the function to calculate angles
    servos_movement_cal(x_axis, y_axis, z_axis);


    // Set the PWM value to move the servo motor
    uint16_t motor1Pwm = angle_to_pwm(motor_1);
    motor_run(&htim3, TIM_CHANNEL_1, motor1Pwm);
	HAL_Delay(1000);

	TaskAdd(dropItem, "Task04", 512, NULL, 9, &dropItem_TaskHandle);
	vTaskDelay(1000);


    uint16_t motor2_pwm = angle_to_pwm(motor_2);
    motor_run(&htim3, TIM_CHANNEL_2, motor2_pwm);
	HAL_Delay(1000);

	uint16_t motor3_pwm = angle_to_pwm(motor_3);
	motor_run(&htim3, TIM_CHANNEL_3, motor3_pwm);
	HAL_Delay(1000);

	uint16_t motor4_pwm = angle_to_pwm(motor_4);
	motor_run(&htim3, TIM_CHANNEL_4, motor4_pwm);
}