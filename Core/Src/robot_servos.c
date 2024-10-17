#include "robot_servos.h"

//Function to initialise timmers to generate pwm
void pwm_init(TIM_HandleTypeDef *htim, uint32_t channel) {

	htim_servo = htim;
	tim_channel = channel;


	HAL_TIM_PWM_Start(htim_servo, tim_channel);
}

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

void robot_run(float x, float y, float z){


    // Call the function to calculate angles
    servos_movement_cal(x, y, z);


    uint16_t motor1_pwm = angle_to_pwm(motor_1);
    uint16_t motor2_pwm = angle_to_pwm(motor_2);
    uint16_t motor3_pwm = angle_to_pwm(motor_3);
    uint16_t motor4_pwm = angle_to_pwm(motor_4);



	// Set the PWM value to move the servo motor
	__HAL_TIM_SET_COMPARE(htim_servo, tim_channel, motor1_pwm);
	HAL_Delay(1000);

	__HAL_TIM_SET_COMPARE(htim_servo, tim_channel, motor2_pwm);
	HAL_Delay(1000);

	__HAL_TIM_SET_COMPARE(htim_servo, tim_channel, motor3_pwm);
	HAL_Delay(1000);

	__HAL_TIM_SET_COMPARE(htim_servo, tim_channel, motor4_pwm);
	HAL_Delay(1000);

}
