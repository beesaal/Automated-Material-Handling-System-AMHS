/**
 * Basically agv/car will rotate in a loop (line) where there are mainly two ends one where the car gets loaded
 * with boxes and another where it will unload the boxes. on both the end there will be line on middle of two ir sensor and 
 * infron of the two ir sensor, just to make it stop. when it stop at those points whether to load up if it's not loaded
 * or to unload the boxes FSR sensor will signal the car to move forward blindly for like 2 sec and then the line
 * following algorithm agains kicks in and drive the car to another stop.
 * 1. agv will wait for the arm to load up the box.
 * 2. FSR will detect the box whether it's mounted or not
 * 3. if the FSR is one (meaning the load is mounted) then the car began to move after 2 second time delay.
 * 4. The car will follow the path until there is no obstacle infront of it 
 *      (ultrasonic sensor keeps continuosly detecting for the object).
 * 5. if the ultrasonic sensor detects any object, the car will stop and sound a buzzer.
 * 6. ultrasonic sensor is still detecting whether the object has been removed from the path or not. 
 *      if removed then will send signal as (car_moving = 0 ) when car_moving is zero then car begans to move lets say
 *      after 2 sec.
 * 7. the car will stop when it doesn't see any path :
 *      meaning:
 *              left side closed 
 *              right side closed and 
 *              front side closed too.
 * 8. car will wait for there till the box has been uloaded:
 FSR = 0;
 9. When FSR = 0 the car will move after like 2 second
 10. Then same loop continues.
 * **/
#include "AGV.h"

uint8_t car_moving = 0;
uint16_t current_speed = MIN_SPEED;

static uint8_t load = 0;

// Define AGV states for non-blocking obstacle and line-following handling
typedef enum {
	AGV_STATE_LINE_FOLLOWING,
	AGV_STATE_OBSTACLE_DETECTED,
	AGV_STATE_OBSTACLE_CLEAR,
	AGV_STATE_STOP_STATION
} AGV_State;

AGV_State current_state = AGV_STATE_LINE_FOLLOWING;

void Obstacle_CheckAndHandle() {
	uint16_t distance = 10;
    printf("Obstacle_Check\r\nObstacle distance: %d\r\n", distance);

	switch (current_state) {
	case AGV_STATE_LINE_FOLLOWING:
		// Check for obstacles only in line-following state
		if (distance < OBSTACLE_THRESHOLD) {
			printf("Obstacle detected, stopping AGV...\r\n");
			SmoothStop();
			HAL_GPIO_TogglePin(GPIOB, BUZZER_Pin);  // Turn buzzer on
			current_state = AGV_STATE_OBSTACLE_DETECTED;
		}
		break;

	case AGV_STATE_OBSTACLE_DETECTED:
		// Wait until obstacle clears without blocking
		if (distance >= OBSTACLE_THRESHOLD) {
			printf("Obstacle cleared, resuming AGV...\r\n");
			HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_RESET); // Turn buzzer off
			current_state = AGV_STATE_OBSTACLE_CLEAR;
		}
		break;

	case AGV_STATE_OBSTACLE_CLEAR:
		HAL_Delay(2000);  // Wait briefly before resuming
		SmoothStart();
		current_state = AGV_STATE_LINE_FOLLOWING; // Return to line-following mode
		break;

	case AGV_STATE_STOP_STATION:
		//wait for (2 sec) at the station, to be loaded
		SmoothStop();
		HAL_Delay(2000);  // Pause for 2 seconds (adjust as needed)

		if (load == 0) {
			if (HAL_GPIO_ReadPin(FSR_GPIO_Port, FSR_Pin) == GPIO_PIN_SET) { // Box loaded
				load = 1;
				SmoothStart();
				HAL_Delay(1500);
				current_state = AGV_STATE_LINE_FOLLOWING;
			} else {  // Box not loaded, wait to be loaded
				current_state = AGV_STATE_STOP_STATION;
			}

		} else {
			if (HAL_GPIO_ReadPin(FSR_GPIO_Port, FSR_Pin) == GPIO_PIN_RESET) { // Box unloaded
				load = 0;
				SmoothStart();
				HAL_Delay(1500);
				current_state = AGV_STATE_LINE_FOLLOWING;
			} else {  // Box loaded, wait to be unloaded
				current_state = AGV_STATE_STOP_STATION;
			}
		}
		break;

	default:
		printf("This is a default message.\r\n");
		current_state = AGV_STATE_LINE_FOLLOWING;
	}
}

void LineFollowing() {

    printf("LineFollowing...\r\n");
	// Only perform line-following if in LINE_FOLLOWING state
	if (current_state == AGV_STATE_LINE_FOLLOWING) {
		uint8_t left_sensor = 0;
		uint8_t right_sensor = 0;

		if (HAL_GPIO_ReadPin(LEFT_IR_GPIO_Port, LEFT_IR_Pin)) {
			left_sensor = LINE_DETECTED;
		}
		if (HAL_GPIO_ReadPin(RIGHT_IR_GPIO_Port, RIGHT_IR_Pin)) {
			right_sensor = LINE_DETECTED;
		}

		if (left_sensor == LINE_DETECTED && right_sensor != LINE_DETECTED) {
			printf("Turning Right.\r\n");
			Motor_TurnRight();
		} else if (left_sensor != LINE_DETECTED && right_sensor == LINE_DETECTED) {
			printf("Turning Left.\r\n");
			Motor_TurnLeft();
		} else if (left_sensor == LINE_DETECTED && right_sensor == LINE_DETECTED) {

			printf("Moving Forward.\r\n");
			Motor_MoveForward();
			current_state = AGV_STATE_LINE_FOLLOWING;
		} else {
			printf("Station_Stop.\r\n");
			current_state = AGV_STATE_STOP_STATION;
		}
	}
}

uint16_t Ultrasonic_GetDistance() {
	uint16_t Distance = 0;
	uint32_t pMillis = 0;
	uint32_t Value1 = 0;
	uint32_t Value2 = 0;

	printf("Starting Ultrasonic Distance Measurement...\r\n");

	// Trigger the ultrasonic sensor
	HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin,
			GPIO_PIN_SET);
	printf("TRIG pin set HIGH\r\n");

	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while (__HAL_TIM_GET_COUNTER(&htim6) < 10)
		;  // Wait for 10 us
	HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin,
			GPIO_PIN_RESET);
	printf("TRIG pin set LOW\r\n");

	// Record the start time to avoid infinite loop
	pMillis = HAL_GetTick();
	printf("Waiting for ECHO pin HIGH...\r\n");

	// Wait for the echo pin to go high, timeout extended to 20 ms
	while (!(HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port,
	ULTRASONIC_ECHO_Pin)) && (pMillis + 20 > HAL_GetTick()))
		;

	if (!(HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin))) {
		printf("Timeout waiting for ECHO pin HIGH. Measurement failed.\r\n");
		return 0;
	}
	Value1 = __HAL_TIM_GET_COUNTER(&htim6);
	printf("ECHO pin went HIGH. Value1 captured: %lu\r\n", Value1);

	// Record the start time again to avoid infinite loop
	pMillis = HAL_GetTick();
	printf("Waiting for ECHO pin LOW...\r\n");

	// Wait for the echo pin to go low, timeout extended to 100 ms
	while ((HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin))
			&& (pMillis + 100 > HAL_GetTick()))
		;

	if (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin)) {
		printf("Timeout waiting for ECHO pin LOW. Measurement failed.\r\n");
		return 0;
	}
	Value2 = __HAL_TIM_GET_COUNTER(&htim6);
	printf("ECHO pin went LOW. Value2 captured: %lu\r\n", Value2);

	// Calculate distance
	Distance = (Value2 - Value1) * 0.034 / 2;
	printf("Distance measured: %u cm\r\n", Distance);

	return Distance;
}

// Motor control functions

/**
 *  Input1   Input2  Spinning Direction

 Low(0)  Low(0)  Motor OFF
 High(1) Low(0)  Forward
 Low(0)  High(1) Backward
 High(1) High(1) Motor OFF


 **/

void Motor_Stop() {
    printf("Motor_Stop...\r\n");
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

	car_moving = 0;
}

void Motor_MoveForward() {
    printf("Motor_MoveForward...\r\n");
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

	car_moving = 1;
}

void Motor_MoveBackward() {
    printf("Motor_MoveBackward...\r\n");
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
}

void Motor_TurnLeft() {
    printf("Motor_TurnLeft...\r\n");
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

void Motor_TurnRight() {

    printf("Motor_TurnRight...\r\n");
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

// Smooth start by gradually increasing speed
void SmoothStart() {
    printf("Smooth Start...\r\n");
	for (current_speed = MIN_SPEED; current_speed <= MAX_SPEED; current_speed +=
			SPEED_STEP) {
		SetMotorSpeed(current_speed);
		HAL_Delay(1);  // Adjust delay for smoother/faster acceleration
	}
	Motor_MoveForward();
	car_moving = 1;
}

// Smooth stop by gradually decreasing speed
void SmoothStop() {
    printf("Smooth Stop...\r\n");
	for (current_speed = MAX_SPEED; current_speed >= MIN_SPEED; current_speed -=
			SPEED_STEP) {
		SetMotorSpeed(current_speed);
		HAL_Delay(1);  // Adjust delay for smoother/faster deceleration
	}
	Motor_Stop();
	car_moving = 0;
}
// Function to set motor speed using PWM
void SetMotorSpeed(uint16_t speed) {

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed); //speed for motor 1ENA
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed); //speed for motor 2ENB
}
