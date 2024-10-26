#include "AGV.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == FSR_Pin) {
        HAL_Delay(START_DELAY_MS); // Delay after FSR is pressed
        car_moving = true;
        Motor_MoveForward();
    }
}

void Obstacle_CheckAndHandle() {
    uint16_t distance = Ultrasonic_GetDistance();
    if (distance < OBSTACLE_THRESHOLD) {
        Motor_Stop();
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_SET);  // Turn buzzer on
        while (Ultrasonic_GetDistance() < OBSTACLE_THRESHOLD);  // Wait until obstacle clears
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_RESET); // Turn buzzer off
        Motor_MoveForward();
    }
}

void LineFollowing() {
    GPIO_PinState left_sensor = HAL_GPIO_ReadPin(GPIOA, LEFT_IR_Pin);
    GPIO_PinState right_sensor = HAL_GPIO_ReadPin(GPIOA, RIGHT_IR_Pin);

    if (left_sensor == LINE_DETECTED && right_sensor != LINE_DETECTED) {
        Motor_TurnLeft();
    } else if (right_sensor == LINE_DETECTED && left_sensor != LINE_DETECTED) {
        Motor_TurnRight();
    } else if (left_sensor == LINE_DETECTED && right_sensor == LINE_DETECTED) {
        Motor_MoveForward();
    } else {
        Motor_Stop();  // Stop if the line is lost
    }
}

void StopAtMarkerAndPause() {
    Motor_Stop();
    HAL_Delay(2000);  // Pause for 2 seconds (adjust as needed)
    Motor_MoveForward();
}


// Helper function to get distance from ultrasonic sensor
uint16_t Ultrasonic_GetDistance() {
    // Trigger the ultrasonic sensor
    HAL_GPIO_WritePin(GPIOA, ULTRASONIC_TRIG_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);

    // Wait for echo and measure the response time
    uint32_t startTime = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOA, ULTRASONIC_ECHO_Pin) == GPIO_PIN_RESET) {
        if (HAL_GetTick() - startTime > 100) return 0;  // Timeout for no echo
    }

    startTime = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOA, ULTRASONIC_ECHO_Pin) == GPIO_PIN_SET);
    uint32_t endTime = HAL_GetTick();
    
    // Calculate distance based on time and speed of sound
    uint32_t timeElapsed = endTime - startTime;
    return timeElapsed * 0.034 / 2;
}

// Motor control functions
void Motor_Stop() {
    HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR3_GPIO_Port, MOTOR3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR4_GPIO_Port, MOTOR4_Pin, GPIO_PIN_RESET);
}

void Motor_MoveForward() {
    HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_SET);  // Motor 1 ON
    HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, GPIO_PIN_RESET);  // Motor 2 OFF
    HAL_GPIO_WritePin(MOTOR3_GPIO_Port, MOTOR3_Pin, GPIO_PIN_SET);  // Motor 3 ON
    HAL_GPIO_WritePin(MOTOR4_GPIO_Port, MOTOR4_Pin, GPIO_PIN_RESET);  // Motor 4 OFF
}

void Motor_TurnLeft() {
    HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_RESET);  // Motor 1 OFF
    HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, GPIO_PIN_SET);  // Motor 2 ON
    HAL_GPIO_WritePin(MOTOR3_GPIO_Port, MOTOR3_Pin, GPIO_PIN_SET);  // Motor 3 ON
    HAL_GPIO_WritePin(MOTOR4_GPIO_Port, MOTOR4_Pin, GPIO_PIN_RESET);  // Motor 4 OFF
}

void Motor_TurnRight() {
    HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_SET);  // Motor 1 ON
    HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, GPIO_PIN_RESET);  // Motor 2 OFF
    HAL_GPIO_WritePin(MOTOR3_GPIO_Port, MOTOR3_Pin, GPIO_PIN_RESET);  // Motor 3 OFF
    HAL_GPIO_WritePin(MOTOR4_GPIO_Port, MOTOR4_Pin, GPIO_PIN_SET);  // Motor 4 ON
}
