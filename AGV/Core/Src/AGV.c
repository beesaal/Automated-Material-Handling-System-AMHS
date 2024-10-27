
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
uint8_t current_speed = MIN_SPEED;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == FSR_Pin) {
        if (HAL_GPIO_ReadPin(FSR_GPIO_Port, FSR_Pin) == GPIO_PIN_SET) {  // Load detected
            HAL_Delay(2000);  // Delay after detecting load
            car_moving = 1;
            SmoothStart();
        }
    }
}

void Obstacle_CheckAndHandle() {
    uint16_t distance = Ultrasonic_GetDistance();
    if (distance < OBSTACLE_THRESHOLD) {
        SmoothStop();
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_SET);  // Turn buzzer on
        while (Ultrasonic_GetDistance() < OBSTACLE_THRESHOLD);  // Wait until obstacle clears
        HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, GPIO_PIN_RESET);  // Turn buzzer off
        HAL_Delay(2000);  // Wait for 2 seconds before resuming
        SmoothStart();
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
        StopAtMarkerAndPause();
    }
}

void StopAtMarkerAndPause() {
    SmoothStop();
    HAL_Delay(2000);  // Pause for 2 seconds (adjust as needed)

    if (HAL_GPIO_ReadPin(FSR_GPIO_Port, FSR_Pin) == GPIO_PIN_SET) {  // Box loaded
        car_moving = 1;
        SmoothStart();
    } else {  // Box not loaded, wait to be loaded
        car_moving = 0;
    }
}

// Smooth start by gradually increasing speed
void SmoothStart() {
    for (current_speed = MIN_SPEED; current_speed <= MAX_SPEED; current_speed += SPEED_STEP) {
        SetMotorSpeed(current_speed);
        HAL_Delay(100);  // Adjust delay for smoother/faster acceleration
    }
    Motor_MoveForward();
}

// Smooth stop by gradually decreasing speed
void SmoothStop() {
    for (current_speed = MAX_SPEED; current_speed >= MIN_SPEED; current_speed -= SPEED_STEP) {
        SetMotorSpeed(current_speed);
        HAL_Delay(100);  // Adjust delay for smoother/faster deceleration
    }
    Motor_Stop();
}

// Helper function to get distance from ultrasonic sensor
uint16_t Ultrasonic_GetDistance() {
    HAL_GPIO_WritePin(GPIOA, ULTRASONIC_TRIG_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);

    uint32_t startTime = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOA, ULTRASONIC_ECHO_Pin) == GPIO_PIN_RESET) {
        if (HAL_GetTick() - startTime > 100) return 0;
    }

    startTime = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOA, ULTRASONIC_ECHO_Pin) == GPIO_PIN_SET);
    uint32_t endTime = HAL_GetTick();

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
    HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR3_GPIO_Port, MOTOR3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR4_GPIO_Port, MOTOR4_Pin, GPIO_PIN_SET);
}

void Motor_TurnLeft() {
    HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR3_GPIO_Port, MOTOR3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR4_GPIO_Port, MOTOR4_Pin, GPIO_PIN_SET);
}

void Motor_TurnRight() {
    HAL_GPIO_WritePin(MOTOR1_GPIO_Port, MOTOR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR3_GPIO_Port, MOTOR3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR4_GPIO_Port, MOTOR4_Pin, GPIO_PIN_RESET);
}

// Function to set motor speed using PWM
void SetMotorSpeed(uint8_t speed) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);       //speed for motor 1
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 500);       //speed for motor 2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);       //speed for motor 3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);       //speed for motor 4
}
