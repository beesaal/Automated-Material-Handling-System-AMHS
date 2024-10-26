#ifndef __BLUETOOTH_COMM_H
#define __BLUETOOTH_COMM_H


#include "stm32f4xx_hal.h"  // For HAL functions
#include "FreeRTOS.h"       // For FreeRTOS support
#include "task.h"           // For task functions
#include <math.h>           // For math operations
#include <stdio.h>          // For printf
#include <stdint.h>         // Standard integer types
#include "string.h"

#include <math.h>
#include "stdio.h"

#define BUFFER_SIZE 100
extern UART_HandleTypeDef huart1;
extern uint8_t rxBuffer[BUFFER_SIZE];  // Receiving buffer
extern uint8_t txBuffer[BUFFER_SIZE];  // Transmitting buffer
extern uint8_t received;  // Flag to indicate data received



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void sendBluetoothData_IT(char* data);


#endif /*__BLUETOOTH_COMM_H*/
