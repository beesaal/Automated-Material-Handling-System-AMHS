#include "bluetooth_comm.h"

uint8_t received = 0;
uint8_t rxBuffer[BUFFER_SIZE]; 
uint8_t txBuffer[BUFFER_SIZE];

// Interrupt callback function for UART RX complete
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        received = 1;  // Set the flag when data is received
        HAL_UART_Receive_IT(&huart1, rxBuffer, BUFFER_SIZE);  // Re-enable the interrupt for continuous reception
    }
}

// Send data over Bluetooth using interrupt
void sendBluetoothData_IT(char* data) {
    strcpy((char*)txBuffer, data);  // Copy data to transmission buffer
    HAL_UART_Transmit_IT(&huart1, txBuffer, strlen((char*)txBuffer));  // Transmit in interrupt mode
}

