#include "barcode_reader.h"


/**
 * @brief Initialize QR scanner (I2C).
 */
void QR_Scanner_Init(void) {
    QR_Scanner_Read(); // Start reading QR data with interrupt
}

/**
 * @brief Initiate I2C read for QR scanner using interrupts.
 */
void QR_Scanner_Read(void) {
    HAL_I2C_Master_Receive_IT(&hi2c1, QR_SCANNER_I2C_ADDR, (uint8_t*)qr_data, BUFFER_SIZE);
}

/**
 * @brief Process the QR data once received.
 */
void Process_QR_Data(char *data) {
    printf("QR Code Data: %s\n", data);  // Print the received data
    // Add more processing logic based on QR data if needed
}

/**
 * @brief Callback function called upon completion of I2C receive.
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {  // Check if the interrupt is from I2C1
        data_received = 1;         // Set flag to indicate data received
    }
}



/**
 * @brief I2C1 event interrupt handler.
 */
void I2C1_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

