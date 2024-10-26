#ifndef __BARCODE_READER_H
#define __BARCODE_READER_H



#include "stm32f4xx_hal.h"  // For HAL functions
#include "FreeRTOS.h"       // For FreeRTOS support
#include "task.h"           // For task functions
#include <math.h>           // For math operations
#include <stdio.h>          // For printf
#include <stdint.h>         // Standard integer types
#include <string.h>

#include <math.h>
#include "stdio.h"


extern I2C_HandleTypeDef hi2c2; // Assuming I2C1 is used for QR scanner

#define QR_SCANNER_I2C_ADDR 0x3C << 1  // Replace 0x3C with actual I2C address
#define BUFFER_SIZE 100

extern char qr_data[BUFFER_SIZE];      // Buffer to store QR data
extern int data_received; // Flag to indicate data received (volatile for ISR safety)




void QR_Scanner_Init(void);
void QR_Scanner_Read(void);
void Process_QR_Data(char *data);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void I2C1_EV_IRQHandler(void);


#endif