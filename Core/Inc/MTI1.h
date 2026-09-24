/*
 * MTI1.h
 *
 *  Created on: Sep 24, 2026
 *      Author: juang
 */

#ifndef INC_MTI1_H_
#define INC_MTI1_H_

#include "main.h"

// Estructura para almacenar los datos del MTi-1
typedef struct {
    float acc_x, acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
    float roll, pitch, yaw;
    int test_flag;
} MTi1_Data_t;

// Prototipos de funciones públicas de la librería
void MTi1_Init(UART_HandleTypeDef *huart);
void MTi1_ProcessRx(void);
void MTi1_RxCpltCallback(UART_HandleTypeDef *huart, uint8_t rx_data);
void MTi1_ErrorCallback(UART_HandleTypeDef *huart);
MTi1_Data_t* MTi1_GetData(void);

#endif /* INC_MTI1_H_ */
