/*
 * MTI1.h
 *
 *  Created on: Sep 24, 2026
 *      Author: juang
 *
 *  Recepcion por DMA circular (recomendado) con respaldo por interrupcion
 *  byte a byte si la UART no tiene DMA configurado en CubeMX.
 */

#ifndef INC_MTI1_H_
#define INC_MTI1_H_

#include "main.h"

typedef struct {
    float acc_x, acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
    float q0, q1, q2, q3;
    int   test_flag;
} MTi1_Data_t;

void          MTi1_Init(UART_HandleTypeDef *huart);
void          MTi1_ProcessRx(void);
MTi1_Data_t*  MTi1_GetData(void);

// Devuelve 1 (y limpia la marca) si llego un paquete nuevo desde la ultima llamada.
uint8_t       MTi1_ConsumeNewData(void);
// HAL_GetTick() del ultimo paquete valido (para detectar IMU desconectada).
uint32_t      MTi1_LastPacketTick(void);
// Estadisticas de diagnostico.
uint32_t      MTi1_PacketCount(void);
uint32_t      MTi1_ChecksumErrors(void);

// Llamar desde HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback en main.c.
// La libreria se encarga de rearmar la recepcion (DMA o interrupcion).
void          MTi1_RxCpltCallback(UART_HandleTypeDef *huart);
void          MTi1_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* INC_MTI1_H_ */
