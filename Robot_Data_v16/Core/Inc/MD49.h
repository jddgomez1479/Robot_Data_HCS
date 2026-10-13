/*
 * MD49.h
 *
 *  Created on: Aug 14, 2026
 *      Author: juang
 *
 *  Version no bloqueante: las transacciones con el driver se hacen por
 *  interrupcion (HAL_UART_xxx_IT) y se gestionan con una maquina de estados
 *  en MD49_Task(), que se llama en cada vuelta del lazo principal.
 */

#ifndef INC_MD49_H_
#define INC_MD49_H_

#include "main.h"

// ============================================================================
// COMANDOS DEL DRIVER MD49
// ============================================================================
#define MD49_SYNC_BYTE              0x00

#define MD49_CMD_GET_SPEED1         0x21
#define MD49_CMD_GET_SPEED2         0x22
#define MD49_CMD_GET_ENCODER1       0x23
#define MD49_CMD_GET_ENCODER2       0x24
#define MD49_CMD_GET_ENCODERS       0x25
#define MD49_CMD_GET_VOLTS          0x26
#define MD49_CMD_GET_CURRENT1       0x27
#define MD49_CMD_GET_CURRENT2       0x28
#define MD49_CMD_GET_VERSION        0x29
#define MD49_CMD_GET_ACCELERATION   0x2A
#define MD49_CMD_GET_MODE           0x2B
#define MD49_CMD_GET_VI             0x2C   // volts, corriente 1, corriente 2 (3 bytes)
#define MD49_CMD_GET_ERROR          0x2D
#define MD49_CMD_SET_SPEED1         0x31
#define MD49_CMD_SET_SPEED2         0x32
#define MD49_CMD_SET_MODE           0x34
#define MD49_CMD_RESET_ENCODERS     0x35
#define MD49_CMD_DISABLE_REGULATOR  0x36
#define MD49_CMD_ENABLE_REGULATOR   0x37
#define MD49_CMD_DISABLE_TIMEOUT    0x38
#define MD49_CMD_ENABLE_TIMEOUT     0x39

// ============================================================================
// PARAMETROS DE TIEMPO (ajustables)
// ============================================================================
// Periodo de lectura de encoders. Una lectura ocupa ~2.6 ms del bus a 38400
// baudios (2 bytes TX + 8 bytes RX). Con 10 ms tendrias velocidad a 100 Hz;
// prueba primero con 20 ms y baja a 10 si la tasa de errores sigue en cero.
#define MD49_ENC_PERIOD_MS      20U
#define MD49_VI_PERIOD_MS       100U    // voltaje + corrientes
#define MD49_ERR_PERIOD_MS      500U    // registro de errores
#define MD49_RX_TIMEOUT_MS      10U     // timeout de una transaccion

// ============================================================================
// DATOS PUBLICADOS
// ============================================================================
typedef struct {
    int32_t  enc1, enc2;          // cuentas de encoder
    float    vel1_pps, vel2_pps;  // velocidad en pulsos/s
    float    volts;               // tension de bateria [V]
    float    i1, i2;              // corrientes [A]
    uint8_t  version, mode, accel;
    uint8_t  t_err;               // codigo de error (misma codificacion que antes)
    uint32_t enc_updates;         // contador de lecturas de encoder validas
    uint32_t comm_errors;         // contador de timeouts / errores de UART
} MD49_Data_t;

// ============================================================================
// API
// ============================================================================
// Inicializacion (bloqueante, solo al arranque): resetea encoders y lee info.
void MD49_Init(UART_HandleTypeDef *huart);

// Tarea no bloqueante: llamarla en cada vuelta del while(1).
void MD49_Task(void);

// Encola un cambio de velocidad (se envia en la siguiente ranura libre,
// con prioridad sobre las lecturas). Si llega otro antes, gana el ultimo.
void MD49_RequestSpeed(uint8_t speed1, uint8_t speed2);

// Acceso a los ultimos datos validos.
const MD49_Data_t* MD49_GetData(void);

// Llamar desde HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback.
void MD49_RxCpltCallback(UART_HandleTypeDef *huart);
void MD49_ErrorCallback(UART_HandleTypeDef *huart);

// Funcion bloqueante original (se mantiene para el arranque o pruebas).
HAL_StatusTypeDef MD49_ExCmd(UART_HandleTypeDef *huart, uint8_t cmd, uint8_t *params,
                             uint8_t n_params, uint8_t *rx_data, uint8_t rx_bytes);

#endif /* INC_MD49_H_ */
