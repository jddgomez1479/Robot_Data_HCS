/*
 * MTI1.c
 *
 *  Created on: Sep 24, 2026
 *      Author: juang
 *
 *  Cambios principales:
 *  - Recepcion por DMA circular: cero interrupciones por byte, sin riesgo de
 *    overrun. Si la UART no tiene DMA (hdmarx == NULL) usa interrupcion por
 *    byte como antes.
 *  - Resincronizacion robusta: si un 0xFA falso o un checksum invalido aparece,
 *    se avanza 1 byte (antes se saltaba el paquete entero y se perdian datos).
 *  - Marca de "dato nuevo" para que main envie la trama sincronizada con la IMU.
 */

#include "MTI1.h"
#include <string.h>

#define RX_RING_SIZE   2048U       // potencia de 2
#define XBUS_PRE       0xFA
#define XBUS_BID       0xFF
#define XBUS_MTDATA2   0x36

static uint8_t           rx_ring[RX_RING_SIZE] __attribute__((aligned(4)));
static volatile uint16_t rx_head_it = 0;   // solo en modo interrupcion
static uint16_t          rx_tail = 0;
static uint8_t           rx_byte_it;
static uint8_t           use_dma = 0;
static volatile uint8_t  resync = 0;

static UART_HandleTypeDef *mti_huart = NULL;
static MTi1_Data_t imu_data = {0};

static uint8_t  new_data = 0;
static uint32_t last_packet_tick = 0;
static uint32_t packet_count = 0;
static uint32_t checksum_errors = 0;

// ----------------------------------------------------------------------------
// Utilidades
// ----------------------------------------------------------------------------
static float bytes_to_float_be(const uint8_t *b)
{
    uint32_t u = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
                 ((uint32_t)b[2] << 8)  |  (uint32_t)b[3];
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

static uint8_t calculate_xbus_checksum(const uint8_t *buffer, uint16_t length)
{
    uint8_t sum = 0;
    for (uint16_t i = 1; i < (length - 1); i++) sum += buffer[i];
    return (uint8_t)(0x100 - sum);
}

static inline uint16_t ring_head(void)
{
    if (use_dma) {
        uint16_t h = (uint16_t)(RX_RING_SIZE - __HAL_DMA_GET_COUNTER(mti_huart->hdmarx));
        return (h >= RX_RING_SIZE) ? 0 : h;
    }
    return rx_head_it;
}

static inline uint8_t ring_at(uint16_t idx)
{
    return rx_ring[idx & (RX_RING_SIZE - 1)];
}

static void start_rx(void)
{
    if (use_dma) {
        HAL_UART_Receive_DMA(mti_huart, rx_ring, RX_RING_SIZE);
        // No necesitamos las interrupciones de mitad/fin de buffer
        __HAL_DMA_DISABLE_IT(mti_huart->hdmarx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(mti_huart->hdmarx, DMA_IT_TC);
    } else {
        HAL_UART_Receive_IT(mti_huart, &rx_byte_it, 1);
    }
}

// Devuelve 1 si el paquete trajo datos utiles.
static uint8_t parse_mtdata2(const uint8_t *packet, uint16_t packet_len)
{
    uint8_t  got = 0;
    uint8_t  payload_len = packet[3];
    uint16_t idx = 4;
    uint16_t end = 4 + payload_len;

    while ((idx + 3) <= end) {
        uint16_t xdi = ((uint16_t)packet[idx] << 8) | packet[idx + 1];
        uint8_t  size = packet[idx + 2];
        const uint8_t *d = &packet[idx + 3];

        if ((idx + 3 + size) > end || (idx + 3 + size) > packet_len) break;

        switch (xdi & 0xFFF0) {
        case 0x4020: // Acceleration
            if (size >= 12) {
                imu_data.acc_x = bytes_to_float_be(&d[0]);
                imu_data.acc_y = bytes_to_float_be(&d[4]);
                imu_data.acc_z = bytes_to_float_be(&d[8]);
                got = 1;
            }
            break;
        case 0x8020: // Rate of turn
            if (size >= 12) {
                imu_data.gyr_x = bytes_to_float_be(&d[0]);
                imu_data.gyr_y = bytes_to_float_be(&d[4]);
                imu_data.gyr_z = bytes_to_float_be(&d[8]);
                got = 1;
            }
            break;
        case 0x2010: // Quaternion
            if (size >= 16) {
                imu_data.q0 = bytes_to_float_be(&d[0]);
                imu_data.q1 = bytes_to_float_be(&d[4]);
                imu_data.q2 = bytes_to_float_be(&d[8]);
                imu_data.q3 = bytes_to_float_be(&d[12]);
                imu_data.test_flag = 1;
                got = 1;
            }
            break;
        default:
            break;
        }
        idx += 3 + size;
    }
    return got;
}

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------
void MTi1_Init(UART_HandleTypeDef *huart)
{
    mti_huart = huart;
    use_dma = (huart->hdmarx != NULL);

    uint8_t goto_config[] = {0xFA, 0xFF, 0x30, 0x00, 0x00};
    goto_config[4] = calculate_xbus_checksum(goto_config, sizeof(goto_config));
    HAL_UART_Transmit(mti_huart, goto_config, sizeof(goto_config), 200);
    HAL_Delay(100);

    uint8_t set_output_config[] = {
        0xFA, 0xFF, 0xC0, 0x0C,
        0x40, 0x20, 0x00, 0x64,   // Acceleration  @ 100 Hz
        0x80, 0x20, 0x00, 0x64,   // Rate of turn  @ 100 Hz
        0x20, 0x10, 0x00, 0x64,   // Quaternion    @ 100 Hz
        0x00
    };
    set_output_config[sizeof(set_output_config) - 1] =
        calculate_xbus_checksum(set_output_config, sizeof(set_output_config));
    HAL_UART_Transmit(mti_huart, set_output_config, sizeof(set_output_config), 200);
    HAL_Delay(100);

    uint8_t goto_measurement[] = {0xFA, 0xFF, 0x10, 0x00, 0x00};
    goto_measurement[4] = calculate_xbus_checksum(goto_measurement, sizeof(goto_measurement));
    HAL_UART_Transmit(mti_huart, goto_measurement, sizeof(goto_measurement), 200);
    HAL_Delay(100);

    // Limpiar errores acumulados mientras no se recibia y arrancar recepcion
    volatile uint32_t tmp = mti_huart->Instance->SR;
    tmp = mti_huart->Instance->DR;
    (void)tmp;
    rx_tail = 0;
    rx_head_it = 0;
    start_rx();
}

void MTi1_ProcessRx(void)
{
    if (mti_huart == NULL) return;

    if (resync) {               // la recepcion se reinicio tras un error
        resync = 0;
        rx_tail = ring_head();
    }

    uint16_t head = ring_head();

    while (rx_tail != head) {
        uint16_t available = (uint16_t)((head - rx_tail) & (RX_RING_SIZE - 1));

        // Buscar preambulo + BID
        if (ring_at(rx_tail) != XBUS_PRE) {
            rx_tail = (rx_tail + 1) & (RX_RING_SIZE - 1);
            continue;
        }
        if (available < 2) break;
        if (ring_at(rx_tail + 1) != XBUS_BID) {
            rx_tail = (rx_tail + 1) & (RX_RING_SIZE - 1);
            continue;
        }
        if (available < 5) break;

        uint8_t  len   = ring_at(rx_tail + 3);
        uint16_t total = 5 + len;
        if (len == 0xFF) {      // longitud extendida: no se usa aqui
            rx_tail = (rx_tail + 1) & (RX_RING_SIZE - 1);
            continue;
        }
        if (available < total) break;   // paquete incompleto, esperar

        // Verificar checksum directamente en el anillo
        uint8_t sum = 0;
        for (uint16_t i = 1; i < total; i++) sum += ring_at(rx_tail + i);
        if (sum != 0) {
            checksum_errors++;
            rx_tail = (rx_tail + 1) & (RX_RING_SIZE - 1);   // 0xFA falso: resincronizar
            continue;
        }

        if (ring_at(rx_tail + 2) == XBUS_MTDATA2) {
            uint8_t pkt[260];
            for (uint16_t i = 0; i < total; i++) pkt[i] = ring_at(rx_tail + i);
            if (parse_mtdata2(pkt, total)) {
                new_data = 1;
                packet_count++;
                last_packet_tick = HAL_GetTick();
            }
        }
        rx_tail = (rx_tail + total) & (RX_RING_SIZE - 1);
    }
}

MTi1_Data_t* MTi1_GetData(void)       { return &imu_data; }
uint32_t     MTi1_LastPacketTick(void) { return last_packet_tick; }
uint32_t     MTi1_PacketCount(void)    { return packet_count; }
uint32_t     MTi1_ChecksumErrors(void) { return checksum_errors; }

uint8_t MTi1_ConsumeNewData(void)
{
    uint8_t n = new_data;
    new_data = 0;
    return n;
}

// ----------------------------------------------------------------------------
// Callbacks
// ----------------------------------------------------------------------------
void MTi1_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != mti_huart || use_dma) return;   // en DMA no hay nada que hacer

    uint16_t next = (rx_head_it + 1) & (RX_RING_SIZE - 1);
    if (next != rx_tail) {
        rx_ring[rx_head_it] = rx_byte_it;
        rx_head_it = next;
    }
    HAL_UART_Receive_IT(mti_huart, &rx_byte_it, 1);
}

void MTi1_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != mti_huart) return;

    volatile uint32_t tmp = huart->Instance->SR;   // limpia ORE/NE/FE/PE
    tmp = huart->Instance->DR;
    (void)tmp;

    if (huart->RxState == HAL_UART_STATE_READY) {  // HAL detuvo la recepcion
        if (!use_dma) rx_head_it = 0;
        resync = 1;
        start_rx();
    }
}
