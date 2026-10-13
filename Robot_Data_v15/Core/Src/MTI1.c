/*
 * MTI1.c
 *
 *  Created on: Sep 24, 2026
 *      Author: juang
 */


#include "mti1.h"
#include <string.h>

#define RX_RING_SIZE 2048

// Variables privadas de la librería
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static UART_HandleTypeDef *mti_huart = NULL;

static MTi1_Data_t imu_data = {0};

static float bytes_to_float_be(const uint8_t *buffer);
static uint8_t xsens_validate_checksum(const uint8_t *msg, uint16_t length);
static uint8_t calculate_xbus_checksum(const uint8_t *buffer, uint16_t length);
static void parse_mtdata2(const uint8_t *packet, uint16_t packet_len);

static uint8_t xsens_validate_checksum(const uint8_t *msg, uint16_t length) {
    uint8_t sum = 0;
    for (uint16_t i = 1; i < length; i++) {
        sum += msg[i];
    }
    return (sum == 0);
}

// Funciones auxiliares internas
static float bytes_to_float_be(const uint8_t *buffer) {
    uint32_t b = ((uint32_t)buffer[0] << 24) |
                 ((uint32_t)buffer[1] << 16) |
                 ((uint32_t)buffer[2] << 8)  |
                  (uint32_t)buffer[3];
    float f;
    memcpy(&f, &b, sizeof(float));
    return f;
}

static uint8_t calculate_xbus_checksum(const uint8_t *buffer, uint16_t length) {
    uint8_t sum = 0;
    for (uint16_t i = 1; i < (length - 1); i++) {
        sum += buffer[i];
    }
    return (uint8_t)(0x100 - sum);
}

static void parse_mtdata2(const uint8_t *packet, uint16_t packet_len) {
    if (packet_len < 5) return;
    if (packet[0] != 0xFA || packet[2] != 0x36) return;

    if (!xsens_validate_checksum(packet, packet_len)) {
        return; // Si el checksum falla se descarta el paquete corrupto inmediatamente
    }

    uint8_t payload_len = packet[3];
    uint16_t idx = 4;

    while (idx < (4 + payload_len) && (idx + 3) <= packet_len) {
        uint16_t xdi = (packet[idx] << 8) | packet[idx + 1];
        uint8_t data_size = packet[idx + 2];
        const uint8_t *data_ptr = &packet[idx + 3];

        if ((idx + 3 + data_size) > packet_len) break;

        if ((xdi & 0xFFF0) == 0x4020) { // ACC
            imu_data.acc_x = bytes_to_float_be(&data_ptr[0]);
            imu_data.acc_y = bytes_to_float_be(&data_ptr[4]);
            imu_data.acc_z = bytes_to_float_be(&data_ptr[8]);
        }
        else if ((xdi & 0xFFF0) == 0x8020) { // GYR
            imu_data.gyr_x = bytes_to_float_be(&data_ptr[0]);
            imu_data.gyr_y = bytes_to_float_be(&data_ptr[4]);
            imu_data.gyr_z = bytes_to_float_be(&data_ptr[8]);
        }
        else if ((xdi & 0xFFF0) == 0x2010) { // QUATERNION
			imu_data.q0 = bytes_to_float_be(&data_ptr[0]);
			imu_data.q1 = bytes_to_float_be(&data_ptr[4]);
			imu_data.q2 = bytes_to_float_be(&data_ptr[8]);
			imu_data.q3 = bytes_to_float_be(&data_ptr[12]);
			imu_data.test_flag = 1;
		}

       /*else if ((xdi & 0xFFF0) == 0x2030) { // EULER
            imu_data.roll  = bytes_to_float_be(&data_ptr[0]);
            imu_data.pitch = bytes_to_float_be(&data_ptr[4]);
            imu_data.yaw   = bytes_to_float_be(&data_ptr[8]);
            imu_data.test_flag = 1;
        }*/

        idx += 3 + data_size;
    }
}

// Funciones públicas
void MTi1_Init(UART_HandleTypeDef *huart) {
    mti_huart = huart;

    uint8_t goto_config[] = {0xFA, 0xFF, 0x30, 0x00, 0x00};
    goto_config[4] = calculate_xbus_checksum(goto_config, sizeof(goto_config));
    HAL_UART_Transmit(mti_huart, goto_config, sizeof(goto_config), 200);
    HAL_Delay(100);

    uint8_t set_output_config[] = {
        0xFA, 0xFF, 0xC0, 0x0C,
        0x40, 0x20, 0x00, 0x64,       // ACC @ 100Hz
        0x80, 0x20, 0x00, 0x64,       // GYR @ 100Hz
		0x20, 0x10, 0x00, 0x64,
        //0x20, 0x30, 0x00, 0x64,       // Euler @ 100Hz
        0x00
    };
    set_output_config[sizeof(set_output_config) - 1] = calculate_xbus_checksum(set_output_config, sizeof(set_output_config));

    HAL_UART_Transmit(mti_huart, set_output_config, sizeof(set_output_config), 200);
    HAL_Delay(100);

    uint8_t goto_measurement[] = {0xFA, 0xFF, 0x10, 0x00, 0x00};
    goto_measurement[4] = calculate_xbus_checksum(goto_measurement, sizeof(goto_measurement));
    HAL_UART_Transmit(mti_huart, goto_measurement, sizeof(goto_measurement), 200);
    HAL_Delay(100);
}

void MTi1_ProcessRx(void) {
    while (rx_head != rx_tail) {
        if (rx_ring[rx_tail] != 0xFA) {
            rx_tail = (rx_tail + 1) % RX_RING_SIZE;
            continue;
        }

        uint16_t available = (rx_head >= rx_tail) ? (rx_head - rx_tail) : (RX_RING_SIZE - rx_tail + rx_head);
        if (available < 5) break;

        uint16_t len_idx = (rx_tail + 3) % RX_RING_SIZE;
        uint8_t payload_len = rx_ring[len_idx];
        uint16_t total_len = 5 + payload_len;

        if (total_len > 256) {
            rx_tail = (rx_tail + 1) % RX_RING_SIZE;
            continue;
        }

        if (available < total_len) break;

        uint8_t packet_buf[256];
        for (uint16_t i = 0; i < total_len; i++) {
            packet_buf[i] = rx_ring[(rx_tail + i) % RX_RING_SIZE];
        }

        if (packet_buf[2] == 0x36) {
            parse_mtdata2(packet_buf, total_len);
        }

        rx_tail = (rx_tail + total_len) % RX_RING_SIZE;
    }
}

void MTi1_RxCpltCallback(UART_HandleTypeDef *huart, uint8_t rx_data) {
    if (huart == mti_huart) {
        uint16_t next = (rx_head + 1) % RX_RING_SIZE;
        if (next != rx_tail) {
            rx_ring[rx_head] = rx_data;
            rx_head = next;
        }
    }
}

void MTi1_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == mti_huart) {
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
}

MTi1_Data_t* MTi1_GetData(void) {
    return &imu_data;
}

