/*
 * MD49.c
 *
 *  Created on: Aug 14, 2026
 *      Author: juandg
 *
 *  Driver no bloqueante para el MD49.
 *  - Una sola transaccion en curso a la vez (el MD49 es pregunta/respuesta).
 *  - La recepcion se arma ANTES de transmitir, asi no se pierde la respuesta.
 *  - Cada transaccion tiene timeout; si falla se aborta, se limpia la UART y se
 *    cuenta el error, sin bloquear el lazo principal.
 *  - Velocidad calculada con el contador de ciclos (DWT), resolucion < 1 us.
 */

#include "MD49.h"

typedef enum {
    JOB_NONE = 0,
    JOB_SPEED,
    JOB_ENC,
    JOB_VI,
    JOB_ERR
} md49_job_t;

static struct {
    UART_HandleTypeDef *huart;

    // transaccion en curso
    uint8_t     busy;
    md49_job_t  job;
    uint8_t     rx_len;
    uint32_t    t_start_ms;
    uint8_t     tx[8];
    uint8_t     rx[8];
    volatile uint8_t  rx_done;
    volatile uint8_t  rx_err;
    volatile uint32_t rx_cycles;   // marca de tiempo DWT al completar la RX

    // velocidad pendiente
    volatile uint8_t speed_pending;
    volatile uint8_t s1, s2;

    // planificacion
    uint32_t next_enc, next_vi, next_err;

    // calculo de velocidad
    uint8_t  have_prev;
    int32_t  enc1_prev, enc2_prev;
    uint32_t cyc_prev;
} md;

static MD49_Data_t md_data;

// ----------------------------------------------------------------------------
// Utilidades
// ----------------------------------------------------------------------------
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Descarta cualquier byte pendiente y limpia errores (ORE, FE, NE, PE).
static void uart_flush(UART_HandleTypeDef *huart)
{
    volatile uint32_t tmp;
    tmp = huart->Instance->SR;
    tmp = huart->Instance->DR;
    (void)tmp;
}

static int32_t be32(const uint8_t *b)
{
    return (int32_t)(((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
                     ((uint32_t)b[2] << 8)  |  (uint32_t)b[3]);
}

static uint8_t map_error(uint8_t err)
{
    uint8_t t = 0;
    if (err & 0x04) t = 2;
    if (err & 0x08) t = 3;
    if (err & 0x10) t = 4;
    if (err & 0x20) t = 5;
    if (err & 0x40) t = 6;
    if (err & 0x80) t = 7;
    return t;
}

// Planificacion por plazos: next += periodo (sin deriva). Si se acumulo
// mucho retraso, se resincroniza para no disparar rafagas.
static uint8_t due(uint32_t now, uint32_t *next, uint32_t period)
{
    if ((int32_t)(now - *next) < 0) return 0;
    *next += period;
    if ((int32_t)(now - *next) >= 0) *next = now + period;
    return 1;
}

static void start_job(md49_job_t job, uint8_t tx_len, uint8_t rx_len)
{
    UART_HandleTypeDef *h = md.huart;

    uart_flush(h);
    md.job = job;
    md.rx_len = rx_len;
    md.rx_done = 0;
    md.rx_err = 0;
    md.t_start_ms = HAL_GetTick();

    if (rx_len > 0) {
        if (HAL_UART_Receive_IT(h, md.rx, rx_len) != HAL_OK) {
            HAL_UART_Abort(h);
            md_data.comm_errors++;
            md.busy = 0;
            return;
        }
    }
    if (HAL_UART_Transmit_IT(h, md.tx, tx_len) != HAL_OK) {
        HAL_UART_Abort(h);
        md_data.comm_errors++;
        md.busy = 0;
        return;
    }
    md.busy = 1;
}

static void finish_job(void)
{
    const uint8_t *r = md.rx;

    switch (md.job) {
    case JOB_ENC: {
        int32_t e1 = be32(&r[0]);
        int32_t e2 = be32(&r[4]);
        uint32_t cyc = md.rx_cycles;

        if (md.have_prev) {
            uint32_t dcyc = cyc - md.cyc_prev;
            if (dcyc > 0) {
                float dt_s = (float)dcyc / (float)SystemCoreClock;
                md_data.vel1_pps = (float)(e1 - md.enc1_prev) / dt_s;
                md_data.vel2_pps = (float)(e2 - md.enc2_prev) / dt_s;
            }
        }
        md.have_prev = 1;
        md.enc1_prev = e1;
        md.enc2_prev = e2;
        md.cyc_prev  = cyc;

        md_data.enc1 = e1;
        md_data.enc2 = e2;
        md_data.enc_updates++;
        break;
    }
    case JOB_VI:
        md_data.volts = (float)r[0];
        md_data.i1    = (float)r[1] / 10.0f;
        md_data.i2    = (float)r[2] / 10.0f;
        break;
    case JOB_ERR:
        md_data.t_err = map_error(r[0]);
        break;
    default:
        break;
    }
    md.busy = 0;
}

static void abort_job(void)
{
    HAL_UART_Abort(md.huart);
    uart_flush(md.huart);
    md_data.comm_errors++;
    // Si fallo una lectura de encoder, la siguiente velocidad se calcula
    // igual con el intervalo real (DWT), asi que no hace falta reiniciar.
    md.busy = 0;
}

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------
HAL_StatusTypeDef MD49_ExCmd(UART_HandleTypeDef *huart, uint8_t cmd, uint8_t *params,
                             uint8_t n_params, uint8_t *rx_data, uint8_t rx_bytes)
{
    uint8_t packet_tx[10];
    if (n_params > 8) return HAL_ERROR;

    packet_tx[0] = MD49_SYNC_BYTE;
    packet_tx[1] = cmd;
    for (uint8_t i = 0; i < n_params && params != NULL; i++) {
        packet_tx[2 + i] = params[i];
    }

    uart_flush(huart);
    if (HAL_UART_Transmit(huart, packet_tx, 2 + n_params, 10) != HAL_OK) return HAL_ERROR;
    if (rx_bytes == 0 || rx_data == NULL) return HAL_OK;
    return HAL_UART_Receive(huart, rx_data, rx_bytes, 20);
}

void MD49_Init(UART_HandleTypeDef *huart)
{
    uint8_t b;

    md.huart = huart;
    dwt_init();

    // Operaciones bloqueantes: solo al arranque, cuando no importa el tiempo.
    MD49_ExCmd(huart, MD49_CMD_RESET_ENCODERS, NULL, 0, NULL, 0);
    if (MD49_ExCmd(huart, MD49_CMD_GET_VERSION, NULL, 0, &b, 1) == HAL_OK)      md_data.version = b;
    if (MD49_ExCmd(huart, MD49_CMD_GET_MODE, NULL, 0, &b, 1) == HAL_OK)         md_data.mode = b;
    if (MD49_ExCmd(huart, MD49_CMD_GET_ACCELERATION, NULL, 0, &b, 1) == HAL_OK) md_data.accel = b;
    if (MD49_ExCmd(huart, MD49_CMD_GET_ERROR, NULL, 0, &b, 1) == HAL_OK)        md_data.t_err = map_error(b);

    uint8_t vi[3];
    if (MD49_ExCmd(huart, MD49_CMD_GET_VI, NULL, 0, vi, 3) == HAL_OK) {
        md_data.volts = (float)vi[0];
        md_data.i1 = (float)vi[1] / 10.0f;
        md_data.i2 = (float)vi[2] / 10.0f;
    }

    uint32_t now = HAL_GetTick();
    md.next_enc = now;
    md.next_vi  = now + 3;   // desfasados para no coincidir con los encoders
    md.next_err = now + 7;
}

void MD49_RequestSpeed(uint8_t speed1, uint8_t speed2)
{
    md.s1 = speed1;
    md.s2 = speed2;
    md.speed_pending = 1;
}

const MD49_Data_t* MD49_GetData(void)
{
    return &md_data;
}

void MD49_Task(void)
{
    if (md.huart == NULL) return;
    uint32_t now = HAL_GetTick();

    // 1) Gestionar la transaccion en curso
    if (md.busy) {
        if (md.rx_err) {
            abort_job();
        } else if (md.rx_len == 0) {
            // Solo TX (comando de velocidad): termina cuando la UART queda libre
            if (md.huart->gState == HAL_UART_STATE_READY) md.busy = 0;
            else if ((now - md.t_start_ms) > MD49_RX_TIMEOUT_MS) abort_job();
        } else if (md.rx_done) {
            finish_job();
        } else if ((now - md.t_start_ms) > MD49_RX_TIMEOUT_MS) {
            abort_job();
        }
        if (md.busy) return;
    }

    // 2) Elegir la siguiente transaccion (la velocidad tiene prioridad)
    if (md.speed_pending) {
        md.speed_pending = 0;
        // Ambos comandos en una sola transmision de 6 bytes
        md.tx[0] = MD49_SYNC_BYTE; md.tx[1] = MD49_CMD_SET_SPEED1; md.tx[2] = md.s1;
        md.tx[3] = MD49_SYNC_BYTE; md.tx[4] = MD49_CMD_SET_SPEED2; md.tx[5] = md.s2;
        start_job(JOB_SPEED, 6, 0);
    } else if (due(now, &md.next_enc, MD49_ENC_PERIOD_MS)) {
        md.tx[0] = MD49_SYNC_BYTE; md.tx[1] = MD49_CMD_GET_ENCODERS;
        start_job(JOB_ENC, 2, 8);
    } else if (due(now, &md.next_vi, MD49_VI_PERIOD_MS)) {
        md.tx[0] = MD49_SYNC_BYTE; md.tx[1] = MD49_CMD_GET_VI;
        start_job(JOB_VI, 2, 3);
    } else if (due(now, &md.next_err, MD49_ERR_PERIOD_MS)) {
        md.tx[0] = MD49_SYNC_BYTE; md.tx[1] = MD49_CMD_GET_ERROR;
        start_job(JOB_ERR, 2, 1);
    }
}

// ----------------------------------------------------------------------------
// Callbacks (llamadas desde los callbacks de HAL en main.c)
// ----------------------------------------------------------------------------
void MD49_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == md.huart) {
        md.rx_cycles = DWT->CYCCNT;
        md.rx_done = 1;
    }
}

void MD49_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == md.huart) {
        md.rx_err = 1;
    }
}
