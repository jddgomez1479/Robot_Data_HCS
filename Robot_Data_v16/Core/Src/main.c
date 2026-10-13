/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "MD49.h"
#include "MTI1.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_PERIOD_MS   10U    // 100 Hz: periodo de respaldo si la IMU no envia
#define IMU_STALE_MS      25U    // sin paquetes de IMU por mas de esto => usar temporizador propio
#define CMD_TIMEOUT_MS    0U     // 0 = desactivado. Ej. 500: detiene motores si el PC deja de mandar comandos
#define STOP_RESEND_MS    100U   // reenvio de la orden de parada mientras dure la falla
#define LED_PERIOD_MS     200U
#define MOTOR_STOP        128U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Comandos USB: los escribe CDC_Receive_FS (interrupcion USB) */
volatile char     Buffer_rx[64];
volatile uint8_t  string_ready = 0;

static uint32_t last_comm_time = 0;
static uint8_t  vel_r = MOTOR_STOP, vel_l = MOTOR_STOP;

/* Trama USB con doble buffer: nunca se escribe el buffer que el USB esta enviando */
static char     tx_buf[2][256];
static uint8_t  tx_idx = 0;
static char     err_buf[48];

/* Diagnostico (visibles en el depurador) */
volatile uint32_t frames_sent = 0;
volatile uint32_t frames_dropped = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Process_USB_Command(void);
static void Safety_Check(uint32_t now);
static void Send_Frame(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---------------------------- Callbacks HAL ------------------------------ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)      MTi1_RxCpltCallback(huart);
    else if (huart->Instance == USART2) MD49_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)      MTi1_ErrorCallback(huart);
    else if (huart->Instance == USART2) MD49_ErrorCallback(huart);
}

/* ------------------- Formateo rapido (sin sprintf/%f) -------------------- */
static char *put_str(char *p, const char *s)
{
    while (*s) *p++ = *s++;
    return p;
}

static char *put_u32(char *p, uint32_t v)
{
    char t[10];
    int n = 0;
    do { t[n++] = (char)('0' + v % 10U); v /= 10U; } while (v);
    while (n) *p++ = t[--n];
    return p;
}

/* Numero con 'dec' decimales (0..6), redondeado. Equivale a "%.<dec>f". */
static char *put_fixed(char *p, float v, uint8_t dec)
{
    static const uint32_t p10[7] = {1U, 10U, 100U, 1000U, 10000U, 100000U, 1000000U};
    if (dec > 6) dec = 6;
    if (v != v) v = 0.0f;                         /* NaN */
    uint8_t neg = (v < 0.0f);
    if (neg) v = -v;
    float lim = 4.0e9f / (float)p10[dec];         /* evita desbordar uint32 */
    if (v > lim) v = lim;

    uint32_t x = (uint32_t)(v * (float)p10[dec] + 0.5f);
    if (neg && x) *p++ = '-';
    p = put_u32(p, x / p10[dec]);
    if (dec) {
        uint32_t f = x % p10[dec];
        *p++ = '.';
        for (int i = dec - 1; i >= 0; i--) { p[i] = (char)('0' + f % 10U); f /= 10U; }
        p += dec;
    }
    return p;
}

/* Mismo formato que antes:
 * E,e1,e2;V,v;I,i1,i2;S,err,mod,ac,A,ax,ay,az,G,gx,gy,gz,O,q0,q1,q2,q3,F,f\r\n */
static uint16_t Build_Frame(char *b, const MD49_Data_t *m, const MTi1_Data_t *imu, float freq)
{
    char *p = b;
    p = put_str(p, "E,");  p = put_fixed(p, m->vel1_pps, 2); *p++ = ','; p = put_fixed(p, m->vel2_pps, 2);
    p = put_str(p, ";V,"); p = put_fixed(p, m->volts, 2);
    p = put_str(p, ";I,"); p = put_fixed(p, m->i1, 2); *p++ = ','; p = put_fixed(p, m->i2, 2);
    p = put_str(p, ";S,"); p = put_u32(p, m->t_err); *p++ = ','; p = put_u32(p, m->mode);
    *p++ = ',';            p = put_u32(p, m->accel);
    p = put_str(p, ",A,"); p = put_fixed(p, imu->acc_x, 3); *p++ = ',';
                           p = put_fixed(p, imu->acc_y, 3); *p++ = ',';
                           p = put_fixed(p, imu->acc_z, 3);
    p = put_str(p, ",G,"); p = put_fixed(p, imu->gyr_x, 3); *p++ = ',';
                           p = put_fixed(p, imu->gyr_y, 3); *p++ = ',';
                           p = put_fixed(p, imu->gyr_z, 3);
    p = put_str(p, ",O,"); p = put_fixed(p, imu->q0, 3); *p++ = ',';
                           p = put_fixed(p, imu->q1, 3); *p++ = ',';
                           p = put_fixed(p, imu->q2, 3); *p++ = ',';
                           p = put_fixed(p, imu->q3, 3);
    p = put_str(p, ",F,"); p = put_fixed(p, freq, 1);
    p = put_str(p, "\r\n");
    return (uint16_t)(p - b);
}

/* --------------------------- Envio de la trama --------------------------- */
static void Send_Frame(void)
{
    static uint32_t last_cyc = 0;
    static uint8_t  first = 1;

    /* Frecuencia real entre tramas medida con el contador de ciclos */
    uint32_t cyc = DWT->CYCCNT;
    float freq = 0.0f;
    if (!first && cyc != last_cyc) freq = (float)SystemCoreClock / (float)(cyc - last_cyc);
    first = 0;
    last_cyc = cyc;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;

    char *b = tx_buf[tx_idx];
    uint16_t len = Build_Frame(b, MD49_GetData(), MTi1_GetData(), freq);

    if (CDC_Transmit_FS((uint8_t *)b, len) == USBD_OK) {
        tx_idx ^= 1U;            /* el proximo frame se arma en el otro buffer */
        frames_sent++;
    } else {
        frames_dropped++;        /* USB ocupado: se descarta, nunca se bloquea */
    }
}

/* ------------------------ Comandos desde el PC --------------------------- */
static void Process_USB_Command(void)
{
    if (!string_ready) return;

    char cmd[sizeof(Buffer_rx)];
    __disable_irq();                         /* copia atomica (~1 us) */
    for (uint32_t i = 0; i < sizeof(cmd); i++) cmd[i] = Buffer_rx[i];
    string_ready = 0;
    __enable_irq();
    cmd[sizeof(cmd) - 1] = '\0';

    /* Formato: <letra><vel_r>,<vel_l>   p.ej. "S128,200" (base 10) */
    char *end1, *end2;
    long r = strtol(&cmd[1], &end1, 10);
    if (end1 != &cmd[1] && *end1 == ',') {
        long l = strtol(end1 + 1, &end2, 10);
        if (end2 != end1 + 1 && r >= 0 && r <= 255 && l >= 0 && l <= 255) {
            vel_r = (uint8_t)r;
            vel_l = (uint8_t)l;
            MD49_RequestSpeed(vel_r, vel_l);
            last_comm_time = HAL_GetTick();
            return;
        }
    }

    /* Comando invalido: avisar solo si el USB esta libre */
    char *p = put_str(err_buf, "error: bad command\r\n");
    CDC_Transmit_FS((uint8_t *)err_buf, (uint16_t)(p - err_buf));
}

/* ----------------------------- Seguridad --------------------------------- */
static void Safety_Check(uint32_t now)
{
    static uint32_t last_stop = 0;
    uint8_t lost = (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED);

#if (CMD_TIMEOUT_MS > 0)
    if ((now - last_comm_time) > CMD_TIMEOUT_MS) lost = 1;
#endif

    if (lost && (now - last_stop) >= STOP_RESEND_MS) {
        last_stop = now;
        vel_r = MOTOR_STOP;
        vel_l = MOTOR_STOP;
        MD49_RequestSpeed(vel_r, vel_l);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  MD49_Init(&huart2);      /* bloqueante solo aqui: reset de encoders e info */
  HAL_Delay(200);
  MTi1_Init(&huart1);      /* configura la IMU y arranca la recepcion por DMA */

  uint32_t next_frame_ms = HAL_GetTick() + FRAME_PERIOD_MS;
  uint32_t last_led_tick = HAL_GetTick();
  last_comm_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    	uint32_t now = HAL_GetTick();

    	    /* Tareas no bloqueantes: cada una retorna en microsegundos */
    	    MTi1_ProcessRx();
    	    MD49_Task();
    	    Process_USB_Command();
    	    Safety_Check(now);

    	    /* Trama general: se envia al llegar cada paquete de la IMU (100 Hz).
    	       Si la IMU deja de enviar, se usa un temporizador propio de 10 ms. */
    	    uint8_t imu_new   = MTi1_ConsumeNewData();
    	    uint8_t imu_alive = (now - MTi1_LastPacketTick()) < IMU_STALE_MS;

    	    if (imu_new) {
    	        Send_Frame();
    	        next_frame_ms = now + FRAME_PERIOD_MS;
    	    } else if (!imu_alive && (int32_t)(now - next_frame_ms) >= 0) {
    	        Send_Frame();
    	        next_frame_ms += FRAME_PERIOD_MS;
    	        if ((int32_t)(now - next_frame_ms) >= 0) next_frame_ms = now + FRAME_PERIOD_MS;
    	    }

    	    if ((now - last_led_tick) >= LED_PERIOD_MS) {
    	        last_led_tick += LED_PERIOD_MS;
    	        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	        HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);
    	    }
    	  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Led1_Pin|Led2_Pin|Led3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Led1_Pin Led2_Pin Led3_Pin */
  GPIO_InitStruct.Pin = Led1_Pin|Led2_Pin|Led3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
