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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
char msg_usb[300];

char Buffer_rx[64];
uint8_t string_ready = 0;
uint8_t vel_r = 128, vel_l = 128;

int cnt = 0;
uint32_t last_comm_time = 0;
uint32_t last_led_tick = 0;
uint8_t rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void Safety_Check_Communication(void);
float sanitize_val(float val, float min_limit, float max_limit, float default_val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        MTi1_RxCpltCallback(huart, rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        MTi1_ErrorCallback(huart);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void Safety_Check_Communication(void) {
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        vel_r = 128;
        vel_l = 128;
        MD49_SetSpeedMotors(&huart2, &vel_r, &vel_l);
    }
}

float sanitize_val(float val, float min_limit, float max_limit, float default_val) {
    if (val < min_limit || val > max_limit) {
        return default_val; // Retorna un valor seguro si hay corrupción
    }
    return val;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_USB_DEVICE_Init();
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */

    uint32_t last_loop_tick = 0;
   	uint32_t last_motor_tick = 0;
   	uint32_t last_current_tick = 0;
   	uint32_t last_info_tick = 0;
   	uint32_t last_imu_tick = 0;
   	uint32_t last_led_tick = 0;
   	float current_freq = 0.0f;

   	uint8_t ac = 0, ver = 0, mod = 0, t_err = 0;
   	float enc1, enc2, v = 0.0f, i1 = 0.0f, i2 = 0.0f;

    MD49_ExCmd(&huart2, MD49_CMD_RESET_ENCODERS, NULL, 0, NULL, 0);
    MD49_GetInfo(&huart2, &ver, &mod, &t_err, &v, &ac);


    // Inicialización de la IMU a través de la librería
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    HAL_Delay(200);
    MTi1_Init(&huart1);
    /* USER CODE END 2 */


    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    	if (HAL_GetTick() - last_loop_tick >= 5) {

			uint32_t now = HAL_GetTick();
			uint32_t dt = now - last_loop_tick;
			last_loop_tick = now;
			if (dt > 0) current_freq = 1000.0f / (float) dt;

			MTi1_ProcessRx();
			MTi1_Data_t *imu = MTi1_GetData();


			if (string_ready) {
				int temp_r, temp_l;
				last_comm_time = HAL_GetTick();
				int n_var = sscanf((char*) &Buffer_rx[1], "%i,%i", &temp_r, &temp_l);
				memset(Buffer_rx, 0, sizeof(Buffer_rx));
				string_ready = 0;

				if (n_var == 2) {
					vel_r = (uint8_t) temp_r;
					vel_l = (uint8_t) temp_l;
					MD49_SetSpeedMotors(&huart2, &vel_r, &vel_l);
				} else {
					sprintf(msg_usb, "error: n_var read are %d\r\n", n_var);
					CDC_Transmit_FS((uint8_t*) msg_usb, strlen(msg_usb));
				}
			}


			if (HAL_GetTick() - last_motor_tick >= 45) {
				last_motor_tick = HAL_GetTick();
				MD49_GetVelocity(&huart2, &enc1, &enc2);
			}

			if (HAL_GetTick() - last_current_tick >= 100) {
				last_current_tick = HAL_GetTick();
				MD49_GetCurrent(&huart2, &i1, &i2);
				Safety_Check_Communication();
			}

			if (HAL_GetTick() - last_info_tick >= 10000) {
				last_info_tick = HAL_GetTick();
				MD49_GetInfo(&huart2, &ver, &mod, &t_err, &v, &ac);
			}

			if (HAL_GetTick() - last_imu_tick >= 10) {
				uint32_t now = HAL_GetTick();
				uint32_t dt_imu = now - last_imu_tick;
				last_imu_tick = now;
				if (dt_imu > 0) current_freq = 1000.0f / (float) dt_imu;

				int pos = sprintf(msg_usb,
						"E,%f,%f;V,%.3f;I,%.3f,%.3f;S,%d,%d,%d,A,%.3f,%.3f,%.3f,G,%.3f,%.3f,%.3f,O,%.3f,%.3f,%.3f,F,%.1f\r\n",
						//"E,%f,%f;V,%f;I,%f,%f;S,%d,%d,%d,A,%.4f,%.4f,%.4f,G,%.4f,%.4f,%.4f,O,%.4f,%.4f,%.4f,F,%.1f\r\n",
						enc1, enc2, v, i1, i2, t_err, mod, ac,
						imu->acc_x, imu->acc_y, imu->acc_z,
						imu->gyr_x, imu->gyr_y, imu->gyr_z,
						imu->roll, imu->pitch, imu->yaw,
						current_freq);

				CDC_Transmit_FS((uint8_t*) msg_usb, pos);
			}

			if (HAL_GetTick() - last_led_tick >= 200) {
				last_led_tick = HAL_GetTick();
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
			}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
