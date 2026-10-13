/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Leitura do MTi-1 (ACC, GYR e Euler Angles)
  * Author          : juandg
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float acc_x, acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
    float roll, pitch, yaw;
} MTi1_Data_t;
/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define RX_RING_SIZE 2048
volatile uint8_t rx_ring[RX_RING_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;
uint8_t rx_byte;

char msg_usb[300];
MTi1_Data_t imu_data;
int test_flag = 0;
uint32_t last_led_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
float bytes_to_float_be(const uint8_t *buffer);
uint8_t xsens_validate_checksum(const uint8_t *msg, uint16_t length);
uint8_t calculate_xbus_checksum(const uint8_t *buffer, uint16_t length);
static void parse_mtdata2(const uint8_t *packet, uint16_t packet_len);
void configure_mti1_outputs(void);
void process_rx_ring(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Converte 4 bytes Big-Endian para float Little-Endian
float bytes_to_float_be(const uint8_t *buffer) {
    uint32_t b = ((uint32_t)buffer[0] << 24) |
                 ((uint32_t)buffer[1] << 16) |
                 ((uint32_t)buffer[2] << 8)  |
                  (uint32_t)buffer[3];
    float f;
    memcpy(&f, &b, sizeof(float));
    return f;
}

// Valida Checksum Xbus
uint8_t xsens_validate_checksum(const uint8_t *msg, uint16_t length) {
    uint8_t sum = 0;
    for (uint16_t i = 1; i < length; i++) {
        sum += msg[i];
    }
    return (sum == 0);
}

uint8_t calculate_xbus_checksum(const uint8_t *buffer, uint16_t length) {
    uint8_t sum = 0;
    for (uint16_t i = 1; i < (length - 1); i++) {
        sum += buffer[i];
    }
    return (uint8_t)(0x100 - sum);
}

static void parse_mtdata2(const uint8_t *packet, uint16_t packet_len) {
    if (packet_len < 5) return;
    if (packet[0] != 0xFA || packet[2] != 0x36) return;

    // ¡NUEVO! Validar el checksum antes de confiar en los datos
    if (!xsens_validate_checksum(packet, packet_len)) {
        return; // Si el checksum falla, descartamos el paquete corrupto inmediatamente
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
        else if ((xdi & 0xFFF0) == 0x2030) { // EULER
            imu_data.roll  = bytes_to_float_be(&data_ptr[0]);
            imu_data.pitch = bytes_to_float_be(&data_ptr[4]);
            imu_data.yaw   = bytes_to_float_be(&data_ptr[8]);
            //imu_data.test_flag = 1;
        }

        idx += 3 + data_size;
    }
}

// Processa o Ring Buffer identificando início de pacote (0xFA)
void process_rx_ring(void) {
    // Enquanto houver bytes suficientes para formar um pacote mínimo Xbus (5 bytes)
    while (rx_head != rx_tail) {
        // 1. Procura o Preamble 0xFA
        if (rx_ring[rx_tail] != 0xFA) {
            rx_tail = (rx_tail + 1) % RX_RING_SIZE;
            continue;
        }

        // 2. Calcula quantos bytes temos disponíveis no Ring Buffer
        uint16_t available = (rx_head >= rx_tail) ? (rx_head - rx_tail) : (RX_RING_SIZE - rx_tail + rx_head);
        if (available < 5) break; // Espera chegar o cabeçalho completo

        // 3. Lê o tamanho do Payload (byte na posição rx_tail + 3)
        uint16_t len_idx = (rx_tail + 3) % RX_RING_SIZE;
        uint8_t payload_len = rx_ring[len_idx];
        uint16_t total_len = 5 + payload_len; // Preamble(1) + BusID(1) + MID(1) + Len(1) + Payload + CS(1)

        // Se o pacote for maior que o buffer auxiliar, avança 1 byte para não travar
        if (total_len > 256) {
            rx_tail = (rx_tail + 1) % RX_RING_SIZE;
            continue;
        }

        // Aguarda todos os bytes do pacote chegarem
        if (available < total_len) break;

        // 4. Copia o pacote completo para um buffer temporário
        uint8_t packet_buf[256];
        for (uint16_t i = 0; i < total_len; i++) {
            packet_buf[i] = rx_ring[(rx_tail + i) % RX_RING_SIZE];
        }

        // 5. Processa o pacote se for uma mensagem de dados MTData2 (MID = 0x36)
        if (packet_buf[2] == 0x36) {
            parse_mtdata2(packet_buf, total_len);
        }

        // 6. Descarta o pacote processado do Ring Buffer
        rx_tail = (rx_tail + total_len) % RX_RING_SIZE;
    }
}

// Configura saídas do MTi-1
void configure_mti1_outputs(void) {
    uint8_t goto_config[] = {0xFA, 0xFF, 0x30, 0x00, 0x00};
    goto_config[4] = calculate_xbus_checksum(goto_config, sizeof(goto_config));
    HAL_UART_Transmit(&huart1, goto_config, sizeof(goto_config), 200);
    HAL_Delay(100);

    uint8_t set_output_config[] = {
        0xFA, 0xFF, 0xC0, 0x0C,       // Header
        0x40, 0x20, 0x00, 0x64,       // ACC @ 100Hz
        0x80, 0x20, 0x00, 0x64,       // GYR @ 100Hz
        0x20, 0x30, 0x00, 0x64,       // Euler @ 100Hz
        0x00                          // Checksum
    };
    set_output_config[sizeof(set_output_config) - 1] = calculate_xbus_checksum(set_output_config, sizeof(set_output_config));

    HAL_UART_Transmit(&huart1, set_output_config, sizeof(set_output_config), 200);
    HAL_Delay(100);

    uint8_t goto_measurement[] = {0xFA, 0xFF, 0x10, 0x00, 0x00};
    goto_measurement[4] = calculate_xbus_checksum(goto_measurement, sizeof(goto_measurement));
    HAL_UART_Transmit(&huart1, goto_measurement, sizeof(goto_measurement), 200);
    HAL_Delay(100);
}

// Interrupção da UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uint16_t next = (rx_head + 1) % RX_RING_SIZE;
        if (next != rx_tail) {
            rx_ring[rx_head] = rx_byte;
            rx_head = next;
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Limpa os registradores de erro da UART
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);

        // Reinicia a recepção por interrupção
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  HAL_Delay(200);

  configure_mti1_outputs();

  uint32_t last_loop_tick = 0;
  float current_freq = 0.0f;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //Processa dados da UART sem bloquear
    process_rx_ring();

    //Transmissão USB controlada (a cada 20ms = 50 Hz)
    static uint32_t last_usb_tick = 0;
		if (HAL_GetTick() - last_usb_tick >= 10) {

			uint32_t now = HAL_GetTick();
			uint32_t dt = now - last_loop_tick;
			last_loop_tick = now;

			// Cálculo de la frecuencia real del bucle en Hz
			if (dt > 0) {
				current_freq = 1000.0f / (float) dt;
			}
			last_usb_tick = HAL_GetTick();

			int pos =
					sprintf(msg_usb,
							"ACC: [%.4f, %.4f, %.4f] | GYR: [%.4f, %.4f, %.4f] | EULER: [R: %.4f, P: %.4f, Y: %.4f] | test_flag: %.2f\r\n",
							imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
							imu_data.gyr_x, imu_data.gyr_y, imu_data.gyr_z,
							imu_data.roll, imu_data.pitch, imu_data.yaw,
							current_freq);

			CDC_Transmit_FS((uint8_t*) msg_usb, pos);
		}

    // Rotina piscar LED (Blink a cada 200ms)
    if (HAL_GetTick() - last_led_tick >= 200) {
        last_led_tick = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
  }
  /* USER CODE END 3 */
}

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

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
