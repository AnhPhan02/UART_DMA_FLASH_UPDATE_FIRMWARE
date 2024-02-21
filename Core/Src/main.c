/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
volatile char rx_buff[5820] = {0};
volatile char recv_data_done = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_INIT()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	uint32_t* MODER = (uint32_t*)(0x40020000);
	*MODER |= (0b10 << 4) | (0b10 << 6);

	uint32_t* AFRL = (uint32_t*)(0x40020000 + 0x20);
	*AFRL |= (0b0111 << 8) | (0b0111 << 12);

	uint32_t* BRR = (uint32_t*)(0x40004400 + 0x08);
	*BRR = (104 << 4) | 3;

	uint32_t* CR1 = (uint32_t*)(0x40004400 +0x0C);
	uint32_t* CR3 = (uint32_t*)(0x40004400 + 0x14);
	*CR3 |= (1 << 6);

	*CR1 |= (1 << 2) | (1 << 3) | (1 << 13);
}

void UART_Send(char data)
{
	uint32_t* SR = (uint32_t*)(0x40004400);
	uint32_t* DR = (uint32_t*)(0x40004400 + 0x04);

	*DR = data;
	while(((*SR >> 6)&1) != 1);
	*SR &= ~(1 << 6);
	while(((*SR >> 7)&1) != 1);
}

void UART_Log(char* msg)
{
	int len_msg = strlen(msg);
	for(int i = 0; i < len_msg; i++)
	{
		UART_Send(msg[i]);
	}
}

void DMA_init()
{
	__HAL_RCC_DMA1_CLK_ENABLE();

	uint32_t* S5CR = (uint32_t*)(0x40026000 + 0x10 + 0x18 * 5);
	*S5CR &= ~1;

	uint32_t* S5PAP = (uint32_t*)(0x40026000 + 0x18 + 0x18 * 5);
	*S5PAP = (0x40004404);

	uint32_t* S5M0AR = (uint32_t*)(0x40026000 + 0x1C + 0x18 * 5);
	*S5M0AR = (uint32_t)rx_buff;

	uint32_t* S5NDTR = (uint32_t*)(0x40026000 + 0x14 + 0x18 * 5);
	*S5NDTR = sizeof(rx_buff);

	*S5CR |= (0b100 << 25) | (1 << 10) | (1 << 8) | (1 << 4) | 1;

	//NVIC DMA1_S5
	uint32_t* ISER0 = (uint32_t*)(0xE000E100);
	*ISER0 |= (1 << 16);
}

void DMA1_Stream5_IRQHandler()
{
	uint32_t* HIFCR_CTCIF5 = (uint32_t*)(0x40026000 + 0x0C);
	*HIFCR_CTCIF5 |= (1 << 11);
	recv_data_done = 1;
}
#define FLASH_ADD_BASE	0x40023C00
__attribute__((section(".FuncInRam")))int Erase_sector(int sector_num)
{
	if((sector_num < 0 ) || (sector_num > 8))
		return -1;
	//1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
	//FLASH_SR register
	uint32_t* SR = (uint32_t*)(FLASH_ADD_BASE + 0x0C);
	while(((*SR >> 16)&1) == 1);

	//2. Set the SER bit and select the sector out of the 7 sectors (STM32F411xC/E) in the
	//main memory block you wish to erase (SNB) in the FLASH_CR register
	uint32_t* CR = (uint32_t*)(FLASH_ADD_BASE + 0x10);
	if(((*CR >> 31)&1) == 1)
	{
		uint32_t* KEYR = (uint32_t*)(FLASH_ADD_BASE + 0x04);

		*KEYR = 0x45670123;
		*KEYR = 0xCDEF89AB;
	}
	*CR |= (1 << 1);
	*CR |= (sector_num << 3);

	//3. Set the STRT bit in the FLASH_CR register
	*CR |= (1 << 16);
	//4. Wait for the BSY bit to be cleared
	while(((*SR >> 16)&1) == 1);

	*CR &= ~(1 << 1);
	return 0;
}

__attribute__((section(".FuncInRam")))void Programming(char *add,char *data, int data_len)
{
	//1. Check that no main Flash memory operation is ongoing by checking the BSY bit in the
	//FLASH_SR register.
	uint32_t* SR = (uint32_t*)(FLASH_ADD_BASE + 0x0C);
		while(((*SR >> 16)&1) == 1);
	//2. Set the PG bit in the FLASH_CR register
		uint32_t* CR = (uint32_t*)(FLASH_ADD_BASE + 0x10);
			if(((*CR >> 31)&1) == 1)
			{
				uint32_t* KEYR = (uint32_t*)(FLASH_ADD_BASE + 0x04);

				*KEYR = 0x45670123;
				*KEYR = 0xCDEF89AB;
			}
		*CR |= (1 << 0);
	//3. Perform the data write operation(s) to the desired memory address (inside main
	//memory block or OTP area):
		memcpy(add, data, data_len);
	//4. Wait for the BSY bit to be cleared.
		while(((*SR >> 16)&1) == 1);
		*CR &= ~(1 << 0);
}

__attribute__((section(".FuncInRam")))void reset_system()
{
	//application interrupts and Reset control
	uint32_t* AIRCR = (uint32_t*)(0xE000ED0C);
	*AIRCR = (0x5FA << 16) | (1 << 2);

}

__attribute__((section(".FuncInRam")))void update_firmware()
{
	  Erase_sector(0);
	  Programming((char*)0x08000000, (char*)rx_buff, sizeof(rx_buff));
	  reset_system();
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
//  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
//  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  UART_INIT();
  DMA_init();
  UART_Log("Hello\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(recv_data_done == 1)
	  {
		  update_firmware();
	  }
    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

#ifdef  USE_FULL_ASSERT
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
