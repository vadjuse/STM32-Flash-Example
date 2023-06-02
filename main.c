/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal_flash.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAXLINE 100
#define PROGPAGE 31

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char line[MAXLINE];
uint8_t last_index = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void portA_init();
void USART_init();
void USART2_IRQHandler();
void line_init();
void line_to_usart();
void line_to_flash();
void byte_to_usart(char byte);

void flash_unlock();
void flash_erase(uint8_t page_number);
void flash_program(uint8_t page_number, uint16_t page_byte, uint16_t Data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void portA_init()
{
	  RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;			// Port A clock enable
	  GPIOA->MODER |= 1 << GPIO_MODER_MODER5_Pos;		// Pin5 (Green LED)i n output mode
	  GPIOA->ODR   |= GPIO_PIN_5;				// LED Set

	  GPIOA->MODER |= 0b10 << GPIO_MODER_MODER2_Pos
		       |  0b10 << GPIO_MODER_MODER3_Pos; 	// Alternate Function
	  GPIOA->AFR[0]|= 0b111<< GPIO_AFRL_AFRL2_Pos
		       |  0b111<< GPIO_AFRL_AFRL3_Pos; 		// PA2 PA3 USART2 TX & RX
}
void USART_init()
{
	//APBCLK - 32 MHz
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;			// USART2 clock enable
	USART2->BRR = 3333; 					//baud 9600				
	USART2->CR1|= USART_CR1_TE				// Transmite enable
			   |  USART_CR1_RXNEIE			// Rx not empty interrupt enable
			   |  USART_CR1_RE			// Rx enable
			   |  USART_CR1_UE; 			// USART2 enable

	NVIC_EnableIRQ(USART2_IRQn);				// IRQ enable
}

void USART2_IRQHandler()
{
	if((USART2->ISR & USART_ISR_RXNE) != 0)
	{
		USART2->RQR |= USART_RQR_RXFRQ; //clear pending bit
		int c = USART2->RDR;
		GPIOA->ODR ^= 1 << 5;
		line[last_index++] = c;
		if(c == '\r')
		{
			line_to_usart();
			flash_erase(31);
			line_to_flash();
			line_init();
		}
	}
}

void line_init()
{
	for(int i = 0; i < MAXLINE; i++)
		line[i] = '\0';
	last_index = 0;
}

void line_to_usart()
{
	int i = 0;
	while(line[i] != '\0')
		byte_to_usart(line[i++]);
}
void byte_to_usart(char byte)
{
	USART2->TDR = byte;
	while((USART2->ISR & USART_ISR_TC) == 0);
}

void flash_unlock()
{
	//Unlocking Combination
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
}

void flash_erase(uint8_t page_number) //page number from 0 to 31, but be careful
{
	// Erase of flash page
	while((FLASH->SR & FLASH_SR_BSY) !=  0); 	//wait while flash is busy
	FLASH->CR |= FLASH_CR_PER; 			//Page Erase
	uint32_t Address =  0x08000000 + 0x800*page_number;
	FLASH->AR = Address;    			//Adress of page to erase
	FLASH->CR |= FLASH_CR_STRT;			//Start of Operation
	while((FLASH->SR & FLASH_SR_BSY) !=  0); 	//wait while flash is busy
	if((FLASH->SR & FLASH_SR_EOP) != 0)
	{
		FLASH->SR |= FLASH_SR_EOP; //reset of EOP flag
		GPIOA->ODR &= ~(1<<5);
	}
}

void flash_program(uint8_t page_number, uint16_t page_byte, uint16_t Data)
{
	//Programming of previously Erased memory
	while((FLASH->SR & FLASH_SR_BSY) !=  0); 	//wait while flash is busy
	FLASH->CR &= ~FLASH_CR_PER; 			//Page Erase
	FLASH->CR |= FLASH_CR_PG; 			//Page Programming operation
	uint32_t Address = 0x08000000 + 0x800*page_number + page_byte;
	//this code copied from FLASH_WriteHalfWord
	*(__IO uint16_t*)Address = (uint16_t)Data;  	//Programming 16-bit HalfWord Data to Flash
	///////////////////////////////////////////
	while((FLASH->SR & FLASH_SR_BSY) !=  0); 	//wait while flash is busy
	if((FLASH->SR & FLASH_SR_EOP) != 0)
	{
		FLASH->SR |= FLASH_SR_EOP; 		//reset of EOP flag
		GPIOA->ODR ^= 1<<5;
	}
}

void line_to_flash()
{
	uint16_t i = 0;
	uint16_t Data = 0;
	flash_erase(PROGPAGE); //erase 31th page of flash memory
	while(line[i] != '\0')
	{
		Data = ((uint8_t)line[i] << 8);// + (uint8_t)line[i+1];
		if((uint8_t)line[i+1]!= '\0')
			Data += (uint8_t)line[i+1];
		else
			Data += 0xFF;
		flash_program(PROGPAGE, i, Data);
		i += 2; //Writing in Flash requires 16bit HalfWords
	}
}
void init_from_flash(uint8_t page_number)
{
	uint16_t b;
	uint32_t Address = 0x08000000 + 0x800*page_number;
	for(int i = 0; i < MAXLINE; i++)
	{
		b = *(__IO uint32_t*)(Address + 2*i);
		if(b >> 8 != 0xFF)
		{
			byte_to_usart(b>>8);
			if((b & 0xFF) != 0xFF)
				byte_to_usart(b & 0xFF);
		}
		else
		{
			if((b & 0xFF) != 0xFF)
				byte_to_usart(b & 0xFF);
			else
				break;
		}
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
  /* USER CODE BEGIN 2 */
  portA_init();
  USART_init();
  line_init();
  HAL_Delay(1000);
  flash_unlock();
  init_from_flash(PROGPAGE);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
