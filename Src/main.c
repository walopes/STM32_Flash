/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_flash.h"

/////////// AHAHAHAHAHA /////////////////
//
// OBS.: FAZER UMA FUNCAO ERASE(), PARA APAGAR A MEMORIA
// BASICAMENTE UM WRITE(), MAS ESCREVE 0 NAS POSICOES QUE FORAM UTILIZADAS
//
////////////////////////////////////////

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define MAPSIZE 4
#define WRITE_FLASH 1
#define ERASE_FLASH 0

//uint32_t startAdd = 0x8040000;
//uint32_t startAdd = 0x080000FF; // Comecando em 32K da Flash

// Inicio da posicao na memoria Flash permitida para a escrita do usuario
uint32_t startAdd = 0x08008000; // Comecando em 32K da Flash

typedef struct{
	uint16_t EncoderA_right;
	uint16_t EncoderB_right;
	uint16_t EncoderA_left;
	uint16_t EncoderB_left;
}Enc;

typedef struct{
	Enc Mapping[MAPSIZE];
	uint16_t qtdOfMarcs;
}Map;

void initMap(Map *E)
{
	E->qtdOfMarcs = 0;

	uint16_t x = 0;
	while(x < MAPSIZE)
	{
		E->Mapping[x].EncoderA_right = 0;
		E->Mapping[x].EncoderA_left = 0;
		E->Mapping[x].EncoderB_right = 0;
		E->Mapping[x].EncoderB_left = 0;
		x++;
	}
}

typedef struct{
	uint16_t Val;
	uint16_t Tes;
}Teste;
//
//Teste teste[30];
//
Teste Memoria[30];


void writeFlash(uint16_t identifier, Map *myMap)
{
  uint32_t i, j;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Unlock the Options Bytes *************************************************/
  HAL_FLASH_OB_Unlock();

  // Clear the flags set in the unlocking
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

//  /*******************************************************/
  // Create a Handler to manage the Flash
  FLASH_EraseInitTypeDef Erase;

  Erase.TypeErase = FLASH_TYPEERASE_PAGES;
  //Erase.PageAddress = FLASH_BASE;
  Erase.PageAddress = startAdd;
  Erase.NbPages = 1;

  		//.PageAddress = startAddress;
  		//.NbSectors = 1,
  		//.VoltageRange = FLASH_VOLTAGE_RANGE_3
  		//};

  // Erase the entire page before the data can be written
  uint32_t pageError = 0;
  HAL_StatusTypeDef re = HAL_FLASHEx_Erase(&Erase,&pageError);


  ////////////////  GRAVACAO NA FLASH

  // Utilizado para preencher todo os espaco na Flash com 0s
  uint16_t zero = 0;


  // Primeiro a ser gravado eh a quantidade de marcacoes
  if(identifier == WRITE_FLASH) HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd),myMap->qtdOfMarcs);
  else HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd),zero);

  // Write the mapping in the Flash
  j=i=1;
  //for(i=0;i<30;i++)
  while(i < myMap->qtdOfMarcs)
  {
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd + 4*j),myMap->Mapping[i].EncoderA_right);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd + 4*(j+1)),myMap->Mapping[i].EncoderA_left);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd + 4*(j+2)),myMap->Mapping[i].EncoderB_right);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd + 4*(j+3)),myMap->Mapping[i].EncoderB_left);
	  j=i+4;
	  i++;
		  //HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd + 2*i*30),teste[i].Val);
		  //HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(FLASH_BASE+i*sizeof(uint16_t)),teste[i].Val);
  }

  ////////////////////

	// Lock the Flash again.
	HAL_FLASH_Lock();
}

//void ReadFlash(void)
//{
//	uint32_t i,j;
//	for(i=0;i<30;i++)
//	{
//		//teste[i].Val = *(uint16_t *)(startAdd + 2*i*30);
//		Memoria[i].Val = *(uint16_t *)(startAdd + 2*i*30);
//		//teste[i] = (Teste)0x080E0000;
//	}
//
//}

void ReadFromFlash(Map *myMap)
{
	uint32_t i,j;

	// Primeiro e feito a leitura da quantidade de marcacoes
	myMap->qtdOfMarcs = *(uint16_t *)(startAdd);

	// Leitura das marcacoes
	  i=0;
	  //for(i=0;i<30;i++)
	  while(i < myMap->qtdOfMarcs)
	  {
		  myMap->Mapping[i].EncoderA_right = *(uint16_t *)(startAdd + 4*(j+1));
		  myMap->Mapping[i].EncoderA_left = *(uint16_t *)(startAdd + 4*(j+2));
		  myMap->Mapping[i].EncoderB_right = *(uint16_t *)(startAdd + 4*(j+3));
		  myMap->Mapping[i].EncoderB_left = *(uint16_t *)(startAdd + 4*(j+4));
//
//		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd + 4*j),myMap->Mapping[i].EncoderA_right);
//		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd + 4*(j+1)),myMap->Mapping[i].EncoderA_left);
//		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd + 4*(j+2)),myMap->Mapping[i].EncoderB_right);
//		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(startAdd + 4*(j+3)),myMap->Mapping[i].EncoderB_left);
		  j=i+5;
		  i++;
	  }

}


/* USER CODE END PFP */
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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

  /* USER CODE BEGIN 2 */


  ////////// NEW

  Map myMap, myMap2;
  int x;

  myMap.qtdOfMarcs = 4;

  initMap(&myMap);
  initMap(&myMap2);


  myMap.qtdOfMarcs = 4;

 // ReadFlash();

//  for(x=0;x<4;x++)
//  {
//	  myMap.Mapping[x].EncoderA_right = 1000+x;
//	  myMap.Mapping[x].EncoderB_right = 2000+x;
//	  myMap.Mapping[x].EncoderA_left = 3000+x;
//	  myMap.Mapping[x].EncoderB_left = 4000+x;
//  }

 // writeFlash(WRITE_FLASH,&myMap);

  ReadFromFlash(&myMap2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
	  HAL_Delay(1000);
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
