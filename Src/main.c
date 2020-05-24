
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "Ethernet.h" 
#include "template.h" 

#define DEBUG debug()
#define payload_error  11
#define crc_error 12
#define frame_size_error 13
#define clear 14

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int ifg_tx_flag=1;
int ifg_rx_flag=0;
uint32_t crc =0; ;
int ready_for_llc=0;
static Ethernet_res frame_build ;


uint8_t myMAC[] ={0xcc,0xcc,0xcc,0xcc,0xcc,0xcc}; 	//Our MAC address





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//for debug
	uint8_t tx_data_wire = 0;
	uint8_t tx_vld_wire = 0;
	uint8_t phy_clk_wire = 0;
	uint8_t phy_busy_wire = 0;
	uint8_t phy_reset_wire = 0; 
	uint8_t rx_data_wire = 0;
	uint8_t rx_vld_wire = 0;
	static int payload_size=0;
	
void debug(void)
{
	 tx_data_wire = (uint8_t)((Phy_tx_data_bus_pin0_GPIO_Port->IDR) & 0x0FF);
	 tx_vld_wire = (uint8_t)HAL_GPIO_ReadPin(Phy_tx_valid_GPIO_Port, Phy_tx_valid_Pin);
	 phy_clk_wire = (uint8_t)HAL_GPIO_ReadPin(Phy_clock_GPIO_Port, Phy_clock_Pin);
	 phy_busy_wire = (uint8_t)HAL_GPIO_ReadPin(Phy_tx_busy_GPIO_Port, Phy_tx_busy_Pin);
	 phy_reset_wire = (uint8_t)HAL_GPIO_ReadPin(Phy_reset_GPIO_Port, Phy_reset_Pin);
	 rx_data_wire = (uint8_t)((Phy_rx_data_bus_pin0_GPIO_Port->IDR)>>8);
	 rx_vld_wire = (uint8_t)HAL_GPIO_ReadPin(Phy_rx_valid_GPIO_Port, Phy_rx_valid_Pin);
}
	//end debug



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* Dll Rx functions */
extern uint8_t isRxByteReady(void);									//check if there is new byte received from phy.
extern uint8_t getByte(void); 											//get byte from phy, it's mandatory to use "isRxByteReady()" before calling this function.

/* Dll Tx functions */
extern uint8_t isPhyTxReady(void);									//check if the phy_Tx ready to get another byte to send.
extern uint8_t sendByte(uint8_t data); 							//sent byte to phy, it's mandatory to use "isPhyTxReady()" before call this function
extern uint8_t isNewTxRequest(void); 								//check if the upper_layer sent us new data to transmit.
extern Ethernet_req* getTxRequeset(void);						//get from upper_layer data to transmit, it's mandatory to use "isNewTxRequest()" before call this function.
//Important! - after finish using the Ethernet_req struct, you have to free it and also free the Ethernet_req.payload.
	void Mac_Tx()
	{
		static int j=0;
		static uint8_t *frame;
		static int flagi=0;
		static int size=0;
		static int i=0;
		
		if (isNewTxRequest())
		{
			if (ifg_tx_flag)
			{ 
			Ethernet_req* temp = getTxRequeset();
			size=(temp->payloadSize[0] + (temp->payloadSize[1]*256));
			if (size<=42)
			{
				frame = (uint8_t*) malloc(72*sizeof(uint8_t)); //--------------
			}
			else
			{
				frame = (uint8_t*) malloc((size+30)*sizeof(uint8_t)); //-----------
				
			}
			for(i=0; i<8;i++)
			{
				if(i!=7)
				{
					frame[i]=0xAA;
					
				}
				else
				{
					frame[i]=0xAB;
					
				}
			}
			for(i=8;i<14;i++)
			{
				if (i==8)
				{
					frame[i]=temp->destinationMac[i-8];
					HAL_CRC_Calculate(&hcrc,(uint32_t*)&(temp->destinationMac[i-8]),1);
				}
				else
				{
					frame[i]=temp->destinationMac[i-8];
					crc = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&(temp->destinationMac[i-8]),1);		
				}
			}
			for(i=14;i<20;i++)
			{
				frame[i]=myMAC[i-14];
				crc = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&(myMAC[i-14]),1);
			}
			for(i=20;i<24;i++)
			{
				frame[i]=temp->tagPort[i-20];
				crc = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&(temp->tagPort[i-26]),1);				
			}
			for(i=24;i<26;i++)
			{
				frame[i]=temp->payloadSize[i-24];
				crc = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&(temp->payloadSize[i-24]),1);
			}
			for(i=26;i<size+26;i++)
			{
				frame[i]=temp->payload[i-26];
				crc = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&(temp->payload[i-26]),1);
			}
			for(i=26+size;i<size+30;i++)
				{
					if (i==26+size)
					frame[i] = crc&0xFF;
					if (i==27+size)
					frame[i] = crc&0xFF00;
					if (i==28+size)
					frame[i] = crc&0xFF0000;
					if (i==29+size)
					frame[i] = crc&0xFF000000;					
				}
			for(int l=0;l<72;l++)
			{
				printf("%x,",frame[l]);
			}
			//in the end of using 'temp' you have to free memory:
			free((void*)temp->payload);
			free(temp);
			flagi=1;
			}
		}
		if (isPhyTxReady()&&flagi)
		{
			if(j<((frame[24]+frame[25]*256)+30))
			{
				sendByte(frame[j]);
				j++;
			}
			else
			{
				ifg_tx_flag=0;
				size=0;
				j=0;
				i=0;
				flagi=0;
				HAL_TIM_Base_Start_IT(&htim3);
				free(frame);
			}
		}
	}
	
	void Mac_Rx()
	{
		static int clock_flag=0;
		static int k = 0;
		static uint8_t *new_frame;
		static uint32_t new_crc=0;
		static uint32_t old_crc=0;
		static int flag=1;
		static int i2=0;
		static int i3=0;
		

		if(isRxByteReady())
		{	
			//if (!ifg_rx_flag)
			//{
			if (flag)
			{
				new_frame = (uint8_t*) malloc(sizeof(uint8_t)); //--------------
				flag=0;
			}
			if (clock_flag ==1)
				{	
					HAL_TIM_Base_Stop_IT(&htim2);
					clock_flag=0;
				}
			if (k==0)
			{
				new_frame[k] = getByte();
				k++;
			}
			else
			{
				new_frame = (uint8_t*) calloc(1,sizeof(uint8_t));	//--------------------
				new_frame[k] = getByte();
				k++;
			}
			HAL_TIM_Base_Start_IT(&htim2);
			clock_flag=1;
		}
		else if (ifg_rx_flag)
		{
				for(i2=8;i2<k-4;i2++)
				{
					if (i2==8)
					{
						HAL_CRC_Calculate(&hcrc,(uint32_t*)&(new_frame[i2]),1);	
					}
					else
						new_crc = HAL_CRC_Accumulate(&hcrc,(uint32_t*)&(new_frame[i2]),1);							
				}
				old_crc = new_frame[k-4];
				old_crc += new_frame[k-3]*256;
				old_crc += new_frame[k-2]*65536;
				old_crc += new_frame[k-1]*(2^24);

			if (old_crc!=new_crc&&0)
			{
				printf("nogooddd");
				frame_build.syndrom = crc_error;
				//error
			}
			else
			{
				for(i3=8; i3<k;i3++)
				{
					if ((i3>=8)&&(i3<14))
					{
						frame_build.destinationMac[i3-8] = new_frame[i3];
					}
					if((i3>=14)&&(i3<20))
					{
						frame_build.sourceMac[i3-14] = new_frame[i3];
					}
					if(i3>=24&&i3<=25)
					{
						if (i3==24)
						{
							frame_build.payloadSize[i3-24]=new_frame[i3];
						}
						if (i3==25)
						{
							frame_build.payloadSize[i3-24]=new_frame[i3];
							payload_size = new_frame[i3-1]+new_frame[i3]*256;
	
							if(payload_size>1500)
							{
								frame_build.syndrom = payload_error;
							}
							frame_build.payload = (uint8_t*) malloc((payload_size)*(sizeof(uint8_t))); //---------------------------
						}
					}
					if ((i3>=26)&&(i3<26+payload_size))
					{
						printf("check");
						frame_build.payload[i3-26]=new_frame[i3];
						if(i3 == 25+payload_size)
						{
							printf("helooo, rnjit!!");
							ready_for_llc=1;
							free(new_frame);
							flag=1;
							old_crc=0;
							new_crc=0;
							payload_size=0;
							k=0;
							i2=0;
							i3=0;
							ifg_rx_flag=0;
						}
					}
				}
				
			}
		}
	}
	
	
	void LLC_Rx()
	{
		static int i=0; //check if need to reset the I each time
		if(ready_for_llc)
		{
			printf("New frame is ready:\n");
			switch(frame_build.syndrom)
			{
				case clear:
					printf("Destination Address:");
					for(i=0; i<6;i++)
					{
						printf("%x",frame_build.destinationMac[i]);
					}
					printf("\nSource Address:");
					for(i=0;i<6;i++)
					{
						printf("%x",frame_build.sourceMac[i]);
					}
					printf("\nData:");
					for(i=0;i<payload_size;i++)
					{
						printf("%c",frame_build.payload[i]);
					}
					break;
				case frame_size_error:
					printf("Frame size is wrong\n");
					break;
				case crc_error:
					printf("Data corrupted\n");
					break;
				case payload_error:
					printf("payload_size is too big\n");
					break;
				
			}
			free((void*)frame_build.payload);
			i=0;
			ready_for_llc=0;
		}
	}
	
	
	
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	printf("------------------------------------------------------------\r\n");
	printf("DLL is On! DLL will print Rx data and Tx data \r\n(My MAC address is: %02X:%02X:%02X:%02X:%02X:%02X)\r\n",
				myMAC[0],myMAC[1],myMAC[2],myMAC[3],myMAC[4],myMAC[5]);
	printf("------------------------------------------------------------\r\n");
	printf("Press any key to start\r\n");
	DllAlive = 1; //DLL is on!
	__HAL_RCC_CRC_CLK_ENABLE();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//DLL functions - DO NOT TOUCH
		HAL_UART_Receive_IT(&huart2,&recieved_value,1);
		printer();
		DEBUG;
		//End Of DLL functions
		Mac_Tx();
		Mac_Rx();
		LLC_Rx();
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		
		/* ~Exmple of Tx_flow:~ */		
		/*if (isNewTxRequest())
		{
			Ethernet_req* temp = getTxRequeset();
			
			... some code ...
			
			in the end of using 'temp' you have to free memory:
			free((void*)temp->payload);
			free(temp);
		}*/
		
		
		
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 359;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 917;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Phy_tx_valid_GPIO_Port, Phy_tx_valid_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Phy_tx_data_bus_pin0_Pin|Phy_tx_data_bus_pin1_Pin|Phy_tx_data_bus_pin2_Pin|Phy_tx_data_bus_pin3_Pin 
                          |Phy_tx_data_bus_pin4_Pin|Phy_tx_data_bus_pin5_Pin|Phy_tx_data_bus_pin6_Pin|Phy_tx_data_bus_pin7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Phy_tx_valid_Pin */
  GPIO_InitStruct.Pin = Phy_tx_valid_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Phy_tx_valid_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Phy_tx_data_bus_pin0_Pin Phy_tx_data_bus_pin1_Pin Phy_tx_data_bus_pin2_Pin Phy_tx_data_bus_pin3_Pin 
                           Phy_tx_data_bus_pin4_Pin Phy_tx_data_bus_pin5_Pin Phy_tx_data_bus_pin6_Pin Phy_tx_data_bus_pin7_Pin */
  GPIO_InitStruct.Pin = Phy_tx_data_bus_pin0_Pin|Phy_tx_data_bus_pin1_Pin|Phy_tx_data_bus_pin2_Pin|Phy_tx_data_bus_pin3_Pin 
                          |Phy_tx_data_bus_pin4_Pin|Phy_tx_data_bus_pin5_Pin|Phy_tx_data_bus_pin6_Pin|Phy_tx_data_bus_pin7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Phy_rx_data_bus_pin2_Pin Phy_rx_data_bus_pin3_Pin Phy_rx_data_bus_pin4_Pin Phy_rx_data_bus_pin5_Pin 
                           Phy_rx_data_bus_pin6_Pin Phy_rx_data_bus_pin7_Pin Phy_rx_data_bus_pin0_Pin Phy_rx_data_bus_pin1_Pin */
  GPIO_InitStruct.Pin = Phy_rx_data_bus_pin2_Pin|Phy_rx_data_bus_pin3_Pin|Phy_rx_data_bus_pin4_Pin|Phy_rx_data_bus_pin5_Pin 
                          |Phy_rx_data_bus_pin6_Pin|Phy_rx_data_bus_pin7_Pin|Phy_rx_data_bus_pin0_Pin|Phy_rx_data_bus_pin1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Phy_rx_valid_Pin Phy_tx_busy_Pin Phy_reset_Pin */
  GPIO_InitStruct.Pin = Phy_rx_valid_Pin|Phy_tx_busy_Pin|Phy_reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Phy_clock_Pin */
  GPIO_InitStruct.Pin = Phy_clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Phy_clock_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
