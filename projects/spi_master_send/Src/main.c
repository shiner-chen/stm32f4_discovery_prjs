/**
  ******************************************************************************
  * @file    FatFs/FatFs_USBDisk/Src/main.c 
  * @author  MCD Application Team
  * @brief   Main program body
  *          This sample code shows how to use FatFs with USB disk drive.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"    

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,  
  BUFFER_OFFSET_HALF,  
  BUFFER_OFFSET_FULL,     
}BUFFER_StateTypeDef;
/* Private define ------------------------------------------------------------*/
#define REPEAT_ON        ((uint32_t)0x00) /* Replay Status in ON */
#define REPEAT_OFF       ((uint32_t)0x01) /* Replay Status in OFF */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* SPI handler declaration */
SPI_HandleTypeDef Spi2Handle;
FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
FIL MyFile;                   /* File object */
char USBDISKPath[4];          /* USB Host logical drive path */
USBH_HandleTypeDef hUSB_Host; /* USB Host handle */

/* SPI data length to be played */
static uint32_t SPIDataLength = 0;
/* Audio wave remaining data length to be played */
static __IO uint32_t DataRemSize = 0;
/* Ping-Pong buffer used for audio play */
uint8_t Data_Buffer[DATA_BUFFER_SIZE];
/* Position in the audio play buffer */
static __IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;
/* Re-play Wave file status on/off.
   Defined as external in waveplayer.c file */
__IO uint32_t RepeatState = REPEAT_ON;
static FILINFO FileInfo;
uint32_t count =0;

typedef enum {
  APPLICATION_IDLE = 0,  
  APPLICATION_START,    
  APPLICATION_RUNNING,
}MSC_ApplicationTypeDef;

MSC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/* Private function prototypes -----------------------------------------------*/ 
static void SystemClock_Config(void);
static void Error_Handler(void);
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
static void MSC_Application(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
	/* Configure LED3, LED4, LED5 and LED6 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
	
  
  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
	
	/*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  Spi2Handle.Instance               = SPIx;
	//1M:SPI_BAUDRATEPRESCALER_128;2M:SPI_BAUDRATEPRESCALER_64;4M:SPI_BAUDRATEPRESCALER_32;8M:SPI_BAUDRATEPRESCALER_16;16M:SPI_BAUDRATEPRESCALER_8;
  Spi2Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  Spi2Handle.Init.Direction         = SPI_DIRECTION_2LINES;
  Spi2Handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  Spi2Handle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  Spi2Handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  Spi2Handle.Init.CRCPolynomial     = 7;
  Spi2Handle.Init.DataSize          = SPI_DATASIZE_8BIT;
  Spi2Handle.Init.FirstBit          = SPI_FIRSTBIT_LSB;
  Spi2Handle.Init.NSS               = SPI_NSS_HARD_OUTPUT; //SPI_NSS_SOFT;
  Spi2Handle.Init.TIMode            = SPI_TIMODE_DISABLE;
 
  Spi2Handle.Init.Mode = SPI_MODE_MASTER;

  if(HAL_SPI_Init(&Spi2Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
   /* Configure USER Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  
  /* Wait for USER Button press before starting the Communication */
//  while (BSP_PB_GetState(BUTTON_KEY) != 1)
//  {
//    BSP_LED_Toggle(LED3);
//    HAL_Delay(40);
//  }
//  
//  BSP_LED_Off(LED3);  
  /*##-1- Link the USB Host disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
  {
    /*##-2- Init Host Library ################################################*/
    USBH_Init(&hUSB_Host, USBH_UserProcess, 0);
    
    /*##-3- Add Supported Class ##############################################*/
    USBH_RegisterClass(&hUSB_Host, USBH_MSC_CLASS);
    
    /*##-4- Start Host Process ###############################################*/
    USBH_Start(&hUSB_Host);
    
    /*##-5- Run Application (Blocking mode) ##################################*/
    while (1)
    {
      /* USB Host Background task */
      USBH_Process(&hUSB_Host);
      
      /* Mass Storage Application State Machine */
      switch(Appli_state)
      {
      case APPLICATION_START:
				if (RepeatState == REPEAT_ON)
					MSC_Application();
        Appli_state = APPLICATION_IDLE;
        break;
        
      case APPLICATION_IDLE:
      default:
        break;      
      }
    }
  }
  
  /* TrueStudio compilation error correction */
  while (1)
  {
  }
}

/**
  * @brief  Main routine for Mass Storage Class
  * @param  None
  * @retval None
  */
static void MSC_Application(void)
{
  FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  //uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
  //uint8_t rtext[100];                                   /* File read buffer */
  RepeatState = REPEAT_OFF;
  /* Register the file system object to the FatFs module */
  if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
  {
    /* FatFs Initialization Error */
    Error_Handler();
  }
  else
  {
		/* Create and Open a new text file object with write access */
		if(f_open(&MyFile, "data.pwm", FA_READ) != FR_OK) 
		{
			/* 'STM32.TXT' file Open for write Error */
			Error_Handler();
		}
		else
		{
			/* Set PdmDataLenght to the Speech PDM length */
			f_stat ("data.pwm", &FileInfo);
			SPIDataLength = FileInfo.fsize;
			
			/* Get Data from USB Flash Disk */
			f_lseek(&MyFile, 0);
			f_read (&MyFile, &Data_Buffer[0], DATA_BUFFER_SIZE, &bytesread);
			DataRemSize = SPIDataLength - bytesread;
			count +=2;
			if(HAL_SPI_Transmit_DMA(&Spi2Handle, &Data_Buffer[0], DATA_BUFFER_SIZE) != HAL_OK)
			{
				/* Transfer error in transmission process */
				Error_Handler();
			}
//			while (HAL_SPI_GetState(&Spi2Handle) != HAL_SPI_STATE_READY)
//			{
//			} 
//			while (BSP_PB_GetState(BUTTON_KEY) != 1)
//			{
//				BSP_LED_Toggle(LED3);
//				HAL_Delay(40);
//			}

//			BSP_LED_Off(LED3); 
			/* Check if the device is connected.*/
			while(DataRemSize != 0)
			{ 
					
//				BSP_LED_Toggle(LED3);
					BSP_LED_On(LED3);
//				HAL_Delay(40);
				bytesread = 0;
				
				if(buffer_offset == BUFFER_OFFSET_HALF)
				{
					f_read(&MyFile, 
								 &Data_Buffer[0], 
								 DATA_BUFFER_SIZE/2, 
								 (void *)&bytesread); 
					count++;
					buffer_offset = BUFFER_OFFSET_NONE;
				}
 
				if(buffer_offset == BUFFER_OFFSET_FULL)
				{
					f_read(&MyFile, 
								 &Data_Buffer[DATA_BUFFER_SIZE/2], 
								 DATA_BUFFER_SIZE/2, 
								 (void *)&bytesread); 
					count++;
					buffer_offset = BUFFER_OFFSET_NONE;
				} 
		
				if(DataRemSize > (DATA_BUFFER_SIZE / 2))
				{
					DataRemSize -= bytesread;

				}
				else
				{
					DataRemSize = 0;
				}
			}
			/* Call DMA Stop to disable DMA stream before stopping codec */
			HAL_SPI_DMAStop(&Spi2Handle);
			BSP_LED_Off(LED3);
			BSP_LED_Off(LED4);
			BSP_LED_Off(LED6);
			/* Close file */
			f_close(&MyFile);
		}
	}
	/* Unlink the USB disk I/O driver */
	//FATFS_UnLinkDriver(USBDISKPath);
}

/**
  * @brief  User Process
  * @param  phost: Host handle
  * @param  id: Host Library user message ID
  * @retval None
  */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{  
  switch(id)
  { 
  case HOST_USER_SELECT_CONFIGURATION:
    break;
    
  case HOST_USER_DISCONNECTION:
    Appli_state = APPLICATION_IDLE;
    BSP_LED_Off(LED4); 
    BSP_LED_Off(LED5);  
    f_mount(NULL, (TCHAR const*)"", 0);          
    break;
    
  case HOST_USER_CLASS_ACTIVE:
    Appli_state = APPLICATION_START;
    break;
    
  default:
    break; 
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config  (void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 256; //336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig (&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; //RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}


/**
  * @brief  Sends n-Bytes on the SPI interface.
  * @param  pData: Pointer to data address 
  * @param  Size: Number of data to be written
  */
void SPI_OUT_ChangeBuffer(uint8_t *pData, uint16_t Size)
{
	HAL_SPI_Transmit_DMA(&Spi2Handle, pData, Size);
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hspi: SPI handle
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
   /* Turn LED4 on: Transfer in transmission process is correct */
  BSP_LED_Toggle(LED4);
	buffer_offset = BUFFER_OFFSET_FULL;
  SPI_OUT_ChangeBuffer(&Data_Buffer[0], DATA_BUFFER_SIZE);
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hi2s: I2S handle
  */
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED6 on: Transfer in reception process is correct */
  BSP_LED_Toggle(LED6);
	buffer_offset = BUFFER_OFFSET_HALF;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED5 on: Transfer error in reception/transmission process */
  BSP_LED_On(LED5); 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED5 on */
  BSP_LED_On(LED5);
  while(1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
