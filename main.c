/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "..\..\Middlewares\Third_Party\ioLibrary_Driver\Ethernet\w5500.h"
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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for printf_Mutex */
osMutexId_t printf_MutexHandle;
const osMutexAttr_t printf_Mutex_attributes = {
  .name = "printf_Mutex"
};
/* Definitions for xSemaphore1 */
osSemaphoreId_t xSemaphore1Handle;
const osSemaphoreAttr_t xSemaphore1_attributes = {
  .name = "xSemaphore1"
};
/* USER CODE BEGIN PV */
uint8_t RecvThreadState = 0;
uint8_t dma_tx_flag = 1;
uint8_t dma_rx_flag = 1;

osThreadId_t SocketServerHandle;
const osThreadAttr_t SocketServer_attributes = {
  .name = "SocketServer_task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
osThreadId_t ISRHandleHandle;
const osThreadAttr_t ISRHandle_attributes = {
  .name = "ISRHandle_task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
osThreadId_t SocketReceiveHandle;
const osThreadAttr_t SocketReceive_attributes = {
  .name = "SocketRecv_task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void SocketServer(uint8_t sn);
void rec(uint8_t sn);
void ISRHandleTask(void *argument);
void SocketRecv(uint8_t sn);
void DMA_HalfTransferCallback(DMA_HandleTypeDef * _hdma);
void DMA_FullTransferCallback(DMA_HandleTypeDef * _hdma);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*__attribute__((weak)) int _write(int file, char *ptr, int len)
{
  if(CDC_Transmit_FS(ptr,len) != USBD_OK)
  {
    Error_Handler();
  }
}*/
/*#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  CDC_Transmit_FS(ch,strlen(ch));
  return ch;
}*/

/*(int _write(int fd, char *ptr, int len)
{
  CDC_Transmit_FS((uint8_t*)ptr,len);
  return len;
}*/
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  //HAL_SPI_TransmitReceive_DMA(&hspi1,(uint8_t*)AD_VALUE_Buff1,(uint8_t*)RX_data,1024);
  //HAL_DMA_RegisterCallback(&hspi1,HAL_DMA_XFER_HALFCPLT_CB_ID,DMA_HalfTransferCallback);
  //HAL_DMA_RegisterCallback(&hspi1,HAL_DMA_XFER_CPLT_CB_ID,DMA_FullTransferCallback);
  /* USER CODE END 2 */

  /* Init scheduler */
  //osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of printf_Mutex */
  //printf_MutexHandle = osMutexNew(&printf_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of xSemaphore1 */
  xSemaphore1Handle = osSemaphoreNew(1, 1, &xSemaphore1_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  //ISRHandleHandle = osThreadNew(ISRHandleTask, NULL, &ISRHandle_attributes);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t ret = w5500_init();
  while (1)
  {
    /* USER CODE END WHILE */
	uint8_t ret = 0;
	uint8_t sn = 0;
	uint8_t status = getSn_SR(sn);
	switch(status)
	{
	  case SOCK_CLOSED:
		////printf("Socket%dClosed\r\n",sn);
		//close(sn);
		ret = socket(sn,Sn_MR_TCP,NULL,0x00);
		//ret = socket(sn,Sn_MR_TCP,NULL,Sn_MR_ND);
		////printf("Socket%dEstablishing\r\n",sn);
		if(ret != sn)
		{
		  ////printf("socket%d open fail,ret=%d\r\n",sn,ret);
		}
		else
		{
		  ////printf("socket%d open success\r\n",sn);
		  //setSn_MR(sn,Sn_MR_ND);
		}
		break;
	  case SOCK_INIT:
		ret = listen(sn);
		if(ret == 1)
		{
		  ////printf("socket%d start listening\r\n",sn);
		}
		else
		{
	      ////printf("socket%d listen fail,ret =%d\r\n",sn,ret);
		}
		break;
	  case SOCK_ESTABLISHED:
		if(getSn_IR(sn)&Sn_IR_CON)
		{
		  ////printf("socket%d connected\r\n",sn);
		  setSn_IR(sn,Sn_IR_CON);
		}
		if((RecvThreadState&(1<<sn))!=(1<<sn))
		{
		  ////printf("New Receive Thread%d Creating, state:%d\r\n",sn,RecvThreadState);
		//osThreadId_t aaa =osThreadNew(SocketRecv,sn,&SocketReceive_attributes);
		//printf("%d\r\n",aaa);
		  osDelay(2000);
		}
		break;
	  case SOCK_CLOSE_WAIT:
		ret = disconnect(sn);
		////printf("Socket%d Closing\r\n",sn);
		if(ret == 1)
		{
		  ////printf("Socket%d Closed\r\n", sn);
		  ////printf("Receive Thread Exit, state:%d\r\n",RecvThreadState);
		}
		break;
	  case SOCK_LISTEN:
		//printf("Socket%d listening\r\n",sn);
		break;
	  default:
	    ////printf("default:%d\r\n",getSn_SR(sn));
		break;
	}
	osDelay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|LED_Pin|LGD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin LED_Pin LGD_Pin */
  GPIO_InitStruct.Pin = CS_Pin|LED_Pin|LGD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT1_Pin */
  GPIO_InitStruct.Pin = OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN1_Pin */
  GPIO_InitStruct.Pin = IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */


int _write(int file, char *ptr, int len)
{
  static uint8_t rc = USBD_OK;
  do{
    rc = CDC_Transmit_FS(ptr,len);
    }while(USBD_BUSY == rc);
  if (USBD_FAIL == rc)
  {
    return 0;
  }
  return len;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi==&hspi1)
  {
	dma_tx_flag = 1;
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi==&hspi1)
  {
	dma_rx_flag = 1;
  }
}
/*
void DMA_HalfTransferCallback(DMA_HandleTypeDef * _hdma)
{
  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_14);
}
void DMA_FullTransferCallback(DMA_HandleTypeDef * _hdma)
{
  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_13);
  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_14);
}
*/
void ISRHandleTask(void *argument)
{
  for(;;)
  {
	if(osSemaphoreAcquire(xSemaphore1Handle,osWaitForever)==osOK)
	{
	  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_14);
	  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_13);
	  printf("pushed!\r\n");
	  char info[100];
	  osVersion_t osv;
	  osStatus_t status = osKernelGetInfo(&osv,info,sizeof(info));
	  //osDelay(10000);
	  int len = strlen(info);
	  if (status == osOK){
        }
	  }
	  /*
	  uint8_t buf = 0xaa;
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	  uint8_t ret = HAL_SPI_Transmit_DMA(&hspi2, &buf, 1);
	  while(HAL_SPI_GetState(&hspi2)!=HAL_SPI_STATE_READY);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	  */
  }
}

void SocketServer(uint8_t sn)
{
  for(;;)
  {
	uint8_t ret = 0;
	uint8_t status = getSn_SR(sn);
	switch(status)
	{
	  case SOCK_CLOSED:
		printf("Socket%dClosed\r\n",sn);
		//close(sn);
		ret = socket(sn,Sn_MR_TCP,NULL,0x00);
		//ret = socket(sn,Sn_MR_TCP,NULL,Sn_MR_ND);
		printf("Socket%dEstablishing\r\n",sn);
		if(ret != sn)
		{
		  printf("socket%d open fail,ret=%d\r\n",sn,ret);
		}
		else
		{
		  printf("socket%d open success\r\n",sn);
		  //setSn_MR(sn,Sn_MR_ND);
		}
		break;
	  case SOCK_INIT:
		ret = listen(sn);
		if(ret == 1)
		{
		  printf("socket%d start listening\r\n",sn);
		}
		else
		{
		  printf("socket%d listen fail,ret =%d\r\n",sn,ret);
		}
		break;
	  case SOCK_ESTABLISHED:
		if(getSn_IR(sn)&Sn_IR_CON)
		{
		  printf("socket%d connected\r\n",sn);
		  setSn_IR(sn,Sn_IR_CON);
		}
		if((RecvThreadState&(1<<sn))!=(1<<sn))
		{
	      printf("New Receive Thread%d Creating, state:%d\r\n",sn,RecvThreadState);
		  //osThreadId_t aaa =osThreadNew(SocketRecv,sn,&SocketReceive_attributes);
		  //printf("%d\r\n",aaa);
		  osDelay(2000);
		}
		break;
	  case SOCK_CLOSE_WAIT:
		ret = disconnect(sn);
		printf("Socket%d Closing\r\n",sn);
		if(ret == 1)
		{
		  printf("Socket%d Closed\r\n", sn);
	      printf("Receive Thread Exit, state:%d\r\n",RecvThreadState);
		}
		break;
	  case SOCK_LISTEN:
		//printf("Socket%d listening\r\n",sn);
		break;
	  default:
		printf("default:%d\r\n",getSn_SR(sn));
		break;
	}
	osDelay(100);
  }
}

void SocketRecv(uint8_t sn)
{
  printf("RecvThreadState=%d,sn=%d\r\n",RecvThreadState,sn);
  RecvThreadState |= (1<<sn);
  printf("New Receive Thread Created, state:%d\r\n",RecvThreadState);
  //setSn_MR(sn,Sn_MR_ND);
  for(;;)
  {
	if(getSn_SR(sn)==SOCK_ESTABLISHED)
	{
	  uint8_t buffer[2048];
	  uint16_t size = 0;
	  int32_t ret = 0;
	  int DATA_BUF_SIZE = 2048;
	  if((size = getSn_RX_RSR(sn))>0)
	  {
		printf("%d\r\n",size);
		uint32_t freq = osKernelGetTickFreq();
		uint32_t tick = osKernelGetTickCount();
		printf("freq:%d\r\n",freq);
		printf("tick:%d\r\n",tick);
		memset(buffer,0,sizeof(buffer));
		if(size>DATA_BUF_SIZE) size = DATA_BUF_SIZE;
		ret = recv(sn,buffer,size);
		if(ret<=0)
		{
		  printf("err ret=%d\r\n",ret);
		}
		printf("Socket%d receive:%s\r\n",sn,buffer);
	  }
	}
	else if(getSn_SR(sn)==SOCK_CLOSED)
	{
	  //osMutexAcquire(printf_MutexHandle,0);
	  //osMutexRelease(printf_MutexHandle);
	  RecvThreadState &= ~(1<<sn);
	  osThreadExit();
	}
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN 5 */
  uint8_t ret = 1;
  /* Infinite loop */
  for(;;)
  {
	/* spi_dma test with arduino
    uint8_t buf = 0xaa;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
    uint8_t ret = HAL_SPI_Transmit_DMA(&hspi2, &buf, 1);
    while(HAL_SPI_GetState(&hspi2)!=HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    */

	if(ret != 0){
	  ret = w5500_init();
	  if(ret != 0){
		printf("nic init_fail\r\n");}
	  else{
		printf("nic init_success\r\n");
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_14,GPIO_PIN_RESET);
		//setRTR(0);
		for(int sn=0;sn<2;sn++){
	      osThreadNew(SocketServer,sn,&SocketServer_attributes);
		}
	  }
	}

  /*
  printf("state=%d\r\n",(uint8_t)hspi1.State);
  ret = HAL_SPI_Transmit_DMA(&hspi1, pBuf, len);
  //uint8_t state = hspi1.State;
  while(state != HAL_SPI_STATE_READY);
  //printf("state=%d\r\n",state);
  printf("state2=%d\r\n",(uint8_t)hspi1.State);
  */

    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
