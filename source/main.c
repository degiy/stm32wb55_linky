/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void process_buffer(uint8_t*, uint8_t*);
void decode(uint8_t,uint16_t);
uint16_t comb_algo();
int _write(int, char *, int);

#define NBVALS (1024*2)
#define NBVALSH (NBVALS/2)
uint8_t buf[NBVALS];
uint8_t *const p_half=&(buf[NBVALSH]);
uint8_t *const p_end=&(buf[NBVALS]);


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	process_buffer(p_half,p_end);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	process_buffer(buf,p_half);
}

// third pass for state change and timing reading
uint8_t last_value=0;
uint16_t last_change=0;
uint16_t ic=0;
uint8_t vold=0;
uint8_t voold=0;
uint16_t total=0;
uint16_t ltnrj=0;

void process_buffer(uint8_t *beg, uint8_t *end)
{
	//	first pass for average calculation
	uint8_t *pt=beg;
	uint32_t sum=0;
	do
	{
		uint8_t v=*pt++;
		sum+=v;
	} while (pt<end);
	uint8_t avg=sum/(NBVALSH);

	// second pass for global energy measurement and offset
	sum=0;
	pt=beg;
	do
	{
		uint8_t v=*pt;
		if (v>=avg) v-=avg; else v=avg-v;
		*pt++=v;
		sum+=v;
	} while (pt<end);
	uint16_t nrj=sum/(NBVALSH);
	// smoothing mean energy value on 16 half buffers
	if (ltnrj!=0)
		ltnrj=(ltnrj*15)/16+nrj;
	else
		ltnrj=16*nrj;
	uint16_t level=24*ltnrj/256;	// 1.5x avg nrj on 3 values as level to compare against to catch bearer

	pt=beg;
	do
	{
		uint8_t v=*pt++;
		total+=v;
		last_change++;
		if ( ((total>level)&&(last_value==0)) || ((total<level)&&(last_value==1)) )
		{
			// inverted logic => no bearer = 1 and bearer = 0
			last_value=last_value^1;
			decode(last_value,last_change);
			last_change=0;
		}
		total-=voold;
		voold=vold;
		vold=v;
	} while (pt<end);
}

// to estimate baud rate
#define MIN_SPACES 10		// normaly 1200 bauds should be 150/1.2 or 125 and 9600 150/9.6 or 15, let's start at 10
#define MAX_SPACES 2048 	// 1200 bauds will need 1125 for 9 consecutive bits to same value (0), use 2048
#define SPACES_LEVEL (1<<12) // we need to hit as much the more frequent space between bits, so few seconds of bit train

uint16_t spaces[MAX_SPACES];

#define max_rating (spaces[0])

// ascii buffer to exit linky frame
#define NBC 1024
uint8_t bufc[NBC];
uint8_t *pchars=bufc;

typedef enum {
	UNKNOWN,
	GAP,
	GAP_START,
	STX_GAP,
	STX_GAP_START,
    STX_PSS,
	LF_PSS,
	DATA_PSS,
	CR7,
	CR_PSS,
	ETX_P
} State;

State cur_state=UNKNOWN;
uint16_t tab[]={0,0x200};
uint16_t cb;
uint8_t nbits=255;
uint8_t full_message=0;
uint16_t speed=0;

void decode(uint8_t val,uint16_t len)
{
	if (speed==0)
	{
		// we need to guess to baud rate as the linky can output historical and standard payload (first at 1200 bauds, last at 9600 bauds)
		if ((len>MAX_SPACES-1)||(len<MIN_SPACES))
			return;
		// we value more central value, and less close neighbours

		spaces[len-1]+=1;
		if (spaces[len-1]>max_rating)
			max_rating=spaces[len-1];
		spaces[len]+=3;
		if (spaces[len]>max_rating)
			max_rating=spaces[len];
		spaces[len+1]+=1;
		if (spaces[len+1]>max_rating)
			max_rating=spaces[len+1];
		return;
	}
	else if (speed==1) return;

	uint16_t nb=len/speed; // to improve for 1200 bauds or 9600 bauds : 150000/1200 = 125, 150000/9600 = 15.6

	// try to decode ASCII from serial stream
	// 0 0100000 110 0101000 010 1000001 010
	// [  STX    p][  LF     p][  7bits  p][
	// 010 1011000 11...
	//       CR    not received as such (because of gap)
	// 010 1011000 110 1100000 0 1....
	//       CR          ETX   p not received (gap)

	if (nb>10)
	{
		if (cur_state==CR7)
			cur_state=STX_GAP;
		else
		{
			cur_state=GAP;
			pchars=bufc;
		}
		cb=0;
		nbits=1;
	}
	else
	{
		uint8_t n=nb;
		while (n-->0)
		{
			cb>>=1;
			cb|=tab[val];
			if (--nbits==0)
			{
				if ((cur_state==GAP)&&(cb==0)) // the missing bit start (we didn't get from exiting the gap yet)
				{
					cur_state=GAP_START;
					//*pchars++='{';
					nbits=10;
				}
				else if ((cur_state==STX_GAP)&&(cb==0)) // the missing bit start (we didn't get from exiting the gap yet)
				{
					cur_state=STX_GAP_START;
					//*pchars++='[';
					nbits=10;
				}
				else if ((cur_state==GAP_START)&&(cb==0x182)) // STX 2 with parity (1) and stop bit (1) and start bit (0)
				{
					cur_state=STX_PSS;
					//*pchars++=2;
					//*pchars++='(';
					cb=0;
					nbits=10;
				}
				else if ( ( (cur_state==STX_GAP_START) || (cur_state==STX_PSS) ) &&(cb==0x10a)) // LF (10) + p(0) + stop(1) + start(0)
				{
					cur_state=LF_PSS;
					//*pchars++=10;
					//*pchars++='-';
					cb=0;
					nbits=10;
				}
				else if ( (cur_state==LF_PSS) || (cur_state==DATA_PSS) )
				{
					cur_state=DATA_PSS;
					cb&=0x7f;	// add parity check here
					*pchars++=cb;
					cb=0;
					nbits=10;
				}
				else if ( (cur_state==CR7) &&(cb== 0x18d ) ) // the end of CR (parity, stop bit and start bit)
				{
					cur_state=CR_PSS; // full CR10
					//*pchars++=']';
					cb=0;
					nbits=8; // to search ETX + p
				}
				else if ( (cur_state==CR_PSS) &&(cb== (3<<2) ) ) // ETX 3<<2 = 12
				{
					cur_state=ETX_P;
					//*pchars++=3;
					//*pchars++=')';
					cb=0;
					nbits=99;
					full_message=1;
				}
				else
				{
					// oups
					cur_state=UNKNOWN;
					nbits=255;
				}
			}
			else if (nbits==3)
			{
				if ( (cur_state==DATA_PSS) && (cb== (0xd<<3) ) )
				{
					cur_state=CR7;
					*pchars++='\r';
					*pchars++='\n';
					// maybe more if a ETX is following
				}
			}
			// else we need more bits to come to form something of interest
		}
	}
}

uint16_t comb_algo()
{
	uint32_t max=0;
	uint16_t best_space=0;

	for (uint16_t comb_width=MIN_SPACES;comb_width<MAX_SPACES/10;comb_width++)
	{
		uint32_t sum=0;
		uint16_t space=comb_width;
		for (uint8_t i=1;i<=9;i++)
		{
			if (spaces[space]==0)
				break;
			sum+=spaces[space]*i;
			space+=comb_width;
		}
		if (sum>max)
		{
			max=sum;
			best_space=comb_width;
			printf("new best space %hu, score %lu\r\n",best_space,max);
		}
	}
	// we take a minor of 8% to be sure to find gaps with 9 bits too
	return best_space-best_space/12;
}

void HAL_Delay2(uint32_t d)
{
	d*=64;
	uint32_t t0=DWT->CYCCNT;
	uint32_t t1;
	do { t1=DWT->CYCCNT; } while (t1-t0<d);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  memset(bufc,0,sizeof(bufc));
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  uint32_t t0=DWT->CYCCNT/64;
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
  uint32_t calib=HAL_ADCEx_Calibration_GetValue(&hadc1,ADC_SINGLE_ENDED);
  printf("start ADC conv at %lu calib %lu\r\n",t0,calib);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buf, NBVALS);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int j=0;
  int jj=0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  j++;
	  if ((j%1013==0)||(j%2053==0)||(j%3067==0))
	  {
		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
		  //uint32_t t1=DWT->CYCCNT/64;
		  if (j%997==0)
		  {
			  jj++;
		  }

		  if (jj==77)
		  {
			  jj=0;
			  printf("\r\n !!! big reset !!!\r\n");
			  HAL_Delay(1);
			  memset((uint32_t*)bufc,'_',sizeof(bufc));
			  pchars=bufc;
			  nbits=255;
			  cur_state=UNKNOWN;
			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buf, NBVALS);
		  }
	  }
	  if ((speed==0)&&(max_rating>SPACES_LEVEL))
	  {
		  // skip updates as we calculate the most probable baud rate
		  speed=1;
		  // algo
		  speed=comb_algo();
	  }
	  if (full_message)
	  {
		  full_message=0;
	  	  _write(0,"\r\n",2);
		  HAL_Delay2(10);
		  *pchars++='\r';
		  *pchars++='\n';
		  _write(0,(char*)bufc,pchars-bufc);
		  HAL_Delay2(10);
		  _write(0,"\r\n",2);
	  }
  }
  // ⌂.ISOUSC 30 9..5....BASE 011767463.......PTEC TH......IINST 001 ... IMAX 090
  // 0 0101000 010 1001001 110 1100101 010 1111001 110 1010101 010 1100101 010 1100001 110 0000010 110 1100110 010 0000110 010 0000010 110 1001110 010 1011000 11...
  // [  <LF>   p][   I     p][    S           O           U          S           C         <space>       3            0        <space>       5       [   <CR>  p]
  // 0 0101000 010 0100001 010 1000001 010 1100101 01010100011100000010110000011001010001101101000110110111011011001101100101110110110001011011001101100101100110010000001011001110100101011000
  // [  <LF>   p][   B            A           S
  // 0 0101000 010 0000101 010 0010101 110101000111011000011100000010110001010111000010010100111010010011101001000000101100010010010 1011000
  // [  <LF>   p][   P            T
  // 0 0101000 010 1001001 110 1001001 110011100101011001010100010101110000001011000001100100000110010100011011000000101100001101110 1011000
  // [  <LF>   p][   I            I
  // 0 0101000 010 1001001 110 1011001 01010000010100001101110000001011000001100101001110010000011001000000101100001001010 1011000
  //                 I            M
  // ADCO 812061237xxx (first frame)
  // 0 0100000 110 0101000 010 1000001 010 0010001 010 1100001 110 1111001 110 00000101100001110110100011011001001101100000110...
  //   <STX>        <LF>         A           D           C            O
  //
  // MOTDETAT 000000 B... (last frame))
  // 0 0101000 010 1011001 010 1111001 110 0010101 110 ... 0000010 110 0100001 010 1011000 110 1100000 01....
  //   <LF>         M            O            T            <space>        B          <CR>       <ETX>

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the HSE Prescaler
  */
  __HAL_RCC_HSE_DIV2_ENABLE();

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 2000000;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */
  //HAL_UARTEx_


  /* USER CODE END LPUART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *data, int len)
{
	HAL_StatusTypeDef HalRet = HAL_OK;

	/* wait for uart ready with timeout */
	while( hlpuart1.TxXferCount > 0U);
	HalRet = HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)data, len);

	/* return # of bytes written */
	if( HalRet != HAL_OK ) return 0;
	return len;
}

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
