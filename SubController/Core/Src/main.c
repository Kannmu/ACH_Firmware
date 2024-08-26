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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <math.h>
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

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

DMA_HandleTypeDef hdma_memtomem_dma1_stream0;
DMA_HandleTypeDef hdma_memtomem_dma1_stream1;
DMA_HandleTypeDef hdma_memtomem_dma1_stream2;
DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
DMA_HandleTypeDef hdma_memtomem_dma2_stream3;

static DMA_HandleTypeDef DMAs[5];

/* USER CODE BEGIN PV */

const char *pins[] =
    {
        "PC11",
        "PD2",
        "PD4",
        "PD6",
        "PB4",
        "PB6",
        "PB8",
        "PE0",

        "PC12",
        "PD3",
        "PD5",
        "PD7",
        "PB5",
        "PB7",
        "PB9",
        "PE1",

        "PA3",
        "PA1",
        "PC3",
        "PC1",
        "PC15",
        "PC13",
        "PE5",
        "PE3",

        "PA2",
        "PA0",
        "PC2",
        "PC0",
        "PC14",
        "PE6",
        "PE4",
        "PE2",

        "PC9",
        "PC7",
        "PD15",
        "PD13",
        "PD11",
        "PD9",
        "PB15",
        "PB13",

        "PC8",
        "PC6",
        "PD14",
        "PD12",
        "PD10",
        "PD8",
        "PB14",
        "PB12",

        "PE15",
        "PE13",
        "PE11",
        "PE9",
        "PE7",
        "PB1",
        "PC5",
        "PA7",

        "PE14",
        "PE12",
        "PE10",
        "PE8",
        "PB2",
        "PB0",
        "PC4",
        "PA6",

        "PC10"};

const double calibe_times[] =
    {
      8.8, 9.5, 11, 20.6, 7.4, 21.5, 21.5, 8.5,
      20.6, 21, 19.4, 7, 20, 20.6, 6.4, 20.6,
      19.4, 6.8, 19.5, 20.4, 7.3, 20, 8.2, 20.3,
      7.5, 19, 19.7, 6.5, 19.4, 7, 9.4, 9.5,
      9, 9.6, 5, 6.8, 18.3, 17.8, 19.3, 8.2,
      8.2, 20.4, 7.9, 6.7, 5.4, 7, 20.1, 21.6,
      19, 20.6, 19.2, 20.7, 7, 18.5, 6.8, 19.7,
      5.6, 10.6, 9.2, 20.3, 19.7, 6.4, 18.2, 2.2,
      0
    };

static uint16_t dma_buff[5][DMA_Buffer_Resolution] __attribute__((section(".dma")));

const float GPIO_Output_Offset[5] = {0U, 3, 6.5, 9, 11.5};
// MCU Parameters

// Transducer Array
struct Transducer* TDs[NumTransducer];

// Debug Parameters
const uint8_t LiveLEDPeriod = 1;
uint16_t LEDTicks = 0;
uint16_t DeltaTicks = 0;
uint32_t FPS = 0;
uint32_t Timebase = 0;
uint8_t DMA_Enable_Flag = 0;
double TestFocusPointDistance = 0.1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
void SubControllerInit();
void StartDMAs();
void HAL_Delay_us(uint32_t nus);
GPIO_TypeDef *map_pin_name_to_gpio_port(const char *pin_name);
uint8_t map_pin_name_to_gpio_port_num(const char *pin_name);
uint16_t map_pin_name_to_pin_number(const char *pin_name);
void IndicateLEDBlink();
Simulation TestShootParameters();

void SendDebuggingInfo();
void CalculateFPS();
void UpdateTransducers(Simulation S);
double EulerDistance(const double From[3], const double To[3]);
void FullFire();
void SingleFire(Transducer *,uint8_t);
void UpdateDMA_Buffer();
void UpdateDMA_BufferWith_Duty_AM();
void CleanDMABuffer();
void EnableDMAs();
void DisableDMAs();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  // Initiation
  SubControllerInit();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  DMAs[0] = hdma_memtomem_dma1_stream0;
  DMAs[1] = hdma_memtomem_dma1_stream1;
  DMAs[2] = hdma_memtomem_dma1_stream2;
  DMAs[3] = hdma_memtomem_dma2_stream0;
  DMAs[4] = hdma_memtomem_dma2_stream1;

  // uint8_t TDNum = 0;

# if PhaseTuningMode == 1
  FullFire(TDs);
# endif

  StartDMAs();

# if CircleMode == 1
    Simulation Circle;
    Circle.position[0] = -0.005;                      // X
    Circle.position[1] = 0;                           // Y
    Circle.position[2] = TestFocusPointDistance; // Z
    Circle.spread = 0;
    Circle.strength = 100;
# endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    DeltaTicks = SysTick->VAL;

    // Begin Task
    IndicateLEDBlink();

    // if (!HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin))
    // {
    //   TestFocusPointDistance += TestFocusPointDistance>0? -0.001:0;
    // }
    // if (!HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin))
    // {
    //   TestFocusPointDistance += TestFocusPointDistance<10? 0.001:0;
    // }

# if CircleMode == 1
      Circle.position[0] = -0.01*sin((2*pi*Timebase/2000));
      Circle.position[1] = 0.01*cos((2*pi*Timebase/2000));
      Circle.position[2] = TestFocusPointDistance;
      UpdateTransducers(Circle);
# endif

    HAL_Delay(200);

    // SendDebuggingInfo();
    
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

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 13;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x009034B6;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  __HAL_FREEZE_TIM1_DBGMCU();
  /* USER CODE END TIM1_Init 2 */
}

/**
 * Enable DMA controller clock
 * Configure DMA for memory to memory transfers
 *   hdma_memtomem_dma1_stream0
 *   hdma_memtomem_dma1_stream1
 *   hdma_memtomem_dma1_stream2
 *   hdma_memtomem_dma2_stream0
 *   hdma_memtomem_dma2_stream1
 *   hdma_memtomem_dma2_stream3
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_stream0 on DMA1_Stream0 */
  hdma_memtomem_dma1_stream0.Instance = DMA1_Stream0;
  hdma_memtomem_dma1_stream0.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma1_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_stream0.Init.MemInc = DMA_MINC_DISABLE;
  hdma_memtomem_dma1_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma1_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma1_stream0.Init.Mode = DMA_CIRCULAR;
  hdma_memtomem_dma1_stream0.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma1_stream0.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_memtomem_dma1_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma_memtomem_dma1_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma1_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_stream0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure DMA request hdma_memtomem_dma1_stream1 on DMA1_Stream1 */
  hdma_memtomem_dma1_stream1.Instance = DMA1_Stream1;
  hdma_memtomem_dma1_stream1.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma1_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_stream1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_stream1.Init.MemInc = DMA_MINC_DISABLE;
  hdma_memtomem_dma1_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma1_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma1_stream1.Init.Mode = DMA_CIRCULAR;
  hdma_memtomem_dma1_stream1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma1_stream1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_memtomem_dma1_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma_memtomem_dma1_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma1_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_stream1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure DMA request hdma_memtomem_dma1_stream2 on DMA1_Stream2 */
  hdma_memtomem_dma1_stream2.Instance = DMA1_Stream2;
  hdma_memtomem_dma1_stream2.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma1_stream2.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_stream2.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_stream2.Init.MemInc = DMA_MINC_DISABLE;
  hdma_memtomem_dma1_stream2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma1_stream2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma1_stream2.Init.Mode = DMA_CIRCULAR;
  hdma_memtomem_dma1_stream2.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma1_stream2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_memtomem_dma1_stream2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma_memtomem_dma1_stream2.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma1_stream2.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_stream2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_DISABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_CIRCULAR;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
  hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
  hdma_memtomem_dma2_stream1.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_DISABLE;
  hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream1.Init.Mode = DMA_CIRCULAR;
  hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure DMA request hdma_memtomem_dma2_stream3 on DMA2_Stream3 */
  hdma_memtomem_dma2_stream3.Instance = DMA2_Stream3;
  hdma_memtomem_dma2_stream3.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma2_stream3.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream3.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream3.Init.MemInc = DMA_MINC_DISABLE;
  hdma_memtomem_dma2_stream3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream3.Init.Mode = DMA_CIRCULAR;
  hdma_memtomem_dma2_stream3.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_memtomem_dma2_stream3.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma_memtomem_dma2_stream3.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream3.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream3) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7 | LED2_Pin | LED1_Pin | LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY0_Pin KEY1_Pin */
  GPIO_InitStruct.Pin = KEY0_Pin | KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB13 PB14 PB15 PB4
                           PB5 PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD2 PD3 PD4 PD5
                           PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin LED0_Pin */
  GPIO_InitStruct.Pin = LED2_Pin | LED1_Pin | LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void StartDMAs()
{
  HAL_DMA_Start(&hdma_memtomem_dma1_stream0, (uint32_t)(dma_buff[0]), (uint32_t)(&(GPIOA->ODR)), sizeof(dma_buff[0]) / sizeof(dma_buff[0][0]));
  HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)(dma_buff[1]), (uint32_t)(&(GPIOB->ODR)), sizeof(dma_buff[1]) / sizeof(dma_buff[1][0]));
  HAL_DMA_Start(&hdma_memtomem_dma1_stream2, (uint32_t)(dma_buff[2]), (uint32_t)(&(GPIOC->ODR)), sizeof(dma_buff[2]) / sizeof(dma_buff[2][0]));
  HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)(dma_buff[3]), (uint32_t)(&(GPIOD->ODR)), sizeof(dma_buff[3]) / sizeof(dma_buff[3][0]));
  HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)(dma_buff[4]), (uint32_t)(&(GPIOE->ODR)), sizeof(dma_buff[4]) / sizeof(dma_buff[4][0]));
  HAL_DMA_Start(&hdma_memtomem_dma2_stream3, (uint32_t)(dma_buff[5]), (uint32_t)(&(GPIOF->ODR)), sizeof(dma_buff[5]) / sizeof(dma_buff[5][0]));
  DMA_Enable_Flag = 1;
}

void EnableDMAs()
{
  for (int i = 0; i < 5; i++)
  {
    __HAL_DMA_ENABLE(&DMAs[i]);
  }
  DMA_Enable_Flag = 1;
}

void DisableDMAs()
{
  for (int i = 0; i < 5; i++)
  {
    __HAL_DMA_DISABLE(&DMAs[i]);
  }
  DMA_Enable_Flag = 0;
}

void UpdateDMA_Buffer()
{
  // CleanDMABuffer();
  for (int i = 0; i < NumTransducer; i++)
  {
    Transducer *TempT = TDs[i];
    for (int j = 0; j < DMA_Buffer_Resolution; j++)
    {
      if (((uint8_t)((j+ TempT->calib + TempT->shift_buffer_bits + (GPIO_Output_Offset[TempT->port_num] * BufferGapPerMicroseconds)) / (DMA_Buffer_Resolution / 2)) % 2 == 0))
      {
        dma_buff[TempT->port_num][j] |= (TempT->pin); // Set to High Level
      }
      else
      {
        dma_buff[TempT->port_num][j] &= ~(TempT->pin); // Set to Low Level
      }
    }
  }
  
}

void UpdateDMA_BufferWith_Duty_AM()
{
  // CleanDMABuffer();
  for (int i = 0; i < NumTransducer; i++)
  {
    Transducer *TempT = TDs[i];
    for (int j = 0; j < DMA_Buffer_Resolution; j++)
    {
      uint16_t Total_Index = (j + TempT->calib+ TempT->shift_buffer_bits + (GPIO_Output_Offset[TempT->port_num] * BufferGapPerMicroseconds));

      double duty = 0.35 * sin((2.0 * pi * Total_Index) / DMA_Buffer_Resolution) + 0.6;
      // duty = 1;

      uint16_t High_Level_Buffer_Length = duty * (MainWaveLengthInBuffer / 2);

      if ((Total_Index % MainWaveLengthInBuffer) <= High_Level_Buffer_Length)
      {
        dma_buff[TempT->port_num][j] |= (TempT->pin); // Set to High Level
      }
      else
      {
        dma_buff[TempT->port_num][j] &= ~(TempT->pin); // Set to Low Level
      }
    }
  }
}

void CleanDMABuffer()
{
  memset(dma_buff, 0x0000, sizeof(dma_buff));
}

void SingleFire(Transducer *TempT, uint8_t Clean)
{
  if(Clean == 1U) CleanDMABuffer();

  for (int j = 0; j < DMA_Buffer_Resolution; j++)
  {
    if (((uint16_t)((j+ TempT->calib + TempT->shift_buffer_bits + (GPIO_Output_Offset[TempT->port_num] * BufferGapPerMicroseconds)) / (DMA_Buffer_Resolution / 2)) % 2 == 0))
    {
      dma_buff[TempT->port_num][j] &= ~(TempT->pin); // Set to Low Level
    }
    else
    {
      dma_buff[TempT->port_num][j] |= (TempT->pin); // Set to High Level
    }
  }
}

void FullFire(Transducer *T[])
{
  CleanDMABuffer();
  for (int i = 0; i < NumTransducer; i++)
  {
    Transducer *TempT = TDs[i];
    for (int j = 0; j < DMA_Buffer_Resolution; j++)
    {
      if (((uint16_t)((j+ TempT->calib + (GPIO_Output_Offset[TempT->port_num] * BufferGapPerMicroseconds)) / (DMA_Buffer_Resolution / 2)) % 2 == 0))
      {
        dma_buff[TempT->port_num][j] |= (TempT->pin); // Set to High Level
      }
      else
      {
        dma_buff[TempT->port_num][j] &= ~(TempT->pin); // Set to Low Level
      }
    }
  }
}

void SendDebuggingInfo()
{
  char TargetStr[100];
  const char str[4] = "FPS:";

  strcat(TargetStr, str);

  // uint32_t len = strlen(TargetStr);
  // CDC_Transmit_FS(TargetStr, len);
}

void SubControllerInit()
{
  for (int i = 0; i < NumTransducer; i++)
  {
    TDs[i] = (Transducer *)malloc(sizeof(Transducer));
    TDs[i]->Index = i;
    TDs[i]->port = map_pin_name_to_gpio_port(pins[i]);
    TDs[i]->port_num = map_pin_name_to_gpio_port_num(pins[i]);
    TDs[i]->pin = map_pin_name_to_pin_number(pins[i]);
    TDs[i]->calib = calibe_times[i]*BufferGapPerMicroseconds;
    TDs[i]->row = i / ArraySize;
    TDs[i]->column = i % ArraySize;
    TDs[i]->coordinate[0] = (TDs[i]->row - (ArraySize / 2.0) + 0.5) * TransducerGap;    // X
    TDs[i]->coordinate[1] = (TDs[i]->column - (ArraySize / 2.0) + 0.5) * TransducerGap; // Y
    TDs[i]->coordinate[2] = 0; // Z         
    TDs[i]->phase = 0;                                                
    TDs[i]->duty = 0.5;
    TDs[i]->shift_buffer_bits = 0;
  }

  UpdateTransducers(TestShootParameters());
}

Simulation TestShootParameters()
{
  // Test Shoot Parameters
  Simulation TestShoot;
  TestShoot.position[0] = 0;                      // X
  TestShoot.position[1] = 0;                      // Y
  TestShoot.position[2] = TestFocusPointDistance; // Z
  TestShoot.spread = 0;
  TestShoot.strength = 100;
  return TestShoot;
}

// Update Simulation to Transducers Parameters
void UpdateTransducers(Simulation S)
{
  for (int i = 0; i < NumTransducer; i++)
  {
    // Distance Calculation
    TDs[i]->distance = EulerDistance(TDs[i]->coordinate, S.position);

    // Distance to Phase
    TDs[i]-> phase = (2 * pi) - (fmod((TDs[i]->distance * Wave_K), (2.0 * pi)));

    if(TwinTrap == 1) TDs[i]-> phase += (TDs[i]->column>3) ? pi/2 : 2.5*pi;
    // Phase to Gap Ticks
    TDs[i]->shift_buffer_bits = (TDs[i]->phase / (2 * pi * MainFrequency)) / TimeGapPerDMABufferBit;
  }

  DMA_Buffer_Update;
  // UpdateDMA_Buffer(T);
  // UpdateDMA_BufferWith_Duty_AM(T);
}

double EulerDistance(const double From[3], const double To[3])
{
  double TempDistance = 0.0;
  for (int i = 0; i < 3; i++)
  {
    double diff = From[i] - To[i];
    TempDistance += diff * diff; // Avoid calling pow() for exponent 2, use direct multiplication
  }
  return sqrt(TempDistance);
}

void CalculateFPS()
{
  // Calculate DeltaTicks
  if (SysTick->VAL < DeltaTicks)
  {
    DeltaTicks = SysTick->LOAD - DeltaTicks + SysTick->VAL;
  }
  else
  {
    DeltaTicks = SysTick->VAL - DeltaTicks;
  }

  // Calculate FPS
  FPS = (SystemCoreClock / (float)DeltaTicks);
}

void IndicateLEDBlink()
{
  if (LEDTicks >= LiveLEDPeriod * 500)
  {
    // HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    for (int i = 0; i < DMA_Buffer_Resolution; i++)
    {
      dma_buff[LED0_GPIO_Port_Num][i] ^= LED0_Pin;
    }
    LEDTicks = 0;
  }
}

GPIO_TypeDef *map_pin_name_to_gpio_port(const char *pin_name)
{
  if (pin_name == NULL)
    return NULL;

  switch (pin_name[1])
  {
  case 'A':
    return GPIOA;
  case 'B':
    return GPIOB;
  case 'C':
    return GPIOC;
  case 'D':
    return GPIOD;
  case 'E':
    return GPIOE;
  case 'F':
    return GPIOF;
  default:
    return GPIOA;
  }
}

uint8_t map_pin_name_to_gpio_port_num(const char *pin_name)
{
  if (pin_name == NULL)
    return 0;

  switch (pin_name[1])
  {
  case 'A':
    return 0U;
  case 'B':
    return 1U;
  case 'C':
    return 2U;
  case 'D':
    return 3U;
  case 'E':
    return 4U;
  case 'F':
    return 5U;
  default:
    // Error_Handler();
    return 0U;
  }
}

uint16_t map_pin_name_to_pin_number(const char *pin_name)
{

  if (pin_name == NULL)
    return 0;

  char pin_number_str[4];
  strncpy(pin_number_str, &pin_name[2], 3);
  pin_number_str[3] = '\0';

  int pin_number = atoi(pin_number_str);
  switch (pin_number)
  {
  case 0:
    return GPIO_PIN_0;
  case 1:
    return GPIO_PIN_1;
  case 2:
    return GPIO_PIN_2;
  case 3:
    return GPIO_PIN_3;
  case 4:
    return GPIO_PIN_4;
  case 5:
    return GPIO_PIN_5;
  case 6:
    return GPIO_PIN_6;
  case 7:
    return GPIO_PIN_7;
  case 8:
    return GPIO_PIN_8;
  case 9:
    return GPIO_PIN_9;
  case 10:
    return GPIO_PIN_10;
  case 11:
    return GPIO_PIN_11;
  case 12:
    return GPIO_PIN_12;
  case 13:
    return GPIO_PIN_13;
  case 14:
    return GPIO_PIN_14;
  case 15:
    return GPIO_PIN_15;
  default:
    return GPIO_PIN_0;
  }
}

void HAL_Delay_us(uint32_t nus)
{
  uint32_t told, tnow, tcnt = 0;
  told = SysTick->VAL;
  while (1)
  {
    tnow = SysTick->VAL;
    if (tnow != told)
    {
      if (tnow < told)
        tcnt += told - tnow;
      else
        tcnt += SysTick->LOAD - tnow + told;
      told = tnow;
      if (tcnt >= nus * (SystemCoreClock / 1000000))
        break;
    }
  };
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

  for (size_t i = 0; i < 64; ++i)
  {
    free(TDs[i]);
  }

  while (1)
  {
    TOGGLE_PIN(LED0_GPIO_Port, LED0_Pin);
    TOGGLE_PIN(LED1_GPIO_Port, LED1_Pin);
    TOGGLE_PIN(LED2_GPIO_Port, LED2_Pin);
    HAL_Delay(500);
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
