/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "malloc.h"
#include <stdlib.h>

#include "usbd_cdc_if.h"

/* SSD1306 */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* SSD1306 */

/* W5500 */
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "socket.h"
#include "dhcp.h"
#include "httpServer.h"
/* W5500 */

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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ROTARY ENCODER */
#define MAX_VALUE      100
#define TICKS_PER_STEP 4 // each step of the encoder sends 4 events

int16_t encoderValue = 0;
int16_t prevEncoderValue = -1;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  encoderValue = (int16_t)__HAL_TIM_GET_COUNTER(htim) / TICKS_PER_STEP;
  if (encoderValue < 0) {
    encoderValue = 0;
    __HAL_TIM_SET_COUNTER(htim, encoderValue);
  }
  if (encoderValue >= MAX_VALUE) {
    encoderValue = MAX_VALUE;
    __HAL_TIM_SET_COUNTER(htim, encoderValue * TICKS_PER_STEP);
  }
}
/* ROTARY ENCODER */

extern char *sbrk(int i);

extern char _end;
extern char _sdata;
extern char _estack;
extern char _Min_Stack_Size;

static char *ramend = &_estack;

/* W5500 */
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define MAX_HTTPSOCK    6
#define index_page "<!DOCTYPE html>"\
  "<html>"\
    "<head>"\
      "<title>W5500-STM32 Web Server</title>"\
      "<meta http-equiv='Content-Type' content='text/html; charset=utf-8'>"\
      "<link href=\"data:image/x-icon;base64,A\" rel=\"icon\" type=\"image/x-icon\">"\
      "<style>"\
        "html {display: inline-block; margin: 0px auto; text-align: center;}"\
        "body{margin-top: 50px;}"\
        ".button {display: block;"\
          "width: 70px;"\
          "background-color: #008000;"\
          "border: none;"\
          "color: white;"\
          "padding: 14px 28px;"\
          "text-decoration: none;"\
          "font-size: 24px;"\
          "margin: 0px auto 36px;"\
          "border-radius: 5px;}"\
        ".button-on {background-color: #008000;}"\
        ".button-on:active{background-color: #008000;}"\
        ".button-off {background-color: #808080;}"\
        ".button-off:active {background-color: #808080;}"\
        "p {font-size: 20px;color: #808080;margin-bottom: 20px;}"\
      "</style>"\
    "</head>"\
    "<body>"\
      "<h1>STM32 - W5500</h1>"\
      "<p>Control the light via Ethernet</p>"\
      "<a class=\"button button-on\" href=\"/ledon.html\">ON</a>"\
      "<a class=\"button button-off\" href=\"/ledoff.html\">OFF</a>"\
    "</body>"\
  "</html>"

extern uint8_t *__sbrk_heap_end;

uint8_t socknumlist[] = {2, 3, 4, 5, 6, 7};
uint8_t RX_BUF[1024];
uint8_t TX_BUF[1024];
wiz_NetInfo net_info = {
    .mac  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED },
    .dhcp = NETINFO_DHCP
};

void wizchipSelect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

void wizchipUnselect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

void wizchipReadBurst(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}

void wizchipWriteBurst(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
}

uint8_t wizchipReadByte(void) {
    uint8_t byte;
    wizchipReadBurst(&byte, sizeof(byte));
    return byte;
}

void wizchipWriteByte(uint8_t byte) {
    wizchipWriteBurst(&byte, sizeof(byte));
}

volatile bool ip_assigned = false;

void Callback_IPAssigned(void) {
    ip_assigned = true;
}

void Callback_IPConflict(void) {
    ip_assigned = false;
}

uint8_t dhcp_buffer[1024];
uint8_t dns_buffer[1024];

void W5500Init() {

    // Register W5500 callbacks
    reg_wizchip_cs_cbfunc(wizchipSelect, wizchipUnselect);
    reg_wizchip_spi_cbfunc(wizchipReadByte, wizchipWriteByte);
    reg_wizchip_spiburst_cbfunc(wizchipReadBurst, wizchipWriteBurst);

    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

    // set MAC address before using DHCP
    setSHAR(net_info.mac);
    DHCP_init(DHCP_SOCKET, dhcp_buffer);

    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );

    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString("Waiting IP", Font_7x10, White);
    ssd1306_UpdateScreen();

    uint32_t ctr = 10000;
    while((!ip_assigned) && (ctr > 0)) {
      ssd1306_SetCursor(72, 12);
      if (ctr % 2 == 0) {
        ssd1306_WriteString("   ", Font_7x10, White);
      } else {
        ssd1306_WriteString("...", Font_7x10, White);
      }
      ssd1306_UpdateScreen();

      printf("Waiting IP...\n");
      DHCP_run();
      HAL_Delay(500);
      ctr--;
    }

    if(!ip_assigned) {
      ssd1306_SetCursor(0, 12);
      ssd1306_WriteString("IP not assigned", Font_7x10, White);
      ssd1306_UpdateScreen();
      return;
    }

    getIPfromDHCP(net_info.ip);
    getGWfromDHCP(net_info.gw);
    getSNfromDHCP(net_info.sn);

    char ipStr[30];
    sprintf(ipStr, "IP: %d.%d.%d.%d", net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3]);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString(ipStr, Font_7x10, White);
    ssd1306_UpdateScreen();

    wizchip_setnetinfo(&net_info);

    // set the TCP Timeout, needed when the connection is lost/cut in the middle of a request
    wiz_NetTimeout timeout;
    wizchip_gettimeout(&timeout);
    printf("OLD TIMEOUT: count: %d, time_100us: %d \n", timeout.retry_cnt, timeout.time_100us);

    timeout.retry_cnt = 4;
    timeout.time_100us = 1000;
    wizchip_settimeout(&timeout);

    wizchip_gettimeout(&timeout);
    printf("NEW TIMEOUT: count: %d, time_100us: %d \n", timeout.retry_cnt, timeout.time_100us);
}

void led_on(uint8_t **content, uint32_t *content_len) {
  uint8_t *mem = malloc(10);
  strcpy((char*)mem, "LED ON");

  *content = mem;
  *content_len = strlen((char*)mem);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void led_off(uint8_t **content, uint32_t *content_len) {
  uint8_t *mem = malloc(10);
  strcpy((char*)mem, "LED OFF");

  *content = mem;
  *content_len = strlen((char*)mem);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void get_encoder_value(uint8_t **content, uint32_t *content_len) {
  char str[10];
  sprintf(str, "%d", encoderValue);

  uint8_t *mem = malloc(10);
  strcpy((char*)mem, str);
  *content = mem;
  *content_len = strlen((char*)mem);
}
/* W5500 */

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  ssd1306_Init();

  printf("STM32 cdc printf\n");

  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("STM32 OLED Display", Font_7x10, White);
  ssd1306_SetCursor(0, 18);
  ssd1306_WriteString("", Font_7x10, White);
  ssd1306_UpdateScreen();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // start PWM on PB4
  htim3.Instance->CCR1 = MAX_VALUE;         // set pwm to max (led off)

  W5500Init();
  httpServer_init(TX_BUF, RX_BUF, MAX_HTTPSOCK, socknumlist);
  //reg_httpServer_cbfunc(NVIC_SystemReset, NULL); // remover

  /* Web content registration */
  reg_httpServer_webContent((uint8_t *)"index.html", (uint8_t *)index_page);

  /* Register function to respond to API call */
  reg_httpServer_api((uint8_t *)"api/led/on", led_on);
  reg_httpServer_api((uint8_t *)"api/led/off", led_off);

  // TODO criar uma função que retorne o valor do encoder
  reg_httpServer_api((uint8_t *)"api/encoder", get_encoder_value);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char str[10];
  float widthFactor = 1.28;
  while (1) {
    for(int i = 0; i < MAX_HTTPSOCK; i++) httpServer_run(i); // HTTP Server handler

    if (prevEncoderValue != encoderValue) {
      htim3.Instance->CCR1 = (MAX_VALUE - encoderValue);

      // draw a progress bar indicating the value
      ssd1306_FillRectangle(0, 23, SSD1306_WIDTH, 30, Black);
      ssd1306_FillRectangle(0, 23, encoderValue * widthFactor, 30, White);

      sprintf(str, "%-5d", encoderValue);
      ssd1306_SetCursor(0, 35);
      ssd1306_WriteString(str, Font_7x10, White);
      ssd1306_SetCursor(60, 35);
      if (encoderValue > prevEncoderValue) {
        ssd1306_WriteString(">", Font_7x10, White);
      } else if (encoderValue < prevEncoderValue) {
        ssd1306_WriteString("<", Font_7x10, White);
      } else {
        ssd1306_WriteString("-", Font_7x10, White);
      }
      prevEncoderValue = encoderValue;

      ssd1306_UpdateScreen();
    }


    // each second
    if (HAL_GetTick() % 1000 == 0) {
      struct mallinfo mi = mallinfo();
      char *heapend = (char*)sbrk(0);
      char * stack_ptr = (char*)__get_MSP();

      char *minSP = (char*)(ramend - &_Min_Stack_Size);

      int free = ((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks;

      ssd1306_SetCursor(0, 45);
      sprintf(str, "%d", free);
      ssd1306_WriteString(str, Font_7x10, White);

      ssd1306_UpdateScreen();
    }


    //ssd1306_SetCursor(0, 45);
    //sprintf(str, "%d", HAL_GetTick() % 1000);
    //ssd1306_WriteString(str, Font_7x10, White);


    //counter++;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_CS_Pin */
  GPIO_InitStruct.Pin = W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W5500_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// https://github.com/alexeykosinov/Redirect-printf-to-USB-VCP-on-STM32H7-MCU
int _write(int file, char *ptr, int len) {
    CDC_Transmit_FS((uint8_t*) ptr, len); return len;
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
