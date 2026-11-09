/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GK-407VE (STM32F407VET6) board support, on HAL: clocks, 1 ms tick, LED,
 * console, and the IMU's UART or CAN. In a CubeMX project this is
 * SystemClock_Config(), MX_GPIO_Init(), MX_USART1_UART_Init(),
 * MX_USART2_UART_Init(), MX_DMA_Init() and MX_CAN1_Init().
 */

#include "board.h"

#include <stdio.h>

#include "imu_can.h"
#include "imu_uart.h"
#include "stm32f4xx_hal.h"

#define LED_PORT GPIOA
#define LED_PIN  GPIO_PIN_8

UART_HandleTypeDef huart1;            /* console */
UART_HandleTypeDef huart2;            /* IMU on UART */
#if IMU_UART_DMA
DMA_HandleTypeDef hdma_usart2_rx;     /* USART2_RX: DMA1 stream 5, channel 4 */
#endif
CAN_HandleTypeDef hcan1;              /* IMU on CAN */

static void clock_config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* HSE / M = 1 MHz, x N=336 = 336 MHz VCO, /P=2 = 168 MHz SYSCLK, /Q=7 = 48 MHz. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = HSE_VALUE / 1000000U;
    osc.PLL.PLLN = 336U;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7U;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Board_Fatal("clock setup failed (crystal or PLL)");

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;  /* HCLK  168 MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV4;   /* PCLK1  42 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;   /* PCLK2  84 MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Board_Fatal("clock setup failed (SYSCLK switch)");
}

static void uart_init(UART_HandleTypeDef *h, USART_TypeDef *instance, uint32_t baud, uint32_t pins, uint32_t af)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = pins;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = af;
    HAL_GPIO_Init(GPIOA, &gpio);

    h->Instance = instance;
    h->Init.BaudRate = baud;
    h->Init.WordLength = UART_WORDLENGTH_8B;
    h->Init.StopBits = UART_STOPBITS_1;
    h->Init.Parity = UART_PARITY_NONE;
    h->Init.Mode = UART_MODE_TX_RX;
    h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(h) != HAL_OK) Board_Fatal("UART init failed");
}

/* USART2 to the IMU: PA2 TX -> IMU RX, PA3 RX <- IMU TX, and its interrupt
 * (used in interrupt mode). */
void Board_InitImuUart(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    uart_init(&huart2, USART2, IMU_UART_BAUD, GPIO_PIN_2 | GPIO_PIN_3, GPIO_AF7_USART2);
#if IMU_UART_DMA
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) Board_Fatal("DMA init failed");
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
    /* imu_uart.c needs no DMA interrupt; CubeMX enables it, which does no harm. */
#endif
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* CAN1 to the IMU through a 3.3 V transceiver: PB9 TX -> TXD, PB8 RX <- RXD,
 * and its receive interrupt. */
void Board_InitImuCan(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 14 time quanta a bit, sampled at 85.7 %: 42 MHz / 6 / 14 = 500 kbit/s.
     * PCLK1 must be a multiple of 14 x the bitrate; otherwise pick other
     * time quanta (CubeMX shows the resulting bitrate). */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = HAL_RCC_GetPCLK1Freq() / (IMU_CAN_BITRATE * 14U);
    hcan1.Init.Mode = CAN_MODE_NORMAL;   /* this node acknowledges the IMU's frames */
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;      /* recover from bus-off without software */
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) Board_Fatal("CAN init failed");
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void Board_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_Init();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = LED_PIN;   /* first, so that a clock failure can still blink */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(LED_PORT, &gpio);
    clock_config();

    __HAL_RCC_USART1_CLK_ENABLE();   /* console: PA9 TX, PA10 RX */
    uart_init(&huart1, USART1, 115200U, GPIO_PIN_9 | GPIO_PIN_10, GPIO_AF7_USART1);
}

uint32_t Board_Ms(void)
{
    return HAL_GetTick();
}

void Board_LedToggle(void)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

/* printf() to the console (MicroLIB). With GCC, retarget _write() instead,
 * and link with -u _printf_float, or newlib-nano prints no float values. */
int fputc(int ch, FILE *f)
{
    uint8_t c[2] = {'\r', (uint8_t)ch};
    (void)f;
    if (ch == '\n') HAL_UART_Transmit(&huart1, c, 2, 10);
    else HAL_UART_Transmit(&huart1, c + 1, 1, 10);
    return ch;
}

void Board_Fatal(const char *why)
{
    uint32_t blink = 0U;
    volatile uint32_t n;

    __disable_irq();
    for (;;) {
        if ((blink++ % 16U) == 0U) printf("\nERROR: %s\n", why);
        Board_LedToggle();
        for (n = SystemCoreClock / 80U; n != 0U; n--) {   /* about 0.1 s; SysTick cannot run in here */
        }
    }
}
