/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Interrupt handlers, as CubeMX generates them.
 */

#include "stm32f4xx_it.h"

#include "board.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;

void HardFault_Handler(void)
{
    Board_Fatal("HardFault");
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}
