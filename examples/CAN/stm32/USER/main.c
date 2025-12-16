/*
 * File      : main.c
 * Brief     : Minimal STM32F103 CAN receive demo with JSON output
 * Hardware  : Warship STM32F103 (PA11 CAN_RX, PA12 CAN_TX) with external CAN transceiver
 * Depends   : STM32F10x StdPeriph, drivers/hipnuc_can_common, hipnuc_j1939_parser, canopen_parser
 * Style     : Simple, beginner-friendly, RT-Thread style comments for APIs
 */
#include <stdio.h>
#include "delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_can.h"
#include "misc.h"

#include "hipnuc_can_common.h"
#include "hipnuc_j1939_parser.h"
#include "canopen_parser.h"


#define CAN_RX_FIFO_SIZE 32
static volatile uint8_t can_fifo_head = 0;
static volatile uint8_t can_fifo_tail = 0;
static CanRxMsg can_fifo[CAN_RX_FIFO_SIZE];

#define CAN_BAUD_125K 125
#define CAN_BAUD_250K 250
#define CAN_BAUD_500K 500
#define CAN_BAUD_1M   1000

#define CAN_BAUD_KBPS CAN_BAUD_500K

/*
 * Function   : usart1_init
 * Description: Initialize USART1 for console output (printf) on PA9/PA10.
 * Parameters : baud - UART baudrate (e.g. 115200)
 * Return     : void
 */
static void usart1_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef us;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
    us.USART_BaudRate = baud;
    us.USART_WordLength = USART_WordLength_8b;
    us.USART_StopBits = USART_StopBits_1;
    us.USART_Parity = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &us);
    USART_Cmd(USART1, ENABLE);
}

/*
 * Function   : can1_gpio_init
 * Description: Configure CAN1 pins.
 *              PA11 as input pull-up (CAN RX), PA12 as AF push-pull (CAN TX).
 * Parameters : None
 * Return     : void
 */
static void can1_gpio_init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
}

/*
 * Function   : can_filter_init
 * Description: Configure CAN filter to accept all frames into FIFO0.
 * Parameters : None
 * Return     : void
 */
static void can_filter_init(void)
{
    CAN_FilterInitTypeDef f;
    f.CAN_FilterNumber = 0;
    f.CAN_FilterMode = CAN_FilterMode_IdMask;
    f.CAN_FilterScale = CAN_FilterScale_32bit;
    f.CAN_FilterIdHigh = 0x0000;
    f.CAN_FilterIdLow = 0x0000;
    f.CAN_FilterMaskIdHigh = 0x0000;
    f.CAN_FilterMaskIdLow = 0x0000;
    f.CAN_FilterFIFOAssignment = 0;
    f.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&f);
}

/*
 * Function   : can1_init
 * Description: Initialize CAN1 with common timing (SJW=1, BS1=5, BS2=3).
 *              Prescaler mapped to kbps: 125/250/500/1000 (APB1=36MHz).
 * Parameters : kbps - target bitrate in kbps (125/250/500/1000)
 * Return     : void
 */
static void can1_init(uint16_t kbps)
{
    CAN_InitTypeDef c;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    CAN_DeInit(CAN1);
    c.CAN_TTCM = DISABLE;
    c.CAN_ABOM = DISABLE;
    c.CAN_AWUM = DISABLE;
    c.CAN_NART = DISABLE;
    c.CAN_RFLM = DISABLE;
    c.CAN_TXFP = DISABLE;
    c.CAN_Mode = CAN_Mode_Normal;
    c.CAN_SJW = CAN_SJW_1tq;
    c.CAN_BS1 = CAN_BS1_5tq;
    c.CAN_BS2 = CAN_BS2_3tq;
    switch (kbps)
    {
        case CAN_BAUD_1M:   c.CAN_Prescaler = 4;  break;
        case CAN_BAUD_500K: c.CAN_Prescaler = 8;  break;
        case CAN_BAUD_250K: c.CAN_Prescaler = 16; break;
        case CAN_BAUD_125K: c.CAN_Prescaler = 32; break;
        default:            c.CAN_Prescaler = 8;  break;
    }
    CAN_Init(CAN1, &c);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/*
 * Function   : nvic_can_rx0_enable
 * Description: Enable CAN RX0 interrupt in NVIC.
 * Parameters : None
 * Return     : void
 */
static void nvic_can_rx0_enable(void)
{
    NVIC_InitTypeDef n;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    n.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 1;
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
}


/**
 * @brief Print system clock frequencies and reception mode
 * 
 * Prints the system clock frequencies and the USART reception mode (DMA or UART Interrupt) to the console.
 */
/*
 * Function   : printf_welcome_information
 * Description: Print basic clock info and selected CAN bitrate.
 * Parameters : None
 * Return     : void
 */
static void printf_welcome_information(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    printf("HiPNUC IMU can decode example\r\n");
    
    printf("System Clock Frequencies:\r\n");
    printf("SYSCLK: %d Hz\r\n", RCC_Clocks.SYSCLK_Frequency);
    printf("HCLK: %d Hz\r\n", RCC_Clocks.HCLK_Frequency);
    printf("Baud: %d kbps\r\n", CAN_BAUD_KBPS);
}

/*
 * Function   : USB_LP_CAN1_RX0_IRQHandler
 * Description: CAN RX0 ISR, drain hardware FIFO0 and push frames into software ring buffer.
 * Parameters : None
 * Return     : void
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0)
    {
        CanRxMsg rx;
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
        uint8_t next_head = (uint8_t)((can_fifo_head + 1) % CAN_RX_FIFO_SIZE);
        if (next_head == can_fifo_tail)
        {
            can_fifo_tail = (uint8_t)((can_fifo_tail + 1) % CAN_RX_FIFO_SIZE);
        }
        can_fifo[can_fifo_head] = rx;
        can_fifo_head = next_head;
    }
}

/*
 * Function   : rx_to_common
 * Description: Convert STM32F10x CanRxMsg to generic hipnuc_can_frame_t.
 * Parameters : rx - pointer to STM32 CAN receive message
 * Return     : generic frame structure (by value)
 */
static hipnuc_can_frame_t rx_to_common(const CanRxMsg *rx)
{
    hipnuc_can_frame_t f;
    if (rx->IDE == CAN_Id_Extended)
        f.can_id = HIPNUC_CAN_EFF_FLAG | (rx->ExtId & HIPNUC_CAN_EFF_MASK);
    else
        f.can_id = (rx->StdId & HIPNUC_CAN_SFF_MASK);
    f.can_dlc = rx->DLC;
    for (uint8_t i = 0; i < rx->DLC && i < 8; i++) f.data[i] = rx->Data[i];
    f.hw_ts_us = 0;
    return f;
}

/*
 * Function   : print_can_json_from_rx
 * Description: Parse frame via drivers (J1939 for extended, CANopen for standard)
 *              and print JSON to console. Falls back to raw JSON if unknown.
 * Parameters : rx - pointer to STM32 CAN receive message
 * Return     : void
 */
static void print_can_json_from_rx(const CanRxMsg *rx)
{
    hipnuc_can_frame_t f = rx_to_common(rx);
    can_sensor_data_t data;
    can_json_output_t out;
    int msg_type = CAN_MSG_UNKNOWN;
    data.node_id = hipnuc_can_extract_node_id(f.can_id);
    data.hw_ts_us = f.hw_ts_us;
    if (rx->IDE == CAN_Id_Extended)
        msg_type = hipnuc_j1939_parse_frame(&f, &data);
    else
        msg_type = canopen_parse_frame(&f, &data);

    if (msg_type != CAN_MSG_UNKNOWN && msg_type != CAN_MSG_ERROR)
    {
        hipnuc_can_to_json(&data, msg_type, &out);
        printf("%s", out.buffer);
    }
    else
    {
        printf("{\"id\":%u,\"ide\":\"%s\",\"dlc\":%d,\"data\":[%u,%u,%u,%u,%u,%u,%u,%u]}\n",
               (rx->IDE == CAN_Id_Extended) ? rx->ExtId : rx->StdId,
               (rx->IDE == CAN_Id_Extended) ? "ext" : "std",
               rx->DLC,
               rx->Data[0], rx->Data[1], rx->Data[2], rx->Data[3], rx->Data[4], rx->Data[5], rx->Data[6], rx->Data[7]);
    }
}

/* removed demo sender */

/*
 * Function   : main
 * Description: Initialize peripherals and run receive/print loop consuming software FIFO.
 * Parameters : None
 * Return     : int (not used)
 */
int main(void)
{
    delay_init();
    usart1_init(115200);
    
    printf_welcome_information();
    
    can1_gpio_init();
    can1_init(CAN_BAUD_KBPS);
    can_filter_init();
    nvic_can_rx0_enable();

    uint16_t print_timer_ms = 0;
    while (1)
    {
        if (can_fifo_tail != can_fifo_head)
        {
            CanRxMsg rx = can_fifo[can_fifo_tail];
            can_fifo_tail = (uint8_t)((can_fifo_tail + 1) % CAN_RX_FIFO_SIZE);
            if (print_timer_ms >= 20)
            {
                print_can_json_from_rx(&rx);
                print_timer_ms = 0;
            }
        }
        delay_ms(2);
        print_timer_ms += 2;
    }
}
