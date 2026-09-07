/*
 * HiPNUC CAN (J1939) example for STM32F103 (Keil MDK, StdPeriph).
 *
 * Wiring:  CAN transceiver on PA11 (CAN_RX) / PA12 (CAN_TX), 120 ohm termination
 *          console: PA9 (USART1 TX), 115200 8N1
 *
 * Change the settings below, build, download, open a terminal on USART1.
 * Each J1939 frame carries a few fields; they are merged per device node
 * and printed at a slow rate. Put your own code where "your application"
 * is marked.
 */
#include <stdio.h>
#include "delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_can.h"
#include "misc.h"

#include "hipnuc_j1939.h"

/* ---- settings ---------------------------------------------------------- */
#define CAN_BAUD_KBPS     500      /* device factory default 500 kbit/s: 125, 250, 500 or 1000 */
#define DEVICE_NODE_ID    8        /* J1939 source address of the device (factory default 8) */
#define PRINT_PERIOD_MS   200      /* how often to print; 0 = never */

/* ---- receive queue filled by the CAN RX0 interrupt --------------------- */
#define CAN_RX_FIFO_SIZE 64
static volatile uint8_t can_fifo_head = 0;
static volatile uint8_t can_fifo_tail = 0;
static CanRxMsg can_fifo[CAN_RX_FIFO_SIZE];
static volatile uint32_t can_overruns = 0;

#define CAN_BAUD_125K 125
#define CAN_BAUD_250K 250
#define CAN_BAUD_500K 500
#define CAN_BAUD_1M   1000

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
            can_overruns++;                       /* consumer too slow: drop the oldest frame */
            can_fifo_tail = (uint8_t)((can_fifo_tail + 1) % CAN_RX_FIFO_SIZE);
        }
        can_fifo[can_fifo_head] = rx;
        can_fifo_head = next_head;
    }
}

/*
 * Function   : rx_to_frame
 * Description: Convert an STM32F10x CanRxMsg to the SDK frame type.
 */
static void rx_to_frame(const CanRxMsg *rx, hipnuc_can_frame_t *f)
{
    uint8_t i;
    f->id = (rx->IDE == CAN_Id_Extended) ? rx->ExtId : rx->StdId;
    f->is_extended = (rx->IDE == CAN_Id_Extended) ? 1 : 0;
    f->is_remote = (rx->RTR == CAN_RTR_Remote) ? 1 : 0;
    f->is_error = 0;
    f->len = (rx->DLC > 8) ? 8 : rx->DLC;
    for (i = 0; i < f->len; i++) f->data[i] = rx->Data[i];
}

int main(void)
{
    hipnuc_sample_t merged;       /* fields collected from several PGNs */
    hipnuc_sample_t part;
    uint32_t frames = 0;
    uint16_t print_timer_ms = 0;

    delay_init();
    usart1_init(115200);
    printf_welcome_information();

    can1_gpio_init();
    can1_init(CAN_BAUD_KBPS);
    can_filter_init();
    nvic_can_rx0_enable();
    hipnuc_sample_clear(&merged);

    while (1)
    {
        while (can_fifo_tail != can_fifo_head)
        {
            hipnuc_can_frame_t f;
            CanRxMsg rx = can_fifo[can_fifo_tail];
            can_fifo_tail = (uint8_t)((can_fifo_tail + 1) % CAN_RX_FIFO_SIZE);
            rx_to_frame(&rx, &f);
            if (hipnuc_j1939_parse(&f, &part, NULL) > 0 && part.node_id == DEVICE_NODE_ID)
            {
                hipnuc_j1939_merge(&merged, &part);
                frames++;
                /* ---- your application: `part` holds the fields of this frame,
                 * `merged` the latest value of every field seen so far. -------- */
            }
        }

        delay_ms(1);
        print_timer_ms++;
        if (PRINT_PERIOD_MS && print_timer_ms >= PRINT_PERIOD_MS)
        {
            print_timer_ms = 0;
            if (frames == 0)
            {
                printf("no J1939 frames from node %d: check transceiver, termination, bitrate\r\n", DEVICE_NODE_ID);
            }
            else
            {
                if (merged.valid & HIPNUC_VALID_EULER)
                    printf("roll %7.2f  pitch %7.2f  yaw %7.2f deg", merged.roll * 57.29578f,
                           merged.pitch * 57.29578f, merged.yaw * 57.29578f);
                if (merged.valid & HIPNUC_VALID_ACC)
                    printf("  acc %6.2f %6.2f %6.2f m/s2", merged.acc[0], merged.acc[1], merged.acc[2]);
                if (merged.valid & HIPNUC_VALID_GYR)
                    printf("  gyr %6.2f %6.2f %6.2f rad/s", merged.gyr[0], merged.gyr[1], merged.gyr[2]);
                printf("  frames %lu%s\r\n", (unsigned long)frames, can_overruns ? "  [RX overrun]" : "");
            }
        }
    }
}
