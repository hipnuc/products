#include "hipnuc_board.h"
#include <string.h>
#include "hipnuc_j1939.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_can.h"
#include "misc.h"

#define CAN_RX_FIFO_SIZE 64U          /* 63 usable slots; one distinguishes full from empty */
static CanRxMsg can_fifo[CAN_RX_FIFO_SIZE];
static volatile uint16_t can_fifo_head; /* only the ISR writes */
static volatile uint16_t can_fifo_tail; /* only the main loop writes */
static volatile uint32_t can_received, can_drops, can_overruns, tick_ms;
static uint8_t device_node;
static hipnuc_board_stats_t stats;

void SysTick_Handler(void) { tick_ms++; }
uint32_t hipnuc_board_millis(void) { return tick_ms; }

static void console_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
}

static int can_init(uint16_t kbps)
{
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;
    NVIC_InitTypeDef nvic;

    if (kbps != 125 && kbps != 250 && kbps != 500 && kbps != 1000) return 0;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    CAN_DeInit(CAN1);
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = ENABLE;              /* retain old FIFO frames when hardware is full */
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_5tq;
    can.CAN_BS2 = CAN_BS2_3tq;
    can.CAN_Prescaler = (uint16_t)(4000U / kbps); /* 36 MHz / (9 time quanta * bitrate) */
    if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success) return 0;

    memset(&filter, 0, sizeof(filter));
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);           /* all IDs -> FIFO0; filter source in the main loop */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    return 1;
}

static void receive_fifo0(CanRxMsg *frame)
{
    uint32_t id = CAN1->sFIFOMailBox[0].RIR;
    uint32_t length = CAN1->sFIFOMailBox[0].RDTR;
    uint32_t low = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t high = CAN1->sFIFOMailBox[0].RDHR;
    unsigned i;
    frame->IDE = (uint8_t)(id & 4U);
    frame->RTR = (uint8_t)(id & 2U);
    frame->StdId = id >> 21;
    frame->ExtId = id >> 3;
    frame->DLC = (uint8_t)(length & 15U);
    frame->FMI = (uint8_t)(length >> 8);
    for (i = 0; i < 4; ++i) {
        frame->Data[i] = (uint8_t)(low >> (i * 8));
        frame->Data[i + 4] = (uint8_t)(high >> (i * 8));
    }
    /* StdPeriph CAN_Receive uses |= here, which can clear a new W1C overflow
     * flag during the mailbox copy. Release only; leave that flag observable. */
    CAN1->RF0R = CAN_RF0R_RFOM0;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    for (;;) {
        CanRxMsg received;
        uint16_t next = (uint16_t)((can_fifo_head + 1U) % CAN_RX_FIFO_SIZE);
        if (CAN_GetFlagStatus(CAN1, CAN_FLAG_FOV0) != RESET) {
            can_overruns++;
            CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
        }
        if (!CAN_MessagePending(CAN1, CAN_FIFO0)) break;
        receive_fifo0(&received);
        can_received++;
        if (next == can_fifo_tail) {
            can_drops++;               /* never move the consumer's tail from the ISR */
        } else {
            can_fifo[can_fifo_head] = received;
            __DMB();                  /* publish the complete frame */
            can_fifo_head = next;
        }
    }
}

static int receive_frame(CanRxMsg *frame)
{
    if (can_fifo_head == can_fifo_tail) return 0;
    __DMB();
    *frame = can_fifo[can_fifo_tail];
    __DMB();                          /* an ISR during this copy must not reuse this slot */
    can_fifo_tail = (uint16_t)((can_fifo_tail + 1U) % CAN_RX_FIFO_SIZE);
    return 1;
}

int hipnuc_board_init(uint16_t bitrate_kbps, uint8_t source_address)
{
    memset(&stats, 0, sizeof(stats));
    can_fifo_head = can_fifo_tail = 0;
    can_received = can_drops = can_overruns = tick_ms = 0;
    device_node = source_address;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Config(SystemCoreClock / 1000U);
    console_init();
    return can_init(bitrate_kbps);
}

int hipnuc_board_poll(hipnuc_sample_t *sample)
{
    CanRxMsg received;
    while (receive_frame(&received)) {
        hipnuc_can_frame_t frame;
        int result;
        if (received.IDE != CAN_Id_Extended || (uint8_t)received.ExtId != device_node) continue;
        memset(&frame, 0, sizeof(frame));
        frame.id = received.ExtId;
        frame.is_extended = 1;
        frame.is_remote = received.RTR == CAN_RTR_Remote;
        frame.len = received.DLC;
        if (frame.len <= 8) memcpy(frame.data, received.Data, frame.len);
        result = hipnuc_j1939_parse(&frame, sample, NULL);
        if (result > 0) {
            stats.frames++;
            return 1;
        }
        if (result < 0) stats.invalid_frames++;
    }
    return 0;
}

const hipnuc_board_stats_t *hipnuc_board_stats(void)
{
    stats.received = can_received;
    stats.queue_drops = can_drops;
    stats.hardware_overruns = can_overruns;
    return &stats;
}
