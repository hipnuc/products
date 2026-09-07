/*
 * Board support for the HiPNUC serial example (STM32F103, StdPeriph).
 * See hipnuc_board.h.
 *
 * DMA mode: USART2 RX is written by DMA1 channel 6 into a circular buffer.
 * The main loop consumes from its own read index up to the DMA write index;
 * the transfer-complete interrupt counts wraps so an overrun (consumer more
 * than one buffer behind) is detected instead of silently corrupting data.
 *
 * Interrupt mode: the RXNE handler appends to a ring buffer; the main loop
 * drains it. The interrupt does nothing else.
 */

#include "hipnuc_board.h"

#include <string.h>

#include "hipnuc_dec.h"
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#define RX_SIZE HIPNUC_BOARD_RX_BUFFER_SIZE

static uint8_t rx_buf[RX_SIZE];
static volatile uint32_t rx_wraps;           /* DMA: transfer-complete count */
static volatile uint16_t rx_head;            /* IRQ mode: next write index */
static uint16_t rx_tail;                     /* consumer read index */
static uint32_t rx_consumed_wraps;
static volatile uint32_t tick_ms;

static hipnuc_raw_t decoder;
static hipnuc_board_stats_t stats;

void SysTick_Handler(void)
{
    tick_ms++;
}

uint32_t hipnuc_board_millis(void)
{
    return tick_ms;
}

static void usart2_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_3;                  /* RX */
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;                  /* TX */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);
    USART_Cmd(USART2, ENABLE);
}

#if HIPNUC_BOARD_USE_DMA

static void dma_init(void)
{
    DMA_InitTypeDef dma;
    NVIC_InitTypeDef nvic;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel6);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)rx_buf;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = RX_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel6, &dma);

    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
    nvic.NVIC_IRQChannel = DMA1_Channel6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    DMA_Cmd(DMA1_Channel6, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
}

void DMA1_Channel6_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC6) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_TC6);
        rx_wraps++;
    }
}

/* Bytes available and, if the DMA lapped the consumer, resynchronize. */
static uint16_t rx_available(void)
{
    uint16_t head = (uint16_t)(RX_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6));
    uint32_t wraps = rx_wraps;
    uint32_t behind = wraps - rx_consumed_wraps;

    if (behind > 1U || (behind == 1U && head >= rx_tail)) {
        /* More than one buffer of data arrived since the last poll: the
         * oldest bytes are gone. Drop everything and restart from head. */
        stats.overruns++;
        rx_tail = head;
        rx_consumed_wraps = wraps;
        memset(&decoder, 0, sizeof(decoder));
        return 0;
    }
    if (head >= rx_tail) {
        if (behind == 1U) rx_consumed_wraps = wraps;
        return (uint16_t)(head - rx_tail);
    }
    return (uint16_t)(RX_SIZE - rx_tail + head);
}

#else /* interrupt per byte */

static void irq_init(void)
{
    NVIC_InitTypeDef nvic;
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint16_t next = (uint16_t)((rx_head + 1U) % RX_SIZE);
        uint8_t ch = (uint8_t)USART_ReceiveData(USART2);
        if (next == rx_tail) {
            stats.overruns++;               /* consumer too slow: byte dropped */
        } else {
            rx_buf[rx_head] = ch;
            rx_head = next;
        }
    }
}

static uint16_t rx_available(void)
{
    uint16_t head = rx_head;
    if (head >= rx_tail) return (uint16_t)(head - rx_tail);
    return (uint16_t)(RX_SIZE - rx_tail + head);
}

#endif

void hipnuc_board_init(uint32_t baudrate)
{
    memset(&decoder, 0, sizeof(decoder));
    memset(&stats, 0, sizeof(stats));
    rx_tail = 0;
    rx_head = 0;
    rx_wraps = 0;
    rx_consumed_wraps = 0;

    SysTick_Config(SystemCoreClock / 1000U);
    usart2_init(baudrate);
#if HIPNUC_BOARD_USE_DMA
    dma_init();
#else
    irq_init();
#endif
}

int hipnuc_board_poll(hipnuc_sample_t *sample)
{
    uint16_t avail = rx_available();

    while (avail--) {
        uint8_t ch = rx_buf[rx_tail];
        int ret;
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_SIZE);
        stats.bytes++;
        ret = hipnuc_input(&decoder, ch);
        if (ret > 0) {
            stats.frames++;
            if (hipnuc_sample_from_raw(&decoder, sample)) return 1;
        } else if (ret < 0) {
            if (decoder.crc_error_count > stats.crc_errors) stats.crc_errors = decoder.crc_error_count;
            if (decoder.invalid_count > stats.invalid_frames) stats.invalid_frames = decoder.invalid_count;
        }
    }
    return 0;
}

const hipnuc_board_stats_t *hipnuc_board_stats(void)
{
    return &stats;
}

void hipnuc_board_send_command(const char *command)
{
    const char *p;
    for (p = command; *p; ++p) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART2, (uint16_t)*p);
    }
    for (p = "\r\n"; *p; ++p) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART2, (uint16_t)*p);
    }
}
