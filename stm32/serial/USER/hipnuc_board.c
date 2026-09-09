/*
 * Board support for the HiPNUC serial example (STM32F103, StdPeriph).
 * See hipnuc_board.h.
 *
 * DMA mode: USART2 RX is written by DMA1 channel 6 into a circular buffer.
 * Transfer-complete events and CNDTR give an accumulated byte count. The
 * main loop owns the consumed count; hardware wraps do not reset it.
 *
 * Interrupt mode: the RXNE handler appends to a ring buffer and observes the
 * UART error flags; the main loop drains the buffer.
 */

#include "hipnuc_board.h"

#include <string.h>

#include "hipnuc_dec.h"
#include "nmea_dec.h"
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#define RX_SIZE HIPNUC_BOARD_RX_BUFFER_SIZE
#if RX_SIZE < 2 || RX_SIZE > 32768 || (RX_SIZE & (RX_SIZE - 1)) != 0
#error "The receive buffer must be a power of two between 2 and 32768 bytes"
#endif

static volatile uint8_t rx_buf[RX_SIZE];
#if HIPNUC_BOARD_USE_DMA
static volatile uint32_t rx_wraps;           /* DMA: transfer-complete count */
static uint32_t rx_consumed;                 /* DMA: accumulated bytes consumed */
#else
static volatile uint16_t rx_head;            /* IRQ mode: next write index */
static volatile uint16_t rx_tail;            /* IRQ mode: only the main loop writes */
static volatile uint32_t rx_bytes, rx_dropped;
static uint32_t rx_dropped_seen;
#endif
static volatile uint32_t tick_ms;
static volatile uint32_t rx_hardware_errors;
static uint32_t rx_hardware_errors_seen;

static hipnuc_raw_t decoder;
static nmea_raw_t nmea;
static hipnuc_board_stats_t stats;

static void reset_decoder(void)
{
    /* A receive gap invalidates a partial frame, but not lifetime counters. */
    uint32_t crc = decoder.crc_error_count;
    uint32_t invalid = decoder.invalid_count;
    memset(&decoder, 0, sizeof(decoder));
    decoder.crc_error_count = crc;
    decoder.invalid_count = invalid;
    nmea.nbyte = 0;
}

void SysTick_Handler(void)
{
    tick_ms++;
}

uint32_t hipnuc_board_millis(void)
{
    return tick_ms;
}

/* SR followed by DR clears UART errors. Capture them before that sequence,
 * including when polling TXE while sending a command. Do not let a CPU DR
 * read silently compete with DMA: pause requests and mark a receive gap. */
static uint32_t uart_status(void)
{
    uint32_t mask = __get_PRIMASK();
    uint32_t status;
    __disable_irq();
    status = USART2->SR;
    if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
#if HIPNUC_BOARD_USE_DMA
        USART_DMACmd(USART2, USART_DMAReq_Rx, DISABLE);
#else
        if (status & USART_SR_RXNE) rx_bytes++;
#endif
        (void)USART_ReceiveData(USART2);
        rx_hardware_errors++;
#if HIPNUC_BOARD_USE_DMA
        USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
#endif
    }
    __set_PRIMASK(mask);
    return status;
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

#if HIPNUC_BOARD_USE_DMA

static void dma_init(void)
{
    DMA_InitTypeDef dma;
    NVIC_InitTypeDef nvic;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel6);
    dma.DMA_PeripheralBaseAddr = (uint32_t)(uintptr_t)&USART2->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)(uintptr_t)rx_buf;
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

static uint32_t dma_produced(void)
{
    uint32_t mask = __get_PRIMASK();
    uint32_t produced;
    uint16_t remaining;

    /* DMA keeps running. Count a pending TC ourselves while the ISR cannot
     * race us, and retry if hardware wraps during the CNDTR snapshot. A TC
     * flag holds only one event: interrupts must run within one buffer time. */
    __disable_irq();
    do {
        if (DMA_GetFlagStatus(DMA1_FLAG_TC6) != RESET) {
            DMA_ClearFlag(DMA1_FLAG_TC6);
            rx_wraps++;
        }
        remaining = DMA_GetCurrDataCounter(DMA1_Channel6);
    } while (remaining == 0 || DMA_GetFlagStatus(DMA1_FLAG_TC6) != RESET);
    produced = rx_wraps * RX_SIZE + (RX_SIZE - remaining);
    __DMB();
    __set_PRIMASK(mask);
    return produced;
}

/* Drop the entire affected span when the oldest unread byte is overwritten. */
static uint16_t rx_available(void)
{
    uint32_t hardware_errors = rx_hardware_errors;
    uint32_t produced = dma_produced();
    uint32_t available = produced - rx_consumed;
    stats.bytes = produced;
    if (hardware_errors != rx_hardware_errors_seen) {
        rx_hardware_errors_seen = hardware_errors;
        rx_consumed = produced;
        reset_decoder();
        return 0;
    }
    if (available > RX_SIZE) {
        stats.overruns++;
        rx_consumed = produced;
        reset_decoder();
        return 0;
    }
    return (uint16_t)available;
}

#else /* interrupt per byte */

static uint16_t rx_available(void)
{
    uint16_t head;
    stats.bytes = rx_bytes;
    if (rx_dropped != rx_dropped_seen || rx_hardware_errors != rx_hardware_errors_seen) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        rx_tail = rx_head;
        rx_dropped_seen = rx_dropped;
        rx_hardware_errors_seen = rx_hardware_errors;
        __set_PRIMASK(mask);
        stats.overruns = rx_dropped_seen;
        reset_decoder();
        return 0;
    }
    head = rx_head;
    __DMB();
    return (uint16_t)((head - rx_tail) & (RX_SIZE - 1U));
}

#endif

static void uart_irq_init(void)
{
    NVIC_InitTypeDef nvic;
    USART_ITConfig(USART2, USART_IT_ERR, ENABLE);
#if !HIPNUC_BOARD_USE_DMA
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
#endif
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void USART2_IRQHandler(void)
{
#if HIPNUC_BOARD_USE_DMA
    (void)uart_status();
#else
    uint32_t mask = __get_PRIMASK();
    uint32_t status;
    uint8_t ch = 0;
    int received;
    /* Keep SR+DR together: a preempting ISR must not let a new ORE be
     * silently cleared by the DR read following an older clean SR. */
    __disable_irq();
    status = uart_status();
    received = (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))
               == USART_SR_RXNE;
    if (received) ch = (uint8_t)USART_ReceiveData(USART2);
    __set_PRIMASK(mask);
    if (received) {
        uint16_t next = (uint16_t)((rx_head + 1U) % RX_SIZE);
        rx_bytes++;
        if (next == rx_tail) {
            rx_dropped++;                  /* keep unread bytes; drop this byte */
        } else {
            rx_buf[rx_head] = ch;
            __DMB();                       /* publish only after storing the byte */
            rx_head = next;
        }
    }
#endif
}

void hipnuc_board_init(uint32_t baudrate)
{
    memset(&decoder, 0, sizeof(decoder));
    memset(&stats, 0, sizeof(stats));
#if HIPNUC_BOARD_USE_DMA
    rx_wraps = 0;
    rx_consumed = 0;
#else
    rx_tail = 0;
    rx_head = 0;
    rx_bytes = rx_dropped = rx_dropped_seen = 0;
#endif
    tick_ms = 0;
    rx_hardware_errors = rx_hardware_errors_seen = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Config(SystemCoreClock / 1000U);
    console_init();
    usart2_init(baudrate);
#if HIPNUC_BOARD_USE_DMA
    dma_init();
#endif
    uart_irq_init();
}

int hipnuc_board_poll(hipnuc_sample_t *sample)
{
    while (rx_available()) {
        int ret, in_binary;
#if HIPNUC_BOARD_USE_DMA
        uint8_t ch = rx_buf[rx_consumed & (RX_SIZE - 1U)];
        /* DMA could overtake us during the byte copy or a preempting ISR. */
        if (!rx_available()) return 0;
        rx_consumed++;
#else
        uint8_t ch = rx_buf[rx_tail];
        if (!rx_available()) return 0;
        __DMB();                           /* finish copying before freeing the slot */
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_SIZE);
#endif
        in_binary = decoder.nbyte != 0;
        ret = hipnuc_input(&decoder, ch);
        stats.crc_errors = decoder.crc_error_count;
        stats.invalid_frames = decoder.invalid_count;
        if (in_binary || decoder.nbyte != 0) {
            /* A binary header also cancels a truncated NMEA sentence. */
            nmea.nbyte = 0;
            if (ret > 0) {
                stats.frames++;
                if (hipnuc_sample_from_raw(&decoder, sample)) return 1;
            }
        } else if (nmea_input(&nmea, ch) > 0 && hipnuc_sample_from_nmea(&nmea, sample)) {
            stats.frames++;
            return 1;
        }
    }
    return 0;
}

const hipnuc_board_stats_t *hipnuc_board_stats(void)
{
    stats.hardware_errors = rx_hardware_errors;
    return &stats;
}

void hipnuc_board_send_command(const char *command)
{
    const char *p;
    for (p = command; *p; ++p) {
        while (!(uart_status() & USART_SR_TXE)) {}
        USART_SendData(USART2, (uint16_t)*p);
    }
    for (p = "\r\n"; *p; ++p) {
        while (!(uart_status() & USART_SR_TXE)) {}
        USART_SendData(USART2, (uint16_t)*p);
    }
}
