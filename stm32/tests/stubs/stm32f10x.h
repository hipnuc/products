/* Minimal register/API stand-ins for exercising the real board receive code. */
#ifndef STM32_TEST_HARDWARE_H
#define STM32_TEST_HARDWARE_H
#include <stdint.h>
#include <string.h>

typedef struct { uint32_t SR, DR; } USART_TypeDef;
typedef struct { uint32_t CNDTR; } DMA_Channel_TypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef struct { uint32_t RIR, RDTR, RDLR, RDHR; } CAN_FIFOMailBox_TypeDef;
typedef struct {
    uint32_t RF0R;
    CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
} CAN_TypeDef;
typedef struct { uint32_t GPIO_Pin, GPIO_Mode, GPIO_Speed; } GPIO_InitTypeDef;
typedef struct {
    uint32_t USART_BaudRate, USART_WordLength, USART_StopBits, USART_Parity;
    uint32_t USART_HardwareFlowControl, USART_Mode;
} USART_InitTypeDef;
typedef struct {
    uint32_t DMA_PeripheralBaseAddr, DMA_MemoryBaseAddr, DMA_DIR, DMA_BufferSize;
    uint32_t DMA_PeripheralInc, DMA_MemoryInc, DMA_PeripheralDataSize;
    uint32_t DMA_MemoryDataSize, DMA_Mode, DMA_Priority, DMA_M2M;
} DMA_InitTypeDef;
typedef struct {
    uint32_t NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority;
    uint32_t NVIC_IRQChannelSubPriority, NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;
typedef struct {
    uint32_t CAN_TTCM, CAN_ABOM, CAN_AWUM, CAN_NART, CAN_RFLM, CAN_TXFP;
    uint32_t CAN_Mode, CAN_SJW, CAN_BS1, CAN_BS2, CAN_Prescaler;
} CAN_InitTypeDef;
typedef struct {
    uint32_t CAN_FilterNumber, CAN_FilterMode, CAN_FilterScale, CAN_FilterIdHigh;
    uint32_t CAN_FilterIdLow, CAN_FilterMaskIdHigh, CAN_FilterMaskIdLow;
    uint32_t CAN_FilterFIFOAssignment, CAN_FilterActivation;
} CAN_FilterInitTypeDef;
typedef struct {
    uint32_t StdId, ExtId;
    uint8_t IDE, RTR, DLC, Data[8], FMI;
} CanRxMsg;
typedef struct { uint32_t SYSCLK_Frequency, HCLK_Frequency; } RCC_ClocksTypeDef;

enum {
    RESET = 0, DISABLE = 0, ENABLE = 1, SET = 1, CAN_InitStatus_Success = 1,
    GPIO_Pin_2 = 2, GPIO_Pin_3, GPIO_Pin_9, GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12,
    GPIO_Mode_IN_FLOATING, GPIO_Mode_AF_PP, GPIO_Mode_IPU, GPIO_Speed_50MHz,
    RCC_APB1Periph_USART2, RCC_APB1Periph_CAN1, RCC_APB2Periph_USART1,
    RCC_APB2Periph_GPIOA, RCC_AHBPeriph_DMA1,
    USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_HardwareFlowControl_None,
    USART_Mode_Rx, USART_Mode_Tx, USART_FLAG_TXE, USART_IT_RXNE, USART_IT_ERR, USART_DMAReq_Rx,
    DMA_DIR_PeripheralSRC, DMA_PeripheralInc_Disable, DMA_MemoryInc_Enable,
    DMA_PeripheralDataSize_Byte, DMA_MemoryDataSize_Byte, DMA_Mode_Circular,
    DMA_Priority_High, DMA_M2M_Disable, DMA_IT_TC, DMA1_IT_TC6, DMA1_FLAG_TC6,
    DMA1_Channel6_IRQn, USART2_IRQn, USB_LP_CAN1_RX0_IRQn, NVIC_PriorityGroup_2,
    CAN_FilterMode_IdMask, CAN_FilterScale_32bit,
    CAN_Mode_Normal, CAN_SJW_1tq, CAN_BS1_5tq, CAN_BS2_3tq, CAN_FIFO0,
    CAN_IT_FMP0, CAN_FLAG_FOV0, CAN_IT_FOV0
};
#define USART_SR_RXNE (1U << 5)
#define USART_SR_TXE  (1U << 7)
#define USART_SR_ORE  (1U << 3)
#define USART_SR_NE   (1U << 2)
#define USART_SR_FE   (1U << 1)
#define CAN_Id_Extended 4U
#define CAN_RTR_Remote 2U
#define CAN_RF0R_RFOM0 UINT32_C(0x20)
#define CAN_RF0R_FOVR0 UINT32_C(0x10)
static USART_TypeDef fake_usart1, fake_usart2;
static DMA_Channel_TypeDef fake_dma;
static GPIO_TypeDef fake_gpio;
static CAN_TypeDef fake_can;
#define USART1 (&fake_usart1)
#define USART2 (&fake_usart2)
#define DMA1_Channel6 (&fake_dma)
#define GPIOA (&fake_gpio)
#define CAN1 (&fake_can)
static const uint32_t SystemCoreClock = 72000000;
static uint32_t fake_irq_mask;
static int fake_usart_error_irq_enabled, fake_usart_dma_enabled;
static int fake_usart_reads_while_dma_enabled;
static int fake_tc, fake_can_pending, fake_can_overflow;
static CanRxMsg fake_can_message;
static void (*fake_counter_hook)(void);
static void (*fake_barrier_hook)(void);
static void (*fake_irq_restore_hook)(void);
static void (*fake_can_receive_hook)(void);
static uint32_t __get_PRIMASK(void) { return fake_irq_mask; }
static void __disable_irq(void) { fake_irq_mask = 1; }
static void __set_PRIMASK(uint32_t mask) {
    fake_irq_mask = mask;
    if (!mask && fake_irq_restore_hook) fake_irq_restore_hook();
}
static void __DMB(void) { if (fake_barrier_hook) fake_barrier_hook(); }
static void __DSB(void) { }
static void __ISB(void) { }
static void SysTick_Config(uint32_t clocks) { (void)clocks; }
static void RCC_APB1PeriphClockCmd(uint32_t p, int on) { (void)p; (void)on; }
static void RCC_APB2PeriphClockCmd(uint32_t p, int on) { (void)p; (void)on; }
static void RCC_AHBPeriphClockCmd(uint32_t p, int on) { (void)p; (void)on; }
static void RCC_GetClocksFreq(RCC_ClocksTypeDef *c) { c->SYSCLK_Frequency = c->HCLK_Frequency = SystemCoreClock; }
static void GPIO_Init(GPIO_TypeDef *g, GPIO_InitTypeDef *c) { (void)g; (void)c; }
static void USART_Init(USART_TypeDef *u, USART_InitTypeDef *c) { (void)u; (void)c; }
static void USART_Cmd(USART_TypeDef *u, int on) { (void)u; (void)on; }
static void USART_DMACmd(USART_TypeDef *u, int p, int on) {
    (void)p; if (u == USART2) fake_usart_dma_enabled = on;
}
static void USART_ITConfig(USART_TypeDef *u, int p, int on) {
    if (u == USART2 && p == USART_IT_ERR) fake_usart_error_irq_enabled = on;
}
static int USART_GetITStatus(USART_TypeDef *u, int p) { (void)u; (void)p; return SET; }
static int USART_GetFlagStatus(USART_TypeDef *u, int p) { (void)u; (void)p; return SET; }
static uint16_t USART_ReceiveData(USART_TypeDef *u) {
    if (u == USART2 && fake_usart_dma_enabled) fake_usart_reads_while_dma_enabled++;
    u->SR &= ~(USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE);
    return (uint16_t)u->DR;
}
static void USART_SendData(USART_TypeDef *u, uint16_t d) { u->DR = d; }
static void NVIC_Init(NVIC_InitTypeDef *n) { (void)n; }
static void NVIC_PriorityGroupConfig(int group) { (void)group; }
static void DMA_DeInit(DMA_Channel_TypeDef *d) { d->CNDTR = 0; fake_tc = 0; }
static void DMA_Init(DMA_Channel_TypeDef *d, DMA_InitTypeDef *c) { d->CNDTR = c->DMA_BufferSize; }
static void DMA_ITConfig(DMA_Channel_TypeDef *d, int p, int on) { (void)d; (void)p; (void)on; }
static void DMA_Cmd(DMA_Channel_TypeDef *d, int on) { (void)d; (void)on; }
static int DMA_GetITStatus(int p) { (void)p; return fake_tc; }
static int DMA_GetFlagStatus(int p) { (void)p; return fake_tc; }
static void DMA_ClearITPendingBit(int p) { (void)p; fake_tc = 0; }
static void DMA_ClearFlag(int p) { (void)p; fake_tc = 0; }
static uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef *d) {
    uint16_t count = (uint16_t)d->CNDTR;
    if (fake_counter_hook) fake_counter_hook();
    return count;
}
static void CAN_DeInit(CAN_TypeDef *c) {
    memset(c, 0, sizeof(*c));
    fake_can_pending = fake_can_overflow = 0;
}
static int CAN_Init(CAN_TypeDef *c, CAN_InitTypeDef *i) { (void)c; (void)i; return 1; }
static void CAN_FilterInit(CAN_FilterInitTypeDef *f) { (void)f; }
static void CAN_ITConfig(CAN_TypeDef *c, int p, int on) { (void)c; (void)p; (void)on; }
/* Apply the last register write. FOVR0 is write-one-to-clear, including when
 * StdPeriph's CAN_Receive releases a mailbox with RF0R |= RFOM0. */
static void fake_can_apply_release(CAN_TypeDef *c) {
    if (c->RF0R & CAN_RF0R_RFOM0) {
        if (c->RF0R & CAN_RF0R_FOVR0) fake_can_overflow = 0;
        if (fake_can_pending) fake_can_pending--;
        c->RF0R = fake_can_overflow ? CAN_RF0R_FOVR0 : 0;
    }
}
static int CAN_MessagePending(CAN_TypeDef *c, int fifo) {
    unsigned i;
    int pending;
    (void)fifo;
    fake_can_apply_release(c);
    pending = fake_can_pending;
    c->sFIFOMailBox[0].RIR = (fake_can_message.ExtId << 3) |
                           fake_can_message.IDE | fake_can_message.RTR;
    c->sFIFOMailBox[0].RDTR = fake_can_message.DLC;
    c->sFIFOMailBox[0].RDLR = c->sFIFOMailBox[0].RDHR = 0;
    for (i = 0; i < 4; ++i) {
        c->sFIFOMailBox[0].RDLR |= (uint32_t)fake_can_message.Data[i] << (i * 8);
        c->sFIFOMailBox[0].RDHR |= (uint32_t)fake_can_message.Data[i + 4] << (i * 8);
    }
    if (pending && fake_can_receive_hook) fake_can_receive_hook();
    return pending;
}
static void CAN_Receive(CAN_TypeDef *c, int fifo, CanRxMsg *r) {
    (void)fifo;
    *r = fake_can_message;
    c->RF0R |= CAN_RF0R_RFOM0;
    fake_can_apply_release(c);
}
static int CAN_GetFlagStatus(CAN_TypeDef *c, int flag) {
    (void)flag;
    fake_can_apply_release(c);
    return fake_can_overflow;
}
static void CAN_ClearFlag(CAN_TypeDef *c, int flag) {
    (void)flag;
    fake_can_overflow = 0;
    c->RF0R &= ~CAN_RF0R_FOVR0;
}
#endif
