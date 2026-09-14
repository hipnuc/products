/* HiPNUC J1939 CAN receiver for STM32F407IGT6, implemented with STM32 HAL. */
#include "hipnuc_board.h"

#include <stdio.h>
#include <string.h>

#include "hipnuc_j1939.h"
#include "stm32f4xx_hal.h"

#define SAMPLE_FIFO_SIZE 16U

static CAN_HandleTypeDef hcan1;
static UART_HandleTypeDef huart1;
static hipnuc_sample_t sample_fifo[SAMPLE_FIFO_SIZE];
static volatile uint16_t sample_fifo_head;
static volatile uint16_t sample_fifo_tail;
static volatile uint32_t can_received;
static volatile uint32_t can_drops;
static volatile uint32_t can_overruns;
static volatile uint32_t can_frames;
static volatile uint32_t can_invalid;
static uint8_t device_node;
static hipnuc_board_stats_t stats;

uint32_t hipnuc_board_millis(void)
{
    return HAL_GetTick();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};
    if (huart->Instance != USART1) return;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static int console_init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200U;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    return HAL_UART_Init(&huart1) == HAL_OK;
}

int fputc(int ch, FILE *stream)
{
    uint8_t byte = (uint8_t)ch;
    (void)stream;
    (void)HAL_UART_Transmit(&huart1, &byte, 1U, HAL_MAX_DELAY);
    return ch;
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    GPIO_InitTypeDef gpio = {0};
    if (hcan->Instance != CAN1) return;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &gpio);            /* CAN1_TX */
    HAL_GPIO_Init(GPIOI, &gpio);            /* CAN1_RX */

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}

static int can_timing_for_kbps(uint16_t kbps, CAN_InitTypeDef *init)
{
    /* CAN clock is PCLK1 = 42 MHz. Every option below is exact. */
    switch (kbps) {
    case 125U:
        init->Prescaler = 21U;
        init->TimeSeg1 = CAN_BS1_13TQ;
        init->TimeSeg2 = CAN_BS2_2TQ;
        break;
    case 250U:
        init->Prescaler = 14U;
        init->TimeSeg1 = CAN_BS1_9TQ;
        init->TimeSeg2 = CAN_BS2_2TQ;
        break;
    case 500U:
        init->Prescaler = 7U;
        init->TimeSeg1 = CAN_BS1_9TQ;
        init->TimeSeg2 = CAN_BS2_2TQ;
        break;
    case 1000U:
        init->Prescaler = 3U;
        init->TimeSeg1 = CAN_BS1_11TQ;
        init->TimeSeg2 = CAN_BS2_2TQ;
        break;
    default:
        return 0;
    }
    return 1;
}

static int can_init(uint16_t kbps)
{
    CAN_FilterTypeDef filter = {0};
    hcan1.Instance = CAN1;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = ENABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (!can_timing_for_kbps(kbps, &hcan1.Init)) return 0;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) return 0;

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) return 0;
    if (HAL_CAN_Start(&hcan1) != HAL_OK) return 0;
    return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING |
                                        CAN_IT_RX_FIFO0_OVERRUN) == HAL_OK;
}

static void process_frame(const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    hipnuc_can_frame_t frame;
    hipnuc_sample_t sample;
    int result;
    uint16_t next;

    ++can_received;
    if (header->IDE != CAN_ID_EXT || (uint8_t)header->ExtId != device_node) return;
    memset(&frame, 0, sizeof(frame));
    frame.id = header->ExtId;
    frame.is_extended = 1U;
    frame.is_remote = header->RTR == CAN_RTR_REMOTE;
    frame.len = (uint8_t)header->DLC;
    if (frame.len <= 8U) memcpy(frame.data, data, frame.len);
    result = hipnuc_j1939_parse(&frame, &sample, NULL);
    if (result > 0) {
        ++can_frames;
        next = (uint16_t)((sample_fifo_head + 1U) % SAMPLE_FIFO_SIZE);
        if (next == sample_fifo_tail) {
            ++can_drops;
        } else {
            sample_fifo[sample_fifo_head] = sample;
            __DMB();
            sample_fifo_head = next;
        }
    } else if (result < 0) {
        ++can_invalid;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (hcan != &hcan1) return;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
        process_frame(&header, data);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1 && (HAL_CAN_GetError(hcan) & HAL_CAN_ERROR_RX_FOV0) != 0U) {
        ++can_overruns;
    }
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

static int receive_sample(hipnuc_sample_t *sample)
{
    if (sample_fifo_head == sample_fifo_tail) return 0;
    __DMB();
    *sample = sample_fifo[sample_fifo_tail];
    __DMB();
    sample_fifo_tail = (uint16_t)((sample_fifo_tail + 1U) % SAMPLE_FIFO_SIZE);
    return 1;
}

int hipnuc_board_init(uint16_t bitrate_kbps, uint8_t source_address)
{
    memset(&stats, 0, sizeof(stats));
    sample_fifo_head = sample_fifo_tail = 0U;
    can_received = can_drops = can_overruns = can_frames = can_invalid = 0U;
    device_node = source_address;
    if (!console_init()) return 0;
    return can_init(bitrate_kbps);
}

int hipnuc_board_poll(hipnuc_sample_t *sample)
{
    return receive_sample(sample);
}

const hipnuc_board_stats_t *hipnuc_board_stats(void)
{
    stats.received = can_received;
    stats.frames = can_frames;
    stats.invalid_frames = can_invalid;
    stats.queue_drops = can_drops;
    stats.hardware_overruns = can_overruns;
    return &stats;
}
