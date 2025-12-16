/**
 * @file main.c
 * @brief Serial Port Data Reception Example
 * 
 * This example demonstrates how to receive data from an Inertial Measurement Unit (IMU) 
 * using USART2 and print the decoded results via USART1. The example is primarily intended 
 * for educational purposes to observe data output and should not be used for other purposes.
 * 
 * @hardware_connections:
 * - USART2 (PA02/PA03): Receives data from IMU
 * - USART1 (PA09/PA10): Prints result to the console
 * 
 * @software_configuration:
 * - ENABLE_FIXED_DATA_EXAMPLE: Set to 1 to enable fixed data array decoding example, otherwise set to 0.
 * - ENABLE_USART_DMA: Set to 1 to enable DMA for USART reception, otherwise set to 0 to use interrupt-based reception.
 * 
 * @program_flow:
 * 1. System Initialization:
 *    - Initialize delay, NVIC priority, and USART1 for debug output.
 *    - Configure USART2 for IMU data reception.
 *    - Optionally configure DMA for USART2 reception.
 * 2. Print Welcome Information:
 *    - Print system clock frequencies and reception mode.
 * 3. Main Loop:
 *    - Continuously process received IMU data.
 *    - Decode and print the data if new data is available.
 * 
 * @website:
 * For more information, visit http://www.hipnuc.com
 * 
 * @date 2024-08-07
 * @version 1.0
 * @author 
 */
 
#include "delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "hipnuc_dec.h"


/* Enable/Disable fixed data array decoding example */
#define ENABLE_FIXED_DATA_EXAMPLE     0

/* Enable/Disable DMA for USART reception */
#define ENABLE_USART_DMA              0

#define USART1_BAUD 115200
#define USART2_BAUD 115200

#define UART_RX_BUF_SIZE        (1024)
#define LOG_STRING_SIZE         (1024)

/* IMU stream read/control struct */
static hipnuc_raw_t hipnuc_raw = {0};

/* 0: no new data arrived, 1: new data arrived */
static uint8_t new_data_flag = 0;         

/* The char buffer used to show result */
static char log_buf[LOG_STRING_SIZE];

static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static uint8_t dma_rx_buf[UART_RX_BUF_SIZE];
static uint16_t uart_rx_index = 0;

/* Function prototypes */
static void USART_Configuration(uint32_t usart1_baud, uint32_t usart2_baud);
static void DMA_Configuration(void);
static void app_init(void);
static void printf_welcome_information(void);
static void process_data(void);
static void handle_usart_rx_idle(void);

/**
 * @brief Main program
 * 
 * Initializes the system and enters an infinite loop to process IMU data.
 * 
 * @return int 
 */
int main(void)
{
    app_init();
    printf_welcome_information();

    /* Use macro to control whether to show the fixed data array decoding example */
    #if ENABLE_FIXED_DATA_EXAMPLE
    #include "example_data.h"
    process_example_data();
    #endif
    
    while (1)
    {
        process_data();
    }    
}

/**
 * @brief System initialization
 * 
 * Initializes delay, NVIC priority, USART1 for debug output, USART2 for IMU data reception, 
 * and optionally DMA for USART2 reception. Also initializes the IMU instance.
 */
static void app_init(void)
{
    delay_init();
    delay_ms(1);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    /* Initialize USART1 for debug output */
    /* Initialize USART2 for IMU data reception */
    USART_Configuration(USART1_BAUD, USART2_BAUD);
    
    #if ENABLE_USART_DMA
    /* Initialize DMA for USART2 reception */
    DMA_Configuration();
    #endif
    
    /* Initialize instance */
    memset(&hipnuc_raw, 0, sizeof(hipnuc_raw_t));
    new_data_flag = 0;
}

/**
 * @brief Print system clock frequencies and reception mode
 * 
 * Prints the system clock frequencies and the USART reception mode (DMA or UART Interrupt) to the console.
 */
static void printf_welcome_information(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    printf("HiPNUC IMU data decode example\r\n");
    #if ENABLE_USART_DMA
    printf("USART reception: DMA\r\n");
    #else
    printf("USART reception: UART Interrupt\r\n");
    #endif
    
    printf("System Clock Frequencies:\r\n");
    printf("SYSCLK: %d Hz\r\n", RCC_Clocks.SYSCLK_Frequency);
    printf("HCLK: %d Hz\r\n", RCC_Clocks.HCLK_Frequency);
}

/**
 * @brief Process IMU data
 * 
 * If new data is available, processes the received IMU data by decoding it and printing the results.
 */
static void process_data(void)
{
    if (new_data_flag)
    {
        new_data_flag = 0;

        for (uint16_t i = 0; i < uart_rx_index; i++)
        {
            if (hipnuc_input(&hipnuc_raw, uart_rx_buf[i]))
            {
                /* Convert result to strings */
                hipnuc_dump_packet(&hipnuc_raw, log_buf, sizeof(log_buf));
                /* Display result */
                printf("parse ok, frame len:%d\r\n", hipnuc_raw.len);
                printf("%s\r\n", log_buf);

                /*
                You can use data as for example:
                
                printf("acc:%.3f, %.3f, %.3f\r\n", hipnuc_raw.hi91.acc[0], hipnuc_raw.hi91.acc[1], hipnuc_raw.hi91.acc[2]);
                */
            }
        }

        uart_rx_index = 0; // Reset buffer index after processing
    }
}

/**
 * @brief Configure USART1 USART2 for IMU data reception
 * 
 * Configures GPIO pins, USART1 USART2 settings, and NVIC for USART2 interrupts.
 */
static void USART_Configuration(uint32_t usart1_baud, uint32_t usart2_baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable USART1 USART2 and GPIOA clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* Configure USART1 USART2 Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART1 USART2 Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART configuration */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    /* Enable USART1 */
    USART_InitStructure.USART_BaudRate = usart1_baud;
    USART_Init(USART1, &USART_InitStructure); 
    USART_Cmd(USART1, ENABLE);

    /* Enable USART2 */
    USART_InitStructure.USART_BaudRate = usart2_baud;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);

    /* Enable USART2 IDLE line detection interrupt */
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

    /* Enable USART2 Receive interrupt */
    #if !ENABLE_USART_DMA
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    #endif

    /* NVIC configuration for USART2 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Configure DMA for USART2 reception
 * 
 * Configures DMA1 Channel6 for USART2 reception to use circular mode and high priority.
 */
static void DMA_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* DMA1 Channel6 (triggered by USART2 Rx event) Config */
    DMA_DeInit(DMA1_Channel6);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dma_rx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = UART_RX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);

    /* Enable DMA1 Channel6 */
    DMA_Cmd(DMA1_Channel6, ENABLE);

    /* Enable USART2 DMA Rx request */
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
}

/**
 * @brief USART2 interrupt handler
 * 
 * Handles USART2 interrupts for IDLE line detection and RXNE (Receive Not Empty).
 */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        /* Clear IDLE line detected bit */
        USART_ReceiveData(USART2);

        handle_usart_rx_idle();
    }

    #if !ENABLE_USART_DMA
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = USART_ReceiveData(USART2);
        uart_rx_buf[uart_rx_index++] = ch;
        if (uart_rx_index >= UART_RX_BUF_SIZE)
        {
            uart_rx_index = 0; // Prevent buffer overflow
        }
    }
    #endif
}

/**
 * @brief Handle USART RX data when IDLE line is detected
 * 
 * Processes received data and resets DMA if DMA is used.
 */
static void handle_usart_rx_idle(void)
{
    #if ENABLE_USART_DMA
    /* Disable DMA1 Channel6 */
    DMA_Cmd(DMA1_Channel6, DISABLE);

    /* Get number of bytes received */
    uint16_t rx_size = UART_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);

    /* Process received data */
    for (uint16_t i = 0; i < rx_size; i++)
    {
        uart_rx_buf[uart_rx_index++] = dma_rx_buf[i];
        
        if (uart_rx_index >= UART_RX_BUF_SIZE)
        {
            uart_rx_index = 0; // Prevent buffer overflow
        }
    }

    /* Reset DMA1 Channel6 */
    DMA_SetCurrDataCounter(DMA1_Channel6, UART_RX_BUF_SIZE);
    
    /* Re-enable DMA1 Channel6 */
    DMA_Cmd(DMA1_Channel6, ENABLE);
    #endif
    
    /* Set flag to indicate new data available */
    new_data_flag = 1;
}
