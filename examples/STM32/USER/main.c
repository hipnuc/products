#include "delay.h"
#include "usart.h"
#include "hipnuc.h"


/**********************************************************
 * HiPNUC Beijing Co.
 * Serial port data reception example
 * This example is only for educational purposes, to observe data output,
 * and should not be used for other purposes.
 * USART2 (PA02/PA03): receives data from IMU
 * USART1 (PA09/PA10): print result
 * Website: http://www.hipnuc.com
 **********************************************************/

/* IMU stream read/control struct */
static hipnuc_raw_t hipnuc_raw = {0};

/* 0: no new frame arrived, 1: new frame arrived */
static uint8_t decode_succ = 0;         

/* the char buffer use to show result */
static char log_buf[256];

int main(void)
{       
    delay_init();                                        // Initialize delay function
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);      // Set NVIC interrupt group 2
    uart_init(115200);                                   // Initialize UART to 115200 baud rate

    /* initialize instance */
    memset(&hipnuc_raw, 0, sizeof(hipnuc_raw_t));
    decode_succ = 0;
    
    printf("HiPNUC IMU data decode example\r\n");

    while(1)
    {
        if(decode_succ)
        {
            decode_succ = 0;
            
            /* convert result to strings */
            hipnuc_dump_packet(&hipnuc_raw, log_buf, sizeof(log_buf));
            
            /* display result */
            printf("Decode one frame success, frame len:%d\r\n", hipnuc_raw.len);
            printf("%s\r\n", log_buf);
            
            /*
            you can use data as for example:
            
            printf("acc:%.3f, %.3f, %.3f\r\n", hipnuc_raw.hi91.acc[0], hipnuc_raw.hi91.acc[1], hipnuc_raw.hi91.acc[2]);
            */
            
            delay_ms(200);
        }
    }    
}

void USART2_IRQHandler(void)
{
    uint8_t ch;
    
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(USART2); 
        
        /* put each received byte into decoder

            if you are using DMA to receive data, you should do the following:
            ```
            uint32_t rx_size = DMA_GetRxSize();  // get received bytes from DMA 
            for(int i = 0; i < rx_size; i++)
            {
                decode_succ = hipnuc_input(&hipnuc_raw, dma_rx_buf[i]); // dma_rx_buf is the buffer that stores the usart rx data, put each byte into the decoder(hipnuc_input)
                if(decode_succ)
                {
                    // process the frames that successfully decoded.
                    ....
                }
            }
            ```
        */

        decode_succ = hipnuc_input(&hipnuc_raw, ch);
    }
}
