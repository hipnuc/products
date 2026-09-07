/*
 * HiPNUC serial example for STM32F103 (Keil MDK, StdPeriph).
 *
 * Wiring:  IMU TX  -> PA3 (USART2 RX)      IMU RX <- PA2 (USART2 TX)
 *          IMU GND -> GND                   console: PA9 (USART1 TX), 115200 8N1
 *
 * Change the two settings below, build, download, open a terminal on
 * USART1. Put your own code where "your application" is marked.
 */

#include <stdio.h>

#include "hipnuc_board.h"
#include "stm32f10x.h"
#include "sys.h"

/* ---- settings ---------------------------------------------------------- */
#define IMU_BAUDRATE      115200U   /* device serial speed (factory default 115200) */
#define PRINT_PERIOD_MS   200U      /* how often to print; 0 = never */

/* ---- console on USART1 (PA9/PA10), 115200 ------------------------------- */
static void console_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
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

int main(void)
{
    hipnuc_sample_t sample;
    uint32_t last_print = 0;
    uint32_t last_frames = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    console_init();
    hipnuc_board_init(IMU_BAUDRATE);
    printf("HiPNUC serial example, waiting for data at %u baud\r\n", (unsigned)IMU_BAUDRATE);

    while (1) {
        if (hipnuc_board_poll(&sample)) {
            /* ---- your application: a new sample is available ------------ */
            /* sample.roll / pitch / yaw are in rad, sample.acc in m/s^2,
             * sample.gyr in rad/s. Check the matching HIPNUC_VALID_* bit
             * before using a field (see hipnuc_sample.h). */
            /* ------------------------------------------------------------- */

            if (PRINT_PERIOD_MS && hipnuc_board_millis() - last_print >= PRINT_PERIOD_MS) {
                const hipnuc_board_stats_t *st = hipnuc_board_stats();
                uint32_t now = hipnuc_board_millis();
                uint32_t rate = (st->frames - last_frames) * 1000U / (now - last_print);
                last_print = now;
                last_frames = st->frames;
                if (sample.valid & HIPNUC_VALID_EULER) {
                    printf("roll %7.2f  pitch %7.2f  yaw %7.2f deg",
                           sample.roll * 57.29578f, sample.pitch * 57.29578f, sample.yaw * 57.29578f);
                }
                if (sample.valid & HIPNUC_VALID_ACC) {
                    printf("  acc %6.2f %6.2f %6.2f m/s2", sample.acc[0], sample.acc[1], sample.acc[2]);
                }
                printf("  %lu Hz%s%s%s\r\n", (unsigned long)rate,
                       sample.attitude_converged ? "" : "  [attitude not converged, keep still]",
                       sample.magnetic_disturbance ? "  [magnetic disturbance]" : "",
                       st->overruns ? "  [RX overrun]" : "");
            }
        } else if (hipnuc_board_millis() - last_print >= 2000U && last_print != 0 &&
                   hipnuc_board_stats()->bytes == 0) {
            last_print = hipnuc_board_millis();
            printf("no data: check wiring (IMU TX -> PA3), baudrate and that output is enabled\r\n");
        }
    }
}
