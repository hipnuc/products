/* STM32F407IGT6. CAN transceiver: PI9 RX / PB9 TX (CAN1 AF9).
 * Console: USART1 PB6, 115200 8N1. */
#include <stdio.h>
#include "hipnuc_board.h"
#include "stm32f4xx_hal.h"

#define CAN_BAUD_KBPS      500U
#define DEVICE_NODE_ID       8U
#define PRINT_PERIOD_MS    200U
#define REPORT_PERIOD_MS  2000U

static void error_handler(void)
{
    __disable_irq();
    while (1) {}
}

static void system_clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 25U;
    osc.PLL.PLLN = 336U;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7U;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) error_handler();

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) error_handler();
    SystemCoreClockUpdate();
}

static void print_sample(const hipnuc_sample_t *sample)
{
    printf("node %u ", (unsigned)sample->node_id);
    if (sample->valid & HIPNUC_VALID_ROLL_PITCH) {
        printf("roll %.1f pitch %.1f deg", sample->roll * 57.29578f, sample->pitch * 57.29578f);
    } else if (sample->valid & HIPNUC_VALID_HEADING) {
        printf("heading %.1f deg", sample->heading * 57.29578f);
    } else if (sample->valid & HIPNUC_VALID_ACC) {
        printf("acc %.2f %.2f %.2f m/s2", sample->acc[0], sample->acc[1], sample->acc[2]);
    } else if (sample->valid & HIPNUC_VALID_GYR) {
        printf("gyr %.2f %.2f %.2f rad/s", sample->gyr[0], sample->gyr[1], sample->gyr[2]);
    } else {
        printf("new J1939 sample (see application block)");
    }
    printf("\r\n");
}

int main(void)
{
    hipnuc_sample_t sample;
    hipnuc_board_stats_t previous = {0};
    uint32_t last_print = 0, last_report = 0;
    int was_receiving = -1;

    HAL_Init();
    system_clock_init();
    if (!hipnuc_board_init(CAN_BAUD_KBPS, DEVICE_NODE_ID)) {
        printf("CAN initialization failed: check bitrate and APB1 clock\r\n");
        while (1) { }
    }
    printf("HiPNUC CAN: %u kbit/s, source %u\r\n", CAN_BAUD_KBPS, DEVICE_NODE_ID);
    while (1) {
        uint32_t now;
        if (hipnuc_board_poll(&sample)) {
            /* Application: process this new PGN here. Check sample.valid before
             * using a field; fields from other PGNs are not included. */
            now = hipnuc_board_millis();
            if (PRINT_PERIOD_MS && now - last_print >= PRINT_PERIOD_MS) {
                last_print = now;
                print_sample(&sample);
            }
        }
        now = hipnuc_board_millis();
        if (now - last_report >= REPORT_PERIOD_MS) {
            hipnuc_board_stats_t current = *hipnuc_board_stats();
            uint32_t frames = current.frames - previous.frames;
            if (current.received == previous.received) {
                printf("no CAN frames: check transceiver, termination and bitrate\r\n");
            } else if (!frames) {
                printf("CAN traffic, no J1939 samples from node %u\r\n", DEVICE_NODE_ID);
            } else {
                printf("%s: %lu decoded samples/s\r\n", was_receiving == 0 ? "recovered" : "receiving",
                       (unsigned long)(frames * 1000U / (now - last_report)));
            }
            if (current.queue_drops != previous.queue_drops || current.hardware_overruns != previous.hardware_overruns)
                printf("RX loss: queue %lu, hardware %lu in this window\r\n",
                       (unsigned long)(current.queue_drops - previous.queue_drops),
                       (unsigned long)(current.hardware_overruns - previous.hardware_overruns));
            if (current.invalid_frames != previous.invalid_frames)
                printf("invalid J1939 frames: %lu in this window\r\n",
                       (unsigned long)(current.invalid_frames - previous.invalid_frames));
            was_receiving = frames != 0;
            previous = current;
            last_report = now;
        }
    }
}
