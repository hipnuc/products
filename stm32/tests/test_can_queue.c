#include <stdio.h>
#include <stdlib.h>
#include "stubs/stm32f10x.h"
#define __STM32F10x_H
#include "../can/USER/hipnuc_board.c"

static hipnuc_can_frame_t parsed;
void hipnuc_sample_clear(hipnuc_sample_t *s) { memset(s, 0, sizeof(*s)); }
int hipnuc_j1939_parse(const hipnuc_can_frame_t *f, hipnuc_sample_t *s, uint8_t *p)
{
    (void)p;
    parsed = *f;
    memset(s, 0, sizeof(*s));
    if (f->len != 8 || f->is_remote) return -1;
    s->node_id = (uint8_t)f->id;
    s->valid = f->data[0] == 1 ? HIPNUC_VALID_ACC : HIPNUC_VALID_ROLL_PITCH;
    return 1;
}
#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); exit(1); \
} } while (0)

static void enqueue(uint8_t value)
{
    memset(&fake_can_message, 0, sizeof(fake_can_message));
    fake_can_message.ExtId = UINT32_C(0x18FF0388);
    fake_can_message.IDE = CAN_Id_Extended;
    fake_can_message.DLC = 8;
    fake_can_message.Data[0] = value;
    fake_can_pending = 1;
    USB_LP_CAN1_RX0_IRQHandler();
}

static void interrupt_consumer(void)
{
    fake_barrier_hook = NULL;
    enqueue(99);
}

static void overflow_during_receive(void)
{
    fake_can_receive_hook = NULL;
    fake_can_overflow = 1;
    fake_can.RF0R |= CAN_RF0R_FOVR0;
}

int main(void)
{
    unsigned i;
    CanRxMsg received;
    hipnuc_sample_t sample;
    CHECK(hipnuc_board_init(500, 0x88));
    CHECK(!receive_frame(&received));
    for (i = 0; i < CAN_RX_FIFO_SIZE - 1; ++i) enqueue((uint8_t)i);
    CHECK(can_fifo_head == CAN_RX_FIFO_SIZE - 1 && can_fifo_tail == 0);
    enqueue(99);
    CHECK(can_fifo_tail == 0 && can_fifo[0].Data[0] == 0 && can_drops == 1);

    /* ISR preempts a consumer copy while full: old frame and tail survive. */
    fake_barrier_hook = interrupt_consumer;
    CHECK(receive_frame(&received) && received.Data[0] == 0);
    CHECK(can_drops == 2 && can_fifo_tail == 1);
    enqueue(73);                       /* the newly freed slot may now be reused */
    for (i = 1; i < CAN_RX_FIFO_SIZE - 1; ++i) {
        CHECK(receive_frame(&received));
        CHECK(received.Data[0] == i);
    }
    CHECK(receive_frame(&received) && received.Data[0] == 73);
    CHECK(!receive_frame(&received));

    CHECK(hipnuc_board_init(500, 0x88));
    enqueue(1);
    enqueue(2);
    CHECK(hipnuc_board_poll(&sample) && sample.valid == HIPNUC_VALID_ACC);
    CHECK(sample.node_id == 0x88);
    CHECK(hipnuc_board_poll(&sample) && sample.valid == HIPNUC_VALID_ROLL_PITCH);
    CHECK(!hipnuc_board_poll(&sample) && stats.frames == 2); /* no historical accumulation */

    fake_can_message.ExtId = UINT32_C(0x18FF0308); /* lower seven bits are not a full SA */
    fake_can_pending = 1;
    USB_LP_CAN1_RX0_IRQHandler();
    CHECK(!hipnuc_board_poll(&sample) && stats.frames == 2);

    fake_can_message.ExtId = UINT32_C(0x18FF0388);
    fake_can_message.DLC = 9;
    fake_can_pending = 1;
    USB_LP_CAN1_RX0_IRQHandler();
    CHECK(!hipnuc_board_poll(&sample) && stats.invalid_frames == 1 && parsed.len == 9);
    fake_can_message.DLC = 8;
    fake_can_message.RTR = CAN_RTR_Remote;
    fake_can_pending = 1;
    USB_LP_CAN1_RX0_IRQHandler();
    CHECK(!hipnuc_board_poll(&sample) && stats.invalid_frames == 2 && parsed.is_remote);

    fake_can_overflow = 1;
    USB_LP_CAN1_RX0_IRQHandler();
    CHECK(hipnuc_board_stats()->hardware_overruns == 1 && !fake_can_overflow);

    /* A new overflow after the ISR's flag check must survive FIFO release. */
    CHECK(hipnuc_board_init(500, 0x88));
    fake_can_receive_hook = overflow_during_receive;
    enqueue(1);
    CHECK(hipnuc_board_stats()->hardware_overruns == 1 && !fake_can_overflow);
    CHECK(hipnuc_board_poll(&sample) && sample.valid == HIPNUC_VALID_ACC);

    /* Register mailbox copy preserves both data words and the full CAN ID. */
    for (i = 0; i < 8; ++i) fake_can_message.Data[i] = (uint8_t)(0x80 + i);
    fake_can_pending = 1;
    USB_LP_CAN1_RX0_IRQHandler();
    CHECK(hipnuc_board_poll(&sample));
    CHECK(parsed.id == UINT32_C(0x18FF0388) && parsed.len == 8);
    for (i = 0; i < 8; ++i) CHECK(parsed.data[i] == (uint8_t)(0x80 + i));
    CHECK(!hipnuc_board_init(333, 8));
    puts("CAN: full/drop, interrupt interleaving, wrap, source isolation and error counts passed");
    return 0;
}
