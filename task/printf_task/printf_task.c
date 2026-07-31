//
// Created by charlotte on 7/14/26.
//

#include "printf_task.h"
#include "cmsis_os2.h"
#include "../../bsp/uart/uart.h"
#include "../../application/global_data.h"

void printf_task(void)
{
    struct uart_device *uart7 = uart_get_device("uart7_dma");
    uint16_t next_print_index = 0U;
    uint32_t last_generation = action_sequence.generation;
    action_sequence_state_t last_state = action_sequence.state;

    if (uart7 == NULL) {
        return;
    }

    for (;;) {
        struct protocol_data action;
        uint16_t action_index = 0U;
        uint8_t have_action = 0U;
        uint32_t generation;
        action_sequence_state_t state;
        int32_t lock_state;

        lock_state = osKernelLock();
        generation = action_sequence.generation;
        state = action_sequence.state;

        if (state == ACTION_SEQUENCE_STATE_LOADING &&
            (last_state != ACTION_SEQUENCE_STATE_LOADING ||
             generation != last_generation)) {
            next_print_index = 0U;
        } else if ((state == ACTION_SEQUENCE_STATE_READY ||
                    state == ACTION_SEQUENCE_STATE_RUNNING) &&
                   generation != last_generation &&
                   !(last_state == ACTION_SEQUENCE_STATE_LOADING &&
                     (uint32_t)(generation - last_generation) == 1U)) {
            next_print_index = 0U;
        } else if (action_sequence.count < next_print_index) {
            next_print_index = 0U;
        }

        if (next_print_index < action_sequence.count) {
            action_index = next_print_index;
            action = action_sequence.frames[action_index];
            have_action = 1U;
        }

        last_generation = generation;
        last_state = state;
        (void)osKernelRestoreLock(lock_state);

        if (have_action != 0U &&
            uart7->uart_printf(
                uart7,
                "ACTION[%u] x=%.3f y=%.3f z=%.3f roll=%.1f action=%u\r\n",
                (unsigned int)action_index,
                (double)action.x,
                (double)action.y,
                (double)action.z,
                (double)action.roll / 10.0,
                (unsigned int)action.action) == 0) {
            next_print_index++;
        }

        osDelay(1U);
    }
}
