#include "global_data.h"

#include "../bsp/uart/uart.h"
#include "../dsp/calculation/revrse_calculation.h"

volatile global_data_t global_data = {0};
volatile aim_pose_t aim_pose = {0};
action_sequence_t action_sequence = {0};
volatile uint8_t question_number = 0U;

static void question_number_uart7_receive_callback(
    struct uart_device *device,
    const uint8_t *data,
    uint16_t length
)
{
    uint16_t index;

    (void)device;

    if (data == NULL || length == 0U) {
        return;
    }

    for (index = 0U; index < length; ++index) {
        if (data[index] >= (uint8_t)'1' && data[index] <= (uint8_t)'4') {
            question_number = (uint8_t)(data[index] - (uint8_t)'0');
        } else if (data[index] >= 1U && data[index] <= 4U) {
            question_number = data[index];
        }
    }
}

void question_number_uart7_init(void)
{
    struct uart_device *uart7;

    uart7 = uart_get_device("uart7_dma");
    if (uart7 != NULL) {
        uart7->uart_recv_callback = question_number_uart7_receive_callback;
    }
}

struct four_axis_robotic_arm arm = {
    .l_1 = 114.5f,
    .l_2 = 83.7f,
    .l_3 = 120.1f,
    .l_4_p = 50.0f,
    .l_4_z = 89.0f,
};
