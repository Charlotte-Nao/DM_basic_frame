#include "global_data.h"

#include "../bsp/uart/uart.h"
#include "../dsp/calculation/revrse_calculation.h"

volatile global_data_t global_data = {0};
volatile aim_pose_t aim_pose = {0};
action_sequence_t action_sequence = {0};
volatile uint8_t question_number = 0U;

#define QUESTION_NUMBER_FORWARD_BUFFER_SIZE 256U

static volatile uint8_t
    question_number_forward_buffer[QUESTION_NUMBER_FORWARD_BUFFER_SIZE];
static volatile uint16_t question_number_forward_head;
static volatile uint16_t question_number_forward_tail;

static void question_number_queue_ascii(uint8_t ascii)
{
    uint16_t head = question_number_forward_head;
    uint16_t tail = question_number_forward_tail;

    if ((uint16_t)(head - tail) >= QUESTION_NUMBER_FORWARD_BUFFER_SIZE) {
        return;
    }

    question_number_forward_buffer[
        head & (QUESTION_NUMBER_FORWARD_BUFFER_SIZE - 1U)
    ] = ascii;
    question_number_forward_head = (uint16_t)(head + 1U);
}

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
            question_number_queue_ascii(data[index]);
        } else if (data[index] >= 1U && data[index] <= 4U) {
            question_number = data[index];
            question_number_queue_ascii(
                (uint8_t)((uint8_t)'0' + data[index])
            );
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

uint16_t question_number_uart7_copy_ascii(uint8_t *data, uint16_t max_length)
{
    uint16_t head;
    uint16_t tail;
    uint16_t available;
    uint16_t copy_length;
    uint16_t index;

    if (data == NULL || max_length == 0U) {
        return 0U;
    }

    tail = question_number_forward_tail;
    head = question_number_forward_head;
    available = (uint16_t)(head - tail);
    copy_length = available < max_length ? available : max_length;

    for (index = 0U; index < copy_length; ++index) {
        data[index] = question_number_forward_buffer[
            (tail + index) & (QUESTION_NUMBER_FORWARD_BUFFER_SIZE - 1U)
        ];
    }

    return copy_length;
}

void question_number_uart7_consume_ascii(uint16_t length)
{
    uint16_t tail = question_number_forward_tail;
    uint16_t available = (uint16_t)(question_number_forward_head - tail);

    if (length > available) {
        length = available;
    }

    question_number_forward_tail = (uint16_t)(tail + length);
}

struct four_axis_robotic_arm arm = {
    .l_1 = 114.5f,
    .l_2 = 83.7f,
    .l_3 = 120.1f,
    .l_4_p = 50.0f,
    .l_4_z = 89.0f,
};
