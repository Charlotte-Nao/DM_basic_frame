/**
 * @file test_task.c
 * @brief Aim pose serial forwarding test.
 */

#include "test_task.h"

#include <stdint.h>
#include <string.h>

#include "cmsis_compiler.h"
#include "cmsis_os2.h"
#include "../../device/vacuum/vacuum.h"
#include "../../application/global_data.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/uart/uart.h"
#include "../../protocol/protocol.h"
#include "../../protocol/machine_protocal.h"

#define UART10_RX_RING_SIZE 512U
#define UART10_STATUS_QUERY_INTERVAL_MS 100U
#define UART10_ACK_TIMEOUT_MS 2000U

enum uart10_motion_state {
    UART10_MOTION_WAIT_READY = 0,
    UART10_MOTION_IDLE,
    UART10_MOTION_WAIT_ACK,
    UART10_MOTION_WAIT_IDLE,
    UART10_MOTION_FAULT,
};

struct uart10_rx_ring {
    uint8_t data[UART10_RX_RING_SIZE];
    volatile uint16_t write_index;
    volatile uint16_t read_index;
    volatile uint8_t overflow;
};

static struct uart10_rx_ring uart10_rx;
static struct machine_protocol_context uart10_protocol;
static char uart10_command[MACHINE_PROTOCOL_TX_BUFFER_SIZE];

static int uart_is_port(const struct uart_device *uart, const char *port_name)
{
    size_t port_name_length;

    if (uart == NULL || uart->name == NULL || port_name == NULL) {
        return 0;
    }
    port_name_length = strlen(port_name);
    return strncmp(uart->name, port_name, port_name_length) == 0 &&
           uart->name[port_name_length] == '_';
}

static int16_t roll_to_protocol_phi(float roll)
{
    float raw_phi = roll * 10.0f;

    if (raw_phi >= 0.0f) {
        return (int16_t)(raw_phi + 0.5f);
    }
    return (int16_t)(raw_phi - 0.5f);
}

static int set_seria_target(struct uart_device *uart, const float aim_pose[5])
{
    uint8_t protocol_frame[PROTOCOL_FRAME_SIZE];
    struct protocol_data protocol_target;
    struct uart_device *uart1;
    uart1 = uart_get_device("uart1_dma");
    if (uart == NULL || aim_pose == NULL || uart1 == NULL) {
        LED_YELLOW_SET();
        return -1;
    }
    if (uart_is_port(uart, "uart1")) {
        protocol_target.x = aim_pose[0];
        protocol_target.y = aim_pose[1];
        protocol_target.z = aim_pose[2];
        protocol_target.roll = roll_to_protocol_phi(aim_pose[3]);
        protocol_target.action = aim_pose[4];
        if (protocol_pack(&protocol_target, protocol_frame) != 0) {
            LED_RED_SET();
            return -1;
        }
        // LED_SKY_SET();
        return uart1->uart_send_bytes(uart1, protocol_frame, PROTOCOL_FRAME_SIZE);
    }
    return -1;
}

static void uart10_receive_callback(struct uart_device *device, const uint8_t *data,
                                    uint16_t length)
{
    uint16_t write_index;
    (void)device;
    if (data == NULL) {
        return;
    }
    write_index = uart10_rx.write_index;
    for (uint16_t i = 0U; i < length; ++i) {
        uint16_t next_index = (uint16_t)((write_index + 1U) % UART10_RX_RING_SIZE);
        if (next_index == uart10_rx.read_index) {
            uart10_rx.overflow = 1U;
            break;
        }
        uart10_rx.data[write_index] = data[i];
        write_index = next_index;
    }
    __DMB();
    uart10_rx.write_index = write_index;
}

static int uart10_receive_byte(uint8_t *byte)
{
    uint16_t read_index;
    if (byte == NULL) {
        return 0;
    }
    read_index = uart10_rx.read_index;
    if (read_index == uart10_rx.write_index) {
        return 0;
    }
    *byte = uart10_rx.data[read_index];
    __DMB();
    uart10_rx.read_index = (uint16_t)((read_index + 1U) % UART10_RX_RING_SIZE);
    return 1;
}

static void uart10_handle_event(enum uart10_motion_state *motion_state,
                                const struct machine_protocol_context *protocol, uint32_t events)
{
    if ((events & (MACHINE_PROTOCOL_EVENT_ERROR | MACHINE_PROTOCOL_EVENT_ALARM)) != 0U) {
        *motion_state = UART10_MOTION_FAULT;
        LED_RED_SET();
        return;
    }
    if ((events & MACHINE_PROTOCOL_EVENT_READY) != 0U) {
        if (*motion_state == UART10_MOTION_WAIT_READY) {
            return;
        }
        /* A welcome line during a command means FluidNC reset mid-session. */
        *motion_state = UART10_MOTION_FAULT;
        LED_RED_SET();
        return;
    }
    if ((events & MACHINE_PROTOCOL_EVENT_OK) != 0U && *motion_state == UART10_MOTION_WAIT_ACK) {
        *motion_state = UART10_MOTION_WAIT_IDLE;
    }
    if ((events & MACHINE_PROTOCOL_EVENT_STATUS) == 0U) {
        return;
    }
    if (protocol->controller_state == MACHINE_PROTOCOL_STATE_ALARM) {
        *motion_state = UART10_MOTION_FAULT;
        LED_RED_SET();
    } else if (protocol->controller_state == MACHINE_PROTOCOL_STATE_IDLE) {
        if (*motion_state == UART10_MOTION_WAIT_READY || *motion_state == UART10_MOTION_WAIT_IDLE) {
            *motion_state = UART10_MOTION_IDLE;
        }
    } else if (*motion_state == UART10_MOTION_IDLE) {
        *motion_state = UART10_MOTION_WAIT_READY;
    }
}

static int uart10_send_status_query(struct uart_device *uart10)
{
    static const uint8_t query = (uint8_t)'?';
    return uart10->uart_send_bytes(uart10, &query, 1U);

}


void test_task(void)
{
    struct uart_device *uart1;
    struct uart_device *uart10;
    enum uart10_motion_state uart10_state = UART10_MOTION_WAIT_READY;
    uint32_t last_query_tick;
    uint32_t command_sent_tick = 0U;
    float pending_xyz[3] = {0.0f, 0.0f, 0.0f};
    float last_xyz[3] = {0.0f, 0.0f, 0.0f};
    uint8_t target_pending = 0U;
    uart1 = uart_get_device("uart1_dma");
    uart10 = uart_get_device("uart10_dma");
    while (uart1 == NULL)
    {
        uart1 = uart_get_device("uart1_dma");
        LED_YELLOW_SET();
        osDelay(1000U);
    }
    while (uart10 == NULL)
    {
        uart10 = uart_get_device("uart10_dma");
        LED_YELLOW_SET();
        osDelay(1000U);
    }
    memset(&uart10_rx, 0, sizeof(uart10_rx));
    machine_protocol_init(&uart10_protocol);
    uart10->uart_recv_callback = uart10_receive_callback;
    last_query_tick = osKernelGetTickCount() - UART10_STATUS_QUERY_INTERVAL_MS;

    float aim_pose_array[5] = {0};
    float last_aim_pose_array[5] = {0};
    for (;;) {
        aim_pose_array[0] = aim_pose.x;
        aim_pose_array[1] = aim_pose.y;
        aim_pose_array[2] = aim_pose.z;
        aim_pose_array[3] = aim_pose.roll;
        aim_pose_array[4] = aim_pose.action;
        if (memcmp(aim_pose_array, last_aim_pose_array, sizeof(aim_pose_array)) != 0) {
            if (set_seria_target(uart1, aim_pose_array) == 0) {
                LED_SKY_SET();
            } else {
                LED_RED_SET();
            }
        }
        if (memcmp(aim_pose_array, last_xyz, sizeof(last_xyz)) != 0) {
            memcpy(pending_xyz, aim_pose_array, sizeof(pending_xyz));
            memcpy(last_xyz, aim_pose_array, sizeof(last_xyz));
            target_pending = 1U;
        }
        memcpy(last_aim_pose_array, aim_pose_array, sizeof(aim_pose_array));
        uint8_t received_byte;
        while (uart10_receive_byte(&received_byte) != 0) {
            uint32_t events = machine_protocol_parse(&uart10_protocol, &received_byte, 1U);
            if (events != MACHINE_PROTOCOL_EVENT_NONE) {
                uart10_handle_event(&uart10_state, &uart10_protocol, events);
            }
        }
        if (uart10_rx.overflow != 0U) {
            uart10_state = UART10_MOTION_FAULT;
            LED_RED_SET();
        }
        uint32_t now = osKernelGetTickCount();
        if (uart10_state == UART10_MOTION_WAIT_ACK && (uint32_t)(now - command_sent_tick) >= UART10_ACK_TIMEOUT_MS) {
            uart10_state = UART10_MOTION_FAULT;
            LED_RED_SET();
        }
        if ((uart10_state == UART10_MOTION_WAIT_READY || uart10_state == UART10_MOTION_WAIT_IDLE || uart10_state == UART10_MOTION_FAULT) && (uint32_t)(now - last_query_tick) >= UART10_STATUS_QUERY_INTERVAL_MS) {
            // if (uart10_send_status_query(uart10) == 0) {
            //     last_query_tick = now;
            // }
        }
        if (uart10_state == UART10_MOTION_IDLE && uart10_protocol.controller_state == MACHINE_PROTOCOL_STATE_IDLE && target_pending != 0U) {
            int command_length = machine_protocol_pack(pending_xyz,MACHINE_PROTOCOL_DEFAULT_FEED_MM_PER_MIN, uart10_command, sizeof(uart10_command));
            if (command_length <= 0) {
                uart10_state = UART10_MOTION_FAULT;
                LED_RED_SET();
            } else if (uart10->uart_send_bytes(uart10, (const uint8_t *)uart10_command,(uint16_t)command_length) == 0) {
                target_pending = 0U;
                command_sent_tick = now;
                uart10_state = UART10_MOTION_WAIT_ACK;
                LED_PURPLE_SET();
            }
        }
        osDelay(10U);
    }
}
