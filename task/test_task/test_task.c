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
#include "../../bsp/pwm/pwm.h"
#include "../../bsp/uart/uart.h"
#include "../../protocol/protocol.h"
#include "../../protocol/machine_protocal.h"

#define UART10_RX_RING_SIZE 512U
#define UART10_STATUS_QUERY_INTERVAL_MS 100U
#define UART10_ACK_TIMEOUT_MS 2000U
#define UART10_IDLE_SETTLE_MS 500U
#define FINISH_PWM_RESET_MS 2000U
#define UART10_REQUIRE_CONTROLLER_FEEDBACK 1U

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

static int16_t roll_to_protocol_phi(float roll)
{
    float raw_phi = roll * 10.0f;

    if (raw_phi >= 0.0f) {
        return (int16_t)(raw_phi + 0.5f);
    }
    return (int16_t)(raw_phi - 0.5f);
}

static void aim_pose_to_protocol_data(struct protocol_data *target)
{
    if (target == NULL) {
        return;
    }

    target->x = aim_pose.x;
    target->y = aim_pose.y;
    target->z = aim_pose.z;
    target->roll = roll_to_protocol_phi(aim_pose.roll);
    target->action = aim_pose.action;
}

static int protocol_data_equal(const struct protocol_data *left,
                               const struct protocol_data *right)
{
    if (left == NULL || right == NULL) {
        return 0;
    }

    return left->x == right->x &&
           left->y == right->y &&
           left->z == right->z &&
           left->roll == right->roll &&
           left->action == right->action;
}

static int send_protocol_action(struct uart_device *uart,
                                const struct protocol_data *target)
{
    uint8_t protocol_frame[PROTOCOL_FRAME_SIZE];

    if (uart == NULL || target == NULL) {
        LED_YELLOW_SET();
        return -1;
    }

    if (protocol_pack(target, protocol_frame) != 0) {
        LED_RED_SET();
        return -1;
    }

    return uart->uart_send_bytes(uart, protocol_frame, PROTOCOL_FRAME_SIZE);
}

static int send_uart10_motion(struct uart_device *uart10,
                              const struct protocol_data *target,
                              char command[MACHINE_PROTOCOL_TX_BUFFER_SIZE])
{
    float xyz[3];
    int command_length;

    if (uart10 == NULL || target == NULL || command == NULL) {
        return -1;
    }

    xyz[0] = target->x;
    xyz[1] = target->y;
    xyz[2] = target->z;

    command_length = machine_protocol_pack(xyz,
                                           MACHINE_PROTOCOL_DEFAULT_FEED_MM_PER_MIN,
                                           command,
                                           MACHINE_PROTOCOL_TX_BUFFER_SIZE);
    if (command_length <= 0) {
        return -1;
    }

    return uart10->uart_send_bytes(uart10,
                                   (const uint8_t *)command,
                                   (uint16_t)command_length);
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
                                const struct machine_protocol_context *protocol,
                                uint32_t events,
                                uint32_t *idle_detected_tick)
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
            if (idle_detected_tick != NULL) {
                *idle_detected_tick = osKernelGetTickCount();
            }
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
    uint32_t idle_detected_tick = 0U;
    uint32_t finish_pwm_reset_tick = 0U;
    uint8_t sequence_active = 0U;
    uint8_t finish_pwm_reset_active = 0U;
    uint16_t sequence_count = 0U;
    uint16_t sequence_index = 0U;
    uint32_t sequence_generation = 0U;
    uint8_t legacy_target_pending = 0U;
    uint8_t last_legacy_valid = 0U;
    struct protocol_data action_to_send = {0};
    struct protocol_data legacy_action = {0};
    struct protocol_data last_legacy_action = {0};
    struct protocol_data pending_legacy_action = {0};

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
    aim_pose_to_protocol_data(&last_legacy_action);
    last_legacy_valid = 1U;

    for (;;) {
        uint8_t received_byte;
        uint32_t now;
        action_sequence_state_t sequence_state;
        uint32_t sequence_global_generation;
        int32_t lock_state;

        while (uart10_receive_byte(&received_byte) != 0) {
            uint32_t events = machine_protocol_parse(&uart10_protocol, &received_byte, 1U);
            if (events != MACHINE_PROTOCOL_EVENT_NONE) {
                uart10_handle_event(&uart10_state,
                                    &uart10_protocol,
                                    events,
                                    &idle_detected_tick);
            }
        }
        if (uart10_rx.overflow != 0U) {
            uart10_state = UART10_MOTION_FAULT;
            LED_RED_SET();
        }
        now = osKernelGetTickCount();
        if (uart10_state == UART10_MOTION_WAIT_ACK && (uint32_t)(now - command_sent_tick) >= UART10_ACK_TIMEOUT_MS) {
            uart10_state = UART10_MOTION_FAULT;
            LED_RED_SET();
        }
        if (UART10_REQUIRE_CONTROLLER_FEEDBACK != 0U &&
            (uart10_state == UART10_MOTION_WAIT_READY ||
             uart10_state == UART10_MOTION_WAIT_IDLE ||
             uart10_state == UART10_MOTION_FAULT) &&
            (uint32_t)(now - last_query_tick) >= UART10_STATUS_QUERY_INTERVAL_MS) {
            if (uart10_send_status_query(uart10) == 0) {
                last_query_tick = now;
            }
        }

        if (finish_pwm_reset_active != 0U &&
            (uint32_t)(now - finish_pwm_reset_tick) >= FINISH_PWM_RESET_MS) {
            pwm_set_pulse_us(PWM_CHANNEL_1, 20000U);
            finish_pwm_reset_active = 0U;
        }

        lock_state = osKernelLock();
        sequence_state = action_sequence.state;
        sequence_global_generation = action_sequence.generation;
        (void)osKernelRestoreLock(lock_state);

        if (sequence_active != 0U &&
            sequence_global_generation != sequence_generation &&
            sequence_state != ACTION_SEQUENCE_STATE_RUNNING) {
            sequence_active = 0U;
            sequence_count = 0U;
            sequence_index = 0U;
            legacy_target_pending = 0U;
            aim_pose_to_protocol_data(&last_legacy_action);
            last_legacy_valid = 1U;
        }

        if (finish_pwm_reset_active == 0U &&
            sequence_active == 0U &&
            sequence_state == ACTION_SEQUENCE_STATE_READY) {
            lock_state = osKernelLock();
            if (action_sequence.state == ACTION_SEQUENCE_STATE_READY &&
                action_sequence.count > 0U) {
                action_sequence.state = ACTION_SEQUENCE_STATE_RUNNING;
                action_sequence.active_index = 0U;
                sequence_count = action_sequence.count;
                sequence_generation = action_sequence.generation;
                sequence_index = 0U;
                sequence_active = 1U;
            }
            (void)osKernelRestoreLock(lock_state);

            if (sequence_active != 0U) {
                legacy_target_pending = 0U;
                aim_pose_to_protocol_data(&last_legacy_action);
                last_legacy_valid = 1U;
                LED_GREEN_SET();
            }
        }

        if (sequence_active != 0U && uart10_state == UART10_MOTION_FAULT) {
            lock_state = osKernelLock();
            if (action_sequence.state == ACTION_SEQUENCE_STATE_RUNNING) {
                action_sequence.state = ACTION_SEQUENCE_STATE_ERROR;
                action_sequence.generation++;
            }
            (void)osKernelRestoreLock(lock_state);
            sequence_active = 0U;
            LED_RED_SET();
        }

        if (sequence_active != 0U) {
            if (sequence_index >= sequence_count) {
                if (uart10_state == UART10_MOTION_IDLE) {
                    lock_state = osKernelLock();
                    if (action_sequence.state == ACTION_SEQUENCE_STATE_RUNNING) {
                        action_sequence.active_index = action_sequence.count;
                        action_sequence.state = ACTION_SEQUENCE_STATE_DONE;
                        action_sequence.generation++;
                    }
                    (void)osKernelRestoreLock(lock_state);
                    sequence_active = 0U;
                    aim_pose_to_protocol_data(&last_legacy_action);
                    last_legacy_valid = 1U;
                    pwm_set_pulse_us(PWM_CHANNEL_1, 0U);
                    finish_pwm_reset_tick = now;
                    finish_pwm_reset_active = 1U;
                }
            } else if (uart10_state == UART10_MOTION_IDLE &&
                       uart10_protocol.controller_state == MACHINE_PROTOCOL_STATE_IDLE &&
                       (uint32_t)(now - idle_detected_tick) >= UART10_IDLE_SETTLE_MS) {
                uint8_t have_action = 0U;

                lock_state = osKernelLock();
                if (action_sequence.state == ACTION_SEQUENCE_STATE_RUNNING &&
                    sequence_index < action_sequence.count) {
                    action_to_send = action_sequence.frames[sequence_index];
                    have_action = 1U;
                }
                (void)osKernelRestoreLock(lock_state);

                if (have_action == 0U) {
                    lock_state = osKernelLock();
                    action_sequence.state = ACTION_SEQUENCE_STATE_ERROR;
                    action_sequence.generation++;
                    (void)osKernelRestoreLock(lock_state);
                    sequence_active = 0U;
                    LED_RED_SET();
                } else if (send_protocol_action(uart1, &action_to_send) == 0 &&
                           send_uart10_motion(uart10, &action_to_send, uart10_command) == 0) {
                    sequence_index++;
                    lock_state = osKernelLock();
                    if (action_sequence.state == ACTION_SEQUENCE_STATE_RUNNING) {
                        action_sequence.active_index = sequence_index;
                    }
                    (void)osKernelRestoreLock(lock_state);
                    command_sent_tick = now;
                    uart10_state = UART10_MOTION_WAIT_ACK;
                    LED_PURPLE_SET();
                } else {
                    LED_RED_SET();
                }
            }
        } else if (finish_pwm_reset_active == 0U &&
                   (sequence_state == ACTION_SEQUENCE_STATE_IDLE ||
                    sequence_state == ACTION_SEQUENCE_STATE_DONE ||
                    sequence_state == ACTION_SEQUENCE_STATE_ERROR)) {
            aim_pose_to_protocol_data(&legacy_action);
            if (last_legacy_valid == 0U ||
                protocol_data_equal(&legacy_action, &last_legacy_action) == 0) {
                if (send_protocol_action(uart1, &legacy_action) == 0) {
                    LED_SKY_SET();
                } else {
                    LED_RED_SET();
                }
                pending_legacy_action = legacy_action;
                legacy_target_pending = 1U;
                last_legacy_action = legacy_action;
                last_legacy_valid = 1U;
            }

            if (legacy_target_pending != 0U &&
                uart10_state == UART10_MOTION_IDLE &&
                uart10_protocol.controller_state == MACHINE_PROTOCOL_STATE_IDLE &&
                (uint32_t)(now - idle_detected_tick) >= UART10_IDLE_SETTLE_MS) {
                if (send_uart10_motion(uart10, &pending_legacy_action, uart10_command) == 0) {
                    legacy_target_pending = 0U;
                    command_sent_tick = now;
                    uart10_state = UART10_MOTION_WAIT_ACK;
                    LED_PURPLE_SET();
                } else {
                    LED_RED_SET();
                }
            }
        }

        osDelay(10U);
    }
}
