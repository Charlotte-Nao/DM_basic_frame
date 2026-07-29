/**
 * @file test_task.c
 * @brief Aim pose serial forwarding test.
 */

#include "test_task.h"

#include <stdint.h>
#include <string.h>

#include "cmsis_os2.h"
#include "../../device/vacuum/vacuum.h"
#include "../../application/global_data.h"
#include "../../bsp/LED/LED.h"
#include "../../bsp/uart/uart.h"
#include "../../protocol/protocol.h"
#include "../../protocol/machine_protocal.h"

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
    uint8_t machine_aim[MACHINE_PROTOCAL_FRAME_SIZE];
    struct protocol_data protocol_target;
    struct uart_device *uart1;
    struct uart_device *uart10;
    uart1 = uart_get_device("uart1_dma");
    uart10 = uart_get_device("uart10_dma");

    if (uart == NULL || aim_pose == NULL) {
        return -1;
    }
    if (uart10 == NULL) {
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

        return uart1->uart_send_bytes(uart1, protocol_frame, PROTOCOL_FRAME_SIZE);
    }
    else if (uart_is_port(uart, "uart10")) {

        if (machine_protocol_pack(aim_pose, machine_aim) != 0) {
            LED_RED_SET();
            return -1;
        }

        return uart10->uart_send_bytes(uart10, machine_aim, MACHINE_PROTOCAL_FRAME_SIZE);
    }

    return -1;
}

void test_task(void)
{
    struct uart_device *uart1;

    uart1 = uart_get_device("uart1_dma");

    while (uart1 == NULL)
    {
        uart1 = uart_get_device("uart1_dma");
        osDelay(1000U);
    }

    float aim_pose_array[5] = {0};
    float last_aim_pose_array[5] = {0};

    for (;;) {
        aim_pose_array[0] = aim_pose.x;
        aim_pose_array[1] = aim_pose.y;
        aim_pose_array[2] = aim_pose.z;
        aim_pose_array[3] = aim_pose.roll;
        aim_pose_array[4] = aim_pose.action;

        if (memcmp(aim_pose_array,last_aim_pose_array,sizeof(aim_pose_array))!= 0)
        {
            if (set_seria_target(uart1, aim_pose_array) == 0)
            {
                LED_GREEN_SET();
            }
            else
            {
                LED_RED_SET();
            }
            osDelay(1U);

            if (set_seria_target(uart1, aim_pose_array) == 0)
            {
                LED_GREEN_SET();
            }
            else
            {
                LED_RED_SET();
            }

        }
        memcpy(last_aim_pose_array,aim_pose_array,sizeof(aim_pose_array));

        osDelay(1);
    }
}
