//
// Created by charlotte on 7/14/26.
//

#include "printf_task.h"
#include "cmsis_os2.h"
#include "../../device/motor/motor.h"
#include "../../device/YB_SD15M/YB_SD15M.h"
#include "../../bsp/uart/uart.h"
#include "../../application/global_data.h"
#include "../../dsp/calculation/calculation.h"
#include "../../bsp/pwm/pwm.h"
#include <string.h>


static volatile uint8_t flag = 0U;

static void uart1_echo_callback(struct uart_device *device,
                                const uint8_t *data,
                                uint16_t length)
{
    if (data[0] == 0xAA)
    {
        (void)device->uart_printf(device,"charge\r\n");
        pwm_set_pulse_us(PWM_CHANNEL_1,20000U);
        pwm_set_pulse_us(PWM_CHANNEL_2,0U);
        flag = 1;
    }

    else if (data[0] == 0xBB)
    {
        (void)device->uart_printf(device,"fire\r\n");
        pwm_set_pulse_us(PWM_CHANNEL_1,0U);
        pwm_set_pulse_us(PWM_CHANNEL_2,20000U);
        flag = 2;
    }

    else if (data[0] == 0xCC)
    {
        (void)device->uart_printf(device,"lose_charge\r\n");
        pwm_set_pulse_us(PWM_CHANNEL_1,0U);
        pwm_set_pulse_us(PWM_CHANNEL_2,20000U);
        flag = 3;
    }

    else if (data[0] == 0xDD)
    {
        (void)device->uart_printf(device,"stop\r\n");
        pwm_set_pulse_us(PWM_CHANNEL_1,0U);
        pwm_set_pulse_us(PWM_CHANNEL_2,0U);
        flag = 4;
    }
}

void printf_task(void)
{
    struct uart_device *uart1;

    uart1 = uart_get_device("uart1_dma");
    uart1->uart_recv_callback = uart1_echo_callback;

    for (;;) {
        if (flag == 1)
        {
         osDelay(1);
        }
        else if (flag == 2)
        {
            osDelay(10);
            pwm_set_pulse_us(PWM_CHANNEL_1,0U);
            pwm_set_pulse_us(PWM_CHANNEL_2,0U);
            flag = 4;

        }
        else if (flag == 3)
        {
            osDelay(1);
        }
        else if (flag == 4)
        {
            osDelay(1);
        }
        else
        {
            osDelay(1);
        }




    }
}
