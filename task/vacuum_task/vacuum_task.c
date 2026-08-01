//
// Created by charlotte on 7/20/26.
//

#include "vacuum_task.h"
#include <cmsis_os2.h>
#include "../../bsp/pwm/pwm.h"

void vacuum_task(void)
{
    pwm_power_enable();
    pwm_set_pulse_us(PWM_CHANNEL_1, 20000U);
    for (;;)
    {
        Vacuum_All_Update();
        osDelay(10);
    }
}