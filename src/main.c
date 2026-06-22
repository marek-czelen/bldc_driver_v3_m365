#include "board.h"
#include "clock.h"
#include "timebase.h"
#include "pwm.h"
#include "hall.h"
#include "adc.h"
#include "uart.h"
#include "motor.h"
#include "cli.h"

int main(void)
{
    SystemClock_Config();
    timebase_init();

    /* SysTick co 1 ms (timebase + pętla regulacji silnika). */
    LL_InitTick(SYSCLK_HZ, 1000U);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, 0);

    pwm_init();
    hall_init();
    adc_init();
    uart_init();
    motor_init();
    cli_init();

    for (;;) {
        cli_process();
    }
}

/* SysTick 1 kHz */
void SysTick_Handler(void)
{
    timebase_systick_tick();
    motor_tick_1ms();
}
