#include "timebase.h"
#include "board.h"

static volatile uint32_t s_millis;

void timebase_init(void)
{
    /* Włącz licznik cykli DWT (mikrosekundy) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_millis = 0;
}

void timebase_systick_tick(void)
{
    s_millis++;
}

uint32_t millis(void)
{
    return s_millis;
}

uint32_t micros(void)
{
    return DWT->CYCCNT / (SYSCLK_HZ / 1000000UL);
}

void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) {
        __NOP();
    }
}
