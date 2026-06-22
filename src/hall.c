#include "hall.h"
#include "board.h"
#include "motor.h"

void hall_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB |
                             LL_APB2_GRP1_PERIPH_AFIO);

    /*
     * Odblokowanie PB4 (JTRST): domyślnie zarezerwowany dla JTAG.
     * Tryb NOJTAG zostawia SWD (PA13/PA14) sprawne — programowanie ST-Link działa.
     * Bez tego PB4 (Hall H1) nie działa jako EXTI.
     */
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    LL_GPIO_InitTypeDef g = {0};
    g.Mode = LL_GPIO_MODE_FLOATING;     /* zakładamy zewn. pull-up; */
    g.Pin  = HALL1_PIN | HALL2_PIN | HALL3_PIN;
    /* Dla wewnętrznego pull-up użyj: g.Mode = LL_GPIO_MODE_INPUT; g.Pull = LL_GPIO_PULL_UP; */
    LL_GPIO_Init(HALL_PORT, &g);

    /* Źródła EXTI: PB0->line0, PB4->line4, PB5->line5 */
    LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE0);
    LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE4);
    LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE5);

    LL_EXTI_InitTypeDef e = {0};
    e.Line_0_31   = LL_EXTI_LINE_0 | LL_EXTI_LINE_4 | LL_EXTI_LINE_5;
    e.LineCommand = ENABLE;
    e.Mode        = LL_EXTI_MODE_IT;
    e.Trigger     = LL_EXTI_TRIGGER_RISING_FALLING;
    LL_EXTI_Init(&e);

    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_SetPriority(EXTI4_IRQn, 1);
    NVIC_SetPriority(EXTI9_5_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

uint8_t hall_read(void)
{
    uint8_t s = 0;
    if (LL_GPIO_IsInputPinSet(HALL_PORT, HALL1_PIN)) s |= 0x01;
    if (LL_GPIO_IsInputPinSet(HALL_PORT, HALL2_PIN)) s |= 0x02;
    if (LL_GPIO_IsInputPinSet(HALL_PORT, HALL3_PIN)) s |= 0x04;
    return s;
}

/* --- Obsługa przerwań EXTI --------------------------------------- */
void EXTI0_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
        motor_on_hall();
    }
}

void EXTI4_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
        motor_on_hall();
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
        motor_on_hall();
    }
}
