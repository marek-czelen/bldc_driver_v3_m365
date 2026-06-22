#include "clock.h"
#include "board.h"

/*
 * SYSCLK = 64 MHz: HSI (8 MHz) / 2 -> PLL x16  (identycznie jak v2)
 * Użycie HSI (wewnętrzny RC) zamiast HSE: działa nawet bez zewnętrznego kwarcu.
 * AHB  = 64 MHz (/1)
 * APB1 = 32 MHz (/2)  -> timery APB1 x2 = 64 MHz
 * APB2 = 64 MHz (/1)  -> TIM1 = 64 MHz
 * ADC  = PCLK2 / 6 = 10.67 MHz (< 14 MHz)
 */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
    }

    /* HSI zawsze dostępny — nie czekamy na zewnętrzny kwarc. */
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1) {
    }

    /* HSI/2 = 4 MHz, PLL x16 = 64 MHz */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1) {
    }

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }

    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);

    SystemCoreClock = SYSCLK_HZ;
}
