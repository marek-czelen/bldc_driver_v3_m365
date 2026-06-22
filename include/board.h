/**
 * board.h — definicje sprzętowe i parametry konfiguracyjne.
 * Zgodne z hardware.md (STM32F103C8T6).
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32f1xx.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_pwr.h"

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* Zegar  (brak kwarcu — wewnętrzny oscylator HSI)                     */
/* ------------------------------------------------------------------ */
#define SYSCLK_HZ          64000000UL   /* HSI 8 MHz /2 * PLL x16    */
#define APB2_TIM_HZ        64000000UL   /* TIM1 (APB2 /1)            */

/* ------------------------------------------------------------------ */
/* PWM / TIM1 (mostek 3-fazowy, wyjścia komplementarne)               */
/* ------------------------------------------------------------------ */
/* f_pwm = SYSCLK / (2 * ARR)  -> center-aligned                      */
#define PWM_ARR            1600U        /* 64MHz/(2*1600) = 20 kHz   */
#define PWM_FREQ_HZ        20000U
#define PWM_DUTY_MAX       1520U        /* 95% (rezerwa na bootstrap)*/
#define PWM_DEADTIME_REG   64U          /* ~1 us przy 64 MHz         */

/* Wyjścia PWM high-side: PA8/PA9/PA10 (TIM1_CH1/2/3)                  */
/* Wyjścia PWM low-side : PB13/PB14/PB15 (TIM1_CH1N/2N/3N)            */
#define PWM_HS_PORT        GPIOA
#define PWM_HS_PINS        (LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10)
#define PWM_LS_PORT        GPIOB
#define PWM_LS_PINS        (LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15)

/* ------------------------------------------------------------------ */
/* Czujniki Halla (EXTI)                                              */
/* ------------------------------------------------------------------ */
#define HALL_PORT          GPIOB
#define HALL1_PIN          LL_GPIO_PIN_4   /* bit 0 */
#define HALL2_PIN          LL_GPIO_PIN_5   /* bit 1 */
#define HALL3_PIN          LL_GPIO_PIN_0   /* bit 2 */

/* ------------------------------------------------------------------ */
/* ADC                                                               */
/* ------------------------------------------------------------------ */
#define ADC_CH_VBAT        LL_ADC_CHANNEL_2   /* PA2 */
#define ADC_CH_IA          LL_ADC_CHANNEL_3   /* PA3 */
#define ADC_CH_IB          LL_ADC_CHANNEL_4   /* PA4 */
#define ADC_CH_IC          LL_ADC_CHANNEL_5   /* PA5 */

/* Skalowania (DOSTOSUJ do dzielnika napięcia / czujnika prądu).     */
#define VBAT_MV_PER_COUNT  10.0f   /* placeholder: mV na 1 count ADC */
/* Współczynnik konwersji ADC→mA: mA = (offset - raw) * IPHASE_MA_PER_CNT.
 * Dla M365 (shunt ~10mΩ, wzmacniacz ~50V/V, Vref=3.3V): ~15 mA/count.
 * Kalibracja: zmierz prąd miernikiem, odczytaj surowy ADC, wylicz.
 *   IPHASE_MA_PER_CNT = I_zmierzone_mA / |ADC_offset - ADC_obciążenie|   */
#define IPHASE_MA_PER_CNT  15.0f    /* mA na 1 count ADC (dla M365)  */

/* ------------------------------------------------------------------ */
/* UART (USART3, PB10 TX / PB11 RX)                                  */
/* ------------------------------------------------------------------ */
#define UART_INSTANCE      USART3
#define UART_BAUDRATE      115200U
#define UART_PERIPH_CLK_HZ 32000000UL   /* APB1 /2 = 32 MHz          */

/* ------------------------------------------------------------------ */
/* Silnik                                                            */
/* ------------------------------------------------------------------ */
#define MOTOR_POLE_PAIRS   15U
/* Po jakim czasie bez zbocza Halla uznajemy, że silnik stoi (us).   */
#define HALL_STALL_US      300000UL
#define HALL_TIMEOUT_MS    200U        /* timeout Halla w ms (dla SINUS) */

/* Uczenie sekwencji Halla (komenda L[%]) */
#define LEARN_DUTY_PCT     30.0f      /* domyślne duty podczas uczenia  */
#define LEARN_SETTLE_MS    600U       /* czas na ustabiliz. mechaniczne */

#endif /* BOARD_H */
