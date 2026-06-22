#include "pwm.h"
#include "board.h"

/*
 * Architektura sprzętowa komutacji:
 *
 * TIM1 działa w trybie center-aligned PWM z CCPC=1 (Capture/Compare
 * Preload Control). Oznacza to, że rejestry CCER i CCRx są buforowane —
 * zapis do nich trafia do rejestru cienia (preload), a faktyczne
 * przełączenie następuje DOPIERO przy zdarzeniu COM.
 *
 * Dzięki temu ISR Halla może spokojnie wpisać nowe wartości do preloadu
 * w dowolnym momencie cyklu PWM, a sprzęt przełączy fazy atomowo
 * w jednym, zsynchronizowanym momencie (przy najbliższym COM).
 *
 * Przepływ dla normalnej pracy (RUN):
 *   ISR Halla → pwm_preload_step(hi,lo,duty) → wpis do CCER+CCRx (preload)
 *            → pwm_commutate()                 → TIM1->EGR = COMG
 *            → HW kopiuje preload → active     → fazy przełączone atomowo
 *
 * Tryb bezpośredni (MANUAL / LEARN):
 *   pwm_apply_step_direct() → tymczasowo wyłącza CCPC → wpis bezpośredni
 */

/* Maski bitów CCER dla poszczególnych faz (high + low side).
 * Używamy zapisu 32-bitowego (CCER to jeden rejestr 16-bit z parami CCxE/CCxNE). */
#define CCER_CH1      (TIM_CCER_CC1E  | TIM_CCER_CC1NE)
#define CCER_CH2      (TIM_CCER_CC2E  | TIM_CCER_CC2NE)
#define CCER_CH3      (TIM_CCER_CC3E  | TIM_CCER_CC3NE)
#define CCER_ALL      0x5555U  /* wszystkie 6 kanałów */

/* Tablica: dla fazy PH_A/PH_B/PH_C → maska bitów CCER */
static const uint16_t ccer_mask[3] = { CCER_CH1, CCER_CH2, CCER_CH3 };

/* Mapa: faza → rejestr CCR */
static inline volatile uint32_t *ccr_of(phase_t ph) {
    switch (ph) {
        case PH_A: return &TIM1->CCR1;
        case PH_B: return &TIM1->CCR2;
        case PH_C: return &TIM1->CCR3;
    }
    return &TIM1->CCR1;
}

/* Ustaw mostek: hi=modulowany PWM, lo=stale ON, reszta=wyłączona.
 * ccpc_mode: true = preload (CCPC=1, wartości czekają na COM),
 *            false = bezpośrednio (CCPC=0, wartości od razu aktywne). */
static void apply_step_int(int8_t hi, int8_t lo, uint16_t duty, bool ccpc_mode)
{
    if (duty > PWM_DUTY_MAX) {
        duty = PWM_DUTY_MAX;
    }

    uint16_t ccer_val = 0;

    for (phase_t ph = PH_A; ph <= PH_C; ph++) {
        if ((int8_t)ph == hi) {
            *ccr_of(ph) = duty;
            ccer_val |= ccer_mask[ph];
        } else if ((int8_t)ph == lo) {
            *ccr_of(ph) = 0;
            ccer_val |= ccer_mask[ph];
        } else {
            *ccr_of(ph) = 0;
            /* faza pływa — CCER=0 dla tego kanału */
        }
    }

    if (ccpc_mode) {
        /* CCPC=1: zapis do CCER/CCRx → preload. COM załaduje do aktywnych. */
        TIM1->CCER = ccer_val;
    } else {
        /* Tryb bezpośredni: wyłącz CCPC → zapis idzie od razu do aktywnych. */
        TIM1->CR2 &= ~TIM_CR2_CCPC;
        TIM1->CCER = ccer_val;
        TIM1->EGR = TIM_EGR_COMG;   /* dla pewności synchronizacja */
        TIM1->CR2 |= TIM_CR2_CCPC;
    }
}

/* ------------------------------------------------------------------ */
/* API                                                                */
/* ------------------------------------------------------------------ */

void pwm_init(void)
{
    /* GPIO: PA8/9/10 = HS PWM, PB13/14/15 = LS PWM */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA |
                             LL_APB2_GRP1_PERIPH_GPIOB |
                             LL_APB2_GRP1_PERIPH_AFIO);

    LL_GPIO_InitTypeDef g = {0};
    g.Mode       = LL_GPIO_MODE_ALTERNATE;
    g.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    g.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

    g.Pin = PWM_HS_PINS;
    LL_GPIO_Init(PWM_HS_PORT, &g);

    g.Pin = PWM_LS_PINS;
    LL_GPIO_Init(PWM_LS_PORT, &g);

    /* TIM1 — center-aligned, CCPC=1 (preload na COM) */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* Odblokuj dostęp do rejestrów TIM1 (advanced timer lock). */
    TIM1->BDTR = TIM_BDTR_MOE;

    TIM1->PSC  = 0;
    TIM1->ARR  = PWM_ARR;
    TIM1->CR1  = TIM_CR1_ARPE | TIM_CR1_CMS_1;   /* ARR preload + center-aligned */
    TIM1->CR2  = TIM_CR2_CCPC;                     /* preload CCER/CCRx na COM    */
    TIM1->RCR  = 0;

    /* Kanały OC 1..3 — PWM1 na high-side, low-side komplementarny.
     * Polaryzacja: high active-high, low active-high (z dead-time nie odwracamy). */
    TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;

    TIM1->CCER = 0;  /* wszystkie kanały wyłączone na start */

    /* Dead-time + automatyczne wyjścia */
    TIM1->BDTR = TIM_BDTR_MOE | PWM_DEADTIME_REG;

    /* Generuj UPDATE aby załadować ARR, potem COM dla CCER/CCR */
    TIM1->EGR = TIM_EGR_UG;
    TIM1->EGR = TIM_EGR_COMG;
    TIM1->CR1 |= TIM_CR1_CEN;

    pwm_coast();
}

void pwm_preload_step(int8_t hi, int8_t lo, uint16_t duty)
{
    apply_step_int(hi, lo, duty, true);
}

void pwm_commutate(void)
{
    TIM1->EGR = TIM_EGR_COMG;
}

void pwm_apply_step_direct(int8_t hi, int8_t lo, uint16_t duty)
{
    apply_step_int(hi, lo, duty, false);
}

void pwm_brake(void)
{
    /* Wszystkie low-side ON, high-side OFF.
     * PWM1 z CCR=0: OCxREF=0 zawsze → OCxN (low-side)=1 zawsze. */
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    TIM1->CCER = CCER_ALL;
    TIM1->EGR = TIM_EGR_COMG;
}

void pwm_coast(void)
{
    /* Wszystkie tranzystory wyłączone (wysoka impedancja). */
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    TIM1->CCER = 0;
    TIM1->EGR = TIM_EGR_COMG;
}

void pwm_set_ccpc(bool on)
{
    if (on) {
        TIM1->CR2 |= TIM_CR2_CCPC;
    } else {
        TIM1->CR2 &= ~TIM_CR2_CCPC;
    }
}
