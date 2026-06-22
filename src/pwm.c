#include "pwm.h"
#include "board.h"

/* Maski kanałów (high + complementary low) dla każdej fazy. */
static const uint32_t k_ch[3]  = { LL_TIM_CHANNEL_CH1,  LL_TIM_CHANNEL_CH2,  LL_TIM_CHANNEL_CH3  };
static const uint32_t k_chn[3] = { LL_TIM_CHANNEL_CH1N, LL_TIM_CHANNEL_CH2N, LL_TIM_CHANNEL_CH3N };

static void phase_set_compare(phase_t ph, uint16_t v)
{
    switch (ph) {
        case PH_A: LL_TIM_OC_SetCompareCH1(TIM1, v); break;
        case PH_B: LL_TIM_OC_SetCompareCH2(TIM1, v); break;
        case PH_C: LL_TIM_OC_SetCompareCH3(TIM1, v); break;
    }
}

static void phase_enable(phase_t ph)
{
    LL_TIM_CC_EnableChannel(TIM1, k_ch[ph] | k_chn[ph]);
}

static void phase_disable(phase_t ph)
{
    LL_TIM_CC_DisableChannel(TIM1, k_ch[ph] | k_chn[ph]);
}

static void gpio_init(void)
{
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
}

void pwm_init(void)
{
    gpio_init();

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* Baza czasu: center-aligned (symetryczny PWM). */
    LL_TIM_InitTypeDef t = {0};
    t.Prescaler         = 0;
    t.CounterMode       = LL_TIM_COUNTERMODE_CENTER_UP;
    t.Autoreload        = PWM_ARR;
    t.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
    t.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &t);
    LL_TIM_EnableARRPreload(TIM1);

    /* Kanały OC 1..3 — PWM1, komplementarne, na razie wyłączone. */
    LL_TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = LL_TIM_OCMODE_PWM1;
    oc.OCState      = LL_TIM_OCSTATE_DISABLE;
    oc.OCNState     = LL_TIM_OCSTATE_DISABLE;
    oc.CompareValue = 0;
    oc.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
    oc.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
    oc.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &oc);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &oc);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &oc);

    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

    /* Dead-time + ustawienia mostka. */
    LL_TIM_BDTR_InitTypeDef bd = {0};
    bd.OSSRState       = LL_TIM_OSSR_DISABLE;
    bd.OSSIState       = LL_TIM_OSSI_DISABLE;
    bd.LockLevel       = LL_TIM_LOCKLEVEL_OFF;
    bd.DeadTime        = PWM_DEADTIME_REG;
    bd.BreakState      = LL_TIM_BREAK_DISABLE;
    bd.BreakPolarity   = LL_TIM_BREAK_POLARITY_HIGH;
    bd.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &bd);

    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);

    pwm_coast();
}

void pwm_apply_step(int8_t hi, int8_t lo, uint16_t duty)
{
    if (duty > PWM_DUTY_MAX) {
        duty = PWM_DUTY_MAX;
    }

    for (phase_t ph = PH_A; ph <= PH_C; ph++) {
        if ((int8_t)ph == hi) {
            phase_set_compare(ph, duty);   /* high-side modulowany */
            phase_enable(ph);
        } else if ((int8_t)ph == lo) {
            phase_set_compare(ph, 0);      /* low-side stale ON     */
            phase_enable(ph);
        } else {
            phase_set_compare(ph, 0);
            phase_disable(ph);             /* faza pływa            */
        }
    }
}

void pwm_brake(void)
{
    for (phase_t ph = PH_A; ph <= PH_C; ph++) {
        phase_set_compare(ph, 0);
        phase_enable(ph);
    }
}

void pwm_coast(void)
{
    for (phase_t ph = PH_A; ph <= PH_C; ph++) {
        phase_set_compare(ph, 0);
        phase_disable(ph);
    }
}
