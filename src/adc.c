#include "adc.h"
#include "board.h"
#include "timebase.h"

/* ══════════════════════════════════════════════════════════════════
 * Kalibracja offsetu prądu (zero-current).
 * Przy wyłączonym mostku mierzymy średnią wartość ADC dla każdej fazy.
 * Pod obciążeniem prąd = (offset - raw) * skala.
 * ══════════════════════════════════════════════════════════════════ */
static uint16_t s_offset_ia = 2048U;
static uint16_t s_offset_ib = 2048U;
static uint16_t s_offset_ic = 2048U;
static bool     s_offset_calibrated = false;

void adc_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA |
                             LL_APB2_GRP1_PERIPH_ADC1);

    /* PA2..PA5 jako wejścia analogowe. */
    LL_GPIO_InitTypeDef g = {0};
    g.Mode = LL_GPIO_MODE_ANALOG;
    g.Pin  = LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
    LL_GPIO_Init(GPIOA, &g);

    /* F1: rozdzielczość ADC stała 12-bit (brak SetResolution). */
    LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);

    LL_ADC_Enable(ADC1);
    delay_ms(1);

    /* Kalibracja (wymagana po włączeniu na F1). */
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {
    }
    delay_ms(1);
}

uint16_t adc_read(uint32_t channel)
{
    LL_ADC_SetChannelSamplingTime(ADC1, channel, LL_ADC_SAMPLINGTIME_55CYCLES_5);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);

    LL_ADC_REG_StartConversionSWStart(ADC1);

    /* Timeout: max ~100k iteracji (≈1 ms przy 64 MHz) — bezpiecznik. */
    uint32_t timeout = 100000UL;
    while (!LL_ADC_IsActiveFlag_EOS(ADC1)) {
        if (--timeout == 0) {
            break;
        }
    }
    uint16_t v = LL_ADC_REG_ReadConversionData12(ADC1);
    LL_ADC_ClearFlag_EOS(ADC1);
    return v;
}

/* ══════════════════════════════════════════════════════════════════
 * Kalibracja offsetu — uśredniamy 256 próbek każdej fazy.
 * Mostek musi być wyłączony (coast).
 * ══════════════════════════════════════════════════════════════════ */
#define ADC_CALIB_SAMPLES  256U

void adc_calibrate_current_offset(void)
{
    uint32_t sum_ia = 0, sum_ib = 0, sum_ic = 0;

    for (uint32_t i = 0; i < ADC_CALIB_SAMPLES; i++) {
        sum_ia += adc_read(ADC_CH_IA);
        sum_ib += adc_read(ADC_CH_IB);
        sum_ic += adc_read(ADC_CH_IC);
    }

    s_offset_ia = (uint16_t)(sum_ia / ADC_CALIB_SAMPLES);
    s_offset_ib = (uint16_t)(sum_ib / ADC_CALIB_SAMPLES);
    s_offset_ic = (uint16_t)(sum_ic / ADC_CALIB_SAMPLES);
    s_offset_calibrated = true;
}

bool adc_is_current_calibrated(void)
{
    return s_offset_calibrated;
}

int16_t adc_read_current_ma(uint32_t channel)
{
    uint16_t raw    = adc_read(channel);
    uint16_t offset = 2048U;

    switch (channel) {
        case ADC_CH_IA: offset = s_offset_ia; break;
        case ADC_CH_IB: offset = s_offset_ib; break;
        case ADC_CH_IC: offset = s_offset_ic; break;
        default: break;
    }

    /* Prąd dodatni = płynie z mostka do fazy silnika.
     * W topologii M365 niższy ADC → wyższy prąd, stąd offset - raw. */
    int16_t diff = (int16_t)offset - (int16_t)raw;
    float ma = (float)diff * IPHASE_MA_PER_CNT;

    if (ma > 32767.0f)  ma = 32767.0f;
    if (ma < -32768.0f) ma = -32768.0f;
    return (int16_t)ma;
}
