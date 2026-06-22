#include "adc.h"
#include "board.h"
#include "timebase.h"

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
    while (!LL_ADC_IsActiveFlag_EOS(ADC1)) {
    }
    uint16_t v = LL_ADC_REG_ReadConversionData12(ADC1);
    LL_ADC_ClearFlag_EOS(ADC1);
    return v;
}
