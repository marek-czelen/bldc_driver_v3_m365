/** adc.h — pomiary Vbat i prądów faz (odczyt na żądanie). */
#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);

/* Pojedynczy odczyt wybranego kanału (ADC_CH_*), zwraca surowe 12-bit. */
uint16_t adc_read(uint32_t channel);

#endif /* ADC_H */
