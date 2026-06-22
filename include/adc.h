/** adc.h — pomiary Vbat i prądów faz (odczyt na żądanie).
 * Prąd: odczytywany jako (offset - raw) * skala → mA.
 * Kalibracja offsetu (zero-current) wykonywana przy starcie. */
#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stdbool.h>

void adc_init(void);

/* Pojedynczy odczyt wybranego kanału (ADC_CH_*), zwraca surowe 12-bit. */
uint16_t adc_read(uint32_t channel);

/* Kalibruje offset zera prądu (uśrednia N próbek na postoju).
 * Wołać po inicie ADC, przed wysterowaniem mostka. */
void adc_calibrate_current_offset(void);

/* Odczyt prądu fazy [mA] z uwzględnieniem skalibrowanego offsetu.
 * Dodatni = prąd płynie z mostka do silnika. */
int16_t adc_read_current_ma(uint32_t channel);

/* Czy kalibracja offsetu prądu została już wykonana. */
bool adc_is_current_calibrated(void);

#endif /* ADC_H */
