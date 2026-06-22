/** timebase.h — pomiar czasu: millis (SysTick) i micros (DWT). */
#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

void timebase_init(void);          /* DWT + zmienne; SysTick konfig. w main */
uint32_t millis(void);
uint32_t micros(void);
void delay_ms(uint32_t ms);
void timebase_systick_tick(void);  /* wywoływane z SysTick_Handler */

#endif /* TIMEBASE_H */
