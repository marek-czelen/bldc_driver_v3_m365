/** sinus.h — sterowanie sinusoidalne (SVPWM z 3. harmoniczną). */
#ifndef SINUS_H
#define SINUS_H

#include <stdint.h>
#include <stdbool.h>

void sinus_init(void);
void sinus_start(void);            /* reset estymatora kąta */
void sinus_stop(void);

/* Ustaw amplitudę 0..100%. Zapewnia płynną zmianę (slew rate). */
void sinus_set_amplitude(float pct);

/* Zdarzenie zbocza Halla — aktualizuje estymator kąta i prędkości. */
void sinus_hall_edge(uint8_t hall_state);

/* Główna pętla sinusa — wołana co 1 ms z motor_tick_1ms.
 * Estymuje kąt, generuje 3-fazowy sinus i wpisuje do CCR1/2/3. */
void sinus_update(void);

/* Odczyty */
float  sinus_get_rpm(void);        /* RPM mechaniczne */
float  sinus_get_amplitude(void);  /* aktualna amplituda % */
uint16_t sinus_get_angle(void);    /* kąt elektryczny 0..65535 (0..360°) */
bool sinus_is_running(void);

#endif /* SINUS_H */
