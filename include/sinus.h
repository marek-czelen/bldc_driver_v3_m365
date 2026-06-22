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

/* Ustaw / odczytaj przesunięcie fazowe (advance) w stopniach.
 * Domyślnie 60°. Wyższa wartość = większy kąt wyprzedzenia.
 * Zakres 0..120°. */
void sinus_set_advance_deg(float deg);
float sinus_get_advance_deg(void);

/* Nachylenie redukcji advance od prądu [deg/A].
 * advance_efektywne = advance_bazowe - slope * |I_avg|
 * Domyślnie 0. Przykład: slope=20 → przy 1A advance spada o 20°. */
void sinus_set_advance_slope(float deg_per_a);
float sinus_get_advance_slope(void);

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
bool sinus_get_and_clear_stall(void);   /* zwraca true jeśli był stall, kasuje */

#endif /* SINUS_H */
