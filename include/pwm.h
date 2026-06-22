/** pwm.h — TIM1, 3-fazowy PWM komplementarny + sprzętowa komutacja COM. */
#ifndef PWM_H
#define PWM_H

#include <stdint.h>

/* Identyfikatory faz */
typedef enum {
    PH_A = 0,
    PH_B = 1,
    PH_C = 2,
} phase_t;

/* Inicjalizacja TIM1 z preload przez COM (CCPC=1). */
void pwm_init(void);

/* -------- Tryb sprzętowy (RUN) -------- */

/* Wpisuje nowy stan do rejestrów PRZEDŁADOWANIA (CCER + CCRx).
 * Wywoływane z ISR Halla. Rzeczywiste przełączenie nastąpi przy COM. */
void pwm_preload_step(int8_t hi, int8_t lo, uint16_t duty);

/* Atomowe przełączenie: preload → active (TIM1->EGR = COMG). */
void pwm_commutate(void);

/* -------- Tryb bezpośredni (IDLE / BRAKE / MANUAL / LEARN) -------- */

/* Natychmiastowe ustawienie mostka (omija preload). */
void pwm_apply_step_direct(int8_t hi, int8_t lo, uint16_t duty);

/* Hamowanie zwarciowe: wszystkie low-side załączone. */
void pwm_brake(void);

/* Wybieg: wszystkie tranzystory wyłączone. */
void pwm_coast(void);

#endif /* PWM_H */
