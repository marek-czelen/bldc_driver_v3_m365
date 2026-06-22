/** pwm.h — TIM1, 3-fazowy PWM komplementarny dla mostka BLDC. */
#ifndef PWM_H
#define PWM_H

#include <stdint.h>

/* Identyfikatory faz */
typedef enum {
    PH_A = 0,
    PH_B = 1,
    PH_C = 2,
} phase_t;

void pwm_init(void);

/* Komutacja 6-step: faza "hi" steruje PWM (high-side modulowany),
 * faza "lo" ma stale załączony low-side, trzecia faza pływa.
 * duty: 0..PWM_DUTY_MAX. hi/lo: PH_A/PH_B/PH_C lub -1 (brak). */
void pwm_apply_step(int8_t hi, int8_t lo, uint16_t duty);

/* Hamowanie zwarciowe: wszystkie low-side załączone. */
void pwm_brake(void);

/* Wybieg: wszystkie tranzystory wyłączone (wysoka impedancja). */
void pwm_coast(void);

#endif /* PWM_H */
