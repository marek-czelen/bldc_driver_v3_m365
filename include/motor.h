/** motor.h — sterowanie silnikiem BLDC (tryb BLOCK; SINUS w przyszłości). */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MODE_BLOCK = 0,   /* komutacja 6-step (zaimplementowane)        */
    MODE_SINUS = 1,   /* sterowanie sinusoidalne (szkielet)         */
} motor_mode_t;

typedef enum {
    MSTATE_IDLE = 0,  /* wybieg, mostek wyłączony                   */
    MSTATE_RUN  = 1,  /* praca                                      */
    MSTATE_BRAKE= 2,  /* hamowanie zwarciowe                        */
    MSTATE_FAULT= 3,  /* błąd (np. niepoprawny stan Halla)          */
    MSTATE_MANUAL=4,  /* ręczny wektor (kalibracja, open-loop)      */
} motor_state_t;

typedef enum {
    DIR_FWD = 0,
    DIR_REV = 1,
} motor_dir_t;

typedef enum {
    CTRL_DUTY  = 0,   /* sterowanie wypełnieniem (open-loop)        */
    CTRL_SPEED = 1,   /* sterowanie prędkością (closed-loop PI)     */
} ctrl_mode_t;

void motor_init(void);

/* Sterowanie stanem */
void motor_start(void);
void motor_stop(void);
void motor_brake(void);

/* Wymuszenie wektora komutacji 0..5 (open-loop, bez Halla).
 * Służy do kalibracji tablicy/sprawdzenia mostka. Używa aktualnego duty. */
void motor_force_vector(uint8_t idx);

/* Uczenie sekwencji Halla: wymusza 6 wektorów i buduje tablice komutacji.
 * seq_out (opcjonalne, >=6 elem.) wypełniane odczytanymi stanami Halla.
 * duty_pct: 0 = domyślne. Zwraca true gdy sekwencja poprawna. */
bool motor_learn_hall_with_duty(float duty_pct, uint8_t *seq_out);
bool motor_learn_hall(uint8_t *seq_out);
bool motor_is_learned(void);

/* EEPROM: zapis/odczyt/kasowanie konfiguracji Halla. */
bool motor_config_save(void);          /* zapisz aktualną tablicę do Flash */
bool motor_config_load(void);          /* wczytaj tablicę z Flash (true=OK) */
bool motor_config_erase(void);         /* wyczyść zapisaną konfigurację */
uint8_t motor_get_hall_seq(uint8_t idx); /* odczytaj nauczony stan Halla dla wektora idx (0..5) */

/* Konfiguracja */
void motor_set_mode(motor_mode_t mode);
void motor_set_dir(motor_dir_t dir);
void motor_set_duty_pct(float pct);     /* 0..100, przełącza w CTRL_DUTY  */
void motor_set_target_rpm(float rpm);   /* przełącza w CTRL_SPEED         */
void motor_set_advance_deg(float deg);  /* przesunięcie fazowe SINUS [°]  */
void motor_set_advance_slope(float deg_per_a); /* redukcja advance od prądu [°/A] */

/* Odczyty */
motor_mode_t  motor_get_mode(void);
motor_state_t motor_get_state(void);
motor_dir_t   motor_get_dir(void);
ctrl_mode_t   motor_get_ctrl(void);
float         motor_get_rpm(void);
float         motor_get_duty_pct(void);
uint8_t       motor_get_hall(void);
float         motor_get_advance_deg(void);  /* przesunięcie fazowe SINUS [°] */
float         motor_get_advance_slope(void); /* redukcja advance [°/A]        */

/* Wywoływane z ISR Halla (komutacja) */
void motor_on_hall(void);

/* Pętla regulacji — wywoływana co 1 ms z SysTick. */
void motor_tick_1ms(void);

/* Raportowanie zdarzeń (stall) — wołane z głównej pętli. */
void motor_poll_events(void);

#endif /* MOTOR_H */
