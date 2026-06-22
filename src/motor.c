#include "motor.h"
#include "board.h"
#include "pwm.h"
#include "hall.h"
#include "timebase.h"
#include "eeprom.h"
#include "sinus.h"
#include "adc.h"

/* ------------------------------------------------------------------ */
/* Tablice komutacji 6-step (indeks = stan Halla 0..7).               */
/* Sekwencja zgodna z działającym v2 (block.c):                        */
/*   Hall: 1→3→2→6→4→5                                                */
/*   Krok: A+B- → A+C- → B+C- → B+A- → C+A- → C+B-                    */
/* ------------------------------------------------------------------ */
/*  hall:  0    1    2    3    4    5    6    7                        */
static const int8_t k_hi_fwd[8] = { -1, PH_A, PH_B, PH_A, PH_C, PH_C, PH_B, -1 };
static const int8_t k_lo_fwd[8] = { -1, PH_B, PH_C, PH_C, PH_A, PH_B, PH_A, -1 };

/* Wektory uczenia w kolejności v2 (zgodnej z kierunkiem jazdy). */
/*               krok: 0     1     2     3     4     5                  */
static const int8_t k_vec_hi[6] = { PH_A, PH_A, PH_B, PH_B, PH_C, PH_C };
static const int8_t k_vec_lo[6] = { PH_B, PH_C, PH_C, PH_A, PH_A, PH_B };

/* Aktywne tablice komutacji: [kierunek][stan Halla].                 */
/* Wypełniane domyślnie lub przez uczenie. -1 = stan niepoprawny.     */
static int8_t s_hi[2][8];
static int8_t s_lo[2][8];
static bool   s_learned = false;


/* ------------------------------------------------------------------ */
/* Stan                                                              */
/* ------------------------------------------------------------------ */
static volatile motor_mode_t  s_mode  = MODE_BLOCK;
static volatile motor_state_t s_state = MSTATE_IDLE;
static volatile motor_dir_t   s_dir   = DIR_FWD;
static volatile ctrl_mode_t   s_ctrl  = CTRL_DUTY;

static volatile uint16_t s_duty       = 0;       /* aktualne wypełnienie 0..PWM_DUTY_MAX */
static volatile float    s_target_rpm = 0.0f;
static volatile float    s_rpm        = 0.0f;
static volatile uint8_t  s_hall       = 0;

/* Pomiar prędkości */
static volatile uint32_t s_last_hall_us = 0;

/* Regulator PI (tryb CTRL_SPEED) */
static float s_pi_integ = 0.0f;
static const float PI_KP = 0.05f;   /* [duty/rpm]  — dostrój */
static const float PI_KI = 0.10f;   /* [duty/(rpm*s)] — dostrój */

/* ------------------------------------------------------------------ */
static inline uint16_t duty_pct_to_raw(float pct)
{
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint16_t)((pct * (float)PWM_DUTY_MAX) / 100.0f);
}

/* Załaduj domyślną tablicę komutacji (sekwencja Graya). */
static void load_default_tables(void)
{
    for (uint8_t h = 0; h < 8; h++) {
        s_hi[DIR_FWD][h] = k_hi_fwd[h];
        s_lo[DIR_FWD][h] = k_lo_fwd[h];
        /* odwrócenie kierunku w domyślnej tablicy = zamiana hi/lo */
        s_hi[DIR_REV][h] = (k_hi_fwd[h] < 0) ? -1 : k_lo_fwd[h];
        s_lo[DIR_REV][h] = (k_lo_fwd[h] < 0) ? -1 : k_hi_fwd[h];
    }
    s_learned = false;
}

static void hw_commutate(uint8_t hall)
{
    int8_t hi = s_hi[s_dir][hall & 0x07];
    int8_t lo = s_lo[s_dir][hall & 0x07];

    if (hi < 0 || lo < 0) {
        /* Niepoprawny stan Halla (0 lub 7) -> błąd. */
        pwm_coast();
        s_state = MSTATE_FAULT;
        return;
    }
    /* Preload CCER+CCRx, potem atomowe przełączenie przez COM. */
    pwm_preload_step(hi, lo, s_duty);
    pwm_commutate();
}


/* ------------------------------------------------------------------ */
void motor_init(void)
{
    s_mode  = MODE_BLOCK;
    s_state = MSTATE_IDLE;
    s_dir   = DIR_FWD;
    s_ctrl  = CTRL_DUTY;
    s_duty  = 0;
    s_target_rpm = 0.0f;
    s_rpm = 0.0f;
    load_default_tables();

    /* Spróbuj wczytać zapisaną konfigurację Halla z EEPROM.
     * Jeśli jest poprawna — nadpisuje domyślną tablicę. */
    motor_config_load();
    sinus_init();
    pwm_coast();

    /* Kalibracja offsetu prądu (mostek wyłączony → zero prądu). */
    adc_calibrate_current_offset();
}

void motor_start(void)
{
    if (s_mode == MODE_SINUS) {
        s_pi_integ = 0.0f;
        s_state = MSTATE_RUN;
        s_last_hall_us = micros();
        s_hall = hall_read();
        sinus_set_amplitude(((float)s_duty * 100.0f) / (float)PWM_DUTY_MAX);
        sinus_start();
        return;
    }
    if (s_mode != MODE_BLOCK) {
        return;
    }
    s_pi_integ = 0.0f;
    s_state = MSTATE_RUN;
    s_last_hall_us = micros();
    s_hall = hall_read();
    hw_commutate(s_hall);
}

void motor_stop(void)
{
    s_state = MSTATE_IDLE;
    s_duty  = 0;
    sinus_stop();
    pwm_coast();
}

void motor_brake(void)
{
    s_state = MSTATE_BRAKE;
    s_duty  = 0;
    pwm_brake();
}

void motor_force_vector(uint8_t idx)
{
    if (idx > 5U) {
        return;
    }
    s_state = MSTATE_MANUAL;
    pwm_apply_step_direct(k_vec_hi[idx], k_vec_lo[idx], s_duty);
}

/*
 * Uczenie sekwencji Halla — open-loop, z aktywnym tłumieniem drgań.
 *
 * Procedura dla każdego wektora:
 *   1) przyłóż wektor z pełnym duty (przeskok do nowej pozycji)
 *   2) czekaj LEARN_SETTLE_MS — pozycja zgrubna
 *   3) hamulec zwarciowy 15 ms — tłumi oscylacje mechaniczne
 *   4) coast 5 ms
 *   5) przyłóż wektor z duty trzymającym (~50% uczącego) — delikatne
 *      przyciągnięcie, nie wywołuje nowych oscylacji
 *   6) pętla stabilności: odczytuj Halla co 10 ms, max 20 prób.
 *      Jeśli 3 kolejne odczyty identyczne → stabilny, wyjdź.
 *      Jeśli po 20 próbach brak stabilności → weź ostatni odczyt.
 *
 * duty_pct: wypełnienie uczenia (0 = domyślne LEARN_DUTY_PCT).
 * seq_out (opcjonalne, 6 elem.) -> h0..h5.
 * Zwraca true gdy sekwencja poprawna.
 */
bool motor_learn_hall_with_duty(float duty_pct, uint8_t *seq_out)
{
    if (duty_pct <= 0.0f || duty_pct > 100.0f) {
        duty_pct = LEARN_DUTY_PCT;
    }

    uint8_t h_of_v[6] = {0};
    uint16_t learn_duty = duty_pct_to_raw(duty_pct);
    /* Duty trzymające = ~50% uczącego — trzyma pozycję, nie wywołuje oscylacji */
    uint16_t hold_duty = learn_duty / 2U;
    if (hold_duty < 40U) hold_duty = 40U;  /* absolutne minimum żeby cokolwiek poczuć */
    const uint32_t settle = LEARN_SETTLE_MS;

    /* Zatrzymaj wszystko, ustaw tryb manualny. */
    pwm_coast();
    s_ctrl  = CTRL_DUTY;
    s_state = MSTATE_MANUAL;
    s_duty  = learn_duty;

    /* --- Pas kondycjonujący (2× bez odczytu, rozrywa tarcie) --- */
    for (uint8_t pass = 0; pass < 2; pass++) {
        for (uint8_t v = 0; v < 6; v++) {
            pwm_apply_step_direct(k_vec_hi[v], k_vec_lo[v], learn_duty);
            delay_ms(settle);
        }
    }

    /* --- Pas pomiarowy (2× — drugi odczyt nadpisuje, ostatni wygrywa) --- */
    for (uint8_t lap = 0; lap < 2; lap++) {
        for (uint8_t v = 0; v < 6; v++) {
            /* 1) przeskok do nowej pozycji — pełne duty */
            pwm_apply_step_direct(k_vec_hi[v], k_vec_lo[v], learn_duty);
            delay_ms(settle);

            /* 2) hamulec zwarciowy — tłumi oscylacje wirnika */
            pwm_brake();
            delay_ms(15);

            /* 3) coast — wirnik swobodnie dociąga do równowagi */
            pwm_coast();
            delay_ms(5);

            /* 4) delikatne przytrzymanie — duty trzymające, bez wzbudzania oscylacji */
            pwm_apply_step_direct(k_vec_hi[v], k_vec_lo[v], hold_duty);

            /* 5) pętla stabilności: czekaj aż Hall przestanie się zmieniać */
            uint8_t last = hall_read();
            uint8_t same_cnt = 1;
            for (int retry = 0; retry < 20; retry++) {
                delay_ms(10);
                uint8_t cur = hall_read();
                if (cur == last) {
                    same_cnt++;
                    if (same_cnt >= 3) {
                        /* 3 kolejne identyczne odczyty → stabilny */
                        break;
                    }
                } else {
                    same_cnt = 1;
                    last = cur;
                }
            }

            h_of_v[v] = last;
            if (seq_out) {
                seq_out[v] = last;
            }
        }
    }

    /* Sprzątanie. */
    pwm_coast();
    s_duty  = 0;
    s_state = MSTATE_IDLE;

    /* Walidacja: każdy stan w zakresie 1..6 i wszystkie różne. */
    for (uint8_t v = 0; v < 6; v++) {
        if (h_of_v[v] < 1 || h_of_v[v] > 6) {
            return false;
        }
        for (uint8_t w = v + 1; w < 6; w++) {
            if (h_of_v[v] == h_of_v[w]) {
                return false;
            }
        }
    }

    /* Budowa tablic komutacji z nauczonej sekwencji. */
    for (uint8_t v = 0; v < 6; v++) {
        uint8_t h    = h_of_v[v];
        uint8_t vfwd = (uint8_t)((v + 1U) % 6U);   /* +60° -> jazda w przód */
        uint8_t vrev = (uint8_t)((v + 5U) % 6U);   /* -60° -> jazda w tył   */

        s_hi[DIR_FWD][h] = k_vec_hi[vfwd];
        s_lo[DIR_FWD][h] = k_vec_lo[vfwd];
        s_hi[DIR_REV][h] = k_vec_hi[vrev];
        s_lo[DIR_REV][h] = k_vec_lo[vrev];
    }
    /* stany niepoprawne */
    s_hi[DIR_FWD][0] = s_lo[DIR_FWD][0] = -1;
    s_hi[DIR_FWD][7] = s_lo[DIR_FWD][7] = -1;
    s_hi[DIR_REV][0] = s_lo[DIR_REV][0] = -1;
    s_hi[DIR_REV][7] = s_lo[DIR_REV][7] = -1;

    s_learned = true;

    /* Automatycznie zapisz do EEPROM po udanym uczeniu. */
    motor_config_save();

    return true;
}

/* Wersja z domyślnym duty (dla starych wywołań). */
bool motor_learn_hall(uint8_t *seq_out)
{
    return motor_learn_hall_with_duty(LEARN_DUTY_PCT, seq_out);
}

bool motor_is_learned(void)
{
    return s_learned;
}

/* ------------------------------------------------------------------ */
/* EEPROM — zapis/odczyt konfiguracji Halla                          */
/* ------------------------------------------------------------------ */

bool motor_config_save(void)
{
    eeprom_config_t cfg;
    cfg.magic      = 0x4C48U;  /* 'H' 'L' */
    cfg.version    = 1U;
    cfg.pole_pairs = MOTOR_POLE_PAIRS;

    /* Kopiujemy stany hall=1..6 (indeksy 0 i 7 zawsze niepoprawne). */
    for (uint8_t h = 1; h <= 6; h++) {
        cfg.hi_fwd[h - 1] = s_hi[DIR_FWD][h];
        cfg.lo_fwd[h - 1] = s_lo[DIR_FWD][h];
    }
    cfg.crc16 = 0;

    return eeprom_save(&cfg);
}

bool motor_config_load(void)
{
    eeprom_config_t cfg;
    if (!eeprom_load(&cfg)) {
        return false;
    }

    /* Odtwórz tablice komutacji z zapisanej konfiguracji. */
    for (uint8_t h = 1; h <= 6; h++) {
        s_hi[DIR_FWD][h] = cfg.hi_fwd[h - 1];
        s_lo[DIR_FWD][h] = cfg.lo_fwd[h - 1];
        /* Kierunek wsteczny = zamiana hi/lo */
        if (cfg.hi_fwd[h - 1] < 0) {
            s_hi[DIR_REV][h] = -1;
            s_lo[DIR_REV][h] = -1;
        } else {
            s_hi[DIR_REV][h] = cfg.lo_fwd[h - 1];
            s_lo[DIR_REV][h] = cfg.hi_fwd[h - 1];
        }
    }
    s_hi[DIR_FWD][0] = s_lo[DIR_FWD][0] = -1;
    s_hi[DIR_FWD][7] = s_lo[DIR_FWD][7] = -1;
    s_hi[DIR_REV][0] = s_lo[DIR_REV][0] = -1;
    s_hi[DIR_REV][7] = s_lo[DIR_REV][7] = -1;

    s_learned = true;
    return true;
}

bool motor_config_erase(void)
{
    eeprom_erase();
    /* Przywróć domyślną tablicę. */
    load_default_tables();
    s_learned = false;
    return true;
}

uint8_t motor_get_hall_seq(uint8_t idx)
{
    if (idx > 5U) {
        return 0;
    }
    /* Znajdź stan Halla dla danego wektora (idx = pozycja w sekwencji). */
    /* Wyszukujemy w tablicy s_hi[DIR_FWD] który stan Halla odpowiada */
    /* wektorowi o indeksie idx (gdzie k_vec_hi/lo[idx] to para hi/lo). */
    for (uint8_t h = 1; h <= 6; h++) {
        if (s_hi[DIR_FWD][h] == k_vec_hi[idx] &&
            s_lo[DIR_FWD][h] == k_vec_lo[idx]) {
            return h;
        }
    }
    return 0;
}

void motor_set_mode(motor_mode_t mode)
{
    motor_stop();
    s_mode = mode;
}

void motor_set_dir(motor_dir_t dir)
{
    s_dir = dir;
    if (s_state == MSTATE_RUN) {
        if (s_mode == MODE_SINUS) {
            /* SINUS: zmiana kierunku = odwrócenie kierunku narastania kąta
             * (realizowane wewnętrznie przez sinus_hall_edge i estymator). */
        } else {
            hw_commutate(hall_read());
        }
    }
}

void motor_set_duty_pct(float pct)
{
    s_ctrl = CTRL_DUTY;
    s_duty = duty_pct_to_raw(pct);
    if (s_state == MSTATE_RUN) {
        if (s_mode == MODE_SINUS) {
            sinus_set_amplitude(pct);
            /* R ma być podtrzymane aż do S:
             * jeśli SINUS zatrzymał się po timeout Halla (np. po D0),
             * dodatnie duty ma go ponownie uzbroić bez ponownego R. */
            if (pct > 0.0f && !sinus_is_running()) {
                sinus_start();
            }
        } else {
            hw_commutate(hall_read());
        }
    }
}

void motor_set_target_rpm(float rpm)
{
    s_ctrl = CTRL_SPEED;
    if (rpm < 0.0f) rpm = 0.0f;
    s_target_rpm = rpm;
}

motor_mode_t  motor_get_mode(void)  { return s_mode; }
motor_state_t motor_get_state(void) { return s_state; }
motor_dir_t   motor_get_dir(void)   { return s_dir; }
ctrl_mode_t   motor_get_ctrl(void)  { return s_ctrl; }
float         motor_get_rpm(void)
{
    if (s_mode == MODE_SINUS) {
        return sinus_get_rpm();
    }
    return s_rpm;
}
uint8_t       motor_get_hall(void)  { return s_hall; }

float motor_get_duty_pct(void)
{
    return ((float)s_duty * 100.0f) / (float)PWM_DUTY_MAX;
}

/* ------------------------------------------------------------------ */
/* Wywoływane z ISR Halla.                                            */
/* ------------------------------------------------------------------ */
void motor_on_hall(void)
{
    uint8_t h = hall_read();
    s_hall = h;

    if (s_mode == MODE_SINUS) {
        sinus_hall_edge(h);
        return;
    }

    /* Pomiar prędkości: czas między zboczami (1 zbocze = 60° elektr.). */
    uint32_t now = micros();
    uint32_t dt  = now - s_last_hall_us;
    s_last_hall_us = now;

    if (dt > 0U) {
        float erev_per_s = 1000000.0f / (6.0f * (float)dt);
        float rpm = erev_per_s * 60.0f / (float)MOTOR_POLE_PAIRS;
        s_rpm = s_rpm * 0.7f + rpm * 0.3f;
    }

    if (s_state == MSTATE_RUN) {
        hw_commutate(h);
    }
}

/* ------------------------------------------------------------------ */
/* Pętla regulacji 1 kHz.                                             */
/* ------------------------------------------------------------------ */
void motor_tick_1ms(void)
{
    /* W SINUSIE główna pętla sinusa wykonuje estymację + PWM. */
    if (s_mode == MODE_SINUS) {
        sinus_update();
        return;
    }

    /* Wykrycie zatrzymania -> rpm = 0. */
    if ((micros() - s_last_hall_us) > HALL_STALL_US) {
        s_rpm = 0.0f;
    }

    if (s_state != MSTATE_RUN) {
        return;
    }

    if (s_ctrl == CTRL_SPEED) {
        float cur_rpm = motor_get_rpm();
        const float dt_s = 0.001f;
        float err = s_target_rpm - cur_rpm;

        s_pi_integ += err * dt_s;
        /* ograniczenie całki (anti-windup) */
        float integ_max = (float)PWM_DUTY_MAX / PI_KI;
        if (s_pi_integ > integ_max)  s_pi_integ = integ_max;
        if (s_pi_integ < 0.0f)       s_pi_integ = 0.0f;

        float out = PI_KP * err + PI_KI * s_pi_integ;
        if (out < 0.0f)                    out = 0.0f;
        if (out > (float)PWM_DUTY_MAX)     out = (float)PWM_DUTY_MAX;

        s_duty = (uint16_t)out;

        if (s_mode == MODE_SINUS) {
            sinus_set_amplitude((float)out * 100.0f / (float)PWM_DUTY_MAX);
        } else {
            hw_commutate(s_hall);
        }
    }
}
