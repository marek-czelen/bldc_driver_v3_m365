#include "sinus.h"
#include "board.h"
#include "timebase.h"
#include "hall.h"

/*
 * Sterowanie sinusoidalne — odwzorowanie działającego v2_stm.
 *
 *   - Aktualizacja co 1 kHz (sinus_update, wołana z motor_tick_1ms).
 *   - Kąt estymowany prosto: przy zmianie Halla → snap; między zmianami
 *     → interpolacja liniowa z ostatnio zmierzonego okresu 60°.
 *   - Advance 60°: kąt wysterowania = kąt wirnika + 60°.
 *   - Midpoint injection (SVPWM) na surowych wartościach sinusa.
 *   - Tabela sin3_table: sin + 1/6·sin(3θ), znormalizowana do 100..1900.
 */

/* ── Tabela sinusa + 3. harmonicznej ─────────────────────────── */
static const uint16_t sin3_table[256] = {
    1000, 1038, 1076, 1114, 1152, 1190, 1227, 1263, 1299, 1334, 1369, 1403, 1436, 1468, 1499, 1529,
    1558, 1586, 1612, 1638, 1662, 1685, 1707, 1728, 1747, 1765, 1782, 1798, 1812, 1825, 1837, 1848,
    1857, 1866, 1873, 1880, 1885, 1889, 1893, 1896, 1898, 1899, 1900, 1900, 1900, 1899, 1897, 1896,
    1894, 1892, 1889, 1887, 1885, 1882, 1880, 1877, 1875, 1873, 1871, 1870, 1868, 1867, 1867, 1866,
    1866, 1866, 1867, 1867, 1868, 1870, 1871, 1873, 1875, 1877, 1880, 1882, 1885, 1887, 1889, 1892,
    1894, 1896, 1897, 1899, 1900, 1900, 1900, 1899, 1898, 1896, 1893, 1889, 1885, 1880, 1873, 1866,
    1857, 1848, 1837, 1825, 1812, 1798, 1782, 1765, 1747, 1728, 1707, 1685, 1662, 1638, 1612, 1586,
    1558, 1529, 1499, 1468, 1436, 1403, 1369, 1334, 1299, 1263, 1227, 1190, 1152, 1114, 1076, 1038,
    1000,  962,  924,  886,  848,  810,  773,  737,  701,  666,  631,  597,  564,  532,  501,  471,
     442,  414,  388,  362,  338,  315,  293,  272,  253,  235,  218,  202,  188,  175,  163,  152,
     143,  134,  127,  120,  115,  111,  107,  104,  102,  101,  100,  100,  100,  101,  103,  104,
     106,  108,  111,  113,  115,  118,  120,  123,  125,  127,  129,  130,  132,  133,  133,  134,
     134,  134,  133,  133,  132,  130,  129,  127,  125,  123,  120,  118,  115,  113,  111,  108,
     106,  104,  103,  101,  100,  100,  100,  101,  102,  104,  107,  111,  115,  120,  127,  134,
     143,  152,  163,  175,  188,  202,  218,  235,  253,  272,  293,  315,  338,  362,  388,  414,
     442,  471,  501,  532,  564,  597,  631,  666,  701,  737,  773,  810,  848,  886,  924,  962,
};

/* 65536 * 60°/360° */
#define ADV_OFFSET  10923U

/* ── Stan ──────────────────────────────────────────────────── */
static bool     s_running       = false;
static uint16_t s_angle;          /* kąt wirnika 0..65535 (0..360°) */

static float    s_modulation;     /* 0..1 */
static float    s_rpm;

static uint8_t  s_last_hall;
static uint32_t s_last_hall_ms;
static uint32_t s_hall_period_ms;
static bool     s_speed_valid;

/* Mapowanie Hall → kąt wirnika (sekwencja v2: 1→3→2→6→4→5). */
static const uint16_t hall_angle[8] = {
    [0] = 0,
    [1] = 0,        /*   0° */
    [2] = 21845,    /* 120° */
    [3] = 10923,    /*  60° */
    [4] = 43691,    /* 240° */
    [5] = 54613,    /* 300° */
    [6] = 32768,    /* 180° */
    [7] = 0,
};

/* ══════════════════════════════════════════════════════════════ */
void sinus_init(void)
{
    s_running        = false;
    s_angle          = 0;
    s_modulation     = 0.0f;
    s_rpm            = 0.0f;
    s_last_hall      = 0xFF;
    s_last_hall_ms   = 0;
    s_hall_period_ms = 0;
    s_speed_valid    = false;
}

void sinus_start(void)
{
    uint8_t hall = hall_read();

    /* Kąt startowy: środek sektora (+30° = 65536/12). */
    if (hall >= 1 && hall <= 6) {
        s_angle = hall_angle[hall] + 5461U;
    } else {
        s_angle = 0;
    }

    s_last_hall      = hall;
    s_last_hall_ms   = millis();
    s_hall_period_ms = 0;
    s_speed_valid    = false;
    s_running        = true;

    /* Neutralne 50%, włącz wszystkie kanały (komplementarne). */
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = (PWM_ARR / 2U);
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE;
    TIM1->EGR  = TIM_EGR_COMG;
}

void sinus_stop(void)
{
    s_running = false;
    TIM1->CCER = 0;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    TIM1->EGR  = TIM_EGR_COMG;
    s_modulation  = 0.0f;
    s_speed_valid = false;
}

void sinus_set_amplitude(float pct)
{
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    s_modulation = pct / 100.0f;
}

float sinus_get_amplitude(void)
{
    return s_modulation * 100.0f;
}

/* ── Zdarzenie Halla ────────────────────────────────────────── */
void sinus_hall_edge(uint8_t hall_state)
{
    if (hall_state < 1 || hall_state > 6) return;
    if (hall_state == s_last_hall) return;

    uint32_t now = millis();
    uint32_t dt  = now - s_last_hall_ms;

    if (dt >= 2 && dt < HALL_TIMEOUT_MS) {
        s_hall_period_ms = dt;
        s_speed_valid    = true;
        s_angle = hall_angle[hall_state];
    }
    s_last_hall    = hall_state;
    s_last_hall_ms = now;

    /* RPM */
    if (s_speed_valid && s_hall_period_ms > 0) {
        float erev_per_s = 1000.0f / (6.0f * (float)s_hall_period_ms);
        float rpm = erev_per_s * 60.0f / (float)MOTOR_POLE_PAIRS;
        s_rpm = s_rpm * 0.7f + rpm * 0.3f;
    }
}

/* ── Główna aktualizacja (wołana z motor_tick_1ms, 1 kHz) ───── */
void sinus_update(void)
{
    if (!s_running) return;

    uint32_t now = millis();
    uint8_t  hall = hall_read();

    /* Detekcja zmiany Halla (fallback, gdyby EXTI nie zadziałało). */
    if (hall >= 1 && hall <= 6 && hall != s_last_hall) {
        uint32_t dt = now - s_last_hall_ms;
        if (dt >= 2 && dt < HALL_TIMEOUT_MS) {
            s_hall_period_ms = dt;
            s_speed_valid    = true;
            s_angle = hall_angle[hall];
        }
        s_last_hall    = hall;
        s_last_hall_ms = now;
    }

    /* Interpolacja kąta między zboczami Halla. */
    if (s_speed_valid && s_hall_period_ms > 0) {
        uint32_t elapsed = now - s_last_hall_ms;
        uint32_t inc = (10923UL * elapsed) / s_hall_period_ms;
        if (inc > 10923UL) inc = 10923UL;
        s_angle = (uint16_t)((uint32_t)hall_angle[hall] + inc);
    }

    /* Timeout Halla. */
    if ((now - s_last_hall_ms) > HALL_TIMEOUT_MS) {
        sinus_stop();
        return;
    }

    /* ══════════════════════════════════════════════════════════
     * Generacja sinusa z advance 60° (jak w działającym v2).
     *
     * W v2:  A=sin(θ+60°), B=sin(θ-60°), C=sin(θ-180°)
     * W tablicy (0..255): dodajemy odpowiednie offsety.
     * ══════════════════════════════════════════════════════════ */
    uint16_t adv = s_angle + ADV_OFFSET;

    uint8_t idx_a = (uint8_t)(adv >> 8);
    uint8_t idx_b = (uint8_t)((adv + 43691U) >> 8);   /* -120° ≡ +240° */
    uint8_t idx_c = (uint8_t)((adv + 21845U) >> 8);   /* -240° ≡ +120° */

    /* Surowe wartości z tabeli (100..1900, środek 1000). */
    int16_t sa = (int16_t)(sin3_table[idx_a]) - 1000;
    int16_t sb = (int16_t)(sin3_table[idx_b]) - 1000;
    int16_t sc = (int16_t)(sin3_table[idx_c]) - 1000;

    /* ── Midpoint injection (SVPWM) ── */
    int16_t vmin = sa, vmax = sa;
    if (sb < vmin) vmin = sb;  if (sb > vmax) vmax = sb;
    if (sc < vmin) vmin = sc;  if (sc > vmax) vmax = sc;

    int16_t off = (int16_t)(((int32_t)vmin + (int32_t)vmax) / 2);
    sa = (int16_t)(sa - off);
    sb = (int16_t)(sb - off);
    sc = (int16_t)(sc - off);

    /* Skalowanie 2/√3 ≈ 1155/1000 — pełne wykorzystanie DC. */
    sa = (int16_t)(((int32_t)sa * 1155) / 1000);
    sb = (int16_t)(((int32_t)sb * 1155) / 1000);
    sc = (int16_t)(((int32_t)sc * 1155) / 1000);

    /* Mapowanie do CCR:
     * CCR = PWM_ARR/2 + (sa/500) × swing × modulation
     * swing = PWM_ARR/2 - 100 (margines 5%) */
    int32_t mid   = (int32_t)(PWM_ARR / 2U);
    int32_t swing = mid - 100;
    int32_t mod_i = (int32_t)(s_modulation * 1000.0f);   /* 0..1000 */

    int32_t ccr_a = mid + ((swing * mod_i / 1000) * sa / 500);
    int32_t ccr_b = mid + ((swing * mod_i / 1000) * sb / 500);
    int32_t ccr_c = mid + ((swing * mod_i / 1000) * sc / 500);

    /* Clamp. */
    if (ccr_a < 100) ccr_a = 100;  if (ccr_a > 1900) ccr_a = 1900;
    if (ccr_b < 100) ccr_b = 100;  if (ccr_b > 1900) ccr_b = 1900;
    if (ccr_c < 100) ccr_c = 100;  if (ccr_c > 1900) ccr_c = 1900;

    TIM1->CCR1 = (uint16_t)ccr_a;
    TIM1->CCR2 = (uint16_t)ccr_b;
    TIM1->CCR3 = (uint16_t)ccr_c;
}

/* ── Odczyty ────────────────────────────────────────────────── */
float sinus_get_rpm(void)
{
    return s_rpm;
}

uint16_t sinus_get_angle(void)
{
    return s_angle;
}
