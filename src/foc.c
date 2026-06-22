#include "foc.h"
#include "board.h"
#include "timebase.h"
#include "hall.h"
#include "adc.h"
#include <stdlib.h>

/*
 * FOC — Field Oriented Control z hall sensorami.
 *
 * Architektura:
 *   1. Estymacja kąta elektrycznego z Halla (interpolacja liniowa)
 *   2. Clarke: Ia,Ib,Ic → Iα,Iβ
 *   3. Park:   Iα,Iβ,θ → Id,Iq
 *   4. PI Id (target=0), PI Iq (target=zadany)
 *   5. Inverse Park: Vd,Vq,θ → Vα,Vβ
 *   6. SVPWM: Vα,Vβ → CCR1/2/3
 *
 * Parametry PI są w jednostkach fixed-point:
 *   Err [mA], Out [mV] → przeliczane na duty przez Vbat
 */

/* ── Tabela sin/cos 256 elementów (0..360°) ─────────────────── */
static const int16_t sin_table[256] = {
       0,   804,  1608,  2411,  3212,  4011,  4808,  5602,
    6393,  7180,  7962,  8740,  9512, 10279, 11039, 11793,
   12540, 13279, 14010, 14733, 15447, 16151, 16846, 17531,
   18205, 18868, 19520, 20160, 20788, 21403, 22006, 22595,
   23170, 23732, 24279, 24812, 25330, 25833, 26320, 26791,
   27246, 27684, 28106, 28511, 28899, 29269, 29622, 29957,
   30274, 30572, 30853, 31114, 31357, 31581, 31786, 31971,
   32138, 32285, 32413, 32521, 32610, 32679, 32728, 32758,
   32767, 32758, 32728, 32679, 32610, 32521, 32413, 32285,
   32138, 31971, 31786, 31581, 31357, 31114, 30853, 30572,
   30274, 29957, 29622, 29269, 28899, 28511, 28106, 27684,
   27246, 26791, 26320, 25833, 25330, 24812, 24279, 23732,
   23170, 22595, 22006, 21403, 20788, 20160, 19520, 18868,
   18205, 17531, 16846, 16151, 15447, 14733, 14010, 13279,
   12540, 11793, 11039, 10279,  9512,  8740,  7962,  7180,
    6393,  5602,  4808,  4011,  3212,  2411,  1608,   804,
       0,  -804, -1608, -2411, -3212, -4011, -4808, -5602,
   -6393, -7180, -7962, -8740, -9512,-10279,-11039,-11793,
  -12540,-13279,-14010,-14733,-15447,-16151,-16846,-17531,
  -18205,-18868,-19520,-20160,-20788,-21403,-22006,-22595,
  -23170,-23732,-24279,-24812,-25330,-25833,-26320,-26791,
  -27246,-27684,-28106,-28511,-28899,-29269,-29622,-29957,
  -30274,-30572,-30853,-31114,-31357,-31581,-31786,-31971,
  -32138,-32285,-32413,-32521,-32610,-32679,-32728,-32758,
  -32767,-32758,-32728,-32679,-32610,-32521,-32413,-32285,
  -32138,-31971,-31786,-31581,-31357,-31114,-30853,-30572,
  -30274,-29957,-29622,-29269,-28899,-28511,-28106,-27684,
  -27246,-26791,-26320,-25833,-25330,-24812,-24279,-23732,
  -23170,-22595,-22006,-21403,-20788,-20160,-19520,-18868,
  -18205,-17531,-16846,-16151,-15447,-14733,-14010,-13279,
  -12540,-11793,-11039,-10279, -9512, -8740, -7962, -7180,
   -6393, -5602, -4808, -4011, -3212, -2411, -1608,  -804,
};

/* cos(θ) = sin(θ + 90°) → offset +64 w indeksie */
static inline int16_t foc_sin(uint8_t idx) { return sin_table[idx]; }
static inline int16_t foc_cos(uint8_t idx) { return sin_table[(uint8_t)(idx + 64U)]; }

/* ── Mapowanie Hall → kąt (0..255 = 0..360°) ────────────────── */
static const uint8_t hall_angle_idx[8] = {
    [0]=0, [1]=0,   [2]=85,  [3]=43,   /* 0°, 120°, 60° */
    [4]=171, [5]=213, [6]=128, [7]=0    /* 240°, 300°, 180° */
};

/* ── Stan FOC ────────────────────────────────────────────────── */
static bool     s_running       = false;
static uint8_t  s_angle_idx;      /* kąt elektryczny 0..255 */
static int16_t  s_iq_target = 0;  /* zadany Iq [mA] */

/* Estymacja kąta (jak w sinus.c) */
static uint8_t  s_last_hall;
static uint32_t s_last_hall_ms;
static uint32_t s_hall_period_ms;
static bool     s_speed_valid;
static float    s_rpm;

/* PI — integratory */
static float s_id_integ = 0.0f;
static float s_iq_integ = 0.0f;

/* Ostatnie zmierzone Id/Iq [mA] (filtrowane) */
static int16_t s_id_meas = 0;
static int16_t s_iq_meas = 0;
static float s_id_filt = 0.0f;
static float s_iq_filt = 0.0f;

/* ── Parametry PI (runtime, zmieniane przez CLI) ─────────────── */
static float s_kp_iq = 0.5f;    /* Vq [mV] na mA błędu Iq */
static float s_ki_iq = 1.5f;    /* Vq [mV/ms] na mA błędu Iq */
static float s_kp_id = 0.3f;    /* Vd [mV] na mA błędu Id */
static float s_ki_id = 1.0f;    /* Vd [mV/ms] na mA błędu Id */
static float s_curr_filt = 0.15f; /* 0..1, filtr Id/Iq */
static int32_t s_vbat_mv = 36000; /* napięcie baterii [mV] */

#define FOC_ID_MAX  5000    /* max Id [mA] */
#define FOC_IQ_MAX  15000   /* max Iq [mA] */
#define FOC_VD_MAX  16000   /* max |Vd| [mV] */
#define FOC_VQ_MAX  28000   /* max |Vq| [mV] */

/* ══════════════════════════════════════════════════════════════ */
void foc_init(void)
{
    s_running        = false;
    s_angle_idx      = 0;
    s_iq_target      = 0;
    s_last_hall      = 0xFF;
    s_last_hall_ms   = 0;
    s_hall_period_ms = 0;
    s_speed_valid    = false;
    s_rpm            = 0.0f;
    s_id_integ       = 0.0f;
    s_iq_integ       = 0.0f;
    s_id_meas        = 0;
    s_iq_meas        = 0;
    s_id_filt        = 0.0f;
    s_iq_filt        = 0.0f;
}

void foc_start(void)
{
    uint8_t hall = hall_read();

    if (hall >= 1 && hall <= 6) {
        s_angle_idx = (uint8_t)(hall_angle_idx[hall] + 11U); /* +30° środek sektora */
    } else {
        s_angle_idx = 0;
    }

    s_last_hall      = hall;
    s_last_hall_ms   = millis();
    s_hall_period_ms = 0;
    s_speed_valid    = false;
    s_id_integ       = 0.0f;
    s_iq_integ       = 0.0f;
    s_id_filt        = 0.0f;
    s_iq_filt        = 0.0f;
    s_running        = true;

    /* Włącz PWM: neutralne 50%, wszystkie kanały. */
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = (PWM_ARR / 2U);
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE;
    TIM1->EGR  = TIM_EGR_COMG;
}

void foc_stop(void)
{
    s_running = false;
    TIM1->CCER = 0;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    TIM1->EGR  = TIM_EGR_COMG;
    s_speed_valid = false;
    s_rpm = 0.0f;
    s_id_integ = 0.0f;
    s_iq_integ = 0.0f;
}

void foc_set_iq_target(int16_t iq_ma)
{
    if (iq_ma < 0) iq_ma = 0;
    if (iq_ma > FOC_IQ_MAX) iq_ma = FOC_IQ_MAX;
    s_iq_target = iq_ma;
}

/* ══════════════════════════════════════════════════════════════
 * Clarke: Ia,Ib,Ic → Iα,Iβ
 *   Iα = Ia
 *   Iβ = (Ia + 2*Ib) / √3  ≈ (Ia + 2*Ib) * 18919 / 32768
 * ══════════════════════════════════════════════════════════════ */
static void clarke(int16_t ia, int16_t ib, int16_t ic,
                   int16_t *i_alpha, int16_t *i_beta)
{
    (void)ic; /* Ic = -(Ia+Ib), niepotrzebne do transformaty */
    *i_alpha = ia;
    /* Iβ = (Ia + 2*Ib) / √3  →  1/√3 ≈ 18919/32768 */
    int32_t tmp = (int32_t)ia + 2 * (int32_t)ib;
    *i_beta = (int16_t)((tmp * 18919) >> 15);
}

/* ══════════════════════════════════════════════════════════════
 * Park: Iα,Iβ,θ → Id,Iq
 *   Id =  Iα*cos(θ) + Iβ*sin(θ)
 *   Iq = -Iα*sin(θ) + Iβ*cos(θ)
 * ══════════════════════════════════════════════════════════════ */
static void park(int16_t i_alpha, int16_t i_beta, uint8_t angle_idx,
                 int16_t *id, int16_t *iq)
{
    int16_t s = foc_sin(angle_idx);
    int16_t c = foc_cos(angle_idx);
    /* Id = Iα*c + Iβ*s   (w skali 32767) */
    *id = (int16_t)(((int32_t)i_alpha * c + (int32_t)i_beta * s) >> 15);
    /* Iq = -Iα*s + Iβ*c */
    *iq = (int16_t)(((-(int32_t)i_alpha * s + (int32_t)i_beta * c)) >> 15);
}

/* ══════════════════════════════════════════════════════════════
 * Inverse Park: Vd,Vq,θ → Vα,Vβ
 *   Vα = Vd*cos(θ) - Vq*sin(θ)
 *   Vβ = Vd*sin(θ) + Vq*cos(θ)
 * ══════════════════════════════════════════════════════════════ */
static void ipark(int16_t vd, int16_t vq, uint8_t angle_idx,
                  int16_t *v_alpha, int16_t *v_beta)
{
    int16_t s = foc_sin(angle_idx);
    int16_t c = foc_cos(angle_idx);
    *v_alpha = (int16_t)(((int32_t)vd * c - (int32_t)vq * s) >> 15);
    *v_beta  = (int16_t)(((int32_t)vd * s + (int32_t)vq * c) >> 15);
}

/* ══════════════════════════════════════════════════════════════
 * SVPWM: Vα,Vβ → CCR1,CCR2,CCR3
 *   Va = Vα
 *   Vb = (-Vα + √3*Vβ)/2
 *   Vc = (-Vα - √3*Vβ)/2
 * Potem midpoint injection + skalowanie do PWM_ARR.
 * ══════════════════════════════════════════════════════════════ */
static void svpwm(int16_t v_alpha, int16_t v_beta,
                  uint16_t *ccr_a, uint16_t *ccr_b, uint16_t *ccr_c)
{
    /* Inverse Clarke */
    int32_t va = v_alpha;
    /* √3 ≈ 56756/32768 */
    int32_t vb = (-v_alpha + (v_beta * 56756 >> 15)) >> 1;
    int32_t vc = (-v_alpha - (v_beta * 56756 >> 15)) >> 1;

    /* Midpoint injection */
    int32_t vmin = va, vmax = va;
    if (vb < vmin) vmin = vb; if (vb > vmax) vmax = vb;
    if (vc < vmin) vmin = vc; if (vc > vmax) vmax = vc;
    int32_t off = (vmin + vmax) >> 1;
    va -= off; vb -= off; vc -= off;

    /* Skalowanie: Vα/Vβ są w mV, Vbat w mV.
     * duty = Vphase / Vbat  →  CCR = PWM_ARR/2 * (1 + Vphase/Vbat)
     * Skalujemy przez PWM_ARR/2 / Vbat_mV */
    int32_t scale = (int32_t)(PWM_ARR / 2U) * 32768 / s_vbat_mv;
    int32_t mid = (int32_t)(PWM_ARR / 2U);

    int32_t ca = mid + ((va * scale) >> 15);
    int32_t cb = mid + ((vb * scale) >> 15);
    int32_t cc = mid + ((vc * scale) >> 15);

    /* Clamp 5%..95% PWM_ARR */
    int32_t lo = (int32_t)(PWM_ARR / 20U);
    int32_t hi = (int32_t)(PWM_ARR - lo);
    if (ca < lo) ca = lo; if (ca > hi) ca = hi;
    if (cb < lo) cb = lo; if (cb > hi) cb = hi;
    if (cc < lo) cc = lo; if (cc > hi) cc = hi;

    *ccr_a = (uint16_t)ca;
    *ccr_b = (uint16_t)cb;
    *ccr_c = (uint16_t)cc;
}

/* ══════════════════════════════════════════════════════════════
 * Główna pętla FOC (1 kHz)
 * ══════════════════════════════════════════════════════════════ */
void foc_update(void)
{
    if (!s_running) return;

    uint32_t now = millis();
    uint8_t  hall = hall_read();

    /* ── Estymacja kąta z Halla ── */
    if (hall >= 1 && hall <= 6 && hall != s_last_hall) {
        uint32_t dt = now - s_last_hall_ms;
        if (dt >= 2 && dt < HALL_TIMEOUT_MS) {
            s_hall_period_ms = dt;
            s_speed_valid = true;
            s_angle_idx = hall_angle_idx[hall];
        }
        s_last_hall = hall;
        s_last_hall_ms = now;
    }

    /* Interpolacja kąta */
    if (s_speed_valid && s_hall_period_ms > 0) {
        uint32_t elapsed = now - s_last_hall_ms;
        uint32_t inc = (43UL * elapsed) / s_hall_period_ms; /* 60° → 43 indeksy */
        if (inc > 43UL) inc = 43UL;
        s_angle_idx = (uint8_t)((uint32_t)hall_angle_idx[hall] + inc);
    }

    /* RPM */
    if (s_speed_valid && s_hall_period_ms > 0) {
        float erev_per_s = 1000.0f / (6.0f * (float)s_hall_period_ms);
        float rpm = erev_per_s * 60.0f / (float)MOTOR_POLE_PAIRS;
        s_rpm = s_rpm * 0.7f + rpm * 0.3f;
    } else if (!s_speed_valid) {
        s_rpm = s_rpm * 0.99f;
        if (s_rpm < 1.0f) s_rpm = 0.0f;
    }

    /* Timeout Halla — restart estymatora, nie zatrzymuj */
    if ((now - s_last_hall_ms) > HALL_TIMEOUT_MS) {
        uint8_t hall_now = hall_read();
        if (hall_now >= 1 && hall_now <= 6) {
            s_angle_idx = (uint8_t)(hall_angle_idx[hall_now] + 11U);
        }
        s_last_hall = hall_now;
        s_last_hall_ms = now;
        s_hall_period_ms = 0;
        s_speed_valid = false;
    }

    /* ── Clarke: prądy fazowe → Iα,Iβ ── */
    int16_t ia = adc_read_current_ma(ADC_CH_IA);
    int16_t ib = adc_read_current_ma(ADC_CH_IB);
    int16_t ic = adc_read_current_ma(ADC_CH_IC);
    int16_t i_alpha, i_beta;
    clarke(ia, ib, ic, &i_alpha, &i_beta);

    /* ── Park: Iα,Iβ,θ → Id,Iq ── */
    int16_t id_meas, iq_meas;
    park(i_alpha, i_beta, s_angle_idx, &id_meas, &iq_meas);
    s_id_meas = id_meas;
    s_iq_meas = iq_meas;

    /* ── Filtruj Id/Iq (IIR pierwszego rzędu) ── */
    s_id_filt = s_id_filt * (1.0f - s_curr_filt) + (float)id_meas * s_curr_filt;
    s_iq_filt = s_iq_filt * (1.0f - s_curr_filt) + (float)iq_meas * s_curr_filt;
    int16_t id_f = (int16_t)s_id_filt;
    int16_t iq_f = (int16_t)s_iq_filt;
    s_id_meas = id_f;
    s_iq_meas = iq_f;

    /* ── PI Id (target = 0) ── */
    int16_t id_err = -id_f;
    s_id_integ += (float)id_err * s_ki_id * 0.001f;
    if (s_id_integ > (float)FOC_VD_MAX)  s_id_integ = (float)FOC_VD_MAX;
    if (s_id_integ < -(float)FOC_VD_MAX) s_id_integ = -(float)FOC_VD_MAX;
    int16_t vd = (int16_t)(s_kp_id * (float)id_err + s_id_integ);
    if (vd > FOC_VD_MAX)  vd = FOC_VD_MAX;
    if (vd < -FOC_VD_MAX) vd = -FOC_VD_MAX;

    /* ── PI Iq ── */
    int16_t iq_err = s_iq_target - iq_f;
    s_iq_integ += (float)iq_err * s_ki_iq * 0.001f;
    if (s_iq_integ > (float)FOC_VQ_MAX)  s_iq_integ = (float)FOC_VQ_MAX;
    if (s_iq_integ < -(float)FOC_VQ_MAX) s_iq_integ = -(float)FOC_VQ_MAX;
    int16_t vq = (int16_t)(s_kp_iq * (float)iq_err + s_iq_integ);
    if (vq > FOC_VQ_MAX)  vq = FOC_VQ_MAX;
    if (vq < -FOC_VQ_MAX) vq = -FOC_VQ_MAX;

    /* ── Inverse Park: Vd,Vq,θ → Vα,Vβ ── */
    int16_t v_alpha, v_beta;
    ipark(vd, vq, s_angle_idx, &v_alpha, &v_beta);

    /* ── SVPWM → CCR1/2/3 ── */
    uint16_t ccr_a, ccr_b, ccr_c;
    svpwm(v_alpha, v_beta, &ccr_a, &ccr_b, &ccr_c);

    TIM1->CCR1 = ccr_a;
    TIM1->CCR2 = ccr_b;
    TIM1->CCR3 = ccr_c;
}

/* ── Odczyty ────────────────────────────────────────────────── */
float foc_get_rpm(void)    { return s_rpm; }
int16_t foc_get_iq_ma(void) { return s_iq_meas; }
int16_t foc_get_id_ma(void) { return s_id_meas; }
bool foc_is_running(void)   { return s_running; }

/* ── Parametry runtime ──────────────────────────────────────── */
void foc_set_kp_iq(float v)    { if (v<0.001f) v=0.001f; if (v>5.0f) v=5.0f; s_kp_iq = v; }
void foc_set_ki_iq(float v)    { if (v<0.001f) v=0.001f; if (v>10.0f) v=10.0f; s_ki_iq = v; }
void foc_set_kp_id(float v)    { if (v<0.001f) v=0.001f; if (v>5.0f) v=5.0f; s_kp_id = v; }
void foc_set_ki_id(float v)    { if (v<0.001f) v=0.001f; if (v>10.0f) v=10.0f; s_ki_id = v; }
void foc_set_curr_filt(float v){ if (v<0.01f) v=0.01f; if (v>0.5f) v=0.5f; s_curr_filt = v; }
void foc_set_vbat_mv(int32_t v){ if (v<10000) v=10000; if (v>60000) v=60000; s_vbat_mv = v; }

float   foc_get_kp_iq(void)    { return s_kp_iq; }
float   foc_get_ki_iq(void)    { return s_ki_iq; }
float   foc_get_kp_id(void)    { return s_kp_id; }
float   foc_get_ki_id(void)    { return s_ki_id; }
float   foc_get_curr_filt(void) { return s_curr_filt; }
int32_t foc_get_vbat_mv(void)  { return s_vbat_mv; }
