/** foc.h — Field Oriented Control z hall sensorami i pomiarem prądu. */
#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include <stdbool.h>

#define FOC_IQ_MAX  15000   /* max Iq [mA] */
#define FOC_ID_MAX  5000    /* max Id [mA] */

void foc_init(void);
void foc_start(void);
void foc_stop(void);

/* Ustaw amplitudę prądu Iq [mA]. Zakres 0..FOC_IQ_MAX. */
void foc_set_iq_target(int16_t iq_ma);

/* Główna pętla FOC — wołana co 1 ms z motor_tick_1ms. */
void foc_update(void);

/* Odczyty */
float    foc_get_rpm(void);
int16_t  foc_get_iq_ma(void);
int16_t  foc_get_id_ma(void);
bool     foc_is_running(void);

/* Parametry runtime (0.001..5.0 / 10.0) */
void    foc_set_kp_iq(float v);
void    foc_set_ki_iq(float v);
void    foc_set_kp_id(float v);
void    foc_set_ki_id(float v);
void    foc_set_curr_filt(float v);
void    foc_set_vbat_mv(int32_t v);
float   foc_get_kp_iq(void);
float   foc_get_ki_iq(void);
float   foc_get_kp_id(void);
float   foc_get_ki_id(void);
float   foc_get_curr_filt(void);
int32_t foc_get_vbat_mv(void);

#endif /* FOC_H */
