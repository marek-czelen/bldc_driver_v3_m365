/** hall.h — czujniki Halla na EXTI. */
#ifndef HALL_H
#define HALL_H

#include <stdint.h>

void hall_init(void);

/* Aktualny stan 3 czujników jako 3-bitowa wartość (1..6 = poprawne). */
uint8_t hall_read(void);

#endif /* HALL_H */
