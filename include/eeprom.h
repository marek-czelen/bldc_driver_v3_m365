/** eeprom.h — emulowany EEPROM w ostatniej stronie Flash (STM32F103C8). */
#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include <stdbool.h>
#include "pwm.h"   /* phase_t */

/* Konfiguracja silnika przechowywana w emulowanym EEPROM. */
typedef struct {
    uint16_t magic;         /* 0x4C48 = 'H''L' (little-endian)        */
    uint16_t version;       /* 1                                       */
    uint16_t pole_pairs;    /* liczba par biegunów                     */
    int8_t   hi_fwd[6];     /* hi dla hall=1..6, -1=niepoprawny       */
    int8_t   lo_fwd[6];     /* lo dla hall=1..6                       */
    uint16_t crc16;         /* CRC całej struktury (pole CRC=0 przy liczeniu) */
} eeprom_config_t;

/* Inicjalizacja — nic nie robi (flash nie wymaga inicjalizacji). */
void eeprom_init(void);

/* Zapis konfiguracji do Flash. Zwraca true gdy OK. */
bool eeprom_save(const eeprom_config_t *cfg);

/* Odczyt konfiguracji z Flash. Zwraca true gdy dane poprawne (magic + CRC). */
bool eeprom_load(eeprom_config_t *cfg);

/* Wymazanie strony konfiguracyjnej. */
bool eeprom_erase(void);

#endif /* EEPROM_H */
