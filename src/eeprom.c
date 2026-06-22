#include "eeprom.h"
#include "board.h"

#include <string.h>
#include <stddef.h>

/*
 * Emulowany EEPROM na ostatniej stronie Flash (1 KB).
 * STM32F103C8T6: 64 KB Flash = 64 stron × 1 KB.
 * Strona konfiguracyjna: strona 63, adres 0x0800FC00.
 */
#define EE_PAGE_ADDR    0x0800FC00U
#define EE_PAGE_SIZE    1024U
#define EE_MAGIC        0x4C48U   /* 'H' 'L' little-endian */
#define EE_VERSION      1U

/* CRC16/XMODEM (polynomial 0x1021). */
static uint16_t crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* Odblokowanie pamięci Flash do zapisu/kasowania. */
static void flash_unlock(void)
{
    FLASH->KEYR = 0x45670123U;
    FLASH->KEYR = 0xCDEF89ABU;
}

/* Zablokowanie pamięci Flash. */
static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

/* Czekaj na zakończenie operacji Flash. */
static void flash_wait(void)
{
    while (FLASH->SR & FLASH_SR_BSY) {
    }
}

/* ------------------------------------------------------------------ */

void eeprom_init(void)
{
    /* Flash nie wymaga osobnej inicjalizacji. */
}

bool eeprom_erase(void)
{
    flash_unlock();
    flash_wait();

    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = EE_PAGE_ADDR;
    FLASH->CR |= FLASH_CR_STRT;
    flash_wait();
    FLASH->CR &= ~FLASH_CR_PER;

    flash_lock();
    return true;
}

bool eeprom_save(const eeprom_config_t *cfg)
{
    if (cfg == NULL || cfg->magic != EE_MAGIC) {
        return false;
    }

    /* Oblicz CRC (pole crc16 w strukturze tymczasowo = 0). */
    eeprom_config_t c = *cfg;
    c.crc16 = 0;
    uint16_t crc = crc16((const uint8_t *)&c, sizeof(c));

    /* Skasuj stronę. */
    if (!eeprom_erase()) {
        return false;
    }

    flash_unlock();
    flash_wait();
    FLASH->CR |= FLASH_CR_PG;

    const uint16_t *src = (const uint16_t *)&c;
    uint32_t off = 0;
    /* Zapisz wszystkie słowa OPRÓCZ crc16 (zostanie wpisane osobno). */
    const uint32_t crc_offset = offsetof(eeprom_config_t, crc16);
    while (off < sizeof(c)) {
        if (off == crc_offset) {
            off += 2U;
            src++;
            continue;  /* pomiń CRC — zapiszemy go na końcu */
        }
        *(volatile uint16_t *)(EE_PAGE_ADDR + off) = *src;
        flash_wait();
        off += 2U;
        src++;
    }

    /* Zapisz CRC w miejscu, które było pominięte (nadal 0xFFFF po erase). */
    *(volatile uint16_t *)(EE_PAGE_ADDR + crc_offset) = crc;
    flash_wait();

    FLASH->CR &= ~FLASH_CR_PG;
    flash_lock();
    return true;
}

bool eeprom_load(eeprom_config_t *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    /* Skopiuj zawartość strony do struktury. */
    memcpy(cfg, (const void *)EE_PAGE_ADDR, sizeof(*cfg));

    /* Walidacja magic + CRC. */
    if (cfg->magic != EE_MAGIC || cfg->version != EE_VERSION) {
        return false;
    }

    uint16_t stored_crc = cfg->crc16;
    cfg->crc16 = 0;
    uint16_t calc_crc = crc16((const uint8_t *)cfg, sizeof(*cfg));
    cfg->crc16 = stored_crc;

    return (calc_crc == stored_crc);
}
