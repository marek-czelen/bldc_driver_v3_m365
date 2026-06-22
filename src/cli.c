#include "cli.h"
#include "uart.h"
#include "motor.h"
#include "adc.h"
#include "board.h"

#include <stdlib.h>
#include <ctype.h>

/*
 * Krotki protokol (komendy wysylane przez nadrzedny CPU).
 * Format: <litera>[liczba]\n   (biale znaki ignorowane)
 *
 *   R          start (run)
 *   S          stop (wybieg)
 *   B          brake (hamowanie zwarciowe)
 *   D<n>       duty % 0..100        (open-loop)   np. D5
 *   N<n>       zadana predkosc rpm  (closed-loop) np. N300
 *   F<0|1>     kierunek 0=FWD 1=REV
 *   M<0|1>     tryb 0=BLOCK 1=SINUS
 *   V<0..5>    wymuszenie wektora komutacji (kalibracja, open-loop)
 *   L[%]       uczenie sekwencji Halla, np. "L" lub "L25" dla 25% duty
 *   K          zapisz konfiguracje Halla do EEPROM
 *   P          wypisz zapisana konfiguracje (sekwencja Halla)
 *   X          usun zapisana konfiguracje (powrot do domyslnej)
 *   A          odczyt analogowy: vbat,Ia,Ib,Ic (raw ADC)
 *   ?          status (CSV)
 *   H / I      pomoc
 *
 * Odpowiedzi: "OK" / "E" / linia danych.
 */

static void print_help(void)
{
    uart_write(
        "R start | S stop | B brake\r\n"
        "D<n> duty% | N<n> rpm | F<0|1> dir | M<0|1> tryb\r\n"
        "V<0-5> wektor | L[%] ucz-hall | K save | P cfg | X erase\r\n"
        "A analog | ? status | H pomoc\r\n");
}

/* status: st,md,dir,ct,hall,duty%,rpm */
static void print_status(void)
{
    uart_printf("st=%d md=%d dir=%d ct=%d h=%u d=%d rpm=%d\r\n",
                (int)motor_get_state(),
                (int)motor_get_mode(),
                (int)motor_get_dir(),
                (int)motor_get_ctrl(),
                (unsigned)motor_get_hall(),
                (int)motor_get_duty_pct(),
                (int)motor_get_rpm());
}

static void print_analog(void)
{
    uart_printf("vbat=%u Ia=%u Ib=%u Ic=%u\r\n",
                (unsigned)adc_read(ADC_CH_VBAT),
                (unsigned)adc_read(ADC_CH_IA),
                (unsigned)adc_read(ADC_CH_IB),
                (unsigned)adc_read(ADC_CH_IC));
}

/* Wypisz zapisaną/aktywną konfigurację Halla. */
static void print_config(void)
{
    uart_printf("hall_seq=%u,%u,%u,%u,%u,%u learned=%d\r\n",
                motor_get_hall_seq(0), motor_get_hall_seq(1),
                motor_get_hall_seq(2), motor_get_hall_seq(3),
                motor_get_hall_seq(4), motor_get_hall_seq(5),
                motor_is_learned() ? 1 : 0);
}

static void handle_line(const char *line)
{
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    if (*line == '\0') {
        return;
    }

    char cmd = (char)toupper((unsigned char)*line);
    const char *arg = line + 1;
    while (*arg == ' ' || *arg == '\t') {
        arg++;
    }

    switch (cmd) {
        case 'R':
            motor_start();
            uart_write("OK\r\n");
            break;
        case 'S':
            motor_stop();
            uart_write("OK\r\n");
            break;
        case 'B':
            motor_brake();
            uart_write("OK\r\n");
            break;
        case 'D':
            if (*arg) {
                motor_set_duty_pct((float)atof(arg));
                uart_write("OK\r\n");
            } else {
                uart_write("E\r\n");
            }
            break;
        case 'N':
            if (*arg) {
                motor_set_target_rpm((float)atof(arg));
                uart_write("OK\r\n");
            } else {
                uart_write("E\r\n");
            }
            break;
        case 'F':
            if (*arg == '0') {
                motor_set_dir(DIR_FWD);
                uart_write("OK\r\n");
            } else if (*arg == '1') {
                motor_set_dir(DIR_REV);
                uart_write("OK\r\n");
            } else {
                uart_write("E\r\n");
            }
            break;
        case 'M':
            if (*arg == '0') {
                motor_set_mode(MODE_BLOCK);
                uart_write("OK\r\n");
            } else if (*arg == '1') {
                motor_set_mode(MODE_SINUS);
                uart_write("OK\r\n");
            } else {
                uart_write("E\r\n");
            }
            break;
        case 'V':
            if (*arg) {
                motor_force_vector((uint8_t)atoi(arg));
                uart_write("OK\r\n");
            } else {
                uart_write("E\r\n");
            }
            break;
        case 'L': {
            uint8_t seq[6];
            float dp = 0.0f;
            if (*arg) {
                dp = (float)atof(arg);
            }
            bool ok = motor_learn_hall_with_duty(dp, seq);
            uart_printf("%s seq=%u,%u,%u,%u,%u,%u\r\n",
                        ok ? "OK" : "E",
                        seq[0], seq[1], seq[2], seq[3], seq[4], seq[5]);
            break;
        }
        case 'K':
            if (motor_config_save()) {
                uart_write("OK\r\n");
            } else {
                uart_write("E\r\n");
            }
            break;
        case 'P':
            print_config();
            break;
        case 'X':
            motor_config_erase();
            uart_write("OK\r\n");
            break;
        case 'A':
            print_analog();
            break;
        case '?':
            print_status();
            break;
        case 'H':
        case 'I':
            print_help();
            break;
        default:
            uart_write("E\r\n");
            break;
    }
}

void cli_init(void)
{
    uart_write("BLDC v3 BLOCK\r\n");
}

void cli_process(void)
{
    char line[64];
    if (uart_get_line(line, sizeof(line))) {
        handle_line(line);
    }
}
