#include "cli.h"
#include "uart.h"
#include "motor.h"
#include "adc.h"
#include "board.h"
#include <stdlib.h>

#define RESP_OK()          uart_write("OK\r\n")
#define RESP_E(code)       uart_printf("E %s\r\n", (code))

static int parse_int_arg(const char *arg, int *out)
{
    char *end = NULL;
    long v;

    if (arg == NULL || *arg == '\0') return 0;
    v = strtol(arg, &end, 10);
    if (end == arg || *end != '\0') return 0;
    *out = (int)v;
    return 1;
}

static int parse_float_arg(const char *arg, float *out)
{
    char *end = NULL;
    float v;

    if (arg == NULL || *arg == '\0') return 0;
    v = strtof(arg, &end);
    if (end == arg || *end != '\0') return 0;
    *out = v;
    return 1;
}

/*
 * Protokół: <litera>[argument], terminator CR/LF/CRLF.
 * Białe znaki po literze ignorowane.
 * Odpowiedzi: jedna linia na jedną komendę.
 */

static void cmd_help(void)
{
    uart_write("BLDC v3\r\n"
        "R start S stop B brake W ping\r\n"
        "D<n> duty% N<n> rpm F0|1 dir M0|1 tryb\r\n"
        "V<0-5> wektor L[%%] ucz-hall K save P cfg X erase\r\n"
        "O<deg> advance J<slp> slope A analog ? status H help\r\n");
}

static void cmd_status(void)
{
    int16_t ia = adc_read_current_ma(ADC_CH_IA);
    int16_t ib = adc_read_current_ma(ADC_CH_IB);
    int16_t ic = adc_read_current_ma(ADC_CH_IC);
    uart_printf("st=%d md=%d dir=%d ct=%d h=%u d=%d rpm=%d ia=%d ib=%d ic=%d adv=%.1f slp=%.1f\r\n",
        (int)motor_get_state(), (int)motor_get_mode(),
        (int)motor_get_dir(), (int)motor_get_ctrl(),
        (unsigned)motor_get_hall(), (int)motor_get_duty_pct(),
        (int)motor_get_rpm(),
        (int)ia, (int)ib, (int)ic,
        (double)motor_get_advance_deg(),
        (double)motor_get_advance_slope());
}

static void cmd_analog(void)
{
    uart_printf("V=%u A=%u B=%u C=%u\r\n",
        (unsigned)adc_read(ADC_CH_VBAT),
        (unsigned)adc_read(ADC_CH_IA),
        (unsigned)adc_read(ADC_CH_IB),
        (unsigned)adc_read(ADC_CH_IC));
}

static void cmd_config(void)
{
    uart_printf("seq=%u,%u,%u,%u,%u,%u ok=%d adv=%.1f slp=%.1f\r\n",
        motor_get_hall_seq(0), motor_get_hall_seq(1),
        motor_get_hall_seq(2), motor_get_hall_seq(3),
        motor_get_hall_seq(4), motor_get_hall_seq(5),
        motor_is_learned() ? 1 : 0,
        (double)motor_get_advance_deg(),
        (double)motor_get_advance_slope());
}

void cli_init(void)
{
    cmd_help();
}

void cli_process(void)
{
    char buf[96];
    uint32_t uerr;
    if (!uart_get_line(buf, sizeof(buf))) return;
    if (buf[0] == '\0') return;

    uerr = uart_get_and_clear_errors();
    if (uerr != 0U) {
        uart_printf("E UART %lu\r\n", (unsigned long)uerr);
        return;
    }

    char  cmd = buf[0];
    /* Normalizuj do uppercase (małe litery → duże). */
    if (cmd >= 'a' && cmd <= 'z') cmd -= ('a' - 'A');

    char *arg = buf + 1;
    while (*arg == ' ' || *arg == '\t') arg++;

    switch (cmd) {
    /* ── Podstawowe ── */
    case 'R':
        if (motor_get_state() == MSTATE_FAULT) motor_stop();
        motor_start();
        RESP_OK();
        break;
    case 'S':
        motor_stop();
        RESP_OK();
        break;
    case 'B':
        motor_brake();
        RESP_OK();
        break;
    case 'W':
        RESP_OK();
        break;
    case '?':
        cmd_status();
        break;
    case 'H':
        cmd_help();
        break;

    /* ── Nastawy ── */
    case 'D': {
        float v;
        if (!parse_float_arg(arg, &v)) { RESP_E("ARG"); break; }
        if (v < 0.0f || v > 100.0f)     { RESP_E("RANGE"); break; }
        motor_set_duty_pct(v);
        RESP_OK();
        break;
    }
    case 'N': {
        float v;
        if (!parse_float_arg(arg, &v)) { RESP_E("ARG"); break; }
        if (v < 0.0f)                  { RESP_E("RANGE"); break; }
        motor_set_target_rpm(v);
        RESP_OK();
        break;
    }
    case 'F': {
        int v;
        if (!parse_int_arg(arg, &v))   { RESP_E("ARG"); break; }
        if (!(v == 0 || v == 1))       { RESP_E("RANGE"); break; }
        motor_set_dir((v == 1) ? DIR_REV : DIR_FWD);
        RESP_OK();
        break;
    }
    case 'M': {
        int v;
        if (!parse_int_arg(arg, &v))   { RESP_E("ARG"); break; }
        if (!(v == 0 || v == 1))       { RESP_E("RANGE"); break; }
        motor_set_mode((v == 1) ? MODE_SINUS : MODE_BLOCK);
        RESP_OK();
        break;
    }

    /* ── Kalibracja / konfiguracja ── */
    case 'O': {
        float v;
        if (!parse_float_arg(arg, &v)) { RESP_E("ARG"); break; }
        if (v < 0.0f || v > 120.0f)    { RESP_E("RANGE"); break; }
        motor_set_advance_deg(v);
        motor_config_save();  /* auto-zapis do EEPROM */
        RESP_OK();
        break;
    }
    case 'J': {
        float v;
        if (!parse_float_arg(arg, &v)) { RESP_E("ARG"); break; }
        if (v < 0.0f || v > 50.0f)     { RESP_E("RANGE"); break; }
        motor_set_advance_slope(v);
        motor_config_save();  /* auto-zapis do EEPROM */
        RESP_OK();
        break;
    }
    case 'V': {
        int v;
        if (!parse_int_arg(arg, &v))   { RESP_E("ARG"); break; }
        if (v < 0 || v > 5)            { RESP_E("RANGE"); break; }
        motor_force_vector((uint8_t)v);
        RESP_OK();
        break;
    }
    case 'L': {
        float pct = 0.0f;
        uint8_t seq[6];
        if (*arg != '\0' && !parse_float_arg(arg, &pct)) {
            RESP_E("ARG");
            break;
        }
        if (pct < 0.0f || pct > 100.0f) {
            RESP_E("RANGE");
            break;
        }
        bool ok = motor_learn_hall_with_duty(pct, seq);
        uart_printf("%s %u,%u,%u,%u,%u,%u\r\n", ok ? "OK" : "E LEARN",
            seq[0], seq[1], seq[2], seq[3], seq[4], seq[5]);
        break;
    }
    case 'K':
        if (motor_config_save()) RESP_OK(); else RESP_E("SAVE");
        break;
    case 'P':
        cmd_config();
        break;
    case 'X':
        motor_config_erase();
        RESP_OK();
        break;

    /* ── Pomiary ── */
    case 'A':
        cmd_analog();
        break;

    default:
        RESP_E("CMD");
        break;
    }
}
