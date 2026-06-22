#include "cli.h"
#include "uart.h"
#include "motor.h"
#include "adc.h"
#include "foc.h"
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
        "D<n> duty% N<n> rpm F0|1 dir M0|1|2 tryb\r\n"
        "V<0-5> wektor L[%%] ucz-hall K save P cfg X erase\r\n"
        "O<deg> advance J<slp> slope A analog ? status H help\r\n"
        "G<k> Kp_Iq Z<k> Ki_Iq (FOC tune)\r\n");
}

static void cmd_status(void)
{
    int16_t ia = adc_read_current_ma(ADC_CH_IA);
    int16_t ib = adc_read_current_ma(ADC_CH_IB);
    int16_t ic = adc_read_current_ma(ADC_CH_IC);
    int adv_d1 = (int)(motor_get_advance_deg() * 10.0f + 0.5f);
    int slp_d1 = (int)(motor_get_advance_slope() * 10.0f + 0.5f);
    if (motor_get_mode() == MODE_FOC) {
        int kp_d2 = (int)(foc_get_kp_iq() * 100.0f + 0.5f);
        int ki_d2 = (int)(foc_get_ki_iq() * 100.0f + 0.5f);
        uart_printf("st=%d md=%d dir=%d d=%d rpm=%d ia=%d ib=%d ic=%d iq=%d id=%d kp=%d.%02d ki=%d.%02d\r\n",
            (int)motor_get_state(), (int)motor_get_mode(),
            (int)motor_get_dir(),
            (int)motor_get_duty_pct(), (int)motor_get_rpm(),
            (int)ia, (int)ib, (int)ic,
            (int)foc_get_iq_ma(), (int)foc_get_id_ma(),
            kp_d2 / 100, kp_d2 % 100,
            ki_d2 / 100, ki_d2 % 100);
    } else {
        uart_printf("st=%d md=%d dir=%d ct=%d h=%u d=%d rpm=%d ia=%d ib=%d ic=%d adv=%d.%d slp=%d.%d\r\n",
            (int)motor_get_state(), (int)motor_get_mode(),
            (int)motor_get_dir(), (int)motor_get_ctrl(),
            (unsigned)motor_get_hall(), (int)motor_get_duty_pct(),
            (int)motor_get_rpm(),
            (int)ia, (int)ib, (int)ic,
            adv_d1 / 10, adv_d1 % 10,
            slp_d1 / 10, slp_d1 % 10);
    }
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
    int adv_d1 = (int)(motor_get_advance_deg() * 10.0f + 0.5f);
    int slp_d1 = (int)(motor_get_advance_slope() * 10.0f + 0.5f);
    uart_printf("seq=%u,%u,%u,%u,%u,%u ok=%d adv=%d.%d slp=%d.%d\r\n",
        motor_get_hall_seq(0), motor_get_hall_seq(1),
        motor_get_hall_seq(2), motor_get_hall_seq(3),
        motor_get_hall_seq(4), motor_get_hall_seq(5),
        motor_is_learned() ? 1 : 0,
        adv_d1 / 10, adv_d1 % 10,
        slp_d1 / 10, slp_d1 % 10);
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
        if (!(v == 0 || v == 1 || v == 2)) { RESP_E("RANGE"); break; }
        motor_set_mode((v == 2) ? MODE_FOC : (v == 1) ? MODE_SINUS : MODE_BLOCK);
        RESP_OK();
        break;
    }

    /* ── Kalibracja / konfiguracja ── */
    case 'O': {
        float v;
        if (!parse_float_arg(arg, &v)) { RESP_E("ARG"); break; }
        if (v < 0.0f || v > 120.0f)    { RESP_E("RANGE"); break; }
        motor_set_advance_deg(v);
        if (!motor_config_save()) { RESP_E("SAVE"); break; }
        RESP_OK();
        break;
    }
    case 'J': {
        float v;
        if (!parse_float_arg(arg, &v)) { RESP_E("ARG"); break; }
        if (v < 0.0f || v > 50.0f)     { RESP_E("RANGE"); break; }
        motor_set_advance_slope(v);
        if (!motor_config_save()) { RESP_E("SAVE"); break; }
        RESP_OK();
        break;
    }
    case 'G': {
        float v;
        if (!parse_float_arg(arg, &v)) { RESP_E("ARG"); break; }
        if (v < 0.001f || v > 5.0f)    { RESP_E("RANGE"); break; }
        foc_set_kp_iq(v);
        RESP_OK();
        break;
    }
    case 'Z': {
        float v;
        if (!parse_float_arg(arg, &v)) { RESP_E("ARG"); break; }
        if (v < 0.001f || v > 10.0f)   { RESP_E("RANGE"); break; }
        foc_set_ki_iq(v);
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
