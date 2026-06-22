#include "uart.h"
#include "board.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RX_BUF_SIZE   256U
#define LINE_MAX      96U

static volatile char     rx_buf[RX_BUF_SIZE];
static volatile uint32_t rx_head;  /* ISR pisze */
static volatile uint32_t rx_tail;  /* main czyta */

static char     line[LINE_MAX];
static uint32_t line_len;
static bool     pending_line;
static bool     last_was_cr;
static bool     discard_until_eol;
static volatile uint32_t s_uart_err_flags;

void uart_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB |
                             LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

    LL_GPIO_InitTypeDef g = {0};
    g.Pin = LL_GPIO_PIN_10;
    g.Mode = LL_GPIO_MODE_ALTERNATE;
    g.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    g.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &g);

    g.Pin = LL_GPIO_PIN_11;
    g.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOB, &g);

    LL_USART_InitTypeDef u = {0};
    u.BaudRate = UART_BAUDRATE;
    u.DataWidth = LL_USART_DATAWIDTH_8B;
    u.StopBits = LL_USART_STOPBITS_1;
    u.Parity = LL_USART_PARITY_NONE;
    u.TransferDirection = LL_USART_DIRECTION_TX_RX;
    u.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    LL_USART_Init(UART_INSTANCE, &u);
    LL_USART_Enable(UART_INSTANCE);

    /* Wyczyść ewentualne błędy po inicie. */
    (void)USART3->SR;
    (void)USART3->DR;

    rx_head = 0;
    rx_tail = 0;
    line_len = 0;
    pending_line = false;
    last_was_cr = false;
    discard_until_eol = false;
    s_uart_err_flags = 0;

    LL_USART_EnableIT_RXNE(UART_INSTANCE);
    NVIC_SetPriority(USART3_IRQn, 2);
    NVIC_EnableIRQ(USART3_IRQn);
}

void uart_write(const char *s)
{
    while (*s) {
        while (!(USART3->SR & USART_SR_TXE)) {}
        USART3->DR = (uint8_t)*s++;
    }
    while (!(USART3->SR & USART_SR_TC)) {}
}

void uart_printf(const char *fmt, ...)
{
    char buf[100];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_write(buf);
}

static void line_push_char(char c)
{
    if (line_len < (LINE_MAX - 1U)) {
        line[line_len++] = c;
    } else {
        /* Linia przepełniona: odrzuć bieżącą ramkę do najbliższego terminatora. */
        line_len = 0;
        pending_line = false;
        discard_until_eol = true;
        s_uart_err_flags |= UART_ERR_LINE_OVERFLOW;
    }
}

static void line_finalize(void)
{
    if (line_len > 0U) {
        pending_line = true;
    }
    last_was_cr = false;
}

bool uart_get_line(char *out, int max_len)
{
    if (out == NULL || max_len <= 1) {
        return false;
    }

    while (rx_tail != rx_head) {
        char c = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1U) % RX_BUF_SIZE;

        if (c == '\r') {
            if (discard_until_eol) {
                line_len = 0;
                pending_line = false;
                discard_until_eol = false;
            } else {
                line_finalize();
            }
            last_was_cr = true;
        } else if (c == '\n') {
            /* CRLF traktuj jako jeden terminator. */
            if (discard_until_eol) {
                line_len = 0;
                pending_line = false;
                discard_until_eol = false;
            } else if (!last_was_cr) {
                line_finalize();
            }
            last_was_cr = false;
        } else {
            last_was_cr = false;
            if (!discard_until_eol) {
                line_push_char(c);
            }
        }

        if (pending_line) {
            uint32_t n = (line_len < (uint32_t)(max_len - 1)) ? line_len : (uint32_t)(max_len - 1);
            memcpy(out, line, n);
            out[n] = '\0';
            line_len = 0;
            pending_line = false;
            return true;
        }
    }

    return false;
}

uint32_t uart_get_and_clear_errors(void)
{
    uint32_t e = s_uart_err_flags;
    s_uart_err_flags = 0;
    return e;
}

void USART3_IRQHandler(void)
{
    uint32_t sr = USART3->SR;
    bool dr_read = false;

    /* Gdy RXNE ustawione, odczyt DR pobiera znak i czyści RXNE.
     * ORE/FE/NE też czyścimy przez odczyt DR po SR. */
    if (sr & USART_SR_RXNE) {
        char c = (char)USART3->DR;
        dr_read = true;
        uint32_t next = (rx_head + 1U) % RX_BUF_SIZE;
        if (next != rx_tail) {
            rx_buf[rx_head] = c;
            rx_head = next;
        } else {
            /* Ring buffer overflow: odrzucamy najstarszy znak żeby parser wrócił do życia. */
            rx_tail = (rx_tail + 1U) % RX_BUF_SIZE;
            rx_buf[rx_head] = c;
            rx_head = next;
            s_uart_err_flags |= UART_ERR_RX_OVERFLOW;
        }
    }

    /* Błędy linii: trzeba odczytać DR po SR żeby je skasować. */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        if (!dr_read) {
            (void)USART3->DR;
        }
        s_uart_err_flags |= UART_ERR_RX_OVERFLOW;
    }
}
