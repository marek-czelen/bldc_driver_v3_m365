#include "uart.h"
#include "board.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RX_BUF_SIZE 128U
#define LINE_MAX    96U

static volatile char     s_rx[RX_BUF_SIZE];
static volatile uint32_t s_head; /* zapis (ISR)  */
static volatile uint32_t s_tail; /* odczyt (main)*/

static char     s_line[LINE_MAX];
static uint32_t s_line_len;

void uart_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB |
                             LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

    /* PB10 = TX (AF push-pull), PB11 = RX (floating) */
    LL_GPIO_InitTypeDef g = {0};
    g.Pin        = LL_GPIO_PIN_10;
    g.Mode       = LL_GPIO_MODE_ALTERNATE;
    g.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    g.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &g);

    g.Pin  = LL_GPIO_PIN_11;
    g.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOB, &g);

    LL_USART_InitTypeDef u = {0};
    u.BaudRate            = UART_BAUDRATE;
    u.DataWidth           = LL_USART_DATAWIDTH_8B;
    u.StopBits            = LL_USART_STOPBITS_1;
    u.Parity              = LL_USART_PARITY_NONE;
    u.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    u.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    LL_USART_Init(UART_INSTANCE, &u);
    LL_USART_Enable(UART_INSTANCE);

    LL_USART_EnableIT_RXNE(UART_INSTANCE);
    NVIC_SetPriority(USART3_IRQn, 2);
    NVIC_EnableIRQ(USART3_IRQn);
}

void uart_write_len(const char *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(UART_INSTANCE)) {
        }
        LL_USART_TransmitData8(UART_INSTANCE, (uint8_t)buf[i]);
    }
    while (!LL_USART_IsActiveFlag_TC(UART_INSTANCE)) {
    }
}

void uart_write(const char *s)
{
    uart_write_len(s, (uint32_t)strlen(s));
}

void uart_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        uart_write_len(buf, (uint32_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf) - 1));
    }
}

bool uart_get_line(char *out, uint32_t max_len)
{
    while (s_tail != s_head) {
        char c = s_rx[s_tail];
        s_tail = (s_tail + 1U) % RX_BUF_SIZE;

        if (c == '\n' || c == '\r') {
            if (s_line_len > 0U) {
                uint32_t n = (s_line_len < max_len - 1U) ? s_line_len : max_len - 1U;
                memcpy(out, s_line, n);
                out[n] = '\0';
                s_line_len = 0U;
                return true;
            }
            /* pusta linia — pomiń */
            s_line_len = 0U;
        } else if (s_line_len < (LINE_MAX - 1U)) {
            s_line[s_line_len++] = c;
        }
    }
    return false;
}

void USART3_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_RXNE(UART_INSTANCE)) {
        char c = (char)LL_USART_ReceiveData8(UART_INSTANCE);
        uint32_t next = (s_head + 1U) % RX_BUF_SIZE;
        if (next != s_tail) {
            s_rx[s_head] = c;
            s_head = next;
        }
        /* przepełnienie -> znak odrzucony */
    }
}
