/** uart.h — USART3, 115200, RX interrupt, TX blocking. */
#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

/* Bity błędów warstwy RX (do diagnostyki i auto-recovery). */
#define UART_ERR_RX_OVERFLOW   (1U << 0)
#define UART_ERR_LINE_OVERFLOW (1U << 1)

void uart_init(void);
void uart_write(const char *s);
void uart_printf(const char *fmt, ...);

/* Zwraca true gdy gotowa jest pełna linia.
 * Terminator: CR, LF lub CRLF (CRLF liczone jako jedna linia).
 * out jest zawsze zakończone '\0'. */
bool uart_get_line(char *out, int max_len);

/* Odczyt i kasowanie flag błędów parsera RX. */
uint32_t uart_get_and_clear_errors(void);

#endif /* UART_H */
