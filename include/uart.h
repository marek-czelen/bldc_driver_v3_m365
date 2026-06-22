/** uart.h — USART3, 115200, RX na przerwaniu, TX blokujący. */
#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

void uart_init(void);

void uart_write(const char *s);
void uart_write_len(const char *buf, uint32_t len);
void uart_printf(const char *fmt, ...);

/* Pobiera kompletną linię (zakończoną \n lub \r) do 'out'.
 * Zwraca true gdy linia gotowa. Bufor jest zerowany. */
bool uart_get_line(char *out, uint32_t max_len);

#endif /* UART_H */
