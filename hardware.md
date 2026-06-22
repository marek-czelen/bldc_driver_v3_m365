## Hardware — MCU i peryferia

| Parametr | Wartość |
|---|---|
| MCU | STM32F103C8T6 (Cortex-M3, 64 KB flash, 20 KB SRAM) |
| Oscylator | **Brak kwarcu** — taktowanie z wewnętrznego RC (HSI 8 MHz) |
| Zegar systemowy | 64 MHz (HSI/2 × PLL16) |
| Timer PWM | TIM1 (Advanced Timer, 3 kanały komplementarne) |
| ADC | ADC1, pomiary regularne + injected (TIM1 CH4), DMA |
| UART | USART3, 115200 baud, DMA TX |
| Hall sensors | 3x EXTI (rising + falling) |
| Silnik | BLDC 3-fazowy, 15 par biegunów, czujniki Halla |
| Mostek | 3-fazowy mostek H (6 MOSFET/IGBT) z complementary PWM |


## Pinout

### Wyjścia PWM (TIM1, mostek 3-fazowy)

| Pin   | Funkcja    | Opis |
|-------|-----------|------|
| PA8   | TIM1_CH1  | Faza A — high side |
| PA9   | TIM1_CH2  | Faza B — high side |
| PA10  | TIM1_CH3  | Faza C — high side |
| PB13  | TIM1_CH1N | Faza A — low side (komplementarny) |
| PB14  | TIM1_CH2N | Faza B — low side (komplementarny) |
| PB15  | TIM1_CH3N | Faza C — low side (komplementarny) |

### Czujniki Halla (EXTI)

| Pin  | Funkcja | Bit w stanie Halla |
|------|---------|---------------------|
| PB4  | H1      | bit 0 |
| PB5  | H2      | bit 1 |
| PB0  | H3      | bit 2 |

### Wejścia ADC

| Pin  | Kanał ADC | Funkcja |
|------|-----------|---------|
| PA3  | CH3       | Prąd fazy A (Ia) |
| PA4  | CH4       | Prąd fazy B (Ib) |
| PA5  | CH5       | Prąd fazy C (Ic) |
| PA2  | CH2       | Napięcie baterii (Vbat) |

### UART

| Pin  | Funkcja     |
|------|-------------|
| PB10 | USART3_TX   |
| PB11 | USART3_RX   |


## Uwagi konstrukcyjne

- **Brak rezonatora kwarcowego** — układ taktowany wyłącznie z wewnętrznego oscylatora
  RC (HSI 8 MHz). Konfiguracja PLL: `HSI/2 × 16 = 64 MHz` (`src/clock.c`).
  Nie wolno włączać HSE ani czekać na jego gotowość — spowodowałoby to zawieszenie startu.
  Uwaga: HSI ma dokładność ~±1% i dryf z temperaturą — baud rate UART jest tolerancyjny,
  ale do precyzyjnych zastosowań czasowych należałoby dodać kwarc.
- **PB4 (Hall H1)** to domyślnie pin `JTRST` (JTAG). Firmware wykonuje
  `SWJ_NOJTAG` (`src/hall.c`), aby zwolnić PB4 do funkcji EXTI. SWD (PA13/PA14)
  pozostaje aktywne — programowanie/debug przez ST-Link działa bez zmian.
