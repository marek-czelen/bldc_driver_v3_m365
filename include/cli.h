/** cli.h — interpreter komend tekstowych z UART. */
#ifndef CLI_H
#define CLI_H

void cli_init(void);
void cli_process(void);   /* wywoływane w pętli głównej */

#endif /* CLI_H */
