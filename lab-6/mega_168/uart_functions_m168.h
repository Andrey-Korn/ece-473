//UART Functions header file for Mega 168p only
//Roger Traylor 12.7.15
//Modified for the m168p by Jordan Baxter 11.28.19

//Modified for use with ATmega 168p by Jordan Baxter 11.29.19
//For controlling the UART and sending debug data to a terminal
//as an aid in debugging.

void uart_putc(char data);
void uart_puts(char *str);
void uart_puts_p(const char *str);
void uart_init();
char uart_getc(void);

