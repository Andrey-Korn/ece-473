// Andrey Kornilovich
// lab 5, mega 168 board code

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions_m168.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "twi_master.h"
#include "sht21_functions.h"

char lcd_str_array[16];  //holds string to send to lcd

int i;

volatile uint8_t sht21_wr_buf[2];
volatile uint8_t sht21_rd_buf[2];

// volatile char sht21_temp[16];
volatile uint16_t result;

int main(){
    uart_init();
    init_twi();
    sei();

    // sht21_wr_buf[2] = 0x00;
    // sht21_wr_buf[2] = 0xE3;
    sht21_wr_buf[0] = 0xE3;

    // twi_start_rd(SHT21_ADDRESS, 0x80, 1);

    while(1){
        twi_start_wr(SHT21_ADDRESS, sht21_wr_buf, 1);
        _delay_ms(40);

        twi_start_rd(SHT21_ADDRESS, sht21_rd_buf, 2);
        _delay_ms(40);

        sht21_temp_convert(sht21_rd_buf, result, 0);

        itoa(result, lcd_str_array, 10);

        // uart_puts("ye");
        uart_puts(lcd_str_array);
        uart_putc('\0');

        for(i=0;i<=9;i++){_delay_ms(100);}

    // LED test
    // DDRB = (1 < PB5);

    // while(1){
        // PORTB ^= (1 << PB5);
        // _delay_ms(500);
    // }
   }
}
