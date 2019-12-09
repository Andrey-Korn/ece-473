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

uint8_t sht21_wr_buf[1];
uint8_t sht21_rd_buf[2];

char sht21_str[16];
uint16_t sht21_temp;
uint16_t final_temp;

int main(){
    uart_init();
    init_twi();
    sei();

    sht21_wr_buf[0] = SHT21_TEMP_HOLD;

    while(1){
        twi_start_wr(SHT21_WRITE, sht21_wr_buf, 1);
        twi_start_rd(SHT21_READ, sht21_rd_buf, 2);

        sht21_temp = sht21_rd_buf[0];
        sht21_temp = (sht21_temp << 8);
        sht21_temp |= sht21_rd_buf[1];

        // sht21_temp_convert(sht21_str, sht21_temp, 0);
        sht21_temp_convert(sht21_str, sht21_temp, 1);

        strcpy(lcd_str_array, sht21_str);

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
