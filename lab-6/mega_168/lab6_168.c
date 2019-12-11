// Andrey Kornilovich
// lab6_168.c

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions_m168.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "twi_master.h"
#include "sht21_functions.h"

char lcd_str_array[16];  // holds string to send to lcd
int i; // counter
uint8_t sht21_wr_buf[1]; // message buffer to SHT21
uint8_t sht21_rd_buf[2]; // buffer back from SHT21

char sht21_str[16];      // store string result after conversion
uint16_t sht21_temp;     // 16 bit temperature packet

int main(){
    // init uart, twi, and interrupts
    uart_init();
    init_twi();
    sei();

    // write default message for temperature to buffer
    sht21_wr_buf[0] = SHT21_TEMP_HOLD;

    while(1){
        // SHT21 always needs a write, then a read
        twi_start_wr(SHT21_WRITE, sht21_wr_buf, 1);
        twi_start_rd(SHT21_READ, sht21_rd_buf, 2);

        // shift in result
        sht21_temp = sht21_rd_buf[0];
        sht21_temp = (sht21_temp << 8);
        sht21_temp |= sht21_rd_buf[1];

        // sht21_temp_convert(sht21_str, sht21_temp, 0); // fahrenheit 
        sht21_temp_convert(sht21_str, sht21_temp, 1);    // celcius

        strcpy(lcd_str_array, sht21_str);

        // send string over UART
        uart_puts(lcd_str_array);
        uart_putc('\0');

        // loop delay
        for(i=0;i<=3;i++){_delay_ms(100);}
   }
}

// LED test
// DDRB = (1 < PB5);

// while(1){
    // PORTB ^= (1 << PB5);
    // _delay_ms(500);
// }