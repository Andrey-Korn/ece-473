// Andrey Kornilovich
// lab 5, mega 168 board code

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions_m168.h"
#include <avr/interrupt.h>
#include <util/delay.h>

uint8_t           i;
volatile uint8_t  rcv_rdy;
char              rx_char; 
char              lcd_str_array[16];  //holds string to send to lcd
uint8_t           send_seq=0;         //transmit sequence number
char              lcd_string[3];      //holds value of sequence number

int main(){
    uart_init();

    sei();

    while(1){
        // if uart received, turn on LED
        if(rcv_rdy == 1){
            rcv_rdy = 0;
            PORTB = (1 << PB5);
        }

        // write 

        // uart_putc()
    }

    // LED test
    /*
    DDRB = (1 < PB5);

    while(1){
        PORTB ^= (1 << PB5);
        _delay_ms(500);
   }
   */
}

ISR(USART_RX_vect){
    rcv_rdy = 1;
}