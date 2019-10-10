// count.c 
//Sanity checker. Binary counting pattern on PORTB LEDs.
//

#include <avr/io.h>
#include <util/delay.h>

int main()
{
uint16_t i;
DDRB = 0xFF;  //set port B to all outputs

while(1){
  PORTB++;  // PORTB increments by one
  for(i=0; i<=4; i++) {_delay_ms(100);} //0.5 sec delay
  }//while
}//main
