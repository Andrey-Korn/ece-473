// sht21_functions.h 
// Jordan Baxter 11.28.19
//special defines and functions for the sht21 humidity and temperature sensor

//**This header file and the matching .c file were started from LM73 functions files
//written by Roger Traylor.**

#define F_CPU 16000000UL
#define TRUE  1
#define FALSE 0

#include "twi_master.h"  //my TWCR_START, STOP, NACK, RNACK, SEND
//use the status codes in: usr/local/AVRMacPack/avr-3/include/util/twi.h

#define SHT21_ADDRESS 0x80                      //SHT21, address pin floating
#define SHT21_WRITE (SHT21_ADDRESS | TW_WRITE)  //LSB is a zero to write
#define SHT21_READ  (SHT21_ADDRESS | TW_READ)   //LSB is a one to read
#define SHT21_TEMP_HOLD         0xE3            //SHT21 temperature command, hold clk low while processing
#define SHT21_RH_HOLD           0xE5            //SHT21 relative humidity command, hold clk low while processing
#define SHT21_TEMP_NO_HOLD      0xF3            //SHT21 temp command, do not hold clk low while processing, requires master to poll
#define SHT21_RH_NO_HOLD        0xF5            //SHT21 relative humidity command, do not hold clk low while processing, requires master to poll

//special functions for lm73 temperature sensor
void sht21_temp_convert(char* to_str, uint16_t sense_temp, uint8_t f_not_c);
  
