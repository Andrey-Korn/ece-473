// sht21_functions.c       
// Jordan Baxter

#include <util/twi.h>
#include "sht21_functions.h"
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>



//********************************************************************************
//                              sht21_temp_convert
//
//given a temperature reading from an sht21, the address of a buffer
//array, and a format (deg F or C) it formats the temperature into ascii in 
//the buffer pointed to by the arguement.
//******************************************************************************
void sht21_temp_convert(char* to_str, uint16_t sense_temp, uint8_t f_not_c){
    //sense_temp &= ~3;                                 //Set two LSB to zero (per datasheet) before starting temp calculation
    float temp = (((float)sense_temp/65536)*175.72)-46.85;   //Convert read value to temperature using formula in datasheet
    if(f_not_c){
        //itoa((int)temp, temp_digits, 10); 
        dtostrf(temp, 5, 1, to_str);
    } else {
        temp = temp * 1.8 + 32;
        //itoa((int)temp, temp_digits, 10);
        dtostrf(temp, 5, 1, to_str);
    }

}//sht21_temp_convert
//******************************************************************************
