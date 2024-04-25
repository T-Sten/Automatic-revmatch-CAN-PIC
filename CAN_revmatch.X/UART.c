

#ifndef UART_H
#define UART_H

#include "UART.h"
#include <string.h>

#include <xc.h>


void UART_ini(const uint8_t spbrgreg_value, uint8_t highspeed){
    TXSTA=0x20; //BRGH=0, TXEN = 1, Asynchronous Mode, 8-bit mode
    TXSTAbits.BRGH = highspeed;
    RCSTA=0b10010000; //Serial Port enabled,8-bit reception
    
    SPBRG = spbrgreg_value;
    TXIF=0;
    RCIF=0;
}

void UART_println(uint8_t string[]){
    uint16_t len = strlen(string);
    for(int i=0; i < len; i++){
        UART_send(string[i]);
    }
    UART_send('\n');
}

void UART_send(uint8_t a)
{
    TXREG = a; //fill transmit register
    while(!TRMT); //wait for peripheral register (PIR1) to be empty (0)
}

uint8_t UART_read()
{
    RCIF=0;
    return RCREG;
}

uint8_t UART_received(){
    return RCIF;
}

#endif