/*
 * File:   spi.c
 * Author: Tom Stenvall
 * Created on January 8, 2024, 10:55 PM
 */

#include "spi.h"


void SPI_ini(SPI_SPEED spi_speed){
    
    //Reset the MSSP module
    SSPCONbits.SSPEN = 0; 
    
    TRISC5 = 0; //Set SDO pin as output
    TRISC3 = 0; //Set SCK pin as output (master mode)  
    

    SSPSTATbits.SMP = 0; //Sample input data at the middle of data output time
    SSPSTATbits.CKE = 0; //Data written on the bus on the falling edge of SCK (SPI mode 2)
            

    SSPCON = spi_speed; //SPI to master mode with the desired speed
    SSPCONbits.CKP = 1; //Idle state is high level (SPI mode 2)
    
    
    SSPEN = 1; //Enable MSSP;
}

uint8_t SPI_transfer(uint8_t data){
    
    SSPBUF = data;
    while(!BF);
    return SSPBUF;
    
}