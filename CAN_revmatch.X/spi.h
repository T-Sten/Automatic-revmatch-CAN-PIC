/*
 * File:   spi.h
 * Author: Tom Stenvall
 * Created on January 8, 2024, 10:55 PM
 */

/*  
 * SPI Library for most PIC16F MCUs with the MSSP module
 */

#ifndef SPI_H
#define	SPI_H

#include <xc.h>


//SSPCON1 speed settings
typedef enum{
    SPI_SPEED_FOSC_4,   //0x00
    SPI_SPEED_FOSC_16,  //0x01     
    SPI_SPEED_FOSC_64,  //0x02
}SPI_SPEED;


//Initialises the MSSP module with the desired speed.
void SPI_ini(SPI_SPEED spi_speed);

//Handles both sending and receiving. Send dummy byte when receiving, returns incoming data.
uint8_t SPI_transfer(uint8_t data);


#endif	/* XC_HEADER_TEMPLATE_H */

