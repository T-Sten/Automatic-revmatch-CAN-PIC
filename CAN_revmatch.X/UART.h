/*
 * 
 * UART library by Tom Stenvall for 8-bit PIC MCUs, XC8 compiler
 * 
 */
#include <xc.h>


/*
 * UART_in() Initialises the USART module for UART with a desired baudrate
 * FORMULA TO CALCULATE SPBRG REGISTER VALUE:
 * sbrgreg_value = (Fosc / (64 * BAUDRATE)) - 1
*/   
void UART_ini(const uint8_t spbrgreg_value, uint8_t highspeed);

//Writes one character/byte on RX pin
void UART_send(uint8_t);

//writes a whole string and a newline character
//void UART_println(uint8_t*);

//Waits for reception to finish and returns a char/byte
uint8_t UART_read(void);

//returns 1 if something has been recieved
uint8_t UART_received(void);