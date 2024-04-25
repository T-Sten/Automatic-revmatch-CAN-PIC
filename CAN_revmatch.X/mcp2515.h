
/* 
 * File: mcp2515.h
 * Author: Tom Stenvall
 * Comments: 
 * Revision history: 
 */

#ifndef MCP2515_H
#define MCP2515_H

#include "spi.h"

/*
 * NOTE: These predefined settings only support 8MHZ CLOCK
 */
typedef enum{
    BUS_SPEED_33KBPS,
    BUS_SPEED_95KBPS,
    BUS_SPEED_500KBPS
}BUS_SPEED;

#define SETTING_33KBPS_CNF1 0b1010
#define SETTING_33KBPS_CNF2 0b011001
#define SETTING_33KBPS_CNF3 0x00

#define SETTING_95KBPS_CNF1 0x01
#define SETTING_95KBPS_CNF2 0x35
#define SETTING_95KBPS_CNF3 0x00

#define SETTING_500KBPS_CNF1 0x00
#define SETTING_500KBPS_CNF2 0x02
#define SETTING_500KBPS_CNF3 0x00


//SPI instruction bytes
#define INS_RESET 0b11000000
#define INS_BITMOD 0b00000101
#define INS_READ 0b00000011
#define INS_WRITE 0b00000010
#define INS_LOADTX 0b01000000
#define INS_RTS 0b10000000

#define INS_READRX 0b10010000
#define INS_RXSTATUS 0b10110000

//Register definitions
#define REG_CNF1 0x2A
#define REG_CNF2 0x29
#define REG_CNF3 0x28

#define REG_CANINTE 0x2B
#define REG_CANINTF 0x2C
#define REG_CANCTRL 0x0F

#define REG_CANSTAT 0x0E

#define REG_RXB0CTRL 0x60
#define REG_RXB1CTRL 0x70

//Filter registers
#define REG_RXF0SIDH 0x00
#define REG_RXF0SIDL 0x01

#define REG_RXF1SIDH 0x04
#define REG_RXF1SIDL 0x05

#define REG_RXF2SIDH 0x08
#define REG_RXF2SIDL 0x09

#define REG_RXF3SIDH 0x10
#define REG_RXF3SIDL 0x11

#define REG_RXF4SIDH 0x14
#define REG_RXF4SIDL 0x15

#define REG_RXF5SIDH 0x18
#define REG_RXF5SIDL 0x19

//Filter mask registers
#define REG_RXM0SIDH 0x20
#define REG_RXM0SIDL 0x21
#define REG_RXM1SIDH 0x24
#define REG_RXM1SIDL 0x25

//Interrupts
//These enumerations used for enabling and clearing interrupts
typedef enum{
    MERR = 0x80,    //Message error
    WAKI = 0x40,    //Wakeup on CAN activity
    ERRI = 0x20,    //Error interrupt (multiple sources)
    TX2I = 0x10,    //TX buffer 2 transmit completed
    TX1I = 0x08,    //TX buffer 1 transmit completed
    TX0I = 0x04,    //TX buffer 0 transmit completed
    RX1I = 0x02,    //RX buffer 1 received message
    RX0I = 0x01     //RX buffer 0 received message

}MCP_INTERRUPT;

//Operation modes for the setmode funcrtion
typedef enum {
    MODE_NORMAL = 0x00,
    MODE_SLEEP = 0x20,
    MODE_LOOPBACK = 0x40,
    MODE_LISTEN = 0x60,
    MODE_CONFIGURE = 0x80
}MCP_MODE;

//CLKOUT settings
typedef enum{
    CLKOUT_OFF = 0,
    CLKOUT_1 = 0x04,
    CLKOUT_2 = 0x05,
    CLKOUT_4 = 0x06,
    CLKOUT_8 = 0x07
}MCP_CLKOUT;

//SS/CS pin port options
typedef enum{
    PORT_A,
    PORT_B,
    PORT_C,
    PORT_D
}SS_PIN_PORT;

//Used in ini function
typedef enum{
    ROLLOVER_ENABLE,
    ROLLOVER_DISABLE
}MCP_ROLLOVER;


//attach function writes to an instance of this:
typedef struct{
    uint8_t _ss_pin_bitmask;
    volatile unsigned char *_ss_pin_port;
}Mcp2515;


typedef struct{
    uint8_t rtr;
    uint8_t len;    
    uint16_t id;
    uint8_t data[8];
}Can_frame;

typedef struct{
    uint32_t id;
    uint8_t len;
    uint8_t rtr;
    uint8_t data[];
}Can_extended_id_frame;


typedef struct{
    uint8_t filterhit : 3; 
    uint8_t rx0_full : 1;   
    uint8_t rx1_full : 1;
    uint8_t extended_id : 1;
    uint8_t remote_frame : 1;
    uint8_t rollover : 1;
}Mcp2515_rx_status;

typedef struct{
    
    uint8_t rx0_full : 1;
    uint8_t rx1_full : 1;
    uint8_t tx0_busy : 1;
    uint8_t tx0_sent : 1;
    uint8_t tx1_busy : 1;
    uint8_t tx1_sent : 1;
    uint8_t tx2_busy : 1;
    uint8_t tx2_sent : 1;
    
}Mcp2515_status;
   

void MCP2515_attach(Mcp2515*, SS_PIN_PORT, uint8_t ss_pin);
void MCP2515_ini(Mcp2515*, BUS_SPEED, uint8_t enable_rollover);
void MCP2515_reset(Mcp2515*);

void MCP2515_setmode(Mcp2515*,MCP_MODE);
void MCP2515_setclkout(Mcp2515*, MCP_CLKOUT);

uint8_t MCP2515_get_reg(Mcp2515*, uint8_t reg_address);
void MCP2515_set_reg(Mcp2515*, uint8_t reg_address, uint8_t data);

//Only set filters when in config mode
void MCP2515_setfilter(Mcp2515*, uint16_t *filtersIDs);
void MCP2515_disable_filters(Mcp2515*);

void MCP2515_load_tx(Mcp2515*, Can_frame*, uint8_t buffer);
void MCP2515_load_tx_ext(Mcp2515*, Can_extended_id_frame*, uint8_t buffer);

Mcp2515_status MCP2515_get_status(Mcp2515*);
Mcp2515_rx_status MCP2515_check_rx(Mcp2515*);
void MCP2515_read_rx(Mcp2515*, Can_frame*, uint8_t buffer);

void MCP2515_rts(Mcp2515*, uint8_t buffer);

void MCP2515_interrupt_clear(Mcp2515*, MCP_INTERRUPT);
void MCP2515_interrupt_enable(Mcp2515*, MCP_INTERRUPT);


#endif

