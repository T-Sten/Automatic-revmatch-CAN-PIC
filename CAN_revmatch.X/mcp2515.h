
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
    BUS_SPEED_95KBPS,
    BUS_SPEED_500KBPS
}BUS_SPEED;

#define SETTING_95KBPS_CNF1 0x01
#define SETTING_95KBPS_CNF2 0x35
#define SETTING_95KBPS_CNF3 0x00

#define SETTING_500KBPS_CNF1 0x00
#define SETTING_500KBPS_CNF2 0x0A
#define SETTING_500KBPS_CNF3 0x00

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

void MCP2515_setfilter(Mcp2515*, uint8_t filter_n, uint16_t id);
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

