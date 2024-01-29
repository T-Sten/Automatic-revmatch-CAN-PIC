/*
 * CAN revmatch PIC16F OPEL Vectra C
 * 
 * File:   main.c
 * Author: Tom Stenvall
 *
 * Created on January 2, 2024, 4:30 PM
 */

#include "config.h"

#include <xc.h>
#include "spi.h"
#include "mcp2515.h"
#include "PID.h"


#define _XTAL_FREQ 8000000 //"CAN0" MCP2515 SQWOUT will drive the PIC16F877A clock

#define RELAY PORTBbits.RB0
#define CAN0_INT PORTBbits.RB4
#define CAN1_INT PORTBbits.RB5


//CAN0 IDs
#define SWHEEL_ID 0x175
#define CENTRLOCKING_ID 0x135

//CAN1 IDs
#define RPM_ID 0x110
#define SPEED_ID 0x410
#define CLUTCH_ID 0x300

//PID values
#define PID_P 0.06
#define PID_I 0.00001
#define PID_D 3.5

//Minimum gear to allow revmatch to
#define MIN_GEAR 2

#define PWM_DUTY CCPR1L

//Parses can data to global variables and sets driver input flags
void CAN0_ID_handler(Can_frame*);
void CAN1_ID_handler(Can_frame*);

//Estimates current gear from the current speed and RPM global variables,
//returns current gear or 0 for no match
uint8_t get_gear(); 

//Calculates target RPM for next lower gear, current gear as input parameter. 
//Gets current speed from global variable.
uint16_t get_target_rpm(uint8_t current_gear);

//PID control for the throttle, returns PWM duty cycle that meets the 1-3.6V HIGH pedal signal
uint8_t get_throttle_pwm(uint16_t target_rpm);

//CAN action sequences
void gong(); //Makes a short beep from the instrument cluster
void welcome_action(); //Shortly honks the horn 3 times, turns on highbeams for 2 seconds

enum MODE{
    IDLE,
    WAIT_CLUTCH,
    REVMATCH
};

union driver_input_flags{
    struct{
        uint8_t sWheelUp : 1;
        uint8_t sWheelDown : 1;
        uint8_t doorsUnlocked :1;
    };
    uint8_t all;  
};

enum MODE mode = IDLE;
union driver_input_flags flags;

uint32_t millis;

uint16_t current_rpm = 0;
uint8_t clutch_state, current_speed, current_gear;

//Create can instances and pointers
Mcp2515 can_0;
Mcp2515 can_1;
Mcp2515* can0 = &can_0;
Mcp2515* can1 = &can_1;

void __interrupt() isr(){
    if(CCP2IF){
        CCP2IF = 0;
        millis++;
    }
}

void main(void) {
    TRISBbits.TRISB0 = 0; //Relay output    
    RELAY = 0;
    
    PID_ini(PID_P, PID_I, PID_D);
    
    SPI_ini(SPI_SPEED_FOSC_16);
    
    TRISBbits.TRISB4 = 1;
    OPTION_REGbits.nRBPU = 0;//Enable pullups
    

    
    MCP2515_attach(can0, PORT_D, 2);
    MCP2515_attach(can1, PORT_D, 5);
    MCP2515_ini(can0, BUS_SPEED_95KBPS, 1);
    MCP2515_ini(can1, BUS_SPEED_95KBPS, 1);
    

    
    //Enable specific interrupts to drive the MCP2515 interrupt pin
    MCP2515_interrupt_enable(can0, RX0I | RX1I);   //Interrupts on reception to receive buffer 0 and 1
    MCP2515_interrupt_enable(can1, RX0I | RX1I);          //Interrupt on reception to receive buffer 0
    
    //Set ID filters
    MCP2515_setfilter(can0, 0, CENTRLOCKING_ID);
    MCP2515_setfilter(can0, 1, SWHEEL_ID);
    
    MCP2515_setfilter(can1, 2, CLUTCH_ID);
    MCP2515_setfilter(can1, 3, RPM_ID);
    MCP2515_setfilter(can1, 4, SPEED_ID);
    
    MCP2515_setmode(can0, MODE_NORMAL);
    MCP2515_setmode(can1, MODE_NORMAL);
    
    MCP2515_setclkout(can0, CLKOUT_1);
    MCP2515_setclkout(can1, CLKOUT_OFF);
    
    
    //PWM INI, uses timer 2
    PR2 = 99; //Period
    CCPR1L = 0; //0% duty - (0-100)
    TRISCbits.TRISC2 = 0; //output
    T2CONbits.T2CKPS = 0; //timer prescale to 0
    T2CONbits.TMR2ON = 1; //enable timer2
    CCP1CONbits.CCP1M = 0b1100; //PWM MODE    
    
    //TIMER1 INI (for milliseconds)
    //Prescaler is 0
    T1CONbits.T1CKPS = 0b01; //1:2 prescale from Fosc/4 = 1MHz
    T1CONbits.TMR1ON = 1;
    CCPR2 = 1000; //Interrupt every 1000Hz = every 1ms
    CCP2CONbits.CCP2M = 0b1011; //Compare mode, clears timer 1, sets CCP2IF
    
    //Enable interrupts for millis calculation
    CCP2IE = 1;
    PEIE = 1;
    GIE = 1;
    
    
    while(1){
        
        
        if(!CAN0_INT){//Read CAN0
            Mcp2515_rx_status status = MCP2515_check_rx(can0);
            
            Can_frame frame;
            if(status.rx0_full){
                
                MCP2515_read_rx(can0, &frame, 0);
                MCP2515_interrupt_clear(can0,RX0I);
            }
            else if(status.rx1_full){
                MCP2515_read_rx(can0, &frame, 1);
                MCP2515_interrupt_clear(can0,RX1I);
                CAN0_ID_handler(&frame);
            }  
        }
        
        if(!CAN1_INT){
            Mcp2515_rx_status status = MCP2515_check_rx(can0);
            
            Can_frame frame;
            if(status.rx0_full){
                
                MCP2515_read_rx(can0, &frame, 0);
                MCP2515_interrupt_clear(can0,RX0I);
                
                CAN1_ID_handler(&frame);
            }
            else if(status.rx1_full){
                MCP2515_read_rx(can0, &frame, 1);
                MCP2515_interrupt_clear(can0,RX1I);
                
                CAN1_ID_handler(&frame);
            }
        }
        
        
        switch (mode){
            case IDLE:
                if(flags.sWheelUp && get_gear() > MIN_GEAR && clutch_state < 1){
                    mode = WAIT_CLUTCH;
                    gong();
                }
                else if(flags.doorsUnlocked){
                    welcome_action();
                }
                break;
                
            case WAIT_CLUTCH:
                if(clutch_state > 0){
                    PID_reset();
                    PWM_DUTY = 20; //20% = 1V = 0% Throttle
                    RELAY = 1;
                    mode = REVMATCH;
                }
                break;
                
            case REVMATCH:
                if(clutch_state < 1 || flags.sWheelUp){
                    RELAY = 0;
                    mode = IDLE;
                    gong();
                }
                
                
                static uint16_t last_rpm;
                static uint8_t last_speed;
                
                if(current_rpm == last_rpm && current_speed == last_speed){ break; } //Since PID is time consuming
                last_rpm = current_rpm;
                last_speed = current_speed;
                
                uint8_t current_gear = get_gear();
                uint16_t target_rpm = get_target_rpm(current_gear);
                
                PWM_DUTY = get_throttle_pwm(target_rpm);  
                break;
        }
        
        flags.all = 0;
    }
    return;
}

uint8_t compareData(uint8_t data_1[],uint8_t data_2[], uint8_t size){
    for(uint8_t i=0; i<size; i++){
        if(data_1[i] != data_2[i]){
            return 0;
        }
    }
    
    return 1;
}

void CAN0_ID_handler(Can_frame* frame){
    const uint8_t doors_opened[4] = {0x60,0x6,0xC7,0x4D};
    
    switch(frame->id){
        
        case SWHEEL_ID:
            if(frame->data[5] == 0x10 && frame->data[6] == 0x1F){
                flags.sWheelUp = 1;
            }
            else if(frame->data[5] == 0x20 && frame->data[6] == 0x01){
                flags.sWheelDown = 1;
            }
        break;
        case CENTRLOCKING_ID:
            if(compareData(frame->data, doors_opened, 4)){
                flags.doorsUnlocked = 1;
            }
        break;
    }
}

void CAN1_ID_handler(Can_frame* frame){
    switch(frame->id){
        
        case RPM_ID:
            current_rpm = (frame->data[1] << 6) | ((frame->data[2] >> 2) & 0x3F);
        break;
        case CLUTCH_ID:
            if(frame->data[1] & 0x40){ //This is the clutch state bit
                clutch_state = 1;
            }
            else{
                clutch_state = 0;
            }
        break;
        case SPEED_ID:
            current_speed = (frame->data[1] << 1) | ((frame->data[2] >> 7) & 0x01);
        break;
        
        
    }
}

uint8_t get_throttle_pwm(uint16_t target_rpm){
    
    int16_t pid = PID_calculate(target_rpm, current_rpm, millis);
    
    uint8_t throttle;
    if(pid > 100){
        throttle = 100;
    }
    else if(pid < 0){
        throttle = 0;
    }
    else{
        throttle = (uint8_t)pid;
    }
    
    uint8_t pwm_duty = 20 + (throttle * 0.52);
    
    
    return pwm_duty;
}

uint8_t get_gear(){
    if(current_speed < 1){
        return 0;
    }
    uint8_t ratio = (uint8_t)(current_rpm / current_speed);
    
    if(ratio > 110 && ratio < 150)      {return 1;}
    else if(ratio > 60 && ratio < 75)   {return 2;}
    else if(ratio > 40 && ratio < 50)   {return 3;}
    else if(ratio > 28 && ratio < 35)   {return 4;}
    else if(ratio > 24 && ratio < 28)   {return 5;}
    
    return 0;
}

uint16_t get_target_rpm(uint8_t gear){
    const uint8_t gear_ratio_lookup[6] = {0,130,65,45,32,26}; //RPM-speed ratios for my OPEL Vectra C
    
    return current_speed * gear_ratio_lookup[gear - 1];
}

void gong(){
    Can_frame gongframe;
    gongframe.id = 0x270;
    gongframe.len = 5;
    gongframe.data[0] = 0x69;
    gongframe.data[1] = 0x5;
    gongframe.data[2] = 0x1E;
    gongframe.data[3] = 0x1;
    gongframe.data[4] = 0x33;
    
    MCP2515_load_tx(can0, &gongframe, 0);
    MCP2515_rts(can0, 0);
}

void welcome_action(){
    
    enum buffers{
        NO_INPUT,
        HORN,
        HIGHBEAMS
    };
    
    __delay_ms(100);
    
    Can_frame steering_column;
    steering_column.id = 0x175;
    steering_column.len = 8;
    steering_column.data[0] = 0x00; //No input
    steering_column.data[1] = 0x00;
    steering_column.data[2] = 0x00;
    steering_column.data[3] = 0x00;
    steering_column.data[4] = 0x99;
    steering_column.data[5] = 0x00;
    steering_column.data[6] = 0x00;
    steering_column.data[7] = 0x00;
    MCP2515_load_tx(can0, &steering_column, NO_INPUT);
    
    steering_column.data[0] = 0x10; //Horn
    MCP2515_load_tx(can0, &steering_column, HORN);
    
    steering_column.data[0] = 0x08; //Highbeams
    MCP2515_load_tx(can0, &steering_column, HIGHBEAMS);
    
    for(uint8_t i=0; i < 3; i++){
        MCP2515_rts(can0, HORN);
        __delay_ms(10);
        MCP2515_rts(can0, NO_INPUT);
        __delay_ms(100);
    }
    
    MCP2515_rts(can0, HIGHBEAMS);
    __delay_ms(2000);
    MCP2515_rts(can0, NO_INPUT);
}