/*
 * CAN revmatch PIC16F OPEL Vectra C
 * 
 * File:   main.c
 * Author: Tom Stenvall
 *
 * Created on January 2, 2024, 4:30 PM
 */

#include <xc.h>


#include "config.h"

#include <stdlib.h>
#include <string.h>
//#include <stdio.h> //FOR TESTING (SPRINTF)

#include "spi.h"
#include "mcp2515.h"
#include "PID.h"
#include "UART.h"
#include "dash.h"

#define _XTAL_FREQ 8000000 //"CAN0" MCP2515 SQWOUT will drive the PIC16F877A clock

//HIGH SPEED (BRGH = 1) SPBRG = (Fosc / (16 * BAUDRATE)) - 1
#define SPBRG 4 //100000

#define UART_HIGHSPEED 1

#define RELAY PORTBbits.RB0
#define CAN1_INT PORTBbits.RB4
#define CAN0_INT PORTBbits.RB5

#define SERIAL_BUFFER_SIZE 32

//CAN0 IDs
#define SWHEEL_ID 0x175
#define CENTRLOCKING_ID 0x135
#define PARKSENSOR_ID 0x430

//CAN1 IDs
#define RPM_ID 0x110
#define SPEED_ID 0x410
#define CLUTCH_ID 0x300

/*Good PID values
P 0.06
I 0.00001
D 3.5
*/


//Minimum gear to allow revmatch to
#define MIN_GEAR 2
#define MAX_RPM 5000

#define PWM_DUTY CCPR1L

//Parses can data to global variables and sets driver input flags
void CAN0_ID_handler(Can_frame*);
void CAN1_ID_handler(Can_frame*);

//For interfacing with PC GUI app
void serial_sendCanFrame(uint8_t bus, Can_frame* frame);
void serial_sendConsoleMessage(char *string);
void serial_sendPIDData(uint16_t throttle, uint16_t targetRpm, int16_t *components);
void serial_finishPIDData();
void serial_sendPIDConfig(uint8_t *bytes);

//Sets P, I, D values from the EEPROM
void PID_ini_from_eeprom();

//Estimates current gear from the current speed and RPM global variables,
//returns current gear or 0 for no match
uint8_t get_gear(); 

//Calculates target RPM for next lower gear, current gear as input parameter. 
//Gets current speed from global variable.
uint16_t get_next_target_rpm(uint8_t current_gear);

//PID control for the throttle, returns PWM duty cycle that meets the 1-3.6V HIGH pedal signal
uint8_t get_throttle_pwm(uint16_t target_rpm);

//CAN action sequences, these are all BLOCKING FUNCTIONS!!
void gong(); //Makes a short beep from the instrument cluster
void welcome_action(); //Shortly honks the horn 3 times, turns on highbeams for 2 seconds
void mirror_fold(uint8_t pos); //Folds the mirror up or down for reversing

//Sets idle filters
void CAN_idle_filters();

//For validating of can frame data and serial commands, returns 1 if data is same
uint8_t compareData(uint8_t data_1[],uint8_t data_2[], uint8_t size);


enum MODE{
    IDLE,
    WAIT_CLUTCH,
    REVMATCH,
};

//Serial commands from GUI app
enum serial_command{
    PID_SNIFF_SET,
    SEND,
    FRAME_DUMP_SET,
    ID_DUMP_SET,
    FILTER_SNIFF_START,
    FILTER_SNIFF_END,
    THROTTLE,
    PID_TEST_START,
    PID_TEST_END,
    PID_CONFIG_SET,
    PID_CONFIG_READ
};

//Serial message type to GUI app
typedef enum{
    CANFRAME,
    CONSOLE,
    PID_DATA,
    PID_DATA_END,
    PID_CONFIG
}SERIAL_MESSAGE;

//Tasks set by the GUI app
struct serialTasks{
    uint8_t can_dump_can0 :1;
    uint8_t can_dump_can1 :1;
    
    uint8_t can_sniff :1;
    uint8_t id_dump_can0 : 1;
    uint8_t id_dump_can1 : 1;
    
    uint8_t PID_sniff: 1;
    uint8_t PID_test: 1;
    uint16_t PID_test_targetRpm;
};

//Serial buffer struct
struct serial{
    union{
        struct{
            char sof[3];
            uint8_t len;
            uint8_t command;
            uint8_t data[SERIAL_BUFFER_SIZE - 5];
        };
        uint8_t buffer[SERIAL_BUFFER_SIZE];
    };
    uint8_t received_n;
    uint8_t overflow;
};

union driver_input_flags{
    struct{
        uint8_t sWheelUp : 1;
        uint8_t sWheelDown : 1;
        uint8_t doorsUnlocked :1;
    };
    uint8_t all;  
};

struct modeTasks{
    uint8_t multipleRevMatch : 1;
};

enum MODE mode = IDLE;
union driver_input_flags driverInputFlags;
struct modeTasks modeTasks;
struct serial serial;
struct serialTasks serialTasks;
const char SOF[3] = {'S', 'O', 'F'}; //This is indicates the start of a serial frame to and from the GUI app

uint32_t millis = 0, last_ledsUpdate = 0, last_7segUpdate = 0;

uint16_t current_rpm = 0;
uint8_t clutch_state, current_speed, current_gear;
uint8_t last_gear;

uint8_t dashCleared = 0;

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
    
    //Reads UART
    if(RCIF){
        
        if(serial.received_n >= SERIAL_BUFFER_SIZE-2){
            serial.overflow = 1;
            serial.received_n = 0;
        }
        
        serial.buffer[serial.received_n] = RCREG;
        

        if(serial.received_n < 3 && serial.sof[serial.received_n] != SOF[serial.received_n]){
            serial.received_n = 0;
        }
        else{
            serial.received_n++;
        }


        RCIF = 0;            
    }
}

void main(void) {
    
    
    TRISBbits.TRISB0 = 0; //Relay output    
    RELAY = 0;
    
    SPI_ini(SPI_SPEED_FOSC_4);
    
    UART_ini(SPBRG, UART_HIGHSPEED);
    
    TRISBbits.TRISB4 = 1;
    OPTION_REGbits.nRBPU = 0;//Enable pullups
    
    MCP2515_attach(can0, PORT_D, 2);
    MCP2515_attach(can1, PORT_D, 3);
    MCP2515_ini(can0, BUS_SPEED_33KBPS, 1);
    MCP2515_ini(can1, BUS_SPEED_500KBPS, 1);
    
    MCP2515_setmode(can0, MODE_CONFIGURE);
    MCP2515_setmode(can1, MODE_CONFIGURE);
    
    //Set normal operation filters
    CAN_idle_filters();
    
    //Enable specific interrupts to drive the MCP2515 interrupt pin
    MCP2515_interrupt_enable(can0, RX0I | RX1I);   //Interrupts on reception to receive buffer 0 and 1, will wake up and start oscillator on bus activity
    MCP2515_interrupt_enable(can1, RX0I | RX1I);   //Interrupt on reception to receive buffer 0
    
    MCP2515_setmode(can0, MODE_NORMAL);
    MCP2515_setmode(can1, MODE_NORMAL);
    
    MCP2515_setclkout(can0, CLKOUT_1);
    MCP2515_setclkout(can1, CLKOUT_OFF);
    
    PID_ini_from_eeprom();
    
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
    
    //Enable interrupts for millis calculation, enable UART interrupt
    CCP2IE = 1;
    RCIE = 1;
    PEIE = 1;
    GIE = 1;
    
    serial_sendConsoleMessage("Stenvall's Revbot started...");
    
    
    while(1){
        
        
        //PID test started from the PC app
        if(serialTasks.PID_test){
            int16_t components[3];
            uint16_t throttle = PID_calculate(serialTasks.PID_test_targetRpm, current_rpm, millis, components);
            PWM_DUTY = get_throttle_pwm(throttle);
            
            static uint32_t lastPIDserial = 0;
            if(serialTasks.PID_sniff && millis > lastPIDserial + 50){
                lastPIDserial = millis;
                serial_sendPIDData(throttle, serialTasks.PID_test_targetRpm, components);
            }
        }
        
        //Clear dash if no RPM data received after 500 ms
        if(last_ledsUpdate + 500 < millis && !dashCleared){
            dashCleared = 1;
            dash_clear();
        }
        
        //UART intefrace with PC app
        if(serial.received_n > 3 && serial.received_n >= serial.len){   //Check for complete frame
            
            if(serial.command == SEND){
                Can_frame fr_send;
                uint8_t bus_select = serial.data[0];
                fr_send.id = (serial.data[1] << 8) | serial.data[2];
                fr_send.len = serial.data[3];

                uint8_t *data = &serial.data[4];
                for(uint8_t i=0; i < fr_send.len; i++){
                    fr_send.data[i] = data[i];
                }

                if(!bus_select){
                    MCP2515_load_tx(can0, &fr_send, 0);
                    MCP2515_rts(can0, 0);
                }
                else{
                    MCP2515_load_tx(can1, &fr_send, 0);
                    MCP2515_rts(can1, 0);
                }

            }
            else if(serial.command == THROTTLE){

                if(serial.data[0] > 0){
                    RELAY = 1;
                    PWM_DUTY = get_throttle_pwm((uint16_t)serial.data[0]);
                }
                else{
                    RELAY = 0;
                    PWM_DUTY = 0;
                }
            }
            else if(serial.command == PID_SNIFF_SET){
                serialTasks.PID_sniff = serial.data[0];
            }
            else if(serial.command == PID_TEST_START){
                serialTasks.PID_test = 1;
                serialTasks.PID_test_targetRpm = (serial.data[0] << 8) | serial.data[1];
                PID_reset();
                PWM_DUTY = 20;
                RELAY = 1;
            }
            else if(serial.command == PID_TEST_END){
                RELAY = 0;
                serialTasks.PID_test = 0;
                if(serialTasks.PID_sniff){
                    serial_finishPIDData();
                }
            }
            else if(serial.command == FRAME_DUMP_SET){
                if(serial.data[0] | serial.data[1]){
                    serialTasks.can_sniff = 1;
                    
                    MCP2515_disable_filters(can0);
                    MCP2515_disable_filters(can1);
                }
                else{
                    serialTasks.can_sniff = 0;
                    CAN_idle_filters();
                }
                serialTasks.can_dump_can0 = serial.data[0];
                serialTasks.can_dump_can1 = serial.data[1];
            }
            else if(serial.command == ID_DUMP_SET){
                
                serialTasks.id_dump_can0 = serial.data[0];
                serialTasks.id_dump_can1 = serial.data[1];                
                
                //We need to set can_sniff bit so that sending a can frame to serial is even considered
                //NOTE: received Frame BUS will be checked individually for each frame
                if(serialTasks.id_dump_can0 | serialTasks.id_dump_can1){
                    serialTasks.can_sniff = 1;
                    
                    MCP2515_disable_filters(can0);
                    MCP2515_disable_filters(can1);
                }
                else{
                    serialTasks.can_sniff = 0;

                    CAN_idle_filters();
                }
            }
            else if(serial.command == FILTER_SNIFF_START){
                serialTasks.can_sniff = 1;
                
                
                Mcp2515 *bus = can0;
                for(uint8_t i=0; i < 2; i++){
                    
                    uint16_t filterIDs[6];
                    for(uint8_t j=0; j < 6; j++){
                        uint8_t byte_index = 2*j + 6*i;
                        filterIDs[j] = (serial.data[byte_index] << 8) | serial.data[byte_index + 1];
                    }
                    MCP2515_setfilter(bus, filterIDs);
                    bus = can1;
                }
            }
            else if(serial.command == FILTER_SNIFF_END){
                serialTasks.can_sniff = 0;
                
                CAN_idle_filters();
            }
            else if(serial.command == PID_CONFIG_SET){
                union {
                    uint8_t bytes[4];
                    float floatValue;
                } PID[3];
                
                GIE = 0;
                for(uint8_t i=0; i < 3; i++){
                    
                    for(uint8_t j=0; j < 4; j++){
                        uint8_t byte_index = j + i*4;
                        
                        eeprom_write(byte_index, serial.data[byte_index]);
                        while(WR){};
                        EEIF = 0;
                        PID[i].bytes[j] = serial.data[byte_index];
                    }
                }
                GIE = 1;
                PID_ini(PID[0].floatValue, PID[1].floatValue, PID[2].floatValue);
            }
            else if(serial.command == PID_CONFIG_READ){
                uint8_t bytes[12];
                for(uint8_t i=0; i < 12; i++){
                    bytes[i] = eeprom_read(i);
                }
                
                serial_sendPIDConfig(bytes);
            }
            
            serial.received_n = 0;
        }
        
        //CAN interrupt, read can frame
        if(!CAN0_INT | !CAN1_INT){
            
            Mcp2515 *int_can;
            if(!CAN0_INT){
                int_can = can0;
            }
            else if(!CAN1_INT){
                int_can = can1;
            }
            
            Mcp2515_rx_status status = MCP2515_check_rx(int_can);
            if(status.rx0_full | status.rx1_full){
                
                uint8_t receive_buffer;
                MCP_INTERRUPT int_flag;
                if(status.rx0_full){
                    receive_buffer = 0;
                    int_flag = RX0I;
                }
                else{
                    receive_buffer = 1;
                    int_flag = RX1I;
                }
                
                Can_frame frame;
                MCP2515_read_rx(int_can, &frame, receive_buffer);
                MCP2515_interrupt_clear(int_can, int_flag);
                
                if(int_can == can0){
                    CAN0_ID_handler(&frame);
                }
                else{
                    CAN1_ID_handler(&frame);
                }
                
                if(serialTasks.can_sniff){ //GUI app CAN sniffing
                    uint8_t bus;
                    if(int_can == can0){
                        bus = 0;
                    }
                    else{
                        bus = 1;
                    }            
                    
                    if(serialTasks.id_dump_can0 | serialTasks.id_dump_can1){
                        frame.len = 0;  //if we are just dumping the ID set the length to 0
                        if(bus == 0 && serialTasks.id_dump_can0 || bus == 1 && serialTasks.id_dump_can1){ //Check for correct bus
                            serial_sendCanFrame(bus, &frame);
                        }
                    }
                    else if(serialTasks.can_dump_can0 | serialTasks.can_dump_can1){
                        if(bus == 0 && serialTasks.can_dump_can0 || bus == 1 && serialTasks.can_dump_can1){ //Check for correct bus
                            serial_sendCanFrame(bus, &frame);
                        }
                    }
                    else{
                        serial_sendCanFrame(bus, &frame);
                    }
                }
                

            }
        }
        
        switch (mode){
            case IDLE:
                if((driverInputFlags.sWheelUp || driverInputFlags.sWheelDown) && get_gear() > MIN_GEAR && clutch_state < 1){
                    
                    mode = WAIT_CLUTCH;
                    last_gear = get_gear();
                    gong();
                    if(driverInputFlags.sWheelDown){
                        modeTasks.multipleRevMatch = 1;
                    }
                }
                else if(driverInputFlags.doorsUnlocked){
                    welcome_action();
                }
                break;
                
            case WAIT_CLUTCH:
                if(driverInputFlags.sWheelDown | driverInputFlags.sWheelUp){ //Stops waiting for clutch
                    mode = IDLE;
                }
                else if(clutch_state == 1){
                    mode = REVMATCH;
                    PID_reset();
                    PWM_DUTY = 22; //22% = 1V(output) = 0% Throttle
                    RELAY = 1;
                }
                break;
                
            case REVMATCH:
                
                if((driverInputFlags.sWheelUp | driverInputFlags.sWheelDown) || current_rpm > MAX_RPM){ //Stop revmatching immediately
                    RELAY = 0;
                    gong();
                    mode = IDLE;
                }
                else if(clutch_state == 0){ //Clutch released, stop revmatch
                    RELAY = 0;
                    gong();
                    if(modeTasks.multipleRevMatch && get_gear() > MIN_GEAR){ //Check if we need and are allowed to revmatch for antoher gear
                        mode = WAIT_CLUTCH;
                    }
                    else{
                        mode == IDLE;
                        modeTasks.multipleRevMatch = 0;
                    }
                }
                
                if(serialTasks.PID_sniff && mode != REVMATCH){
                    __delay_ms(50);
                    serial_finishPIDData();
                }
                    
                static uint16_t last_rpm;
                static uint8_t last_speed;
                
                if(current_rpm == last_rpm && current_speed == last_speed){ break; } //Since PID is time consuming
                last_rpm = current_rpm;
                last_speed = current_speed;
                
                uint16_t target_rpm = get_next_target_rpm(last_gear);
                
                if(target_rpm > MAX_RPM){ //Don't exceed MAX RPM
                    RELAY = 0;
                    mode = IDLE;
                    gong();
                    break;
                }
                
                int16_t components[3];
                uint16_t throttle = PID_calculate(target_rpm, current_rpm, millis, components);
                PWM_DUTY = get_throttle_pwm(throttle);
                
                if(serialTasks.PID_sniff){ //PID data for GUI app
                    serial_sendPIDData(throttle, target_rpm, components);
                }
                
                break;
        }
        
        driverInputFlags.all = 0;
    }
    return;
}


void serial_startSend(uint8_t len, SERIAL_MESSAGE message){
    UART_send(SOF[0]);
    UART_send(SOF[1]);
    UART_send(SOF[2]);  
    UART_send(len + 5);
    UART_send(message);
}

void serial_finishPIDData(){
    serial_startSend(0, PID_DATA_END);
}

void serial_sendPIDData(uint16_t throttle, uint16_t targetRpm, int16_t *components){
    serial_startSend(9, PID_DATA);
    
    UART_send(current_rpm >> 8); //RPM high
    UART_send(current_rpm); //RPM LOW
    
    UART_send(targetRpm >> 8); //RPM target high
    UART_send(targetRpm); //RPM target LOW
    
    UART_send(throttle >> 8); //Throttle
    UART_send(throttle);
    
    UART_send(components[0]);
    UART_send(components[1]);
    UART_send(components[2]);
}

void serial_sendConsoleMessage(char *string){
    uint8_t len = strlen(string);
    serial_startSend(len, CONSOLE);
    for(uint8_t i=0; i < len; i++){
        UART_send(string[i]);
    }
}

void serial_sendCanFrame(uint8_t bus, Can_frame* frame){
    
    serial_startSend(frame->len + 4, CANFRAME);
    
    UART_send(bus);
    UART_send(frame->id >> 8);
    UART_send(frame->id);
    UART_send(frame->len);
    
    for(uint8_t i=0; i < frame->len; i++){
        UART_send(frame->data[i]);
    }
}

void serial_sendPIDConfig(uint8_t *bytes){
    serial_startSend(12, PID_CONFIG);
    
    for(uint8_t i=0; i < 12; i++){
        UART_send(bytes[i]);
    }
}

uint8_t compareData(uint8_t data_1[],uint8_t data_2[], uint8_t size){
    for(uint8_t i=0; i<size; i++){
        if(data_1[i] != data_2[i]){
            return 0;
        }
    }
    
    return 1;
}

void PID_ini_from_eeprom(){
    union {
        uint8_t bytes[4];
        float floatValue;
    } PID[3];
    
    for(uint8_t i=0; i < 3; i++){

        for(uint8_t j=0; j < 4; j++){
            PID[i].bytes[j] = eeprom_read(j + i*4);
        }
    }
    
    PID_ini(PID[0].floatValue, PID[1].floatValue, PID[2].floatValue);
}

void CAN_idle_filters(){
    
    uint16_t idleFilters[2][6] = 
    {
        {CENTRLOCKING_ID, SWHEEL_ID, PARKSENSOR_ID, 0x00, 0x00, 0x00},
        {CLUTCH_ID, SPEED_ID, RPM_ID, 0x00, 0x00, 0x00}
    };
    
    MCP2515_setfilter(can0, idleFilters[0]);
    MCP2515_setfilter(can1, idleFilters[1]);
}

void CAN0_ID_handler(Can_frame* frame){
    const uint8_t doors_opened[4] = {0x60,0x6,0xC7,0x4D};
    
    switch(frame->id){
        
        case SWHEEL_ID:
            if(frame->data[5] == 0x10 && frame->data[6] == 0x1F){
                driverInputFlags.sWheelUp = 1;
            }
            else if(frame->data[5] == 0x20 && frame->data[6] == 0x01){
                driverInputFlags.sWheelDown = 1;
            }
        break;
        case CENTRLOCKING_ID:
            if(compareData(frame->data, doors_opened, 4)){
                driverInputFlags.doorsUnlocked = 1;
            }
        break;
        case PARKSENSOR_ID:
            if(frame->data[0] == 0x73){
                mirror_fold(1);
            }
            else if(frame->data[0] == 0x03){
                mirror_fold(0);
            }
        break;
    }
}

void CAN1_ID_handler(Can_frame* frame){
    switch(frame->id){
        
        case RPM_ID:
            current_rpm = (frame->data[1] << 6) | ((frame->data[2] >> 2) & 0x3F);
            
            
            if(millis >= last_ledsUpdate + 100){
                last_ledsUpdate = millis;
                
                if(mode == WAIT_CLUTCH && !dashCleared){
                    
                    dashCleared = 1;
                    dash_setLeds(0);
                }
                else{
                    dashCleared = 0;
                    dash_setLeds(current_rpm);
                }
            }
        break;
        case CLUTCH_ID:
            if(frame->data[1] & 0x40){ //This is the clutch state bit
                clutch_state = 1; //Clutch pressed down
            }
            else{
                clutch_state = 0; //Clutch released
            }
        break;
        case SPEED_ID:
            current_speed = (frame->data[1] << 1) | ((frame->data[2] >> 7) & 0x01);
            if(millis >= last_7segUpdate + 500){
                last_7segUpdate = millis;
                dash_set7seq(get_gear());
            }
        break;
    }
}

uint8_t get_throttle_pwm(uint16_t pid){
    
    uint8_t throttle;
    if(pid > 100){
        throttle = 100;
    }
    else{
        throttle = (uint8_t)pid;
    }
    
    //PWM circuit has a voltage division circuit so the rms voltage needs to be 1.1 time higher than the required output voltage
    //      1 / (4700ohm / (4700ohm + 470ohm)) = 1.1
    
    
    uint8_t pwm_duty = (22 + (throttle * 0.55)); 
    
    
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

uint16_t get_next_target_rpm(uint8_t gear){
    const uint8_t gear_ratio_lookup[6] = {0,130,65,45,32,26}; //RPM-speed ratios for my OPEL Vectra C
    
    return current_speed * gear_ratio_lookup[gear - 1];
}

void mirror_fold(uint8_t enable){
    
    static uint8_t mirrorFolded = 0;
    
    if(enable == mirrorFolded){
        return;
    }

    Can_frame frame;
    frame.id = 0x405;
    frame.len = 6;
    
    frame.data[0] = 0x00;
 
    if(enable){
        mirrorFolded = 1;
        frame.data[1] = 0x08;
    }
    else{
        mirrorFolded = 0;
        frame.data[1] = 0x04;
    }
    frame.data[2] = 0x00;   
    frame.data[3] = 0x10;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    
    MCP2515_load_tx(can0, &frame, 0);
    
    frame.data[1] = 0x00;
    MCP2515_load_tx(can0, &frame, 1);
    
    for(uint8_t i=0; i < 40; i++){
        MCP2515_rts(can0, 0);
        __delay_ms(50);
    }
    MCP2515_rts(can0, 1);
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