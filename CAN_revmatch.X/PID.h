
/* 
 * File:   PID.h
 * Author: Tom Stenvall
 * Comments:
 * Revision history: 
 */

#ifndef PID_H
#define	PID_H

#include <xc.h>  


//Initialise pid with kp, ki and kd values
void PID_ini(float new_kp, float new_ki, float new_kd);

//Resets the static variables of the PID loop, use this when starting a new PID loop
void PID_reset();

//Calculates and returns the PID value
int16_t PID_calculate(int16_t target_value, int16_t current_value, uint32_t time_ms, int16_t *components);

//Same as PID_calculate but components of the PID value will be written to a array
int16_t PID_debug(int16_t target_value, int16_t current_value, uint32_t time_ms, int16_t *components);


#endif

