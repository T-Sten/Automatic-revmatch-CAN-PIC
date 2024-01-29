
/* 
 * File:   PID.h
 * Author: Tom Stenvall
 * Comments:
 * Revision history: 
 */

#ifndef PID_H
#define	PID_H

#include <xc.h>  

void PID_ini(float new_kp, float new_ki, float new_kd);
void PID_reset();
int16_t PID_calculate(int16_t target_value, int16_t current_value, uint32_t time_ms);
int16_t PID_debug(int16_t target_value, int16_t current_value, uint32_t time_ms, int16_t *components);



#endif

