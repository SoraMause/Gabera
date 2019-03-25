#ifndef __RUN_H
#define __RUN_H

#include "config.h"
#include "variable.h"

void calctrapezoid( t_trapezoid *trape, float distance, float accele, 
                float max_vel, float start_vel, float end_vel );
void controlAccele( t_run *ideal, t_trapezoid *trape );
void integral( t_run *run_data );
void integralDistance( float *velocity, float *distance );

void setStraight( float distance, float accele, float max_vel, float start_vel, float end_vel );
void waitStraight( void );

void setRotation( float angle, float angular_accele, float max_angular_vel, float center_velocity );
void waitRotation( void );

void waitMotion( volatile int32_t wait_time );

void waitSlaromOut( void );
void waitSearchStraight( void );
#endif /* __RUN_H */