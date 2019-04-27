#ifndef __TARGET_GENERATOR_H
#define __TARGET_GENERATOR_H

#include <stdint.h>
#include "variable.h"
#include "config.h"

void setSenDiffValue( int16_t value );
void setPIDGain( t_PID_param *param, float kp, float ki, float kd );
void setFrontWallP( float _p_gain );

void PIDControl( t_run *ideal, t_run *left, t_run *right, t_deviation *left_deviation, t_deviation *right_deviation, 
                t_PID_param *gain, t_trapezoid *trape, t_duty *duty, int8_t rotation_control );
void sideWallControl( void );
void frontWallControl( void );

void feedForwardTranslation( float left_vel, float right_vel, float accel, float velocity, t_duty *duty, float vBat, uint8_t backright_flag );

#endif /* __TARGET_GENERATOR_H */