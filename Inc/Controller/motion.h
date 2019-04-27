#ifndef __MOTION_H
#define __MOTION_H

#include "variable.h"

void setSlaromOffset( t_slarom_parameter *slarom, float left_in, float left_out, float right_in, 
                      float right_out, float ang_accel, float max_ang_vel );
                      
void setNormalRunParam( t_normal_param *param, float accel, float max_vel );

void adjFront( float accel, float run_vel );
void adjBack( void );
void straightOneBlock( float run_vel );
void straightHalfBlockStop( float accel , float run_vel );
void straightHalfBlockStart( float accel , float run_vel );

void pivoTurnLeft( float accel, float run_vel );
void pivoTurnRight( float accel, float run_vel );
void pivoTurn180( float accel, float run_vel );

void slaromLeft( float run_vel );
void slaromRight( float run_vel );

// 最短用
void runStraight( float accel , float distance, float start_vel, float run_vel, float end_vel );

void slaromCenterLeft( float accel );
void slaromCenterRight( float accel );

void slaromCenterLeft180( float accel );
void slaromCenterRight180( float accel );

void slaromCenterLeft45( float accel );
void slaromCenterRight45( float accel );

void slaromCenterLeft135( float accel );
void slaromCenterRight135( float accel );

void slaromLeftV90( void );
void slaromRightV90( void );

void slaromReturnDiaLeft45( float accel );
void slaromReturnDiaRight45( float accel );

void slaromReturnDiaLeft135( float accel );
void slaromReturnDiaRight135( float accel );
#endif /* __MOTION_H */