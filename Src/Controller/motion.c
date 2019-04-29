#include "motion.h"

#include "run.h"

#include "maze.h"

void setSlaromOffset( t_slarom_parameter *slarom, float left_in, float left_out, float right_in, 
                      float right_out, float ang_accel, float max_ang_vel )
{
  slarom->left.in = left_in;
  slarom->left.out = left_in;
  slarom->right.in = right_in;
  slarom->right.out = right_out;
  slarom->angular_accel = ang_accel;
  slarom->max_angular_velocity = max_ang_vel;
}

void setNormalRunParam( t_normal_param *param, float accel, float max_vel )
{
  param->accel = accel;
  param->velocity = max_vel;
}

void adjFront( float accel, float run_vel )
{ 
  sidewall_control_flag = 1;  // 壁制御有効
  setStraight( ADJ_FRONT_DISTANCE, accel, run_vel, 0.0f, run_vel );
  waitSearchStraight();
}

void adjBack( void )
{
  setStraight( -50.0f, 1000.0f, 150.0f, 0.0f, 0.0f );
  waitStraight();
}

void straightOneBlock( float run_vel )
{
  sidewall_control_flag = 1;  // 壁制御有効
  setStraight( ONE_BLOCK_DISTANCE, 0.0f, run_vel, run_vel, run_vel );
  waitSearchStraight();
}

void straightHalfBlockStop( float accel , float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  frontwall_control_flag = 1;
  setStraight( HALF_BLOCK_DISTANCE, accel, run_vel, run_vel, 0.0f );
  waitStraight(); 
  waitMotion( 300 );
}

void straightHalfBlockStart( float accel , float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  frontwall_control_flag = 1;
  setStraight( HALF_BLOCK_DISTANCE, accel, run_vel, 0.0f, run_vel );
  waitStraight(); 
}

void pivoTurnLeft( float accel, float run_vel )
{
  setRotation( 90.0f, accel, run_vel, 0.0f );
  waitRotation();
  waitMotion( 300 );
}

void pivoTurnRight( float accel, float run_vel )
{
  setRotation( -90.0f, accel, run_vel, 0.0f );
  waitRotation();
  waitMotion( 300 );
}

void pivoTurn180( float accel, float run_vel )
{
  setRotation( 180.0f, accel, run_vel, 0.0f );
  waitRotation();
  waitMotion( 300 );
}

void slaromLeft( float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom500.left.in, 0.0f, run_vel, run_vel, run_vel );
  waitStraight();
  setRotation( 90.0f, slarom500.angular_accel, slarom500.max_angular_velocity, run_vel );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom500.left.out, 0.0f, run_vel, run_vel, run_vel );
  waitSlaromOut();
}

void slaromRight( float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom500.right.in, 0.0f, run_vel, run_vel, run_vel );
  waitStraight();
  setRotation( -90.0f, slarom500.angular_accel, slarom500.max_angular_velocity, run_vel );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom500.right.out, 0.0f, run_vel, run_vel, run_vel );
  waitSlaromOut();
}

// to do 最短のやつを作りまくる
void runStraight( float accel , float distance, float start_vel, float run_vel, float end_vel )
{
  setStraight( distance, accel, run_vel, start_vel, end_vel );
  waitStraight(); 
}

// 中心から90度
void slaromCenterLeft( float accel )
{
  sidewall_control_flag = 1;
  while( sen_l.now > sen_l.threshold );
  if ( translation_ideal.distance < 15.0f ) translation_ideal.distance = 9.6f;
  setStraight( 35.0f, accel, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( 90.0f, 20000.0f, 900.0f, 1600.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 43.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

void slaromCenterRight( float accel )
{
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold );
  if ( translation_ideal.distance < 15.0f ) translation_ideal.distance = 8.0f;
  setStraight( 35.0f, accel, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1600.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 43.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

// 中心から180度
void slaromCenterLeft180( float accel )
{
  sidewall_control_flag = 1;
  while( sen_l.now > sen_l.threshold );
  if ( translation_ideal.distance < 15.0f ) translation_ideal.distance = 9.6f;
  setStraight( 35.0f, accel, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( 180.0f, 20000.0f, 1080.0f, 1600.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 45.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

void slaromCenterRight180( float accel )
{
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold );
  if ( translation_ideal.distance < 15.0f ) translation_ideal.distance = 8.0f;
  setStraight( 35.0f, accel, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( -180.0f, 20000.0f, 1080.0f, 1600.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 45.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();

}

// 中心から45度
void slaromCenterLeft45( float accel )
{
  sidewall_control_flag = 1;
  while( sen_l.now > sen_l.threshold );
  if ( translation_ideal.distance < 15.0f ) translation_ideal.distance = 9.6f;
  setStraight( 14.0f, accel, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( 45.0f, 25000.0f, 900.0f, 1600.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 58.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

void slaromCenterRight45( float accel )
{
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold );
  if ( translation_ideal.distance < 15.0f ) translation_ideal.distance = 8.0f;
  setStraight( 14.0f, accel, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( -45.0f, 25000.0f, 900.0f, 1600.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 58.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

// 中心から135度
void slaromCenterLeft135( float accel )
{
  sidewall_control_flag = 1;
  while( sen_l.now > sen_l.threshold );
  if ( translation_ideal.distance < 15.0f ) translation_ideal.distance = 9.6f;
  setStraight( 43.0f, accel, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( 135.0f, 30000.0f, 1200.0f, 1600.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 36.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

void slaromCenterRight135( float accel )
{
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold );
  if ( translation_ideal.distance < 15.0f ) translation_ideal.distance = 8.0f;
  setStraight( 43.0f, accel, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( -135.0f, 30000.0f, 1200.0f, 1600.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 36.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

// 斜め90度 ( V90 )
void slaromLeftV90( void )
{
  dirwall_control_flag = 1;
  while( sen_l.now > sen_l.threshold && translation_ideal.distance < 10.0f );
  if ( translation_ideal.distance < 10.0f ) translation_ideal.distance = 4.8f;
  setStraight( 11.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( 90.0f, 30000.0f, 1200.0f, 1600.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 21.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

void slaromRightV90( void )
{
  dirwall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 10.0f );
  if ( translation_ideal.distance < 10.0f ) translation_ideal.distance = 4.8f;
  setStraight( 11.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( -90.0f, 30000.0f, 1200.0f, 1600.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 21.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

// 斜めから復帰
void slaromReturnDiaLeft45( void )
{
  dirwall_control_flag = 1;
  while( sen_l.now > sen_l.threshold && translation_ideal.distance < 10.0f );
  if ( translation_ideal.distance < 10.0f ) translation_ideal.distance = 4.8f;
  setStraight( 47.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( 45.0f, 25000.0f, 900.0f, 1600.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 22.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

void slaromReturnDiaRight45( void )
{
  dirwall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 10.0f );
  if ( translation_ideal.distance < 10.0f ) translation_ideal.distance = 4.8f;
  setStraight( 47.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( -45.0f, 25000.0f, 900.0f, 1600.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 22.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

// 斜めから135度ターン復帰
void slaromReturnDiaLeft135( void )
{
  dirwall_control_flag = 1;
  while( sen_l.now > sen_l.threshold && translation_ideal.distance < 10.0f );
  if ( translation_ideal.distance < 10.0f ) translation_ideal.distance = 4.8f;
  setStraight( 28.0f, 0.0, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( 135.0f, 30000.0f, 1200.0f, 1600.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 53.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}

void slaromReturnDiaRight135( void )
{
  dirwall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 10.0f );
  if ( translation_ideal.distance < 10.0f ) translation_ideal.distance = 4.8f;
  setStraight( 28.0f, 0.0, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
  setRotation( -135.0f, 30000.0f, 1200.0f, 1600.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 53.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
  waitStraight();
}