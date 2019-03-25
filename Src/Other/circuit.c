#include "run.h"
#include "variable.h"
#include "maze.h"
#include "mode.h"
#include "targetGenerator.h"
#include "tim.h"
#include "led.h"
#include "function.h"
#include "timer.h"
#include "logger.h"

void circuit( void )
{
  setPIDGain( &translation_gain, 2.6f, 45.0f, 0.0f ); 
  setPIDGain( &rotation_gain, 0.50f, 60.0f, 0.0f );
  setSenDiffValue( 60 ); 
  startAction();
  setControlFlag( 0 );
  setLogFlag( 0 );
  funControl( FUN_ON );
  waitMotion( 1000 );
  setLogFlag( 1 );
  setControlFlag( 1 );
  
  sidewall_control_flag = 1;
  setStraight( 220.0f, 20000.0f, 1700.0f, 0.0f, 1700.0f );
  waitStraight();
  // 一回目
  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 13.0f, 20000.0f, 4400.0f, 1200.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 5.0f, 20000.0f, 3000.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 13.0f, 20000.0f, 4500.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 5.0f, 20000.0f, 3000.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  // 一回目
  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 13.0f, 20000.0f, 4500.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 5.0f, 20000.0f, 3000.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 13.0f, 20000.0f, 4500.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 5.0f, 20000.0f, 3000.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  // 一回目
  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 13.0f, 20000.0f, 4500.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 5.0f, 20000.0f, 3000.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 13.0f, 20000.0f, 4500.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  sidewall_control_flag = 1;
  setStraight( ONE_BLOCK_DISTANCE * 5.0f, 20000.0f, 3000.0f, 1700.0f, 1700.0f );
  waitStraight();

  fullColorLedOut( 0x02 );
  sidewall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0 ) {
    translation_ideal.distance = 6.8f;
  }
  setStraight( 27.0f, 20000.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();
  setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
  waitRotation();
  sidewall_control_flag = 1;
  setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
  waitStraight();

  setStraight( 180.0f, 20000.0f, 1700.0f, 1700.0f, 0.0f );
  waitStraight();

  funControl( FUN_OFF );
  setControlFlag( 0 );
  while( getPushsw() == 0 );
  showLog();  
}