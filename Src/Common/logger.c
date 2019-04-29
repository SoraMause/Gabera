#include "logger.h"

#include "variable.h"

#include <stdio.h>

#include "run.h"

#include "led.h"

t_log_data logger;

static int8_t log_flag = 0;

static int16_t log_count = 0;

void log_init( void )
{
  log_count = 0;
  for ( int i = 0; i < LOG_NUMBER; i++ ){
    logger.sensor_left[i] = 0;
    logger.sensor_right[i] = 0;
    //logger.sensor_front[i] = 0;
    logger.trans_ideal_vel[i] = 0;
    //logger.trans_vel[i] = 0;
    //logger.rotation_ideal[i] = 0;
    //logger.rotation_vel[i] = 0;
    //logger.batt_data[i] = 0;
  }
}

void setLog( void )
{
  if ( log_flag == 1 ){
    if ( log_count < LOG_NUMBER ){
      logger.sensor_left[log_count] = sen_l.now;
      logger.sensor_right[log_count] = sen_r.now;
      //logger.sensor_front[log_count] = sen_front.now;
      logger.trans_ideal_vel[log_count] = (int16_t)translation_ideal.velocity;
      //logger.trans_vel[log_count] = (int16_t)right_real.velocity;
      //logger.trans_ideal_vel[log_count] = (int16_t)rotation_ideal.velocity;
      //logger.trans_vel[log_count] = (int16_t)rotation_real.velocity;
      //logger.rotation_ideal[log_count] = (int16_t)rotation_ideal.velocity;
      //logger.rotation_vel[log_count] = (int16_t)rotation_real.velocity;
      //logger.batt_data[log_count] = (int8_t)(batt_monitor * 10.0f);
      logger.trans_dis[log_count] = translation_ideal.distance;
      log_count++;
    } else {
      log_flag = 0;
    }
  }

}

void showLog( void )
{
  printf( "sen l, sen r, ideal vel, real vel, ideal_dis\r\n");
  for ( int i  = 0; i < log_count; i++ ){
    printf( "%d,%d,%d,%f\r\n",logger.sensor_left[i], logger.sensor_right[i],
            logger.trans_ideal_vel[i], logger.trans_dis[i] );

    certainLedOut( ( i & 0x04 ) );
  }

  certainLedOut( 0x00 );
  waitMotion( 2000 );
  
}

void setLogFlag( int8_t _flag )
{
  log_flag = _flag;
}