#include "targetGenerator.h"

static float sidewall_control_value = 0.0f;
static float frontwall_control_value = 0.0f;

static int16_t diff_value = 10;

void setSenDiffValue( int16_t value )
{
  diff_value = value;
}

void setPIDGain( t_PID_param *param, float kp, float ki, float kd )
{
  param->kp = kp;
  param->ki = ki;
  param->kd = kd;  
}

void PIDControl( t_run *ideal, t_run *left, t_run *right, t_deviation *left_deviation, t_deviation *right_deviation, 
                t_PID_param *gain, t_trapezoid *trape, t_duty *duty, int8_t rotation_control )
{
  int32_t duty_left, duty_right;
  float left_error, right_error;
  float kp, ki, kd;
  float left_p, right_p;
  float left_i, right_i;
  float left_d, right_d;
  kp = gain->kp;
  ki = gain->ki;
  kd = gain->kd;

  if ( trape->back_rightturn_flag == 1 ){
    left->velocity = -1.0f * ( left->velocity + right->velocity ) / 2.0f;
    right->velocity = left->velocity;
  } else {
    left->velocity = ( left->velocity + right->velocity ) / 2.0f;
    right->velocity = left->velocity;
  }

  // 壁制御を入力
  if ( rotation_control == 1 ){
    left->velocity += sidewall_control_value;
    right->velocity += sidewall_control_value;
  } else {
    left->velocity += frontwall_control_value;
    right->velocity += frontwall_control_value;
  }

  // 現在の偏差を計算
  left_error = ( ideal->velocity - left->velocity );
  right_error = ( ideal->velocity - right ->velocity );

  left_p = left_error * kp;
  right_p = right_error * kp;

  // 積分値を計算
  left_deviation->cumulative += left_error * dt;
  right_deviation->cumulative += right_error * dt;

  left_i = left_deviation->cumulative * ki;
  right_i = right_deviation->cumulative * ki;

  left_d = ( left_error - left_deviation->difference ) / dt * kd;
  right_d = ( right_error - right_deviation->difference ) / dt * kd;

  // 一つ前の値を計算( 微分用 )
  left_deviation->difference = left_error;
  right_deviation->difference = right_error;

  duty_left = (int32_t)(left_p + left_i + left_d);
  duty_right = (int32_t)(right_p + right_i + right_d);

  if ( rotation_control == 1 ){
    duty_left = -1 * duty_left; // 回転方向逆にする 
  }

  if ( trape->back_rightturn_flag == 1 ){
    duty_left = -1 * duty_left;
    duty_right = -1 * duty_right;
  }

  duty->left += duty_left;
  duty->right += duty_right;

}

void sideWallControl( void )
{
  // sidewall_control_valueをいじる
  // kp については無理やり角速度になおせるような値を求める。
  // 横壁制御フラグが1のときのみ制御を行う
  
  float sen_error = 0.0f;

  if ( sidewall_control_flag == 1 && (sen_l.diff < diff_value) && (sen_r.diff < diff_value) && (translation_ideal.velocity > 300.0f) ){
    if ( sen_l.is_wall == 1 && sen_r.is_wall == 1 ){
      sen_error = (float)( ( sen_l.now - sen_l.reference ) - ( sen_r.now - sen_r.reference ) );
      if ( sen_error > 100.0f ){
        sen_error = 100.0f;
      } else if ( sen_error < -100.0f ){
        sen_error = -100.0f;
      }
      sidewall_control_value = sensor_gain.kp * sen_error;
    } else if ( sen_l.is_wall == 1 && sen_r.is_wall == 0 ){
      sen_error = (float)( sen_l.now - sen_l.reference );
      if ( sen_error > 100.0f ){
        sen_error = 100.0f;
      } else if ( sen_error < -100.0f ){
        sen_error = -100.0f;
      }
      sidewall_control_value = 2.0f * sensor_gain.kp * sen_error;
    } else if( sen_r.is_wall == 1 && sen_l.is_wall == 0 ){
      sen_error = (float)( sen_r.now - sen_r.reference );
      if ( sen_error > 100.0f ){
        sen_error = 100.0f;
      } else if ( sen_error < -100.0f ){
        sen_error = -100.0f;
      }
      sidewall_control_value = -2.0f * sensor_gain.kp * sen_error;
    } else {
      sidewall_control_value = 0.0f;
    }
  } else if ( dirwall_control_flag == 1 ){
    sidewall_control_value = 0.0f;
    // 4つのセンサのそれぞれの値の閾値を決めてそれに対して制御量を気持ち与える。
    // 2019 1/25 fl: , l: , fr: , r:
    if ( sen_fl.now > 120 && sen_fl.diff < 100 ){
      sidewall_control_value = (float)0.6f * ( sen_fl.now - 80 );
    } else if ( sen_l.now > 690 && sen_l.diff_1ms < 100 ){
      sidewall_control_value = (float)0.6f * ( sen_l.now - 660 );
    } else if ( sen_fr.now > 110 && sen_fr.diff < 100 ){
      sidewall_control_value = (float)-0.6f * ( sen_fr.now - 70 );
    } else if ( sen_r.now > 690 && sen_r.diff_1ms < 100 ){
      sidewall_control_value = (float)-0.6f * ( sen_r.now - 660 );
    }
  } else {
    sidewall_control_value = 0.0f;
  }

}

void frontWallControl( void )
{
  // 前壁については普通にゲイン調整して合わせること。
  // 前壁制御フラグが1のときのみ制御を行う
  if ( frontwall_control_flag == 1 && sen_front.is_wall == 1 && right_real.velocity < 200.0f ){
    frontwall_control_value = (float) 0.5f * (sen_front.now - sen_front.reference);
  } else {
    frontwall_control_value = 0.0f;
  }
}