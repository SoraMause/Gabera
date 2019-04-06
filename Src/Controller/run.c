#include "run.h"

void calctrapezoid( t_trapezoid *trape, float distance, float accele, 
                float max_vel, float start_vel, float end_vel )
{
  // 後ろに下がる場合はフラグを制御してモーターの回転方向を変える。
  // 加速度は常にプラス
  // 加速度に0.0fを入れた場合は等速
  trape->back_rightturn_flag = 0;
  
  if ( distance < 0.0f ){
    trape->back_rightturn_flag = 1;
    distance = -1.0f * distance;
  } 

  trape->accel = accele;
  trape->distance = distance;
  trape->max_velocity = max_vel;
  trape->start_velocity = start_vel;
  trape->end_velocity = end_vel;

  if ( accele != 0.0f ){
    trape->accele_distance = ( max_vel * max_vel - start_vel * start_vel ) / ( 2.0f * accele );
    trape->decele_distance = ( max_vel * max_vel - end_vel * end_vel ) / ( 2.0f * accele );
  } else {
    trape->accele_distance = 0.0f;
    trape->decele_distance = 0.0f;
  }

}

void controlAccele( t_run *ideal, t_trapezoid *trape )
{
  if ( ideal->distance < trape->accele_distance ){
    if ( ideal->velocity < trape->max_velocity ){
      ideal->accel = trape->accel;
    } else {
      ideal->accel = 0.0f;
      ideal->velocity = trape->max_velocity;
    }
  } else if ( ideal->distance < trape->distance - trape->decele_distance ){
    if ( ideal->velocity < trape->max_velocity ){
      ideal->accel = trape->accel;
    } else {
      ideal->accel = 0.0f;
      ideal->velocity = trape->max_velocity;
    }
  } else if ( ideal->velocity > trape->end_velocity ){
    ideal->accel = -trape->accel;
  } else {
    ideal->accel = 0.0f;
    ideal->velocity = trape->end_velocity;
    trape->run_flag = 0;
  }
}

void integral( t_run *run_data )
{
  run_data->velocity += run_data->accel * dt;
  run_data->distance += run_data->velocity * dt + run_data->accel * dt * dt / 2.0f;
}

void integralDistance( float *velocity, float *distance )
{
  *distance += *velocity * dt;
}

void setStraight( float distance, float accele, float max_vel, float start_vel, float end_vel )
{
  calctrapezoid( &translation_trape_param,distance, accele, max_vel, start_vel, end_vel );
  // wall control on
  // もし壁あてなら壁制御切るなりなんなりする
  translation_trape_param.run_flag = 1;
  translation_ideal.velocity = translation_trape_param.start_velocity;

}

void waitStraight( void )
{
  while( translation_trape_param.run_flag == 1 ){
    // 動作終了までメインは動作を待機状態にする
  }

  // reset ideal data
  translation_ideal.accel = 0.0f;
  translation_ideal.distance = 0.0f;
  translation_ideal.velocity = translation_trape_param.end_velocity;
  
  // reset trape back turn flag
  translation_trape_param.back_rightturn_flag = 0;
  
  // reset pid parameter 
  run_left_deviation.difference = 0.0f;
  run_left_deviation.cumulative = 0.0f;
  run_right_deviation.difference = 0.0f;
  run_right_deviation.cumulative = 0.0f;

  sidewall_control_flag = 0;  // 壁制御は切る
  frontwall_control_flag = 0;
  dirwall_control_flag = 0;

  rotation_trape_param.back_rightturn_flag = 0;
  
}

void setRotation( float angle, float angular_accele, float max_angular_vel, float center_velocity )
{
  calctrapezoid( &rotation_trape_param, angle, angular_accele, max_angular_vel, 0.0f, 0.0f );
  // 壁制御OFF
  rotation_ideal.velocity = 0.0f;
  translation_ideal.accel = 0.0f;
  translation_ideal.velocity = center_velocity;
  
  rotation_trape_param.run_flag = 1;
}

void waitRotation( void )
{
  while( rotation_trape_param.run_flag == 1 ){
    // 動作終了までメインは動作を待機状態にする
  }

  // 移動距離をリセット
  translation_ideal.distance = 0.0f;
  // 回転方向の理想値をそれぞれリセット
  rotation_ideal.distance = 0.0f;
  rotation_ideal.accel = 0.0f;
  rotation_ideal.velocity = 0.0f;
  
  // trape関連をリセット
  rotation_trape_param.back_rightturn_flag = 0;
}

void waitMotion( volatile int32_t wait_time )
{
  cnt_motion = 0;
  wait_flag= 1;
  
  rotation_deviation.cumulative = 0.0f;

  while( cnt_motion < wait_time ){
    // 動作をしない

  }
  rotation_deviation.cumulative = 0.0f;
  wait_flag = 0;

}

void waitSlaromOut( void )
{
  while( translation_trape_param.run_flag == 1 ){
    // 動作終了までメインは動作を待機状態にする
    if ( sen_front.now > 220 ) break;
  }


  if ( sen_front.is_wall == 1 ){
    while( sen_front.now < 220 );
  }

  // reset ideal data
  translation_ideal.accel = 0.0f;
  translation_ideal.distance = 0.0f;
  translation_ideal.velocity = translation_trape_param.end_velocity;
  
  // reset trape back turn flag
  translation_trape_param.back_rightturn_flag = 0;
  
  // reset pid parameter 
  run_left_deviation.difference = 0.0f;
  run_left_deviation.cumulative = 0.0f;
  run_right_deviation.difference = 0.0f;
  run_right_deviation.cumulative = 0.0f;

  sidewall_control_flag = 0;  // 壁制御は切る
  frontwall_control_flag = 0;

  rotation_trape_param.back_rightturn_flag = 0;
  
}

void waitSearchStraight( void )
{
  while( translation_trape_param.run_flag == 1 ){
    // 動作終了までメインは動作を待機状態にする
    if ( sen_front.now > 220 ) break;
  }

  if ( sen_front.is_wall == 1 ){
    while( sen_front.now < 220 );
  }

  // reset ideal data
  translation_ideal.accel = 0.0f;
  translation_ideal.distance = 0.0f;
  translation_ideal.velocity = translation_trape_param.end_velocity;
  
  // reset trape back turn flag
  translation_trape_param.back_rightturn_flag = 0;
  
  // reset pid parameter 
  run_left_deviation.difference = 0.0f;
  run_left_deviation.cumulative = 0.0f;
  run_right_deviation.difference = 0.0f;
  run_right_deviation.cumulative = 0.0f;

  sidewall_control_flag = 0;  // 壁制御は切る
  frontwall_control_flag = 0;
  dirwall_control_flag = 0;

  rotation_trape_param.back_rightturn_flag = 0;
  
}