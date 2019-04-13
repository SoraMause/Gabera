#include "timer.h"

#include "variable.h"

#include "tim.h"
#include "spi.h"
#include "adc.h"

#include "function.h"

#include "run.h"
#include "targetGenerator.h"

#include "led.h"
#include "buzzer.h"
#include "logger.h"

static int8_t control_flag = 0;
static int8_t failsafe_count;
static float gyro_z_prev = 0.0f;
static float gyro_z_amountofchange = 0.0f;

// timer.c intrrupt if ( wall_out_flag == 1 ) 動作

void interrupt( void )
{

  cnt_motion++;
  
  // ジャイロセンサーのデータ更新
  if ( MPU6500_calc_check() == 0 ) {
    MPU6500_z_axis_offset_calc();
  } else {
    rotation_real.velocity = MPU6500_read_gyro_z();
    gyro_z_amountofchange = rotation_real.velocity - gyro_z_prev;
    gyro_z_prev = rotation_real.velocity;
    accel_data = MPU6500_read_accel_x();
  }

  if ( control_flag == 1 ){  
    if ( ( translation_trape_param.back_rightturn_flag == 0 && translation_ideal.velocity >= 300 && ( left_real.velocity < 100.0f || right_real.velocity < 100.0f ) )   
        || gyro_z_amountofchange > 500.0f || gyro_z_amountofchange < -500.0f || ( translation_ideal.accel == 0.0f && (accel_data > 5.0f || accel_data< -5.0f) ) ){
        failsafe_count++;
        if ( failsafe_count > 5 ){
          failSafe_flag = 1;  
          fullColorLedOut( LED_WHITE );
          certainLedOut( LED_BOTH );
          setIrledPwm( IRLED_OFF );
          funControl( FUN_OFF );
          motorControl( 0, 0 );
        }
    } else {
      failsafe_count = 0;
    }
  }

  // エンコーダの値の取得 , 速度変換
  update_encoder( &enc_value, &left_real, &right_real );

  // 処理開始！

  if( control_flag == 1 && failSafe_flag == 0 ){
    // 壁制御
    sideWallControl();
    frontWallControl();

    //if ( wall_out_flag > 0 ) checkWallOut();
    
    if ( translation_trape_param.run_flag == 1 ){
      controlAccele( &translation_ideal, &translation_trape_param );
    }

    if ( rotation_trape_param.run_flag == 1 ){
      controlAccele( &rotation_ideal, &rotation_trape_param );
      integral( &rotation_ideal );  // 回転方向の計算
    }

    feedForwardTranslation( left_real.velocity, right_real.velocity, translation_ideal.accel, translation_ideal.velocity, 
                              &duty, batt_monitor, translation_trape_param.back_rightturn_flag);

    PIDControl( &translation_ideal, &left_real, &right_real, &run_left_deviation, &run_right_deviation,
            &translation_gain, &translation_trape_param, &duty, 0 );

    if ( translation_trape_param.back_rightturn_flag == 0 || translation_ideal.velocity > 100.0f ){
      PIDControl( &rotation_ideal, &rotation_real, &rotation_real, &rotation_deviation, &rotation_deviation, &rotation_gain,
                  &rotation_trape_param, &duty, 1 );
    }  

    integral(&translation_ideal); // 直線の距離計算

    integralDistance( &right_real.velocity, &right_real.distance );

    // motor 出力を行う
    motorControl( duty.left, duty.right );

    // 出力をリセット
    duty.left = 0;
    duty.right = 0;

  } else {
    motorControl( 0, 0 );
    if ( !(mode_counter & 0x80) ) {
      integralDistance( &right_real.velocity, &mode_distance );
    } else {
      integralDistance( &left_real.velocity, &mode_distance );
    }
  }

  setLog();

  buzzerOutPut();

}

void setControlFlag( int8_t _flag )
{
  control_flag = _flag;
  waitMotion( 3 );
}
