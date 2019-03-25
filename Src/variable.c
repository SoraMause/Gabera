#include "variable.h"

//---------------------------------------------------------------------
// 構造体
//---------------------------------------------------------------------
// 速度、台形加速関連
t_run left_real;         // 左モーターの速度
t_run right_real;        // 右モーターの速度
t_run rotation_real;     // 角加速度
t_run translation_ideal; // 理想の併進方向情報
t_run rotation_ideal;    // 理想の回転方向情報
t_trapezoid translation_trape_param; // 併進方向の台形加速情報
t_trapezoid rotation_trape_param;    // 回転方向の台形加速情報
t_normal_param run_param;          // 直線の台形加速の速度
t_normal_param rotation_param;     // 回転方向の台形加速の速度 

// スラローム関連
t_slarom_parameter slarom500;  // 500 mm/sec のスラロームパラメータ

// PID関連
t_PID_param translation_gain;      // 速度ゲイン
t_PID_param rotation_gain;         // 回転ゲイン
t_PID_param sensor_gain;           // センサのゲイン
t_deviation run_left_deviation;    // 直線の差分
t_deviation run_right_deviation;   // 直線の差分
t_deviation rotation_deviation;    // 回転の差分

// duty , encoder 
t_enc_value enc_value;  // エンコーダ
t_duty duty;  // モーターpwm

// sensor data 
t_sensor sen_front = {0,0,0,0,0,0}; // 前壁
t_sensor sen_fl;
t_sensor sen_fr;
t_sensor sen_l = {0,0,0,0,0,0}; // 横左壁
t_sensor sen_r = {0,0,0,0,0,0}; // 横右壁

t_sensosr_log sen_fl_log;  // 前左壁
t_sensosr_log sen_fr_log;  // 前右壁

t_sensosr_log sen_l_log;  // 横左壁
t_sensosr_log sen_r_log;  // 横右壁

t_walldata wall_data; // 壁情報
t_walldata wall_bit; // すべてに壁が入ってる壁情報
t_position mypos;     // 座標状況

//---------------------------------------------------------------------
// 変数
//---------------------------------------------------------------------
float batt_monitor;   // 電圧チェック

int8_t mode_counter;  // modeのカウンタ
float mode_distance;  // modeの選択用距離
volatile int32_t cnt_motion;   // motionカウンタ
volatile int32_t cnt_act;      // 動作の時間を保持するカウンタ

int8_t sidewall_control_flag;  // 横壁制御用フラグ
int8_t frontwall_control_flag;  // 前壁制御フラグ
int8_t dirwall_control_flag;    // 斜め壁制御用フラグ

uint8_t motion_queue[256]; // 最短の動作を入れておく変数
int16_t motion_last;     // motion last
int16_t motion_end;      // motion end

float accel_data;  // フェイルセーフ用の加速度の値を取得する変数
int8_t failSafe_flag; // フェイルセーフ用フラグ
volatile int8_t wait_flag;   // 待っているときかどうかのフラグ

uint32_t maze_step[32][32];   // 歩数マップ  

uint16_t ADCBuff[ADC_CONVERT_DATA_SIZE];     // AD変換の結果を保持するバッファ
uint16_t ADCOffData[ADC_CONVERT_DATA_SIZE];   // AD変換の消した時の値を保持
uint16_t ADCOntData[ADC_CONVERT_DATA_SIZE];   // AD変換の光っているときの値を保持
int16_t adc_counter;                          // AD変換のタイミングを指定