#ifndef __CONFIG_H
#define __CONFIG_H

// macro define
#define dt 0.001f
//---------------------------------------------------------------------
// マシンデータ( 物理パラメータ )
//---------------------------------------------------------------------
#define MACHINE_WHEEL_RADIUS      0.01225f
// マシンのトレッド
#define MACHINE_TREAD_WIDTH       0.060f
// マシンの重さ
#define MACHINE_WEIGHT            0.115f
// ギア比
#define GEAR_RATION               3.5f // 42 / 10 = 4.2

//---------------------------------------------------------------------
// モーターデータ ( 1717 3v version ie 512 )
//---------------------------------------------------------------------
#define MOTOR_RESISTOR                  1.07f   // R
#define MOTOR_TORQUE_CONSTANT           0.00198f   // Kt
#define MOTOR_REVERSE_VOLTAGE_CONSTANT  0.000207f  // ke

#define MOTOR_CONTROL_PERIOD 400

// タイヤが一回転するまでのエンコーダの値
#define MACHINE_ENC_CNT_PER_ROT 7168 // 512 * 4 * 3.5 

#endif /* __CONFIG_H */