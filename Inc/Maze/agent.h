#ifndef __AGENT_H
#define __AGENT_H

#include <stdint.h>
#include "variable.h"

// macro 定義( 最短走行ターン速度 )
#define PARAM_1400 0  // use fun 
#define PARAM_1600 1	// use fun
#define PARAM_1700 2
#define MAX_PARAM 3	// 速度を複合

// macro 定義( モーションの名前 )
#define END                 0
#define END_MOTION          1
#define DELAY               2
#define FRONTPD_DELAY       3
// 最短用
#define SET_STRAIGHT        4
#define SET_DIA_STRAIGHT    5
#define SET_FRONT_PD_STRAIGHT 6
// 最短用
#define SLAROM_LEFT         11
#define SLAROM_RIGHT        12
// 区画の中心から斜め #0
#define DIA_CENTER_LEFT   13
#define DIA_CENTER_RIGHT  14
// 区画の中心から90度ターン #1
#define CENRTER_SLAROM_LEFT    15
#define CENRTER_SLAROM_RIGHT   16
// 斜め動作から直線復帰 #5
#define RETURN_DIA_RIGHT      17
#define RETURN_DIA_LEFT       18
// 180度大廻りターン #2
#define SLAROM_LEFT_180       19
#define SLAROM_RIGHT_180      20
// 斜め90度大廻ターン #3
#define DIA_LEFT_TURN       21
#define DIA_RIGHT_TURN      22
// 斜め135度ターンから直線復帰 #4
#define RETURN_DIA_LEFT_135   23
#define RETURN_DIA_RIGHT_135  24
// 区画の中心から斜め135度 #6
#define DIA_CENTER_LEFT_135   25
#define DIA_CENTER_RIGHT_135  26

typedef enum {
	short_left = 8,
	short_right = 9,
	diagonal = 10,	// 斜め
	dir_left = 11,	// 斜め左
	dir_right = 12, // 斜め右
  left_return = 13, // 斜め左から戻る
  right_return = 14, // 斜め右から戻る
	dia_turn_left = 15,				// 斜め左90度大廻り
	dia_turn_right = 16,			// 斜め右90度大廻り
	left_return_135 = 17,	// 斜め左135度復帰
	right_return_135 = 18, // 斜め右135度復帰
	right_180 = 19,				// 右180度
	left_180 = 20,				// 左180度
	dir_left_135 = 21,	// 斜め左135
	dir_right_135 = 22, // 斜め右135
	end_maze = 32,
}t_fastpath_act;

// 最短用
typedef struct {
  float speed;
  float start_speed;
  float end_speed;
  float distance;
}t_fastpath;

extern t_fastpath fast_path[256];

int8_t agentGetShortRoute( uint8_t gx, uint8_t gy, t_walldata *wall, uint8_t maze_scale, float *all_time, uint8_t method, uint8_t outflag, uint8_t boost );
void agentSetShortRoute( uint8_t gx, uint8_t gy, t_walldata *wall, uint8_t maze_scale, uint8_t outflag, uint8_t boost );
int8_t agentDijkstraRoute( int16_t gx, int16_t gy, t_walldata *wall, uint8_t maze_scale, int8_t _straight, int8_t speed_mode, int8_t out_flag );

void setFastPathParameter1400( int8_t motion_buff[256], int8_t motion_data[256], uint8_t *cnt_motion, int8_t out_flag );
void setFastPathParameter1600( int8_t motion_buff[256], int8_t motion_data[256], uint8_t *cnt_motion, int8_t out_flag );
void setFastPathParameter1700( int8_t motion_buff[256], int8_t motion_data[256], uint8_t *cnt_motion, int8_t out_flag );
void setFastPathParameterMax( int8_t motion_buff[256], int8_t motion_data[256], uint8_t *cnt_motion, int8_t out_flag );

#endif /* __AGENT_H */

