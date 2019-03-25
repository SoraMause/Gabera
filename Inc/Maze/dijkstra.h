#ifndef __DIJKSTRA_H
#define __DIJKSTRA_H

#include <stdint.h>
#include "variable.h"

#define FIELDX 32//2の乗数である必要がある
#define FIELDY 32
//コマンドリスト（GO1は１区画前進を意味する）
#define GO1    1//必ず１である必要がある
#define GO2    2
#define GO30  30
#define GO31  31
#define TURNR 32//GO31の次にあれば何でもよい
#define TURNL 33
#define DIA_TO_CLOTHOIDR 34
#define DIA_TO_CLOTHOIDL 35
#define DIA_FROM_CLOTHOIDR 36
#define DIA_FROM_CLOTHOIDL 37
#define DIA_TURNR 38
#define DIA_TURNL 39
#define DIA_GO1 40
#define DIA_GO63 102
#define SNODE 103//ストップを意味する

int8_t getRouteArray( int16_t gx, int16_t gy, int16_t route[256], t_walldata *wall, uint8_t maze_scale, int8_t _straight, int8_t out_flag );
void inputMazeWallData( t_walldata *wall );

#endif /* __DIJKSTRA_H */