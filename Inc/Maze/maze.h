#ifndef __MAZE_H
#define __MAZE_H

#include <stdint.h>

#include "walldata.h"
#include "variable.h"

#define MAZE_CLASSIC_SIZE 16
#define MAZE_HALF_MAX_SIZE 32

#define HALF_BLOCK_DISTANCE   90.0f
#define ONE_BLOCK_DISTANCE    180.0f
#define ADJ_FRONT_DISTANCE		137.0f

#define SLATING_ONE_BLOCK_DISTANCE 127.28f 

#define MAX_STEP 65535

// 重ね探索について考える
uint8_t checkAllSearch( void );

void goalData_Init( uint8_t maze_scale );
void setGoalDataBit( uint8_t x, uint8_t y );
void clearGoalDataBit( uint8_t x, uint8_t y );
void setVirtualGoal( uint8_t maze_scale, t_walldata *wall );

int8_t getNextDirKnown(uint8_t direction, uint8_t x, uint8_t y, t_walldata *wall, t_walldata *wall_bit, uint8_t maze_scale);

void setMazeGoalSize( uint8_t size );

void positionReset( t_position *pos );

void mazeUpdateMap( uint8_t gx, uint8_t gy, t_walldata *wall, uint8_t maze_scale );

void mazeUpdatePosition( uint8_t dir, t_position *pos );
int8_t getNextDir( uint8_t direction, uint8_t x, uint8_t y, t_walldata *wall, uint8_t maze_scale );

void mazeUpdateShortestMap( uint8_t gx, uint8_t gy, t_walldata *wall, uint8_t maze_scale );

#endif /* __MAZE_H */