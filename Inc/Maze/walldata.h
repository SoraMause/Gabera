#ifndef __WALL_DATA_H
#define __WALL_DATA_H

#include <stdint.h>

#include "variable.h"

int8_t getWallData( uint8_t x, uint8_t y, uint8_t direction, t_walldata *wall );
void printWallData( t_walldata *wall, uint8_t maze_scale );

void wall_Init( t_walldata *wall, uint8_t maze_scale );

void wallBIt_Init( t_walldata *wall, uint8_t maze_scale );

int8_t searchCompartment( uint8_t x, uint8_t y, t_walldata *wall );

void addNorthWall( uint8_t x, uint8_t y, t_walldata *wall );
void addEastWall( uint8_t x, uint8_t y, t_walldata *wall );
void addSouthWall( uint8_t x, uint8_t y, t_walldata *wall );
void addWestWall( uint8_t x, uint8_t y, t_walldata *wall );

void removeNorthWall( uint8_t x, uint8_t y, t_walldata *wall );
void removeEastWall( uint8_t x, uint8_t y, t_walldata *wall );
void removeSouthWall( uint8_t x, uint8_t y, t_walldata *wall );
void removeWestWall( uint8_t x, uint8_t y, t_walldata *wall );

void addWall( t_position *pos ,t_walldata *wall );

#endif /* __WALL_DATA_H */