#ifndef __MODE_H
#define __MODE_H

#include <stdint.h>
#include "variable.h"

void mode_init( void );
void startAction( void );
void modeSelect( int8_t mode );

void writeFlashData( t_walldata *wall );
void loadWallData( t_walldata *wall );

void mode0( void );
void mode1( void );
void mode2( void );
void mode3( void );
void mode4( void );
void mode5( void );
void mode6( void );
void mode7( void );
void mode8( void );

#endif /* __MODE_H */