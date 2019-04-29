#ifndef __MAZE_RUN_H
#define __MAZE_RUN_H

#include <stdint.h>
#include "variable.h"

void adachiSearchRun( int8_t gx, int8_t gy, t_normal_param *translation, t_normal_param *rotation, t_walldata *wall, t_walldata *bit, t_position *pos, uint8_t maze_scale );

void adachiSearchRunKnown( int8_t gx, int8_t gy, t_normal_param *translation, t_normal_param *rotation, t_walldata *wall, t_walldata *bit, t_position *pos, uint8_t maze_scale );

void adachiFastRun( t_normal_param *translation, t_normal_param *rotation );

void adachiFastRunDiagonal1400( t_normal_param *translation, t_normal_param *rotation );
void adachiFastRunDiagonal1600( t_normal_param *translation, t_normal_param *rotation );
void adachiFastRunDiagonal1700( t_normal_param *translation, t_normal_param *rotation );

#endif /*__MAZE_RUN_H */