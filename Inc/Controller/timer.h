#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

void interrupt( void );
void setControlFlag( int8_t _flag );

#endif /* __TIMER_H */