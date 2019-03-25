#ifndef __BUZZER_H
#define __BUZZER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

// music scale
#define NORMAL 799
#define C_SCALE 3058
#define D_SCALE 2727
#define E_SCALE 2427
#define F_SCALE 2290
#define G_SCALE 2040
#define A_SCALE 1818
#define B_SCALE 1620
#define C_H_SCALE 1529

void buzzerSetMonophonic( uint16_t scale, uint16_t time_beep );
void buzzerOutPut( void );
void buzzermodeSelect( int8_t mode );

#ifdef __cplusplus
}
#endif
#endif /* __BUZZER_H */