#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

// led の光らせ方
#define LED_OFF         0x00
#define LED_LEFT        0x02
#define LED_RIGHT       0x01
#define LED_FRONT       0x04
#define LED_BOTH        0x03
#define LED_ALL         0x07
#define LED_RED         0x01
#define LED_GREEN       0x02
#define LED_BLUE        0x03
#define LED_YELLOW      0x04
#define LED_MAGENTA     0x05
#define LED_CYAN        0x06
#define LED_WHITE       0x07

uint8_t getPushsw( void );
void fullColorLedOut( uint8_t led );
void certainLedOut( uint8_t led );

#ifdef __cplusplus
 }
#endif

#endif /* __LED_H */