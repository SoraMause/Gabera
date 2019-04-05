#include "led.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "gpio.h"

///////////////////////////////////////////////////////////////////////
// read push switch(tail) 
// [argument] nothing
// [Substitutiong] nothing
// [return] ON 1 OFF 0
///////////////////////////////////////////////////////////////////////
uint8_t getPushsw( void )
{
  if ( HAL_GPIO_ReadPin( pushsw_GPIO_Port ,pushsw_Pin ) == 1 ){
    return 1;
  } else {
    return 0;
  }
}

///////////////////////////////////////////////////////////////////////
// certain led ( tail )
// [argument] LED_RIGHT_SIDE,LED_LEFT_SIDE,LED_BOTH,LED_OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void certainLedOut( uint8_t led )
{
  uint8_t _right = ( ~led & 0x01 );
  uint8_t _left = ( ~led & 0x02 ) >> 1;  
  uint8_t _front = ( ~led & 0x04 ) >> 2;
  HAL_GPIO_WritePin( led1_GPIO_Port, led1_Pin, _right );
  HAL_GPIO_WritePin( led2_GPIO_Port, led2_Pin, _left );
  HAL_GPIO_WritePin( led3_GPIO_Port, led3_Pin, _front );
}

///////////////////////////////////////////////////////////////////////
// certain led ( full color )
// [argument] 7 colors,LED_OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void fullColorLedOut( uint8_t led )
{
  uint8_t _red = ( ~led & 0x01 );
  uint8_t _green = ( ( ~led & 0x02 ) >> 1 );
  uint8_t _blue = ( ( ~led & 0x04 ) >> 2 );
  HAL_GPIO_WritePin( fled_red_GPIO_Port, fled_red_Pin, _red );
  HAL_GPIO_WritePin( fled_green_GPIO_Port, fled_green_Pin, _green );
  HAL_GPIO_WritePin( fled_blue_GPIO_Port, fled_blue_Pin, _blue );
}