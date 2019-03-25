#include "tim.h"
#include "buzzer.h"

static uint16_t buzzer_counter = 1;
static uint16_t beep_time = 0;

void buzzerSetMonophonic( uint16_t scale, uint16_t time_beep )
{
    buzzerPwmOut( 99, scale ); 
    buzzer_counter = 0;
    beep_time = time_beep;
}

void buzzerOutPut( void )
{
  if ( buzzer_counter > beep_time ){
    buzzerPwmOut( 0, NORMAL );
  } else {
    buzzer_counter++;
  }

}

void buzzermodeSelect( int8_t mode )
{
  switch( mode ){
    case 0:
      break;

    case 1:
      buzzerSetMonophonic( C_SCALE, 200 );
      break;

    case 2:
      buzzerSetMonophonic( D_SCALE, 200 );
      break;

    case 3:
      buzzerSetMonophonic( E_SCALE, 200 );
      break;
    
    case 4:
      buzzerSetMonophonic( F_SCALE, 200 );
      break;

    case 5:
      buzzerSetMonophonic( G_SCALE, 200 );
      break;
    
    case 6:
      buzzerSetMonophonic( A_SCALE, 200 );
      break;

    case 7:
      buzzerSetMonophonic( B_SCALE, 200 );
      break;

    case 8:
      buzzerSetMonophonic( C_H_SCALE, 200 );
      break;

    default:
      break;
  }
}