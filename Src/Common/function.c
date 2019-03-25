#include "function.h"
// peripheeral
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "spi.h"

//common
#include "led.h"
#include "buzzer.h"
#include "flash.h"

#define OFF_VALUE 0
#define SIDE_VALUE 1
#define FRONT_VALUE 2
#define FINISH_CONVERT 3

static float batt_calc_const = 0.0f;
static uint8_t ctr_irled = 0;

///////////////////////////////////////////////////////////////////////
// machine_init
// [argument] nothinh
// [Substitutiong] batt_voltage
// [return] nothing
///////////////////////////////////////////////////////////////////////
void machine_init( void )
{
  setbuf( stdout, NULL );
  setIrledPwm( IRLED_OFF );
  failSafe_flag = 0;
  buzzerSetMonophonic( NORMAL, 100 );
  HAL_Delay( 101 );
  certainLedOut( LED_OFF );
  fullColorLedOut( LED_OFF );
  MPU6500_init();
  buzzerSetMonophonic( NORMAL, 100 );
  HAL_Delay( 100 );
  HAL_TIM_Encoder_Start( &htim1, TIM_CHANNEL_ALL ); // encoder
  HAL_TIM_Encoder_Start( &htim8, TIM_CHANNEL_ALL ); // encoder
  batt_calc_const = 3.3f / 1024.0f * ( 2200.0f + 1000.0f ) / 1000.0f;
  mode_counter = 0;
  mode_distance = 0.0f;
  adc_counter = FINISH_CONVERT;
  HAL_Delay( 300 );

  //MPU6500_z_axis_offset_calc_start();
}

///////////////////////////////////////////////////////////////////////
// batt voltage calculation
// [argument] nothinh
// [Substitutiong] batt_voltage
// [return] nothing
///////////////////////////////////////////////////////////////////////
float battMonitor( void )
{
  int16_t batt_analog = 0;
  float batt_voltage;

  HAL_ADC_Start( &hadc2 );
  HAL_ADC_PollForConversion( &hadc2,100 );  // trans
  batt_analog = HAL_ADC_GetValue( &hadc2 );   // get value
  HAL_ADC_Stop( &hadc2 );
  
  batt_voltage = (float)( batt_calc_const * batt_analog );

  return batt_voltage;
}

///////////////////////////////////////////////////////////////////////
// able ir led
// [argument] ired 1:paluse emit ON 0:OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void setIrledPwm( uint8_t able )
{
	ctr_irled = able;
}


void setSensorConstant( t_sensor *sen, int16_t reference, int16_t threshold )
{
  sen->reference = reference;
  sen->threshold = threshold;
}

void update_sensor_data( void )
{

  sen_front.now = ( sen_fl.now + sen_fr.now ) / 2;

  sen_fr.diff = ( sen_fr_log.now - sen_fr_log.before_3ms );
  if ( sen_fr.diff < 0 ) sen_fr.diff = -1 * sen_fr.diff;
  sen_fr.diff_1ms = ( sen_fr_log.now - sen_fr_log.before_1ms );
  if ( sen_fr.diff_1ms < 0 ) sen_fr.diff_1ms = -1 * sen_fr.diff_1ms;

  sen_r.diff = ( sen_r_log.now - sen_r_log.before_3ms );
  if ( sen_r.diff < 0 ) sen_r.diff = -1 * sen_r.diff;
  sen_r.diff_1ms = ( sen_r_log.now - sen_r_log.before_1ms );
  if ( sen_r.diff_1ms < 0 ) sen_r.diff_1ms = -1 * sen_r.diff_1ms;

  sen_l.diff = ( sen_l_log.now - sen_l_log.before_3ms );
  if ( sen_l.diff < 0 ) sen_l.diff = -1 * sen_l.diff;
  sen_l.diff_1ms = ( sen_l_log.now - sen_l_log.before_1ms );
  if ( sen_l.diff_1ms < 0 ) sen_l.diff_1ms = -1 * sen_l.diff_1ms;

  sen_fl.diff = ( sen_fl_log.now - sen_fl_log.before_3ms );
  if ( sen_fl.diff < 0 ) sen_fl.diff = -1 * sen_fl.diff;
  sen_fl.diff_1ms = ( sen_fl_log.now - sen_fl_log.before_1ms );
  if ( sen_fl.diff_1ms < 0 ) sen_fl.diff_1ms = -1 * sen_fl.diff_1ms;

  if ( sen_front.now < sen_front.threshold ){
    sen_front.is_wall = 0;
  } else {
    sen_front.is_wall = 1;
  }

  if ( sen_l.now < sen_l.threshold ){
    sen_l.is_wall = 0;
  } else {
    sen_l.is_wall = 1;
  }

  if ( sen_r.now < sen_r.threshold ){
    sen_r.is_wall = 0;
  } else {
    sen_r.is_wall = 1;
  }
}

void adcStart( void )
{
  setIrledPwm( IRLED_ON );
  adc_counter = 0;
  HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
}

void adcEnd( void )
{
  setIrledPwm( IRLED_OFF );
  adc_counter = FINISH_CONVERT;
}

void adcCheckConvert( void )
{
  uint8_t led_light = 0;
  if ( adc_counter == FINISH_CONVERT ){
    batt_monitor = battMonitor();
    if ( ctr_irled == 1 ){
      update_sensor_data();
      adc_counter = 0;
      HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
    }
  }

  if ( ctr_irled == 1 ){
    led_light |= sen_r.is_wall;
    led_light |= (sen_l.is_wall << 1);
    led_light |= (sen_front.is_wall << 2);
    certainLedOut( led_light );
  }
}

// DMA の変換式を記載
void getADSensor( int16_t *adcount )
{
  volatile int i;
  switch( *adcount ) {
    case OFF_VALUE:
      HAL_ADC_Stop_DMA( &hadc1 );
      ADCOffData[0] = ADCBuff[0];
      ADCOffData[1] = ADCBuff[1];
      ADCOffData[2] = ADCBuff[2];
      ADCOffData[3] = ADCBuff[3];

      HAL_GPIO_WritePin( sensor_paluseSide_GPIO_Port, sensor_paluseSide_Pin, GPIO_PIN_SET );
      for( i = 0; i < 200; i++ ){

      }

      *adcount = SIDE_VALUE;

      HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
      break;

    case SIDE_VALUE:
      HAL_GPIO_WritePin( sensor_paluseSide_GPIO_Port, sensor_paluseSide_Pin, GPIO_PIN_RESET );
      HAL_ADC_Stop_DMA( &hadc1 );

      ADCOntData[2] = ADCBuff[2];
      ADCOntData[3] = ADCBuff[3];

      sen_r.now = ADCOntData[2] - ADCOffData[2];
      sen_r_log.before_5ms = sen_r_log.before_4ms;
      sen_r_log.before_4ms = sen_r_log.before_3ms;
      sen_r_log.before_3ms = sen_r_log.before_2ms;
      sen_r_log.before_2ms = sen_r_log.before_1ms;
      sen_r_log.before_1ms = sen_r_log.now;
      sen_r_log.now = sen_r.now;

      sen_l.now = ADCOntData[3] - ADCOffData[3];
      sen_l_log.before_5ms = sen_l_log.before_4ms;
      sen_l_log.before_4ms = sen_l_log.before_3ms;
      sen_l_log.before_3ms = sen_l_log.before_2ms;
      sen_l_log.before_2ms = sen_l_log.before_1ms;
      sen_l_log.before_1ms = sen_l_log.now;
      sen_l_log.now = sen_l.now;


      HAL_GPIO_WritePin( sensor_paluseFront_GPIO_Port, sensor_paluseFront_Pin, GPIO_PIN_SET );
      for( i = 0; i < 400; i++ ){

      }

      *adcount = FRONT_VALUE;

      HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
      break;

    case FRONT_VALUE:
      HAL_GPIO_WritePin( sensor_paluseFront_GPIO_Port, sensor_paluseFront_Pin, GPIO_PIN_RESET );
      HAL_ADC_Stop_DMA( &hadc1 );

      ADCOntData[0] = ADCBuff[0];
      ADCOntData[1] = ADCBuff[1];

      sen_fr.now = ADCOntData[0] - ADCOffData[0];
      sen_fr_log.before_5ms = sen_fr_log.before_4ms;
      sen_fr_log.before_4ms = sen_fr_log.before_3ms;
      sen_fr_log.before_3ms = sen_fr_log.before_2ms;
      sen_fr_log.before_2ms = sen_fr_log.before_1ms;
      sen_fr_log.before_1ms = sen_fr_log.now;
      sen_fr_log.now = sen_fr.now;

      sen_fl.now = ADCOntData[1] - ADCOffData[1];
      sen_fl_log.before_5ms = sen_fl_log.before_4ms;
      sen_fl_log.before_4ms = sen_fl_log.before_3ms;
      sen_fl_log.before_3ms = sen_fl_log.before_2ms;
      sen_fl_log.before_2ms = sen_fl_log.before_1ms;
      sen_fl_log.before_1ms = sen_fl_log.now;
      sen_fl_log.now = sen_fl.now;

      *adcount = FINISH_CONVERT;
      for( i = 0; i < 100; i++ ){

      }
      break;

    default:
      break;
  }
}