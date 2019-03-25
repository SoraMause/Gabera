#include "search.h"

#include "maze.h"
#include "agent.h"

#include "tim.h"

#include "variable.h"

#include "mode.h"

#include "motion.h"
#include "run.h"
#include "timer.h"
#include "buzzer.h"
#include "led.h"
#include "logger.h"

#define SEARCH_MAX_TIME 150000

void adachiSearchRun( int8_t gx, int8_t gy, t_normal_param *translation, t_normal_param *rotation, t_walldata *wall, t_walldata *bit, t_position *pos, uint8_t maze_scale )
{

  int8_t next_dir = front;

  // もし、ゴール座標が探索ならマシンの動作時間を0にする。 
  if ( gx != 0 && gy != 0 ){
    cnt_act = 0;
  }

  mazeUpdatePosition( front, pos );
  adjFront( translation->accel, translation->velocity );
  while( pos->x != gx || pos->y != gy ){
    addWall( pos, wall );
    addWall( pos, bit ); 
    mazeUpdateMap( gx, gy, wall, maze_scale );
    next_dir = getNextDir( pos->direction,pos->x, pos->y, wall, maze_scale );

    switch( next_dir ){
      case front:
        mazeUpdatePosition( front, pos );
        straightOneBlock( translation->velocity );
        break;

      case left:
        mazeUpdatePosition( left, pos );
        slaromLeft( translation->velocity );
        break;

      case right:
        mazeUpdatePosition( right, pos );
        slaromRight( translation->velocity );
        break;

      case rear:

        straightHalfBlockStop( translation->accel, translation->velocity );
        mazeUpdatePosition( rear, pos );
        pivoTurn180( rotation->accel, rotation->velocity );
        adjBack();
        adjFront( translation->accel, translation->velocity );
        break;

      case pivo_rear:
        straightHalfBlockStop( translation->accel, translation->velocity );
        mazeUpdatePosition( rear, pos );
        pivoTurn180( rotation->accel, rotation->velocity );
        straightHalfBlockStart( translation->accel, translation->velocity );
        break;
    }

    // 探索時間が2分30秒以上たっていた場合打ち切り。
    if( cnt_act > SEARCH_MAX_TIME ) break;
  }
    
  addWall( pos, wall );
  addWall( pos, bit ); 
  straightHalfBlockStop( translation->accel, translation->velocity );
  waitMotion( 300 );
  pivoTurn180( rotation->accel, rotation->velocity );
  adjBack();
  mypos.direction = (mypos.direction + 2) % 4;
  setControlFlag( 0 );
  buzzerSetMonophonic( C_H_SCALE, 100 );
  waitMotion( 150 );
}

void adachiSearchRunKnown( int8_t gx, int8_t gy, t_normal_param *translation, t_normal_param *rotation, t_walldata *wall, t_walldata *bit, t_position *pos, uint8_t maze_scale )
{
  int8_t next_dir = front;
  int8_t block = 0;

  // もし、ゴール座標が探索ならマシンの動作時間を0にする。 
  if ( gx != 0 && gy != 0 ){
    cnt_act = 0;
  }

  mazeUpdatePosition( front, pos );
  adjFront( translation->accel, translation->velocity );
  while( pos->x != gx || pos->y != gy ){
    addWall( pos, wall );
    addWall( pos, bit ); 
    mazeUpdateMap( gx, gy, wall, maze_scale );
    next_dir = getNextDirKnown( pos->direction,pos->x, pos->y, wall, bit, maze_scale );

    if ( next_dir == front ){
        mazeUpdatePosition( front, pos );
        straightOneBlock( translation->velocity );
    } else if ( next_dir == left ){
        mazeUpdatePosition( left, pos );
        slaromLeft( translation->velocity );
    } else if ( next_dir == right ){
        mazeUpdatePosition( right, pos );
        slaromRight( translation->velocity );
    } else if ( next_dir == rear ){
      straightHalfBlockStop( translation->accel, translation->velocity );
      mazeUpdatePosition( rear, pos );
      pivoTurn180( rotation->accel, rotation->velocity );
      adjBack();
      adjFront( translation->accel, translation->velocity );
    } else if ( next_dir == pivo_rear ){
      straightHalfBlockStop( translation->accel, translation->velocity );
      mazeUpdatePosition( rear, pos );
      pivoTurn180( rotation->accel, rotation->velocity );
      straightHalfBlockStart( translation->accel, translation->velocity );
    } else if ( next_dir > 10 ){
        sidewall_control_flag = 1;  // 壁制御有効
        block = next_dir - 10;
        mazeUpdatePosition( next_dir, pos );
        setStraight( ONE_BLOCK_DISTANCE * block, translation->accel,translation->velocity + 500.0f,translation->velocity,translation->velocity );
        waitSearchStraight();
    }

    // 探索時間が2分30秒以上たっていた場合打ち切り。
    if( cnt_act > SEARCH_MAX_TIME ) break;

    if ( checkAllSearch() == 1 ) break;

  }
    
  addWall( pos, wall );
  addWall( pos, bit ); 
  straightHalfBlockStop( translation->accel, translation->velocity );
  pivoTurn180( rotation->accel, rotation->velocity );
  adjBack();
  mypos.direction = (mypos.direction + 2) % 4;
  setControlFlag( 0 );
  buzzerSetMonophonic( C_H_SCALE, 100 );
  waitMotion( 150 );
}

void adachiFastRun( t_normal_param *translation, t_normal_param *rotation )
{
  while( motion_queue[motion_last] != END ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SLAROM_LEFT:
        slaromLeft( translation->velocity );
        break;

      case SLAROM_RIGHT:
        slaromRight( translation->velocity );
        break;

      case FRONTPD_DELAY:
        waitMotion( 300 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        sidewall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;
    } // end switch 
    motion_last++;
  }

  waitMotion( 100 );
  buzzerSetMonophonic( NORMAL, 100 );
  waitMotion( 100 );
}

void adachiFastRunDiagonal1400( t_normal_param *translation, t_normal_param *rotation )
{
  
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 1000 );
  funControl( FUN_ON );
  waitMotion( 1000 );
  setLogFlag( 1 );
  setControlFlag( 1 );
  
  while( motion_queue[motion_last] != 0 ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        fullColorLedOut( LED_OFF );
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SET_DIA_STRAIGHT:
        fullColorLedOut( 0x01 );
        dirwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        dirwall_control_flag = 0;
        break;

      // 中心から90度
      case CENRTER_SLAROM_LEFT:
        fullColorLedOut( 0x02 );
        sidewall_control_flag = 1;
        while( sen_l.now > sen_l.threshold );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 8.4f;
        }
        setStraight( 20.0f, translation->accel, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( 90.0f, 12000.0f, 720.0f, 1400.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 26.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      case CENRTER_SLAROM_RIGHT:
        fullColorLedOut( 0x02 );
        sidewall_control_flag = 1;
        while( sen_r.now > sen_r.threshold );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 9.8f;
        }
        setStraight( 20.0f, translation->accel, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( -90.0f, 12000.0f, 720.0f, 1400.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 26.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      // 中心から180度
      case SLAROM_LEFT_180:
        fullColorLedOut( 0x03 );
        sidewall_control_flag = 1;
        while( sen_l.now > sen_l.threshold );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 8.4f;
        }
        setStraight( 30.0f, translation->accel, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( 180.0f, 12000.0f, 960.0f, 1400.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 47.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      case SLAROM_RIGHT_180:
        fullColorLedOut( 0x03 );
        sidewall_control_flag = 1;
        while( sen_r.now > sen_r.threshold );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 9.8f;
        }
        setStraight( 30.0f, translation->accel, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( -180.0f, 12000.0f, 960.0f, 1400.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 47.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      // 中心から45度
      case DIA_CENTER_LEFT:
        fullColorLedOut( 0x04 );
        sidewall_control_flag = 1;
        while( sen_l.now > sen_l.threshold );
        setStraight( 17.0f, translation->accel, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( 45.0f, 20000.0f, 900.0f, 1400.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 61.0f, 18000.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      case DIA_CENTER_RIGHT:
        fullColorLedOut( 0x04 );
        sidewall_control_flag = 1;
        while( sen_r.now > sen_r.threshold );
        setStraight( 17.0f, translation->accel, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( -45.0f, 20000.0f, 900.0f, 1400.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 61.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      // 中心から135度
      case DIA_CENTER_LEFT_135:
        fullColorLedOut( 0x05 );
        sidewall_control_flag = 1;
        while( sen_l.now > sen_l.threshold );
        setStraight( 40.0f, translation->accel, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( 135.0f, 25000.0f, 1000.0f, 1400.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 33.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      case DIA_CENTER_RIGHT_135:
        fullColorLedOut( 0x05 );
        sidewall_control_flag = 1;
        while( sen_r.now > sen_r.threshold );
        setStraight( 40.0f, translation->accel, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( -135.0f, 25000.0f, 1000.0f, 1400.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 33.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      // 斜め90度 ( V90 )
      case DIA_LEFT_TURN:
        fullColorLedOut( 0x06 );
        dirwall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 9.8f;
        }

        setStraight( 26.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( 90.0f, 30000.0f, 1200.0f, 1400.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 34.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      case DIA_RIGHT_TURN:
        fullColorLedOut( 0x06 );
        dirwall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 8.4f;
        }

        setStraight( 26.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( -90.0f, 30000.0f, 1200.0f, 1400.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 34.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      // 斜めから復帰
      case RETURN_DIA_LEFT:
        fullColorLedOut( 0x07 );
        dirwall_control_flag = 1;

        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 9.8f;
        }
        
        setStraight( 55.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( 45.0f, 20000.0f, 900.0f, 1400.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 22.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      case RETURN_DIA_RIGHT:
        fullColorLedOut( 0x07 );
        dirwall_control_flag = 1;
        
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 8.4f;
        }

        setStraight( 55.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( -45.0f, 20000.0f, 900.0f, 1400.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 22.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      // 斜めから135度ターン復帰
      case RETURN_DIA_LEFT_135:
        fullColorLedOut( 0x07 );
        dirwall_control_flag = 1;
        
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 9.8f;
        }

        setStraight( 47.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( 135.0f, 25000.0f, 1200.0f, 1400.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 71.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      case RETURN_DIA_RIGHT_135:
        fullColorLedOut( 0x07 );
        dirwall_control_flag = 1;

        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 8.4f;
        }

        setStraight( 47.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        setRotation( -135.0f, 25000.0f, 1200.0f, 1400.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 71.0f, 0.0f, 1400.0f, 1400.0f, 1400.0f );
        waitStraight();
        break;

      case FRONTPD_DELAY:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        waitMotion( 100 );
        break;

      case DELAY:
        waitMotion( 50 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;

      default:
        break;
    } // end switch 
    motion_last++;
  }

  funControl( FUN_OFF );
  buzzerSetMonophonic( NORMAL, 100 );
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 100 );
}

void adachiFastRunDiagonal1600( t_normal_param *translation, t_normal_param *rotation )
{
  
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 1000 );
  funControl( FUN_ON );
  waitMotion( 1000 );
  setLogFlag( 1 );
  setControlFlag( 1 );
  
  while( motion_queue[motion_last] != 0 ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        fullColorLedOut( LED_OFF );
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SET_DIA_STRAIGHT:
        fullColorLedOut( LED_OFF );
        dirwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        dirwall_control_flag = 0;
        break;

      // 中心から90度
      case CENRTER_SLAROM_LEFT:
        fullColorLedOut( 0x01 );
        slaromCenterLeft( translation->accel );
        break;

      case CENRTER_SLAROM_RIGHT:
        fullColorLedOut( 0x01 );
        slaromCenterRight( translation->accel );
        break;

      // 中心から180度
      case SLAROM_LEFT_180:
        fullColorLedOut( 0x03 );
        slaromCenterLeft180( translation->accel );
        break;

      case SLAROM_RIGHT_180:
        fullColorLedOut( 0x03 );
        slaromCenterRight180( translation->accel );
        break;

      // 中心から45度
      case DIA_CENTER_LEFT:
        fullColorLedOut( 0x04 );
        slaromCenterLeft45( translation->accel );
        break;

      case DIA_CENTER_RIGHT:
        fullColorLedOut( 0x04 );
        slaromCenterRight45( translation->accel );
        break;

      // 中心から135度
      case DIA_CENTER_LEFT_135:
        fullColorLedOut( 0x05 );
        slaromCenterLeft135( translation->accel );
        break;

      case DIA_CENTER_RIGHT_135:
        fullColorLedOut( 0x05 );
        slaromCenterRight135( translation->accel );
        break;

      // 斜め90度 ( V90 )
      case DIA_LEFT_TURN:
        fullColorLedOut( LED_OFF );
        slaromLeftV90();
        break;

      case DIA_RIGHT_TURN:
        fullColorLedOut( LED_OFF );
        slaromRightV90();
        break;

      // 斜めから復帰
      case RETURN_DIA_LEFT:
        fullColorLedOut( 0x0f );
        slaromReturnDiaLeft45();
        break;

      case RETURN_DIA_RIGHT:
        fullColorLedOut( 0x0f );
        slaromReturnDiaRight45();
        break;

      // 斜めから135度ターン復帰
      case RETURN_DIA_LEFT_135:
        fullColorLedOut( 0x06 );
        slaromReturnDiaLeft135();
        break;

      case RETURN_DIA_RIGHT_135:
        fullColorLedOut( 0x06 );
        slaromReturnDiaRight135();
        break;

      case FRONTPD_DELAY:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        waitMotion( 100 );
        break;

      case DELAY:
        waitMotion( 50 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;

      default:
        break;
    } // end switch 
    motion_last++;
  }

  funControl( FUN_OFF );
  buzzerSetMonophonic( NORMAL, 100 );
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 100 );
}


void adachiFastRunDiagonal1700( t_normal_param *translation, t_normal_param *rotation )
{
  
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 1000 );
  funControl( FUN_ON );
  waitMotion( 1000 );
  setLogFlag( 1 );
  setControlFlag( 1 );
  
  while( motion_queue[motion_last] != 0 ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        fullColorLedOut( LED_OFF );
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SET_DIA_STRAIGHT:
        fullColorLedOut( 0x01 );
        dirwall_control_flag = 1;
        if ( motion_queue[motion_last+1] == DIA_LEFT_TURN || motion_queue[motion_last+1] == DIA_RIGHT_TURN ){
          runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                      fast_path[motion_last].speed, 1600.0f );
        } else {
          runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                      fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        }
        dirwall_control_flag = 0;
        break;

      // 中心から90度
      case CENRTER_SLAROM_LEFT:
        fullColorLedOut( 0x02 );
        sidewall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0 ) {
          translation_ideal.distance = 6.8f;
        }
        setStraight( 27.0f, translation->accel, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( 90.5f, 20000.0f, 900.0f, 1700.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        break;

      case CENRTER_SLAROM_RIGHT:
        fullColorLedOut( 0x02 );
        sidewall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0 ) {
          translation_ideal.distance = 6.8f;
        }
        setStraight( 27.0f, translation->accel, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( -90.0f, 20000.0f, 900.0f, 1700.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 34.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        break;

      // 中心から180度
      case SLAROM_LEFT_180:
        fullColorLedOut( 0x03 );
        sidewall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0 ) {
          translation_ideal.distance = 6.8f;
        }
        setStraight( 35.0f, translation->accel, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( 180.0f, 26000.0f, 1100.0f, 1700.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 45.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        break;

      case SLAROM_RIGHT_180:
        fullColorLedOut( 0x03 );
        sidewall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0 ) {
          translation_ideal.distance = 6.8f;
        }
        setStraight( 35.0f, translation->accel, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( -180.0f, 26000.0f, 1100.0f, 1700.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 45.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        break;

      // 中心から45度
      case DIA_CENTER_LEFT:
        fullColorLedOut( 0x04 );
        sidewall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 10.0f );
        if ( translation_ideal.distance < 10.0 ) {
          translation_ideal.distance = 6.8f;
        }
        setStraight( 11.0f, translation->accel, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( 45.0f, 25000.0f, 1000.0f, 1700.0f );
        waitRotation();
        dirwall_control_flag = 1;
        if ( motion_queue[motion_last+1] == DIA_LEFT_TURN || motion_queue[motion_last+1] == DIA_RIGHT_TURN ){
          setStraight( 56.0f, 20000.0f, 1700.0f, 1700.0f, 1600.0f );
        } else {
          setStraight( 56.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        }
        waitStraight();
        break;

      case DIA_CENTER_RIGHT:
        fullColorLedOut( 0x04 );
        sidewall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 10.0f );
        if ( translation_ideal.distance < 10.0 ) {
          translation_ideal.distance = 6.8f;
        }
        setStraight( 11.0f, translation->accel, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( -45.0f, 25000.0f, 1000.0f, 1700.0f );
        waitRotation();
        dirwall_control_flag = 1;
        if ( motion_queue[motion_last+1] == DIA_LEFT_TURN || motion_queue[motion_last+1] == DIA_RIGHT_TURN ){
          setStraight( 56.0f, 20000.0f, 1700.0f, 1700.0f, 1600.0f );
        } else {
          setStraight( 56.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        }
        waitStraight();
        break;

      // 中心から135度
      case DIA_CENTER_LEFT_135:
        fullColorLedOut( 0x05 );
        sidewall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 25.0f );
        if ( translation_ideal.distance < 15.0 ) {
          translation_ideal.distance = 6.8f;
        }
        setStraight( 27.0f, translation->accel, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( 136.0f, 30000.0f, 1200.0f, 1700.0f );
        waitRotation();
        dirwall_control_flag = 1;
        if ( motion_queue[motion_last+1] == DIA_LEFT_TURN || motion_queue[motion_last+1] == DIA_RIGHT_TURN ){
          setStraight( 20.0f, 20000.0f, 1700.0f, 1700.0f, 1600.0f );
        } else {
          setStraight( 20.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        }
        waitStraight();
        break;

      case DIA_CENTER_RIGHT_135:
        fullColorLedOut( 0x05 );
        sidewall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 25.0f );
        if ( translation_ideal.distance < 15.0 ) {
          translation_ideal.distance = 6.8f;
        }
        setStraight( 27.0f, translation->accel, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( -135.0f, 30000.0f, 1200.0f, 1700.0f );
        waitRotation();
        dirwall_control_flag = 1;
        if ( motion_queue[motion_last+1] == DIA_LEFT_TURN || motion_queue[motion_last+1] == DIA_RIGHT_TURN ){
          setStraight( 20.0f, 20000.0f, 1700.0f, 1700.0f, 1600.0f );
        } else {
          setStraight( 20.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        }
        waitStraight();
        break;

      // 斜め90度 ( V90 )
      case DIA_LEFT_TURN:
        fullColorLedOut( 0x06 );
        dirwall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 10.0f ){
          translation_ideal.distance = 8.0f;
        }
        setStraight( 11.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
        waitStraight();
        setRotation( 90.0f, 30000.0f, 1200.0f, 1600.0f );
        waitRotation();
        sidewall_control_flag = 1;
        if ( motion_queue[motion_last+1] == DIA_LEFT_TURN || motion_queue[motion_last+1] == DIA_RIGHT_TURN ){
          setStraight( 21.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
        } else {
          setStraight( 21.0f, 20000.0f, 1700.0f, 1600.0f, 1700.0f );
        }
        waitStraight();
        break;

      case DIA_RIGHT_TURN:
        fullColorLedOut( 0x06 );
        dirwall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 10.0f ){
          translation_ideal.distance = 8.0f;
        }
        setStraight( 11.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
        waitStraight();
        setRotation( -90.0f, 30000.0f, 1200.0f, 1600.0f );
        waitRotation();
        sidewall_control_flag = 1;
        if ( motion_queue[motion_last+1] == DIA_LEFT_TURN || motion_queue[motion_last+1] == DIA_RIGHT_TURN ){
          setStraight( 21.0f, 0.0f, 1600.0f, 1600.0f, 1600.0f );
        } else {
          setStraight( 21.0f, 20000.0f, 1700.0f, 1600.0f, 1700.0f );
        }
        waitStraight();
        break;

      // 斜めから復帰
      case RETURN_DIA_LEFT:
        fullColorLedOut( 0x07 );
        dirwall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0 ) {
          translation_ideal.distance = 8.5f;
        }
        setStraight( 45.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( 47.0f, 25000.0f, 1000.0f, 1700.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 22.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        break;

      case RETURN_DIA_RIGHT:
        fullColorLedOut( 0x07 );
        dirwall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0 ) {
          translation_ideal.distance = 8.5f;
        }
        setStraight( 45.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( -45.0f, 25000.0f, 1000.0f, 1700.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 22.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        break;

      // 斜めから135度ターン復帰
      case RETURN_DIA_LEFT_135:
        fullColorLedOut( 0x07 );
        dirwall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 12.0f );
        setStraight( 15.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( 136.0f, 30000.0f, 1200.0f, 1700.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 35.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        break;

      case RETURN_DIA_RIGHT_135:
        fullColorLedOut( 0x07 );
        dirwall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 12.0f );
        setStraight( 15.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        setRotation( -135.0f, 30000.0f, 1200.0f, 1700.0f );
        waitRotation();
        sidewall_control_flag = 1;
        setStraight( 35.0f, 0.0f, 1700.0f, 1700.0f, 1700.0f );
        waitStraight();
        break;

      case FRONTPD_DELAY:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        waitMotion( 100 );
        break;

      case DELAY:
        waitMotion( 50 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;

      default:
        break;
    } // end switch 
    motion_last++;
  }

  funControl( FUN_OFF );
  buzzerSetMonophonic( NORMAL, 100 );
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 100 );
}
