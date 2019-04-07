#include "agent.h"
#include "maze.h"
#include "dijkstra.h"
#include <stdio.h>

t_fastpath fast_path[256];

void fast_path_init( void )
{
  for ( int i = 0; i < 256; i++ ){
    fast_path[i].speed = 0.0f;
    fast_path[i].end_speed = 0.0f;
    fast_path[i].start_speed = 0.0f;
    fast_path[i].distance = 0.0f;
  }
}


int8_t agentGetShortRoute( uint8_t gx, uint8_t gy, t_walldata *wall, uint8_t maze_scale, float *all_time, uint8_t method, uint8_t outflag, uint8_t boost )
{
  uint8_t nextdir = front;
  uint8_t count = 1;
  int8_t path[256];
  uint8_t motion_buff[256];
  int16_t path_number = 0;
  float rec_time = 0.0f;
  uint8_t counter = 0;

  float short_boost = 0.0f;
  float middle_boost = 0.0f;

  t_position pos;

  // to do boost によって直線を早くするソフトを入れる。
  if ( boost == 0 ){
    middle_boost = 0.0f;
    short_boost = 0.0f; 
  } else if ( boost == 1 ){
    middle_boost = 200.0f;
    short_boost = 100.0f; 
  } else if ( boost == 2 ){
    middle_boost = 400.0f;
    short_boost = 200.0f; 
  } else if ( boost == 3 ){
    middle_boost = 600.0f;
    short_boost = 300.0f; 
  } else if ( boost == 4 ){
    middle_boost = 800.0f;
    short_boost = 400.0f; 
  } else if ( boost == 5 ){
    middle_boost = 1000.0f;
    short_boost = 500.0f; 
  }

  for ( int i = 0; i < 256; i++ ){
    motion_queue[i] = 0;
    path[i] = 0;
    motion_buff[i] = 0;
  }

  motion_queue[0] = front;
  path[0] = 0;
  motion_buff[0] = front;

  // 初期化処理
  fast_path_init();
  positionReset( &pos );

  if ( method == 0 ){
    mazeUpdateShortestMap( gx, gy, wall, maze_scale );
  } else {
    mazeUpdateShortestMap( gx, gy, wall, maze_scale );
  }
  
  if ( outflag == 1 ) printWallData( wall, maze_scale );

  mazeUpdatePosition( front, &pos );

  while( 1 ){
    nextdir = getNextDir( pos.direction, pos.x, pos.y, wall, maze_scale );

    if ( nextdir != rear ){
      motion_queue[count] = nextdir;
      count++;
    } else {
      counter++;
    }

    mazeUpdatePosition( nextdir, &pos );

    if ( pos.x == gx && pos.y == gy ) break;

    if ( counter == 5 ){
      break;
    }
  }

  if ( counter == 5 ){
    printf( "経路が見つからなかった\n" );
    return 0;
  } else {
    positionReset( &pos );
    #if 0
    if ( outflag == 1 ){
      for( int i= 0; i < count; i++ ){
        switch( motion_queue[i] ){
          case front:
            printf( "data : %3d front ",i );
            break;

          case left:
            printf( "data : %3d left  ",i );
            break;

          case right:
            printf( "data : %3d right ",i );
            break;
        }

        if ( i % 4 == 0 ) printf( "\r\n" );
      }

      printf( "\r\n");
    }
    #endif

    for ( int i = 0; i < count; i++ ){
      if ( motion_queue[i] == front ){
        if ( i == 0 ) {
          path[0] = 0;
        } else {
          path[path_number] += 1;
        }

      } else  {
        if ( motion_queue[i-1] == front ) {
          path_number++;
        }
        if ( motion_queue[i] == left ){
          path[path_number] = -1;
        } else if ( motion_queue[i] == right ){
          path[path_number] = -8;
        }
        
        path_number++;
      }
    }

    if ( path[path_number] > 0 ) {
      path_number++;
    } 
    
    for ( int i = 0; i < path_number; i++ ){
      if ( i == 0 && path[0] == 0 ){
        fast_path[0].distance = 130.0f;
        rec_time += 0.5f;
      } else if ( path[i] > 0 ){
        if ( i == 0 ){
          fast_path[i].distance = ( ONE_BLOCK_DISTANCE * path[0] ) + 130.0f;
          
          if ( path[i] < 2 ){
            rec_time += 0.18f * path[i] + 0.4f;
          } else {
            rec_time += 0.18f * path[i] + 0.4f;
          }
        } else {
          fast_path[i].distance = ONE_BLOCK_DISTANCE * path[i];
          if ( path[i] < 3 ){
            rec_time += 0.18f * path[i];
          } else {          
            rec_time += 0.18f * path[i];
          }
          
        }
        motion_buff[i] = front;
      } else if ( path[i] == -1 ){
        fast_path[i].distance = 0.0f;
        motion_buff[i] = left;
        rec_time += 0.3f;
      } else if ( path[i] == -8 ){
        fast_path[i].distance = 0.0f;
        motion_buff[i] = right;
        rec_time += 0.3f;
      }
    }

    // motion_queue の中の更新した部分を全て初期化する
    for( int i= 0; i < 256; i++ ){
      motion_queue[i] = 0;
    }
    
    for ( int i = 0; i < path_number; i++ ){
      if ( motion_buff[i] == front ){
        if ( i == 0 ) {
          fast_path[i].start_speed = 0.0f;
          if ( fast_path[i].distance >= ONE_BLOCK_DISTANCE * 7.0f ){
            fast_path[i].speed = 2000.0f + middle_boost;
            if ( fast_path[i].speed > 3000.0f ) fast_path[i].speed = 3000.0f;
          } else if ( fast_path[i].distance >= ONE_BLOCK_DISTANCE * 4.0f ){
            fast_path[i].speed = 1500.0f + short_boost;
            if ( fast_path[i].speed > 2000.0f ) fast_path[i].speed = 2000.0f;
          } else if ( fast_path[i].distance >= ONE_BLOCK_DISTANCE * 2.0f ){
            fast_path[i].speed = 1000.0f + short_boost;
            if ( fast_path[i].speed > 1500.0f ) fast_path[i].speed = 1500.0f;
          } else {
            fast_path[i].speed = 500.0f;
          }
          fast_path[i].end_speed = 500.0f;
        } else {
          if ( fast_path[i].distance >= ONE_BLOCK_DISTANCE * 7.0f ){
            fast_path[i].speed = 2000.0f + middle_boost;
            if ( fast_path[i].speed > 3000.0f ) fast_path[i].speed = 3000.0f;
          } else if ( fast_path[i].distance >= ONE_BLOCK_DISTANCE * 4.0f ){
            fast_path[i].start_speed = fast_path[i-1].end_speed;
            fast_path[i].speed = 1500.0f + short_boost;
            if ( fast_path[i].speed > 2000.0f ) fast_path[i].speed = 2000.0f;
            fast_path[i].end_speed = 500.0f;
          } else if ( fast_path[i].distance >= ONE_BLOCK_DISTANCE * 2.0f ){
            fast_path[i].start_speed = fast_path[i-1].end_speed;
            fast_path[i].speed = 1000.0f + short_boost;
            if ( fast_path[i].speed > 1500.0f ) fast_path[i].speed = 1500.0f;
            fast_path[i].end_speed = 500.0f;
          } else {
            fast_path[i].start_speed = fast_path[i-1].end_speed;
            fast_path[i].speed = 700.0f;
            fast_path[i].end_speed = 500.0f;
          }
        }

        motion_queue[i] = SET_STRAIGHT;
      } else if ( motion_buff[i] == left ){
          fast_path[i].start_speed = fast_path[i-1].end_speed;
          fast_path[i].speed = 500.0f;
          fast_path[i].end_speed = 500.0f;
          motion_queue[i] = SLAROM_LEFT;
      } else if ( motion_buff[i] == right ){
          fast_path[i].start_speed = fast_path[i-1].end_speed;
          fast_path[i].speed = 500.0f;
          fast_path[i].end_speed = 500.0f;
          motion_queue[i] = SLAROM_RIGHT;
      }
    }

    // 最後に半区画ストップを入れておく
    motion_queue[path_number] = SET_STRAIGHT;
    fast_path[path_number].distance = HALF_BLOCK_DISTANCE;
    fast_path[path_number].start_speed = fast_path[path_number-1].end_speed;
    fast_path[path_number].speed = 500.0f;
    fast_path[path_number].end_speed = 0.0f;
    path_number++;
    
    motion_queue[path_number] = FRONTPD_DELAY;
    path_number++;
    motion_queue[path_number] = END_MOTION;
    path_number++;

    if ( outflag == 1 ){
      for ( int i = 0; i < path_number; i++ ){
        if ( motion_queue[i] == SET_STRAIGHT  ){
          printf( "fast_path[%3d] : distance : %5.5f, speed start : %5.5f, end : %5.5f, speed : %5.5f\r\n",i,
          fast_path[i].distance, fast_path[i].start_speed, fast_path[i].end_speed, fast_path[i].speed );
        } else if ( motion_queue[i] == SLAROM_LEFT ){
          printf( "fast_path[%3d] : slarom : left , speed start : %5.5f, end : %5.5f, speed : %5.5f\r\n",i,
                      fast_path[i].start_speed, fast_path[i].end_speed, fast_path[i].speed );
        } else if ( motion_queue[i] == SLAROM_RIGHT ){
          printf( "fast_path[%3d] : slarom : right , speed start : %5.5f, end : %5.5f, speed : %5.5f\r\n",i,
                      fast_path[i].start_speed, fast_path[i].end_speed, fast_path[i].speed );
        } else if ( motion_queue[i] == DELAY ){
          printf( "fast_path[%3d] : delay \r\n",i); 
        } else {
          printf( "motion end\r\n");
        }
      }
      
      printf( "record_time : %f \r\n",rec_time );
    }

    motion_end = path_number-1; // 最後のカウンタの値を保存しておく

    *all_time = rec_time;

    return 1;
  }
}

void agentSetShortRoute( uint8_t gx, uint8_t gy, t_walldata *wall, uint8_t maze_scale, uint8_t outflag, uint8_t boost )
{
  uint8_t checkA = 0;
  uint8_t checkB = 0;
  float timeA = 0.0f;
  float timeB = 0.0f;

  checkA = agentGetShortRoute( gx, gy, wall, maze_scale, &timeA, 0, outflag, boost );
  checkB = agentGetShortRoute( gx, gy, wall, maze_scale, &timeB, 1, outflag, boost );

  if ( checkA == 1 && checkB == 1 ){
    if ( timeA < timeB ){
      agentGetShortRoute( gx, gy, wall, maze_scale, &timeA, 0, outflag, boost );
    } else {
      agentGetShortRoute( gx, gy, wall, maze_scale, &timeB, 0, outflag, boost );
    }
  } else if ( checkA == 0 ){
    agentGetShortRoute( gx, gy, wall, maze_scale, &timeB, 0, outflag, boost );
  } else if ( checkB == 0 ){
    agentGetShortRoute( gx, gy, wall, maze_scale, &timeA, 0, outflag, boost );
  } else {
    return;
  }
}


int8_t agentDijkstraRoute( int16_t gx, int16_t gy, t_walldata *wall, uint8_t maze_scale, int8_t _straight, int8_t speed_mode, int8_t out_flag )
{

  int16_t route[256];
  int8_t motion_buff[256];
  int8_t motion_data[256];
  uint8_t cnt_dijkstra = 0;
  int i = 0;
  uint8_t cnt_motion = 0;

  for ( int i = 0; i < 256; i++ ){
    motion_queue[i] = 0;
    motion_queue[i] = 0;
    motion_data[i] = 0;
    route[i] = 0;
  }
  
  if ( getRouteArray( gx, gy, route, wall, maze_scale,_straight ,out_flag ) ){
		//printf("以下の経路が見つかりました\r\n");
    for( i = 0;route[i]!=SNODE;i++){
      if(GO1<=route[i] && route[i]<=GO31){
        cnt_dijkstra++;      
      }
          
      if(DIA_GO1<=route[i] && route[i]<=DIA_GO63)
        cnt_dijkstra++;
      

      if(route[i]==TURNR)
        cnt_dijkstra++;
      
        
      if(route[i]==TURNL)
        cnt_dijkstra++;
        
      if(route[i]==DIA_TO_CLOTHOIDR)
        cnt_dijkstra++;
        
      if(route[i]==DIA_TO_CLOTHOIDL)
        cnt_dijkstra++;
        
      if(route[i]==DIA_FROM_CLOTHOIDR)
        cnt_dijkstra++;
      
      if(route[i]==DIA_FROM_CLOTHOIDL)
        cnt_dijkstra++;

      if(route[i]==DIA_TURNR)
        cnt_dijkstra++;

      if(route[i]==DIA_TURNL)
        cnt_dijkstra++;

      if(route[i]==SNODE)
        cnt_dijkstra++;
        
    }
  } else {
    printf( "経路が見つかりませんでした。\r\n" );
    return 0;
  }

  motion_queue[cnt_dijkstra] = end_maze;
  cnt_dijkstra++; 

  /*
   to do 
   中心から右斜め　45度ターン #0
   x= 90 y = 180
   中心から右斜め　右斜めから直線　90度ターン　#1
   x = 180 y = 180
   中心から右斜め　右90度ターン　右斜めから45度ターン 180 度ターン #2
   x = 180 y = 0
   右斜め　右90度ターン　斜め１区画　斜め90度ターン #3
   x = 180 y = 0
   右斜め90度ターン　右斜めから直線復帰 135度直線復帰ターン #4
   x = 180 y = -90
   右斜めから復帰
   x = 90 y = 180　
   motion_queue にモーションを入れてしまう！
   */

  i  = 0;
  motion_queue[cnt_motion] = SET_STRAIGHT;
  cnt_motion = 1;
  while( i < cnt_dijkstra ){
    if(GO1<=route[i] && route[i]<=GO31){
      motion_queue[cnt_motion] = SET_STRAIGHT;
      motion_buff[cnt_motion] = front;
      motion_data[cnt_motion] = route[i];
      cnt_motion++;
      i++;      
    }
          
    if(DIA_GO1<=route[i] && route[i]<=DIA_GO63){
      motion_queue[cnt_motion] = SET_DIA_STRAIGHT;
      motion_buff[cnt_motion] = diagonal;
      motion_data[cnt_motion] = route[i]-DIA_GO1 + 1;
      cnt_motion++;
      i++;
    }
      

    if(route[i]==TURNR){
      motion_queue[cnt_motion] = CENRTER_SLAROM_RIGHT;
      motion_buff[cnt_motion] = right;
      motion_data[cnt_motion] = 1;
      cnt_motion++;
      i++;
    }
         
    if(route[i]==TURNL){
      motion_queue[cnt_motion] = CENRTER_SLAROM_LEFT;
      motion_buff[cnt_motion] = left;
      motion_data[cnt_motion] = 1;
      cnt_motion++;
      i++;
    }
        
    if(route[i]==DIA_TO_CLOTHOIDR){
      if ( route[i+1] == DIA_FROM_CLOTHOIDR ){
        // 次の動作が右斜め直線復帰なら90度ターンに変更
        motion_queue[cnt_motion] = CENRTER_SLAROM_RIGHT;
        motion_buff[cnt_motion] = short_right;
        motion_data[cnt_motion] = 1;
        cnt_motion++;
        i+=2;
      } else if ( route[i+1] == DIA_TURNR ){
        // もし次の動作が右90度なら
        if( route[i+2] == DIA_FROM_CLOTHOIDR ){
          // もしその次の動作が右斜め直線復帰なら
          motion_queue[cnt_motion] = SLAROM_RIGHT_180;
          motion_buff[cnt_motion] = right_180;
          motion_data[cnt_motion] = 1;
          cnt_motion++;
          i+=3;
        } else {
          // to do 135度ターン斜めを入れる
          if ( route[i+2] == DIA_GO1 ){
            motion_queue[cnt_motion] = DIA_CENTER_RIGHT_135;
            motion_buff[cnt_motion] = dir_right_135;
            motion_data[cnt_motion] = 1;
            cnt_motion++;
            i+=3;   
          } else {
            motion_queue[cnt_motion] = DIA_CENTER_RIGHT_135;
            motion_buff[cnt_motion] = dir_right_135;
            motion_data[cnt_motion] = 1;
            cnt_motion++;
            i+=2;
            motion_queue[cnt_motion] = SET_DIA_STRAIGHT;
            motion_buff[cnt_motion] = diagonal;
            motion_data[cnt_motion] = route[i]-DIA_GO1;
            cnt_motion++;
            i++;  
          }
        }
      } else {
        // 1区画の場合とそれ以外の場合で分ける
        if ( route[i+1] == DIA_GO1 ){
          motion_queue[cnt_motion] = DIA_CENTER_RIGHT;
          motion_buff[cnt_motion] = dir_right;
          motion_data[cnt_motion] = 1;
          cnt_motion++;
          i+=2;   
        } else {
          motion_queue[cnt_motion] = DIA_CENTER_RIGHT;
          motion_buff[cnt_motion] = dir_right;
          motion_data[cnt_motion] = 1;
          cnt_motion++;
          i++;
          motion_queue[cnt_motion] = SET_DIA_STRAIGHT;
          motion_buff[cnt_motion] = diagonal;
          motion_data[cnt_motion] = route[i]-DIA_GO1;
          cnt_motion++;
          i++;  
        }
      }

    }
              
    if(route[i]==DIA_TO_CLOTHOIDL){
      if ( route[i+1] == DIA_FROM_CLOTHOIDL ){
        // 次の動作が左斜め直線復帰なら90度ターンに変更
        motion_queue[cnt_motion] = CENRTER_SLAROM_LEFT;
        motion_buff[cnt_motion] = short_left;
        motion_data[cnt_motion] = 1;
        cnt_motion++;
        i+=2;
      } else if ( route[i+1] == DIA_TURNL ){
        // もし次の動作が左90度なら
        if ( route[i+2] == DIA_FROM_CLOTHOIDL ){
          // もしその次の動作が左斜め直線復帰なら
          motion_queue[cnt_motion] = SLAROM_LEFT_180;
          motion_buff[cnt_motion] = left_180;
          motion_data[cnt_motion] = 1;
          cnt_motion++;
          i+=3;
        } else {
          // to do 135度ターン斜めを入れる
          if ( route[i+2] == DIA_GO1 ){
            motion_queue[cnt_motion] = DIA_CENTER_LEFT_135;
            motion_buff[cnt_motion] = dir_left_135;
            motion_data[cnt_motion] = 1;
            cnt_motion++;
            i+=3;   
          } else {
            motion_queue[cnt_motion] = DIA_CENTER_LEFT_135;
            motion_buff[cnt_motion] = dir_left_135;
            motion_data[cnt_motion] = 1;
            cnt_motion++;
            i+=2;
            motion_queue[cnt_motion] = SET_DIA_STRAIGHT;
            motion_buff[cnt_motion] = diagonal;
            motion_data[cnt_motion] = route[i]-DIA_GO1;
            cnt_motion++;
            i++;  
          }
        }
      } else {
        // 1区画の場合とそれ以外の場合で分ける
        if ( route[i+1] == DIA_GO1 ){
          motion_queue[cnt_motion] = DIA_CENTER_LEFT;
          motion_buff[cnt_motion] = dir_left;
          motion_data[cnt_motion] = 1;
          cnt_motion++;
          i+=2;   
        } else {
          motion_queue[cnt_motion] = DIA_CENTER_LEFT;
          motion_buff[cnt_motion] = dir_right;
          motion_data[cnt_motion] = 1;
          cnt_motion++;
          i++;
          motion_queue[cnt_motion] = SET_DIA_STRAIGHT;
          motion_buff[cnt_motion] = diagonal;
          motion_data[cnt_motion] = route[i]-DIA_GO1;
          cnt_motion++;
          i++;  
        }
      }
    }
        
    if(route[i]==DIA_FROM_CLOTHOIDR){
      motion_queue[cnt_motion] = RETURN_DIA_RIGHT;
      motion_buff[cnt_motion] = right_return;
      motion_data[cnt_motion] = 1;
      cnt_motion++;
      i++;
    }
      
    if(route[i]==DIA_FROM_CLOTHOIDL){
      motion_queue[cnt_motion] = RETURN_DIA_LEFT;
      motion_buff[cnt_motion] = left_return;
      motion_data[cnt_motion] = 1;
      cnt_motion++;
      i++;
    }

    if(route[i]==DIA_TURNR){
      if ( route[i+1] == DIA_FROM_CLOTHOIDR ){
        // もしその次の動作が右斜めから直線復帰なら
        motion_queue[cnt_motion] = RETURN_DIA_RIGHT_135;
        motion_buff[cnt_motion] = right_return_135;
        motion_data[cnt_motion] = 1;
        cnt_motion++;
        i+=2;  
      } else if (DIA_GO1<=route[i+1] && route[i+1]<=DIA_GO63 ){
        if( route[i+1] == DIA_GO1 ){
          // もし次の動作が１区画斜め直進なら
          motion_queue[cnt_motion] = DIA_RIGHT_TURN;
          motion_buff[cnt_motion] = dia_turn_right;
          motion_data[cnt_motion] = 1;
          cnt_motion++;
          i+=2;
        } else {
          // もし1区画以上なら
          motion_queue[cnt_motion] = DIA_RIGHT_TURN;
          motion_buff[cnt_motion] = dia_turn_right;
          motion_data[cnt_motion] = 1;
          cnt_motion++;
          i++;
          motion_queue[cnt_motion] = SET_DIA_STRAIGHT;
          motion_buff[cnt_motion] = diagonal;
          motion_data[cnt_motion] = route[i]-DIA_GO1;
          cnt_motion++;
          i++;
        } 
      } 
    }

    if(route[i]==DIA_TURNL){
      if ( route[i+1] == DIA_FROM_CLOTHOIDL ){
        // もしその次の動作が左斜めから直線復帰なら
        motion_queue[cnt_motion] = RETURN_DIA_LEFT_135;
        motion_buff[cnt_motion] = left_return_135;
        motion_data[cnt_motion] = 1;
        cnt_motion++;
        i+=2;       
      } else if (DIA_GO1<=route[i+1] && route[i+1]<=DIA_GO63 ){
        if( route[i+1] == DIA_GO1 ){
          // もし次の動作が１区画斜め直進なら
          motion_queue[cnt_motion] = DIA_LEFT_TURN;
          motion_buff[cnt_motion] = dia_turn_left;
          motion_data[cnt_motion] = 1;
          cnt_motion++;
          i+=2;
        } else {
          // もし1区画以上なら
          motion_queue[cnt_motion] = DIA_LEFT_TURN;
          motion_buff[cnt_motion] = dia_turn_left;
          motion_data[cnt_motion] = 1;
          cnt_motion++;
          i++;
          motion_queue[cnt_motion] = SET_DIA_STRAIGHT;
          motion_buff[cnt_motion] = diagonal;
          motion_data[cnt_motion] = route[i]-DIA_GO1;
          cnt_motion++;
          i++;
        }
      } 
    }
    
        
    if(route[i]==SNODE){
      motion_queue[cnt_motion] = END_MOTION;
      motion_buff[cnt_motion] = end_maze;
      motion_data[cnt_motion] = 0;      
      cnt_motion++;
      i++;
    }
  }

  if ( speed_mode == PARAM_1400 ){
    setFastPathParameter1400( motion_buff, motion_data, &cnt_motion, out_flag );
  } else if ( speed_mode == PARAM_1600 ){
    setFastPathParameter1600( motion_buff, motion_data, &cnt_motion, out_flag );
  } else if ( speed_mode == PARAM_1700 ){
    setFastPathParameter1700( motion_buff, motion_data, &cnt_motion, out_flag );
  }
  
  
  // 結果の表示
  if ( out_flag == 1 ){

    printf( "\r\n最短圧縮後\r\n\r\n");
    for( i = 0; i < cnt_motion; i++ ){
      if( motion_buff[i] == front )
        printf("%dマス直進,distance %f,speed target %f, start %f, end %f\r\n", 
        motion_data[i],fast_path[i].distance, fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed );
      if( motion_buff[i] == diagonal )
        printf("%dマス斜め直進,distance %f,speed target %f, start %f, end %f\r\n",
        motion_data[i],fast_path[i].distance, fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed );
      if( motion_buff[i] == right )
        printf("右９０度方向へスラローム,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == left )
        printf("左９０度方向へスラローム,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == dir_right )
        printf("直進から１マス使って斜め右方向へ #0,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == dir_left )
        printf("直進から１マス使って斜め左方向へ #0,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if ( motion_buff[i] == dir_right_135 )
        printf("直進から１マス使って右135度から斜め方向へ #6,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if ( motion_buff[i] == dir_left_135 )
        printf("直進から１マス使って左135度から斜め方向へ #6,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == right_return )
        printf("斜め右方向から直進へ #5,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == left_return )
        printf("斜め左方向から直進へ #5,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == dia_turn_right )
        printf("斜めから右９０度方向へ大廻りターンして斜めへ #3,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == dia_turn_left )
        printf("斜めから左９０度方向へ大廻りターンして斜めへ #3,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == short_right )
        printf("右９０度方向へ区画の中心からスラローム #1,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == short_left )
        printf("左９０度方向へ区画の中心からスラローム #1,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == right_return_135 )
        printf("右135度方向から直線へ #4,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == left_return_135 )
        printf("左135度方向から直線へ #4,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == right_180 )
        printf("右180度ターンから直線へ #2,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == left_180 )
        printf("左180度ターンから直線へ #2,speed target %f, start %f, end %f\r\n"
        ,fast_path[i].speed,fast_path[i].start_speed, fast_path[i].end_speed);
      if( motion_buff[i] == end_maze ){
        if ( motion_queue[i] == FRONTPD_DELAY ){
          printf("delay\r\n");
        } else {
          printf("おわり\r\n");
        }
        
      } 
    }
  }

  motion_last = 0;
  motion_end = cnt_motion-1; // 最後のカウンタの値を保存しておく

  return 1;

}

void setFastPathParameter1400( int8_t motion_buff[256], int8_t motion_data[256], uint8_t *cnt_motion, int8_t out_flag )
{
  
  fast_path_init();
  motion_buff[0] = front;
  fast_path[0].start_speed = 0.0f;
  fast_path[0].end_speed = 1100.0f;
  fast_path[0].speed = 1100.0f;
  fast_path[0].distance = 47.0f;
  for( int i = 1; i < *cnt_motion; i++ ){
    if ( i == *cnt_motion -2 ){
      fast_path[i].start_speed = 1400.0f;
      fast_path[i].end_speed = 0.0f;
    } else if ( i == *cnt_motion - 1 ){
      if ( motion_queue[i] == END_MOTION && out_flag == 1 ) printf( "check end motion\r\n" );
    } else {
      fast_path[i].start_speed = 1400.0f;
      fast_path[i].end_speed = 1400.0f;
    }

    if ( motion_buff[i] == front ){
      fast_path[i].distance = motion_data[i] * ONE_BLOCK_DISTANCE;
      // to do 距離によって変更
      if ( fast_path[i].distance > 10.0f * ONE_BLOCK_DISTANCE ) {
         fast_path[i].speed = 4000.0f; 
      } else if ( fast_path[i].distance > 7.0f * ONE_BLOCK_DISTANCE ) {
         fast_path[i].speed = 3500.0f; 
      } else if ( fast_path[i].distance > 5.0f * ONE_BLOCK_DISTANCE ) {
         fast_path[i].speed = 3000.0f;
      } else if ( fast_path[i].distance > 3.0f * ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2400.0f;
      } else if ( fast_path[i].distance > 1.0f * ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 1800.0f;
      } else {
        fast_path[i].speed = 1400.0f;
      }
    } else if ( motion_buff[i] == diagonal ){
      fast_path[i].distance = motion_data[i] * SLATING_ONE_BLOCK_DISTANCE;

      if ( fast_path[i].distance > 12.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 3500.0f;
      } else if ( fast_path[i].distance > 9.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 3000.0f;
      } else if ( fast_path[i].distance > 6.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2600.0f;
      } else if ( fast_path[i].distance > 4.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2200.0f;
      } else if ( fast_path[i].distance > 2.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 1800.0f;
      } else {
        fast_path[i].speed = 1400.0f;
      }
    } else {
      fast_path[i].distance = 0.0f;
      fast_path[i].speed = 1400.0f;
    }
  }

  // もし、END_MOTIONの前がスラロームターンの場合
  // 前壁制御を有効にして20mm進めようとする 必須条件　加速度 16 m /ss
  // 停止する前に前壁制御を有効にしたdelay
  if ( motion_queue[*cnt_motion-2] != SET_STRAIGHT ){
    // 終了速度を低速のまま
    fast_path[*cnt_motion-2].end_speed = 1400.0f;
    // 次の動作をEND_MOTIONから直線30mm 停止へ
    motion_buff[*cnt_motion-1] = front;
    fast_path[*cnt_motion-1].distance = 10.0f;
    fast_path[*cnt_motion-1].speed = 100.0f;
    fast_path[*cnt_motion-1].start_speed = 1400.0f;
    fast_path[*cnt_motion-1].end_speed = 0.0f;
    // 壁制御を有効なストレートモードを作成すること
    motion_queue[*cnt_motion-1] = SET_FRONT_PD_STRAIGHT;
    // delay
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = FRONTPD_DELAY;
    (*cnt_motion)++;
    // cnt_motionにEND_MOTIONを入れてその後
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = END_MOTION;
    (*cnt_motion)++;
  } else {
    // 終了速度を低速のまま
    fast_path[*cnt_motion-2].end_speed = 100.0f;
    fast_path[*cnt_motion-2].distance -= 20.0f;
    // 次の動作をEND_MOTIONから直線30mm 停止へ
    motion_buff[*cnt_motion-1] = front;
    fast_path[*cnt_motion-1].distance = 20.0f;
    fast_path[*cnt_motion-1].speed = 100.0f;
    fast_path[*cnt_motion-1].start_speed = 100.0f;
    fast_path[*cnt_motion-1].end_speed = 0.0f;
    // 壁制御を有効なストレートモードを作成すること
    motion_queue[*cnt_motion-1] = SET_FRONT_PD_STRAIGHT;
    // delay
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = DELAY;
    (*cnt_motion)++;
    // cnt_motionにEND_MOTIONを入れてその後
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = END_MOTION;
    (*cnt_motion)++;
  }
  
}

void setFastPathParameter1600( int8_t motion_buff[256], int8_t motion_data[256], uint8_t *cnt_motion, int8_t out_flag )
{
  
  fast_path_init();
  motion_buff[0] = front;
  fast_path[0].start_speed = 0.0f;
  fast_path[0].end_speed = 1200.0f;
  fast_path[0].speed = 1200.0f;
  fast_path[0].distance = 47.0f;
  for( int i = 1; i < *cnt_motion; i++ ){
    if ( i == *cnt_motion -2 ){
      fast_path[i].start_speed = 1600.0f;
      fast_path[i].end_speed = 0.0f;
    } else if ( i == *cnt_motion - 1 ){
      if ( motion_queue[i] == END_MOTION && out_flag == 1 ) printf( "check end motion\r\n" );
    } else {
      fast_path[i].start_speed = 1600.0f;
      fast_path[i].end_speed = 1600.0f;
    }

    if ( motion_buff[i] == front ){
      fast_path[i].distance = motion_data[i] * ONE_BLOCK_DISTANCE;
      // to do 距離によって変更
      if ( fast_path[i].distance > 10.0f * ONE_BLOCK_DISTANCE ) {
         fast_path[i].speed = 4500.0f; 
      } else if ( fast_path[i].distance > 8.0f * ONE_BLOCK_DISTANCE ) {
        fast_path[i].speed = 4000.0f; 
      } else if ( fast_path[i].distance > 6.0f * ONE_BLOCK_DISTANCE ) {
        fast_path[i].speed = 3500.0f; 
      } else if ( fast_path[i].distance > 4.0f * ONE_BLOCK_DISTANCE ) {
        fast_path[i].speed = 3000.0f;
      } else if ( fast_path[i].distance > 2.0f * ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2500.0f;
      } else if ( fast_path[i].distance > 1.0f * ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2000.0f;
      } else {
        fast_path[i].speed = 1600.0f;
      }
    } else if ( motion_buff[i] == diagonal ){
      fast_path[i].distance = motion_data[i] * SLATING_ONE_BLOCK_DISTANCE;

      if ( fast_path[i].distance > 12.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 3500.0f;
      } else if ( fast_path[i].distance > 9.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 3100.0f;
      } else if ( fast_path[i].distance > 6.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2700.0f;
      } else if ( fast_path[i].distance > 4.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2300.0f;
      } else if ( fast_path[i].distance > 2.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 1900.0f;
      } else {
        fast_path[i].speed = 1600.0f;
      }
    } else {
      fast_path[i].distance = 0.0f;
      fast_path[i].speed = 1600.0f;
    }
  }

  // もし、END_MOTIONの前がスラロームターンの場合
  // 前壁制御を有効にして20mm進めようとする 必須条件　加速度 16 m /ss
  // 停止する前に前壁制御を有効にしたdelay
  if ( motion_queue[*cnt_motion-2] != SET_STRAIGHT ){
    // 終了速度を低速のまま
    fast_path[*cnt_motion-2].end_speed = 1600.0f;
    // 次の動作をEND_MOTIONから直線30mm 停止へ
    motion_buff[*cnt_motion-1] = front;
    fast_path[*cnt_motion-1].distance = 10.0f;
    fast_path[*cnt_motion-1].speed = 100.0f;
    fast_path[*cnt_motion-1].start_speed = 1600.0f;
    fast_path[*cnt_motion-1].end_speed = 0.0f;
    // 壁制御を有効なストレートモードを作成すること
    motion_queue[*cnt_motion-1] = SET_FRONT_PD_STRAIGHT;
    // delay
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = FRONTPD_DELAY;
    (*cnt_motion)++;
    // cnt_motionにEND_MOTIONを入れてその後
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = END_MOTION;
    (*cnt_motion)++;
  } else {
    // 終了速度を低速のまま
    fast_path[*cnt_motion-2].end_speed = 300.0f;
    fast_path[*cnt_motion-2].distance -= 45.0f;
    // 次の動作をEND_MOTIONから直線30mm 停止へ
    motion_buff[*cnt_motion-1] = front;
    fast_path[*cnt_motion-1].distance = 45.0f;
    fast_path[*cnt_motion-1].speed = 300.0f;
    fast_path[*cnt_motion-1].start_speed = 300.0f;
    fast_path[*cnt_motion-1].end_speed = 0.0f;
    // 壁制御を有効なストレートモードを作成すること
    motion_queue[*cnt_motion-1] = SET_FRONT_PD_STRAIGHT;
    // delay
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = DELAY;
    (*cnt_motion)++;
    // cnt_motionにEND_MOTIONを入れてその後
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = END_MOTION;
    (*cnt_motion)++;
  }
  
}

void setFastPathParameter1700( int8_t motion_buff[256], int8_t motion_data[256], uint8_t *cnt_motion, int8_t out_flag )
{
  
  fast_path_init();
  motion_buff[0] = front;
  fast_path[0].start_speed = 0.0f;
  fast_path[0].end_speed = 1300.0f;
  fast_path[0].speed = 1300.0f;
  fast_path[0].distance = 47.0f;
  for( int i = 1; i < *cnt_motion; i++ ){
    if ( i == *cnt_motion -2 ){
      fast_path[i].start_speed = 1700.0f;
      fast_path[i].end_speed = 0.0f;
    } else if ( i == *cnt_motion - 1 ){
      if ( motion_queue[i] == END_MOTION && out_flag == 1 ) printf( "check end motion\r\n" );
    } else {
      fast_path[i].start_speed = 1700.0f;
      fast_path[i].end_speed = 1700.0f;
    }

    if ( motion_buff[i] == front ){
      fast_path[i].distance = motion_data[i] * ONE_BLOCK_DISTANCE;
      // to do 距離によって変更
      if ( fast_path[i].distance > 10.0f * ONE_BLOCK_DISTANCE ) {
         fast_path[i].speed = 4500.0f; 
      } else if ( fast_path[i].distance > 8.0f * ONE_BLOCK_DISTANCE ) {
        fast_path[i].speed = 4000.0f; 
      } else if ( fast_path[i].distance > 6.0f * ONE_BLOCK_DISTANCE ) {
        fast_path[i].speed = 3500.0f; 
      } else if ( fast_path[i].distance > 4.0f * ONE_BLOCK_DISTANCE ) {
        fast_path[i].speed = 3000.0f;
      } else if ( fast_path[i].distance > 2.0f * ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2600.0f;
      } else if ( fast_path[i].distance > 1.0f * ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2200.0f;
      } else {
        fast_path[i].speed = 1700.0f;
      }
    } else if ( motion_buff[i] == diagonal ){
      fast_path[i].distance = motion_data[i] * SLATING_ONE_BLOCK_DISTANCE;

      if ( fast_path[i].distance > 12.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 3500.0f;
      } else if ( fast_path[i].distance > 9.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 3100.0f;
      } else if ( fast_path[i].distance > 6.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2800.0f;
      } else if ( fast_path[i].distance > 4.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2500.0f;
      } else if ( fast_path[i].distance > 2.0f * SLATING_ONE_BLOCK_DISTANCE ){
        fast_path[i].speed = 2200.0f;
      } else {
        fast_path[i].speed = 1700.0f;
      }
    } else {
      fast_path[i].distance = 0.0f;
      fast_path[i].speed = 1700.0f;
    }
  }

  // もし、END_MOTIONの前がスラロームターンの場合
  // 前壁制御を有効にして20mm進めようとする 必須条件　加速度 16 m /ss
  // 停止する前に前壁制御を有効にしたdelay
  if ( motion_queue[*cnt_motion-2] != SET_STRAIGHT ){
    // 終了速度を低速のまま
    fast_path[*cnt_motion-2].end_speed = 1700.0f;
    // 次の動作をEND_MOTIONから直線30mm 停止へ
    motion_buff[*cnt_motion-1] = front;
    fast_path[*cnt_motion-1].distance = 10.0f;
    fast_path[*cnt_motion-1].speed = 100.0f;
    fast_path[*cnt_motion-1].start_speed = 1700.0f;
    fast_path[*cnt_motion-1].end_speed = 0.0f;
    // 壁制御を有効なストレートモードを作成すること
    motion_queue[*cnt_motion-1] = SET_FRONT_PD_STRAIGHT;
    // delay
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = FRONTPD_DELAY;
    (*cnt_motion)++;
    // cnt_motionにEND_MOTIONを入れてその後
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = END_MOTION;
    (*cnt_motion)++;
  } else {
    // 終了速度を低速のまま
    fast_path[*cnt_motion-2].end_speed = 300.0f;
    fast_path[*cnt_motion-2].distance -= 45.0f;
    // 次の動作をEND_MOTIONから直線30mm 停止へ
    motion_buff[*cnt_motion-1] = front;
    fast_path[*cnt_motion-1].distance = 45.0f;
    fast_path[*cnt_motion-1].speed = 300.0f;
    fast_path[*cnt_motion-1].start_speed = 300.0f;
    fast_path[*cnt_motion-1].end_speed = 0.0f;
    // 壁制御を有効なストレートモードを作成すること
    motion_queue[*cnt_motion-1] = SET_FRONT_PD_STRAIGHT;
    // delay
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = DELAY;
    (*cnt_motion)++;
    // cnt_motionにEND_MOTIONを入れてその後
    motion_buff[*cnt_motion] = end_maze;
    motion_queue[*cnt_motion] = END_MOTION;
    (*cnt_motion)++;
  }
  
}