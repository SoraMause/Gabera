#include "walldata.h"

#include <stdio.h>

#include "maze.h"

/**
 * 壁情報を初期化
*/ 
void wall_Init( t_walldata *wall, uint8_t maze_scale )
{
  if ( maze_scale == MAZE_CLASSIC_SIZE ){
    for ( int i = 0; i <= maze_scale; i++ ){
      // 壁情報初期化
      wall->row[i] = 0xffff0000;
      wall->column[i] = 0xffff0000;

      // 探索情報初期化
      wall->row_known[i] = 0xffff0000;
      wall->column_known[i] = 0xffff0000;
    }

    for ( int i = maze_scale+1; i <= MAZE_HALF_MAX_SIZE; i++ ){
      // 壁情報初期化
      wall->row[i] = 0xffffffff;
      wall->column[i] = 0xffffffff;

      // 探索情報初期化
      wall->row_known[i] = 0xffffffff;
      wall->column_known[i] = 0xffffffff;      
    }
  } else {
    for ( int i = maze_scale+1; i <= MAZE_HALF_MAX_SIZE; i++ ){
      // 壁情報初期化
      wall->row[i] = 0;
      wall->column[i] = 0;

      // 探索情報初期化
      wall->row_known[i] = 0;
      wall->column_known[i] = 0;      
    } 
  }


  // 外周の壁を初期化
  wall->row[0] = 0xffffffff;
  wall->column[0] = 0xffffffff;
  wall->row[maze_scale] = 0xffffffff;
  wall->column[maze_scale] = 0xffffffff;
  
  if ( maze_scale == MAZE_CLASSIC_SIZE ){
    // スタートマスの横の壁の代入
    wall->column[1] = 0xffff0001;
    wall->column_known[1] = 0xffff0001;
    wall->row_known[1] = 0xffff0001;
  } else {
    wall->column[1] = 0x00000001;
    wall->column_known[1] = 0x00000001;
    wall->row_known[1] = 0x00000001;
  }
  
  // 探索情報の外周を探索済みにしておく
  wall->row_known[0] = 0xffffffff;
  wall->column_known[0] = 0xffffffff;
  wall->row_known[maze_scale] = 0xffffffff;
  wall->column_known[maze_scale] = 0xffffffff;
  
}

void wallBIt_Init(t_walldata *wall, uint8_t maze_scale)
{
	for (int i = 0; i <= maze_scale; i++) {
		wall->column[i] = 0xffffffff;
		wall->row[i] = 0xffffffff;
		wall->row_known[i] = 0x00;
		wall->column_known[i] = 0x00;
	}
	wall->row[1] = 0xfffffffe;
}

/**
 * 壁情報を追加
 * それぞれの向きに対して壁を追加、削除する関数
 * 全体を通して追加をする関数を用意した。
*/
void addNorthWall( uint8_t x, uint8_t y, t_walldata *wall )
{
  uint32_t data = 1;
  data <<= x;
  wall->row[y+1] |= data;
  wall->row_known[y+1] |= data;
}

void addEastWall( uint8_t x, uint8_t y, t_walldata *wall )
{
  uint32_t data = 1;
  data <<= y;
  wall->column[x+1] |= data;
  wall->column_known[x+1] |= data;
}

void addSouthWall( uint8_t x, uint8_t y, t_walldata *wall )
{
  uint32_t data = 1;
  data <<= x;
  wall->row[y] |= data;
  wall->row_known[y] |= data;
}

void addWestWall( uint8_t x, uint8_t y, t_walldata *wall )
{
  uint32_t data = 1;
  data <<= y;
  wall->column[x] |= data;
  wall->column_known[x] |= data;
}


void removeNorthWall( uint8_t x, uint8_t y, t_walldata *wall )
{
  uint32_t data = 1;
  data <<= x;
  wall->row[y+1] &= ~data;
  wall->row_known[y+1] |= data;
}

void removeEastWall( uint8_t x, uint8_t y, t_walldata *wall )
{
  uint32_t data = 1;
  data <<= y;
  wall->column[x+1] &= ~data;
  wall->column_known[x+1] |= data;
}

void removeSouthWall( uint8_t x, uint8_t y, t_walldata *wall )
{
  uint32_t data = 1;
  data <<= x;
  wall->row[y] &= ~data;
  wall->row_known[y] |= data;
}

void removeWestWall( uint8_t x, uint8_t y, t_walldata *wall )
{
  uint32_t data = 1;
  data <<= y;
  wall->column[x] &= ~data;
  wall->column_known[x] |= data;
}

void addWall( t_position *pos ,t_walldata *wall )
{
  // 探索済みでない場合壁情報の更新を行う。
  //if ( searchCompartment( pos->x, pos->y, wall ) == 0 ){
    // 方向が北向きのとき
    if ( pos->direction == North ){
      // 西壁
      if ( sen_l.is_wall == 1 ){
        addWestWall( pos->x, pos->y, wall );
      } else {
        removeWestWall( pos->x, pos->y, wall );
      }
      // 東壁
      if ( sen_r.is_wall == 1 ){
        addEastWall( pos->x, pos->y, wall );
      } else {
        removeEastWall( pos->x, pos->y, wall );
      }
      // 北壁
      if ( sen_front.is_wall == 1 ){
        addNorthWall( pos->x, pos->y, wall );
      } else {
        removeNorthWall( pos->x, pos->y, wall );
      }
    // 方向が西向きのとき
    } else if ( pos->direction == West ){
      // 南壁
      if ( sen_l.is_wall == 1 ){
        addSouthWall( pos->x, pos->y, wall );
      } else {
        removeSouthWall( pos->x, pos->y, wall );
      }
      // 北壁
      if ( sen_r.is_wall == 1 ){
        addNorthWall( pos->x, pos->y, wall );
      } else {
        removeNorthWall( pos->x, pos->y, wall );
      }
      // 西壁
      if ( sen_front.is_wall == 1 ){
        addWestWall( pos->x, pos->y, wall );
      } else {
        removeWestWall( pos->x, pos->y, wall );
      }
    // 方向が南向きのとき
    } else if ( pos->direction == South ){
      // 東壁
      if ( sen_l.is_wall == 1 ){
        addEastWall( pos->x, pos->y, wall );
      } else {
        removeEastWall( pos->x, pos->y, wall );
      }
      // 西壁
      if ( sen_r.is_wall == 1 ){
        addWestWall( pos->x, pos->y, wall );
      } else {
        removeWestWall( pos->x, pos->y, wall );
      }
      // 南壁
      if ( sen_front.is_wall == 1 ){
        addSouthWall( pos->x, pos->y, wall );
      } else {
        removeSouthWall( pos->x, pos->y, wall );
      }    
    // 方向が東向きのとき
    } else if ( pos->direction == East ){
      // 北壁
      if ( sen_l.is_wall == 1 ){
        addNorthWall( pos->x, pos->y, wall );
      } else {
        removeNorthWall( pos->x, pos->y, wall );
      }
      // 南壁
      if ( sen_r.is_wall == 1 ){
        addSouthWall( pos->x, pos->y, wall );
      } else {
        removeSouthWall( pos->x, pos->y, wall );
      }
      // 東壁
      if ( sen_front.is_wall == 1 ){
        addEastWall( pos->x, pos->y, wall );
      } else {
        removeEastWall( pos->x, pos->y, wall );
      }    
    }
  }
//}

/**
 * 探索済みでどうか知らせる関数
 * 探索済み : 1 探索していない : 0
*/
int8_t searchCompartment( uint8_t x, uint8_t y, t_walldata *wall )
{
  uint32_t check_known = 1;
  int8_t north_known = 0, east_known = 0, south_known = 0, west_known = 0;

  // 北の情報
  check_known <<= x;
  check_known &= wall->row_known[y+1];
  if ( check_known != 0 ){
    north_known = 1;
  }
  check_known = 1;

  // 東の情報
  check_known <<= y;
  check_known &= wall->column_known[x+1];
  if ( check_known != 0 ){
    east_known = 1;
  }
  check_known = 1;

  // 南の情報
  check_known <<= x;
  check_known &= wall->row_known[y];
  if ( check_known != 0 ){
    south_known = 1;
  }
  check_known = 1;

  // 西の情報
  check_known <<= y;
  check_known &= wall->column_known[x];
  if ( check_known != 0 ){
    west_known = 1;
  }
  
  if ( north_known == 1 && east_known == 1 && south_known == 1 && west_known == 1 ){
    return 1;
  } else {
    return 0;
  }
}

/**
 * 壁情報を取得
 * 壁あり : 1 壁なし : 0
*/ 
int8_t getWallData( uint8_t x, uint8_t y, uint8_t direction, t_walldata *wall )
{
  uint32_t check_wall = 1;

  if ( direction > 3 ){
    direction = direction - 4;
  }

  if ( direction == North ){
		check_wall <<= x;
		check_wall &= wall->row[y + 1];
		if (check_wall != 0) {
			check_wall = 1;
		}
  } else if ( direction == East ){
		check_wall <<= y;
		check_wall &= wall->column[x + 1];
		if (check_wall != 0) {
			check_wall = 1;
		}
  } else if ( direction == South ){
		check_wall <<= x;
		check_wall &= wall->row[y];
		if (check_wall != 0) {
			check_wall = 1;
		}
  } else if ( direction == West ){
		check_wall <<= y;
		check_wall &= wall->column[x];
		if (check_wall != 0) {
			check_wall = 1;
		}
  }

  return check_wall;
}


/**
 * 壁情報をstdoutに出力
*/
void printWallData( t_walldata *wall, uint8_t maze_scale )
{
  int x,y;
  printf("\r\n");

  for ( y = maze_scale-1; y >= 0; y-- ){
    printf("+");
    for ( x = 0; x < maze_scale; x++ ){
      if ( getWallData( x, y, North, wall ) == 1 ){
        printf("-----");
      } else {
        printf("     ");
      }
      printf("+");
    }

    printf("\r\n");
    for ( x = 0; x < maze_scale; x++ ){
      if ( getWallData( x, y, West, wall) == 1 ){
        printf("|");
      } else {
        printf(" ");
      }

      // step を表示
      if ( mypos.x == x && mypos.y == y ){
        printf( "  M  " );
      } else if ( maze_step[x][y] == 0 ) {
        printf( "  G  ");
      } else {
        printf("%5ld",maze_step[x][y] );
      }
      
    }

    if ( getWallData( maze_scale -1, y, East, wall ) == 1 ){
      printf("|");
    } else {
      printf(" ");
    }

    printf("\r\n");
  }

  printf("+");
  for( x = 0; x < maze_scale; x++ ){
    if ( getWallData( x, 0, South, wall ) == 1 ){
      printf("-----");
    } else {
      printf("     ");
    }

    printf("+");
  }

  printf("\r\n");
}