#include "maze.h"

#include <stdio.h>

// ゴール座標を複数与える場合
static uint8_t goal_size = 1;

// ゴール座標指定用変数
static uint32_t goal_data[32];

// もし、すべての値に初期値が入ったら全面探索終了
static uint8_t check_all_search_flag = 0;

/**
* 全面探索が終わったかどうかを返す
* 終わっている場合 1 終わってない場合 0
*/
uint8_t checkAllSearch( void )
{
	return check_all_search_flag;
}

/**
* ゴール座標を初期化
*/
void goalData_Init(uint8_t maze_scale)
{
	for (int y = 0; y < maze_scale; y++) {
		goal_data[y] = 0;
	}
	check_all_search_flag = 0;
}

/**
* ゴール座標に追加
*/
void setGoalDataBit(uint8_t x, uint8_t y)
{
	goal_data[y] |= (1 << x);
}

/**
* ゴール座標から取り除く
*/
void clearGoalDataBit(uint8_t x, uint8_t y)
{
	goal_data[y] = (goal_data[y] & (~(1 << x)));
}

/**
* 見探索区画を探して見探索区画を仮想ゴールに設定する
*/
void setVirtualGoal(uint8_t maze_scale, t_walldata *wall)
{
	for (int x = 0; x < maze_scale; x++) {
		for (int y = 0; y < maze_scale; y++) {
			if (searchCompartment(x, y, wall) == 1) {
				clearGoalDataBit(x, y);
			}
			else {
				setGoalDataBit(x, y);
			}
		}
	}
}

/**
* ゴールのサイズ ex クラッシクなら4マスといった内容
* */
void setMazeGoalSize(uint8_t size)
{
	goal_size = size;
}

void positionReset(t_position *pos)
{
	pos->direction = front;
	pos->x = 0;
	pos->y = 0;
}

void mazeUpdateMap(uint8_t gx, uint8_t gy, t_walldata *wall, uint8_t maze_scale)
{
	uint16_t distPositionList[1025];
	uint16_t head = 0, tail = 1;

	if (gx == 0 && gy == 0) {
		tail = 0;
		if (check_all_search_flag == 1) {
			// 2019 add intial step
			for (int x = 0; x < maze_scale; x++) {
				for (int y = 0; y < maze_scale; y++) {
					maze_step[x][y] = MAX_STEP;
				}
			}
			maze_step[gx][gy] = 0;
			distPositionList[0] = gx * 256 + gy;
			tail = 1;
		} else {
			for (int x = 0; x < maze_scale; x++) {
				for (int y = 0; y < maze_scale; y++) {
					if (searchCompartment(x, y, wall) == 1 && ((goal_data[y] >> x) & 0x01) == 1) {
						clearGoalDataBit(x, y);
					}

					if (((goal_data[y] >> x) & 0x01) == 1) {
						maze_step[x][y] = 0;
						distPositionList[tail] = x * 256 + y;
						tail++;
					}
					else {
						maze_step[x][y] = MAX_STEP;
					}
				}
			}

			// ほぼすべて探索済みなら真のゴール座標を入力
			// 2018 11/17 新たに追加
			// 2019 1/7 add check_all_search_flag = 1
			// 2019 1 / 7 change 5 > maze_scale 
			if (tail < 4) {
				maze_step[gx][gy] = 0;
				distPositionList[0] = gx * 256 + gy;
				tail = 1;
				check_all_search_flag = 1;
			}
		}
	} else {
		for (int x = 0; x < maze_scale; x++) {
			for (int y = 0; y < maze_scale; y++) {
				maze_step[x][y] = MAX_STEP;
			}
		}

		if (goal_size == 9) {

		}
		else if (goal_size == 4) {
			maze_step[gx + 1][gy] = 0;
			distPositionList[tail] = (gx + 1) * 256 + gy;
			tail++;
			maze_step[gx][gy + 1] = 0;
			distPositionList[tail] = gx * 256 + (gy + 1);
			tail++;
			maze_step[gx + 1][gy + 1] = 0;
			distPositionList[tail] = (gx + 1) * 256 + (gy + 1);
			tail++;
		}
		// goal座標は絶対一つあるのでその処理はここで行う。
		// 2018 11/17 新たに追加
		maze_step[gx][gy] = 0;
		distPositionList[0] = gx * 256 + gy;
	}

	while (head != tail) {

		//head = tail なら更新する区画がない
		int8_t Y = distPositionList[head] & 0x00ff;
		int8_t X = (distPositionList[head] & 0xff00) >> 8;

		head++;

		// 北壁の情報を更新
		if (Y < maze_scale - 1) {
			if ((getWallData(X, Y, North, wall) == 0) && (maze_step[X][Y + 1] == MAX_STEP)) {
				maze_step[X][Y + 1] = maze_step[X][Y] + 1;
				distPositionList[tail] = (X << 8) | (Y + 1);
				tail++;
			}
		}

		// 東壁の情報
		if (X < maze_scale - 1) {
			if ((getWallData(X, Y, East, wall) == 0) && (maze_step[X + 1][Y] == MAX_STEP)) {
				maze_step[X + 1][Y] = maze_step[X][Y] + 1;
				distPositionList[tail] = ((X + 1) << 8) | (Y);
				tail++;
			}
		}

		// 南壁の情報を更新
		if (Y > 0) {
			if ((getWallData(X, Y, South, wall) == 0) && (maze_step[X][Y - 1] == MAX_STEP)) {
				maze_step[X][Y - 1] = maze_step[X][Y] + 1;
				distPositionList[tail] = (X << 8) | (Y - 1);
				tail++;
			}
		}

		// 西壁の情報を更新
		if (X > 0) {
			if ((getWallData(X, Y, West, wall) == 0) && (maze_step[X - 1][Y] == MAX_STEP)) {
				maze_step[X - 1][Y] = maze_step[X][Y] + 1;
				distPositionList[tail] = ((X - 1) << 8) | (Y);
				tail++;
			}
		}

	}

}

#if 0
//改造前
void mazeUpdateMap(uint8_t gx, uint8_t gy, t_walldata *wall, uint8_t maze_scale)
{
	uint16_t distPositionList[1025];
	uint16_t head = 0, tail = 1;

	for (int x = 0; x < maze_scale; x++) {
		for (int y = 0; y < maze_scale; y++) {
			maze_step[x][y] = MAX_STEP;
		}
	}

	if (goal_size == 9) {

	}
	else if (goal_size == 4) {
		maze_step[gx + 1][gy] = 0;
		distPositionList[tail] = (gx + 1) * 256 + gy;
		tail++;
		maze_step[gx][gy + 1] = 0;
		distPositionList[tail] = gx * 256 + (gy + 1);
		tail++;
		maze_step[gx + 1][gy + 1] = 0;
		distPositionList[tail] = (gx + 1) * 256 + (gy + 1);
		tail++;
	}

	// goal座標は絶対一つあるのでその処理はここで行う。
	maze_step[gx][gy] = 0;
	distPositionList[0] = gx * 256 + gy;

	while (head != tail) {

		//head = tail なら更新する区画がない
		int8_t Y = distPositionList[head] & 0x00ff;
		int8_t X = (distPositionList[head] & 0xff00) >> 8;

		head++;

		// 北壁の情報を更新
		if (Y < maze_scale - 1) {
			if ((getWallData(X, Y, North, wall) == 0) && (maze_step[X][Y + 1] == MAX_STEP)) {
				maze_step[X][Y + 1] = maze_step[X][Y] + 1;
				distPositionList[tail] = (X << 8) | (Y + 1);
				tail++;
			}
		}

		// 東壁の情報
		if (X < maze_scale - 1) {
			if ((getWallData(X, Y, East, wall) == 0) && (maze_step[X + 1][Y] == MAX_STEP)) {
				maze_step[X + 1][Y] = maze_step[X][Y] + 1;
				distPositionList[tail] = ((X + 1) << 8) | (Y);
				tail++;
			}
		}

		// 南壁の情報を更新
		if (Y > 0) {
			if ((getWallData(X, Y, South, wall) == 0) && (maze_step[X][Y - 1] == MAX_STEP)) {
				maze_step[X][Y - 1] = maze_step[X][Y] + 1;
				distPositionList[tail] = (X << 8) | (Y - 1);
				tail++;
			}
		}

		// 西壁の情報を更新
		if (X > 0) {
			if ((getWallData(X, Y, West, wall) == 0) && (maze_step[X - 1][Y] == MAX_STEP)) {
				maze_step[X - 1][Y] = maze_step[X][Y] + 1;
				distPositionList[tail] = ((X - 1) << 8) | (Y);
				tail++;
			}
		}

	}

}
#endif

int8_t getNextDir(uint8_t direction, uint8_t x, uint8_t y, t_walldata *wall, uint8_t maze_scale)
{
	uint16_t step = 0;
	uint8_t nextdir = 0, i = 0;
	int8_t a = 0, b = 0;

	if (direction == North) {
		i = 0;
		a = 1;
		b = 0;
	}
	else if (direction == West) {
		i = 1;
		a = 0;
		b = 1;
	}
	else if (direction == South) {
		i = 2;
		a = -1;
		b = 0;
	}
	else if (direction == East) {
		i = 3;
		a = 0;
		b = -1;
	}
	if ((y - a >= 0) && (x + b < maze_scale)) {
		if ((getWallData(x, y, (North + i) % 4, wall) == 1)
			&& (getWallData(x, y, (East + i) % 4, wall) == 1)
			&& (getWallData(x, y, (West + i) % 4, wall) == 1)) {
			nextdir = 2;
		}
		else {
			nextdir = 4;
		}
		step = maze_step[x + b][y - a];
	}
	if ((x - a >= 0) && (y - b >= 0)
		&& (getWallData(x, y, (West + i) % 4, wall) == 0)) {
		if (maze_step[x - a][y - b] <= step) {
			nextdir = 1;
			step = maze_step[x - a][y - b];
		}
	}
	if ((x + a < maze_scale) && (y + b < maze_scale)
		&& (getWallData(x, y, (East + i) % 4, wall) == 0)) {
		if (maze_step[x + a][y + b] <= step) {
			nextdir = 3;
			step = maze_step[x + a][y + b];
		}
	}
	if ((y + a < maze_scale) && (x - b >= 0)
		&& (getWallData(x, y, (North + i) % 4, wall) == 0)) {
		if (maze_step[x - b][y + a] <= step) {
			nextdir = 0;
			step = maze_step[x - b][y + a];
		}
	}

	if (step == MAX_STEP) {
		check_all_search_flag = 1;
	}

	return nextdir;
}

int8_t getNextDirKnown(uint8_t direction, uint8_t x, uint8_t y, t_walldata *wall, t_walldata *wall_bit, uint8_t maze_scale)
{
	uint16_t step = 0;
	uint8_t nextdir = 0, i = 0;
	int8_t a = 0, b = 0;
	int8_t boost = 0;

	if (direction == North) {
		i = 0;
		a = 1;
		b = 0;
	}
	else if (direction == West) {
		i = 1;
		a = 0;
		b = 1;
	}
	else if (direction == South) {
		i = 2;
		a = -1;
		b = 0;
	}
	else if (direction == East) {
		i = 3;
		a = 0;
		b = -1;
	}
	if ((y - a >= 0) && (x + b < maze_scale)) {
		if ((getWallData(x, y, (North + i) % 4, wall) == 1)
			&& (getWallData(x, y, (East + i) % 4, wall) == 1)
			&& (getWallData(x, y, (West + i) % 4, wall) == 1)) {
			nextdir = 2;
		}
		else {
			nextdir = 4;
		}
		step = maze_step[x + b][y - a];
	}
	if ((x - a >= 0) && (y - b >= 0)
		&& (getWallData(x, y, (West + i) % 4, wall) == 0)) {
		if (maze_step[x - a][y - b] <= step) {
			nextdir = 1;
			step = maze_step[x - a][y - b];
		}
	}
	if ((x + a < maze_scale) && (y + b < maze_scale)
		&& (getWallData(x, y, (East + i) % 4, wall) == 0)) {
		if (maze_step[x + a][y + b] <= step) {
			nextdir = 3;
			step = maze_step[x + a][y + b];
		}
	}
	if ((y + a < maze_scale) && (x - b >= 0)
		&& (getWallData(x, y, (North + i) % 4, wall) == 0)) {
		if (maze_step[x - b][y + a] <= step) {
			nextdir = 0;
			step = maze_step[x - b][y + a];
		}
	}

	if (nextdir == front && (getWallData(x, y, (North + i) % 4, wall_bit) == 0)) {
		while ((getWallData(x, y, (North + i) % 4, wall_bit) == 0) && (getNextDir(direction, x, y, wall_bit, maze_scale) == 0)) {
			boost++;
			if (direction == North && y < maze_scale - 1) {
				y++;
			}
			else if (direction == East && x < maze_scale - 1) {
				x++;
			}
			else if (direction == South && y > 0) {
				y--;
			}
			else if (direction == West && x > 0) {
				x--;
			}
			else {
				break;
			}
		}
		if (boost < 2) {
			nextdir = 0;
		}
		else {
			nextdir = boost + 10;	// +10はオフセット
		}

	}

	if (step == MAX_STEP) {
		check_all_search_flag = 1;
	}

	return nextdir;
}

/**
* 入力された次の動作がに対して座標の更新を行う
*/
void mazeUpdatePosition(uint8_t dir, t_position *pos)
{
	// 次の動作で向き、座標を更新する	
	if (dir == front) {
		switch (pos->direction) {
		case North:
			pos->y++;
			break;
		case East:
			pos->x++;
			break;
		case South:
			pos->y--;
			break;
		case West:
			pos->x--;
			break;
		default:
			break;
		}
	}
	else if (dir == left) {
		switch (pos->direction) {
		case North:
			pos->x--;
			pos->direction = West;
			break;
		case East:
			pos->y++;
			pos->direction = North;
			break;
		case South:
			pos->x++;
			pos->direction = East;
			break;
		case West:
			pos->y--;
			pos->direction = South;
			break;
		default:
			break;
		}
	}
	else if (dir == right) {
		switch (pos->direction) {
		case North:
			pos->x++;
			pos->direction = East;
			break;
		case East:
			pos->y--;
			pos->direction = South;
			break;
		case South:
			pos->x--;
			pos->direction = West;
			break;
		case West:
			pos->y++;
			pos->direction = North;
			break;
		default:
			break;
		}
	}
	else if (dir == rear) {
		switch (pos->direction) {
		case North:
			pos->y--;
			pos->direction = South;
			break;
		case East:
			pos->x--;
			pos->direction = West;
			break;
		case South:
			pos->y++;
			pos->direction = North;
			break;
		case West:
			pos->x++;
			pos->direction = East;
			break;
		default:
			break;
		}
	}
	else if (dir > 10) {
		// 既知区間加速
		for (int i = 10; i < dir; i++) {
			switch (pos->direction) {
			case North:
				pos->y++;
				break;
			case East:
				pos->x++;
				break;
			case South:
				pos->y--;
				break;
			case West:
				pos->x--;
				break;
			default:
				break;
			}
		}
	}
}

void mazeUpdateShortestMap(uint8_t gx, uint8_t gy, t_walldata *wall, uint8_t maze_scale)
{
	uint16_t distPositionList[1025];
	uint16_t head = 0, tail = 1;

	for (int x = 0; x < maze_scale; x++) {
		for (int y = 0; y < maze_scale; y++) {
			maze_step[x][y] = MAX_STEP;
		}
	}

	if (goal_size == 9) {

	}
	else if (goal_size == 4) {
		maze_step[gx + 1][gy] = 0;
		distPositionList[tail] = (gx + 1) * 256 + gy;
		tail++;
		maze_step[gx][gy + 1] = 0;
		distPositionList[tail] = gx * 256 + (gy + 1);
		tail++;
		maze_step[gx + 1][gy + 1] = 0;
		distPositionList[tail] = (gx + 1) * 256 + (gy + 1);
		tail++;
	}


	// goal座標は絶対一つあるのでその処理はここで行う。
	maze_step[gx][gy] = 0;
	distPositionList[0] = gx * 256 + gy;

	while (head != tail) {

		//head = tail なら更新する区画がない
		int8_t Y = distPositionList[head] & 0x00ff;
		int8_t X = (distPositionList[head] & 0xff00) >> 8;

		head++;

		// 北壁の情報を更新
		if (Y < maze_scale - 1) {
			if ((getWallData(X, Y, North, wall) == 0) && (maze_step[X][Y + 1] == MAX_STEP) && (searchCompartment(X, Y, wall) == 1)) {
				maze_step[X][Y + 1] = maze_step[X][Y] + 1;
				distPositionList[tail] = (X << 8) | (Y + 1);
				tail++;
			}
		}

		// 東壁の情報
		if (X < maze_scale - 1) {
			if ((getWallData(X, Y, East, wall) == 0) && (maze_step[X + 1][Y] == MAX_STEP) && (searchCompartment(X, Y, wall) == 1)) {
				maze_step[X + 1][Y] = maze_step[X][Y] + 1;
				distPositionList[tail] = ((X + 1) << 8) | (Y);
				tail++;
			}
		}

		// 南壁の情報を更新
		if (Y > 0) {
			if ((getWallData(X, Y, South, wall) == 0) && (maze_step[X][Y - 1] == MAX_STEP) && (searchCompartment(X, Y, wall) == 1)) {
				maze_step[X][Y - 1] = maze_step[X][Y] + 1;
				distPositionList[tail] = (X << 8) | (Y - 1);
				tail++;
			}
		}

		// 西壁の情報を更新
		if (X > 0) {
			if ((getWallData(X, Y, West, wall) == 0) && (maze_step[X - 1][Y] == MAX_STEP) && (searchCompartment(X, Y, wall) == 1)) {
				maze_step[X - 1][Y] = maze_step[X][Y] + 1;
				distPositionList[tail] = ((X - 1) << 8) | (Y);
				tail++;
			}
		}

	}

}

