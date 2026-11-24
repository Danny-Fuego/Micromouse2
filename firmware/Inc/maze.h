#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	WALL_EXIT = 0,
	WALL_PRESENT = 1,
	WALL_UNKNOWN = 2,
	WALL_VIRTUAL = 3
} WallState;

typedef struct {
	uint8_t north : 2;
	uint8_t east : 2;
	uint8_t south : 2;
	uint8_t west : 2;
} WallInfo;

typedef enum {
    MASK_OPEN   = 0x01,
    MASK_CLOSED = 0x03
} MazeMask;

typedef enum {
	HEADING_NORTH = 0,
	HEADING_EAST,
	HEADING_SOUTH,
	HEADING_WEST,
	HEADING_COUNT,
	HEADING_BLOCKED = 99
} Heading;

Heading Heading_RightFrom(Heading h);
Heading Heading_LeftFrom(Heading h);
Heading Heading_AheadFrom(Heading h);
Heading Heading_BehindFrom(Heading h);

typedef enum {
	DIR_AHEAD,
	DIR_RIGHT,
	DIR_BACK,
	DIR_LEFT,
	DIRECTION_COUNT
} Direction;

#define MAZE_WIDTH 10
#define MAZE_HEIGHT 10
#define MAZE_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)
#define MAX_COST (MAZE_CELL_COUNT - 1)

typedef struct {
	uint8_t x;
	uint8_t y;
} Location;

Location Location_Make(uint8_t x, uint8_t y);
bool     Location_IsInMaze(Location loc);
bool     Location_Equals(Location a, Location b);
Location Location_North(Location loc);
Location Location_East(Location loc);
Location Location_South(Location loc);
Location Location_West(Location loc);
Location Location_Neighbour(Location loc, Heading heading);

#define START_LOCATION (Location_Make(0, 0))

void     Maze_Init(void);

Location Maze_GetGoal(void);
void     Maze_SetGoal(Location goal);
bool     Maze_IsGoal(Location loc);     // NEW: true for any cell of the goal block

WallInfo Maze_GetWalls(Location cell);
bool    Maze_HasUnkownWalls(Location cell);
uint8_t  Maze_WallCount(Location cell);
bool     Maze_CellIsVisited(Location cell);
bool     Maze_IsExit(Location cell, Heading heading);

void     Maze_UpdateWallState(Location cell, Heading heading, WallState state);
void     Maze_ForceWallState(Location cell, Heading heading, WallState state);   // NEW: overwrites a stored wall (used when stopped at a cell centre)

void     Maze_SetMask(MazeMask mask);
MazeMask Maze_GetMask(void);

uint16_t Maze_NeighbourCost(Location cell, Heading heading);
uint16_t Maze_Cost(Location cell);

void     Maze_Flood(Location target);
bool     Maze_FloodQueueOverflow(void);

Heading  Maze_HeadingToSmallest(Location cell, Heading start_heading);

#endif /* MAZE_H */
