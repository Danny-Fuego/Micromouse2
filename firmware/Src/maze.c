// maze.c
// Fixes applied:
//   1. Maze_FloodQueueOverflow() capitalization now matches maze.h
//   2. FLOOD_QUEUE_CAPACITY raised to MAZE_CELL_COUNT (was /4, could overflow silently)
//   3. Removed the hardcoded START_LOCATION east/north wall assumptions in
//      Maze_Init() — they were pre-marking the start cell's walls as
//      "known," which caused Maze_UpdateWallState() (which only writes
//      when a wall is still WALL_UNKNOWN) to silently discard the real
//      IR sensor reading on move 1. Now the start cell's walls stay
//      unknown like every other cell until actually sensed.
//   4. NEW: goal is now a BLOCK. Odd maze (5x5) = one centre cell, exactly as
//      before. Even maze (10x10, 16x16) = the 2x2 centre block; Maze_Flood()
//      seeds all four cells with cost 0 and Maze_IsGoal() is true for any of
//      them, so the mouse stops at the FIRST centre cell it reaches.

#include "maze.h"

Heading Heading_RightFrom(Heading h) { return (Heading)((h + 1) % HEADING_COUNT); }
Heading Heading_LeftFrom(Heading h) { return (Heading)((h + HEADING_COUNT - 1) % HEADING_COUNT); }
Heading Heading_AheadFrom(Heading h) { return h; }
Heading Heading_BehindFrom(Heading h) { return (Heading)((h + 2) % HEADING_COUNT); }

Location Location_Make(uint8_t x, uint8_t y) { Location loc; loc.x = x; loc.y = y; return loc; }
bool Location_IsInMaze(Location loc) { return (loc.x < MAZE_WIDTH) && (loc.y < MAZE_HEIGHT); }
bool Location_Equals(Location a, Location b) { return (a.x == b.x) && (a.y == b.y); }
Location Location_North(Location loc) { return Location_Make(loc.x, (loc.y + 1) % MAZE_HEIGHT); }
Location Location_East(Location loc) { return Location_Make((loc.x + 1) % MAZE_WIDTH, loc.y); }
Location Location_South(Location loc) { return Location_Make(loc.x, (loc.y + MAZE_HEIGHT - 1) % MAZE_HEIGHT); }
Location Location_West(Location loc) { return Location_Make((loc.x + MAZE_WIDTH - 1) % MAZE_WIDTH, loc.y); }

Location Location_Neighbour(Location loc, Heading heading)
{
	switch (heading)
	{
		case HEADING_NORTH: return Location_North(loc);
		case HEADING_EAST: return Location_East(loc);
		case HEADING_SOUTH: return Location_South(loc);
		case HEADING_WEST: return Location_West(loc);
		default: return loc;
	}
}

static WallInfo m_walls[MAZE_WIDTH][MAZE_HEIGHT];
static uint8_t m_cost[MAZE_WIDTH][MAZE_HEIGHT];
static MazeMask m_mask = MASK_OPEN;
static Location m_goal;
static uint8_t m_goal_x1, m_goal_x2, m_goal_y1, m_goal_y2;   // NEW: goal block (inclusive)
static bool m_flood_queue_overflow = false;

#define FLOOD_QUEUE_CAPACITY MAZE_CELL_COUNT

typedef struct {
	Location items[FLOOD_QUEUE_CAPACITY];
	uint16_t head;
	uint16_t count;
} FloodQueue;

static void FloodQueue_Init(FloodQueue *q) { q->head = 0; q->count = 0; }

static bool FloodQueue_Add(FloodQueue *q, Location loc)
{
	if (q->count >= FLOOD_QUEUE_CAPACITY) return false;
	uint16_t tail = (q->head + q->count) % FLOOD_QUEUE_CAPACITY;
	q->items[tail] = loc;
	q->count++;
	return true;
}

static Location FloodQueue_Pop(FloodQueue *q)
{
	Location loc = q->items[q->head];
	q->head = (q->head + 1) % FLOOD_QUEUE_CAPACITY;
	q->count--;
	return loc;
}

static uint16_t FloodQueue_Size(FloodQueue *q) { return q->count; }

static void Maze_SetWallState(Location loc, Heading heading, WallState state)
{
	switch (heading)
	{
		case HEADING_NORTH: { Location n = Location_North(loc); m_walls[loc.x][loc.y].north = state; m_walls[n.x][n.y].south = state; break; }
		case HEADING_EAST: { Location e = Location_East(loc); m_walls[loc.x][loc.y].east = state; m_walls[e.x][e.y].west = state; break; }
		case HEADING_WEST: { Location w = Location_West(loc); m_walls[loc.x][loc.y].west = state; m_walls[w.x][w.y].east = state; break; }
		case HEADING_SOUTH: { Location s = Location_South(loc); m_walls[loc.x][loc.y].south = state; m_walls[s.x][s.y].north = state; break; }
		default: break;
	}
}

void Maze_Init(void)
{
	for (int x = 0; x < MAZE_WIDTH; x++)
		for (int y = 0; y < MAZE_HEIGHT; y++)
		{
			m_walls[x][y].north = WALL_UNKNOWN;
			m_walls[x][y].east = WALL_UNKNOWN;
			m_walls[x][y].south = WALL_UNKNOWN;
			m_walls[x][y].west = WALL_UNKNOWN;
		}

	for (int x = 0; x < MAZE_WIDTH; x++)
	{
		m_walls[x][0].south  = WALL_PRESENT;
		m_walls[x][MAZE_HEIGHT - 1].north = WALL_PRESENT;
	}
	for (int y = 0; y < MAZE_HEIGHT; y++)
	{
		m_walls[0][y].west = WALL_PRESENT;
		m_walls[MAZE_WIDTH - 1][y].east = WALL_PRESENT;
	}

	m_goal = Location_Make(MAZE_WIDTH / 2, MAZE_HEIGHT / 2);
	// NEW: even size -> 2x2 centre block, odd size -> the single centre cell
	m_goal_x2 = (uint8_t)(MAZE_WIDTH / 2);
	m_goal_y2 = (uint8_t)(MAZE_HEIGHT / 2);
	m_goal_x1 = (MAZE_WIDTH  % 2 == 0) ? (uint8_t)(m_goal_x2 - 1) : m_goal_x2;
	m_goal_y1 = (MAZE_HEIGHT % 2 == 0) ? (uint8_t)(m_goal_y2 - 1) : m_goal_y2;

	Maze_SetMask(MASK_OPEN);
	Maze_Flood(Maze_GetGoal());
}

Location Maze_GetGoal(void) { return m_goal; }
void Maze_SetGoal(Location goal)
{
	m_goal = goal;
	// NEW: a goal set by hand is a single cell
	m_goal_x1 = m_goal_x2 = goal.x;
	m_goal_y1 = m_goal_y2 = goal.y;
}

// NEW: true if loc is inside the goal block
bool Maze_IsGoal(Location loc)
{
	return (loc.x >= m_goal_x1) && (loc.x <= m_goal_x2) &&
	       (loc.y >= m_goal_y1) && (loc.y <= m_goal_y2);
}
WallInfo Maze_GetWalls(Location cell) { return m_walls[cell.x][cell.y]; }

bool Maze_HasUnkownWalls(Location cell)
{
	WallInfo w = m_walls[cell.x][cell.y];
	return (w.north == WALL_UNKNOWN) || (w.east == WALL_UNKNOWN) ||
			(w.south == WALL_UNKNOWN) || (w.west == WALL_UNKNOWN);
}

uint8_t Maze_WallCount(Location cell)
{
	uint8_t count = 4;
	if (Maze_IsExit(cell, HEADING_NORTH)) count--;
	if (Maze_IsExit(cell, HEADING_EAST)) count--;
	if (Maze_IsExit(cell, HEADING_SOUTH)) count--;
	if (Maze_IsExit(cell, HEADING_WEST)) count--;
	return count;
}

bool Maze_CellIsVisited(Location cell) { return !Maze_HasUnkownWalls(cell); }

bool Maze_IsExit(Location cell, Heading heading)
{
	WallInfo w = m_walls[cell.x][cell.y];
	uint8_t wallVal;
	switch (heading)
	{
		case HEADING_NORTH: wallVal = w.north; break;
		case HEADING_EAST: wallVal = w.east; break;
		case HEADING_SOUTH: wallVal = w.south; break;
		case HEADING_WEST: wallVal = w.west; break;
		default: return false;
	}
	return (wallVal & m_mask) == WALL_EXIT;
}

void Maze_UpdateWallState(Location cell, Heading heading, WallState state)
{
	WallInfo w = m_walls[cell.x][cell.y];
	uint8_t current;
	switch (heading)
	{
		case HEADING_NORTH: current = w.north; break;
		case HEADING_EAST: current = w.east; break;
		case HEADING_SOUTH: current = w.south; break;
		case HEADING_WEST: current = w.west; break;
		default: return;
	}
	if (current != WALL_UNKNOWN) return;
	Maze_SetWallState(cell, heading, state);
}

// NEW: like Maze_UpdateWallState but it OVERWRITES what is already stored. Used only when the mouse is
// standing still at a cell centre, where the side IR reads are trustworthy (the reads taken while driving into the
// cell can be stale, and Maze_UpdateWallState keeps the first reading forever). Never touches the outer border.
void Maze_ForceWallState(Location cell, Heading heading, WallState state)
{
	switch (heading)
	{
		case HEADING_NORTH: if ((cell.y + 1) >= MAZE_HEIGHT) return; break;
		case HEADING_EAST:  if ((cell.x + 1) >= MAZE_WIDTH)  return; break;
		case HEADING_SOUTH: if (cell.y == 0) return; break;
		case HEADING_WEST:  if (cell.x == 0) return; break;
		default: return;
	}
	Maze_SetWallState(cell, heading, state);
}

void Maze_SetMask(MazeMask mask) { m_mask = mask; }
MazeMask Maze_GetMask(void) { return m_mask; }

uint16_t Maze_NeighbourCost(Location cell, Heading heading)
{
	if (!Maze_IsExit(cell, heading)) return MAX_COST;
	Location next = Location_Neighbour(cell, heading);
	return m_cost[next.x][next.y];
}

uint16_t Maze_Cost(Location cell) { return m_cost[cell.x][cell.y]; }

void Maze_Flood(Location target)
{
	for (int x = 0; x < MAZE_WIDTH; x++)
		for (int y = 0; y < MAZE_HEIGHT; y++)
			m_cost[x][y] = (uint8_t)MAX_COST;

	m_flood_queue_overflow = false;

	FloodQueue queue;
	FloodQueue_Init(&queue);

	if (Location_Equals(target, m_goal))
	{
		// NEW: flooding to the goal seeds every cell of the goal block with 0
		for (int gx = m_goal_x1; gx <= m_goal_x2; gx++)
			for (int gy = m_goal_y1; gy <= m_goal_y2; gy++)
			{
				Location g = Location_Make((uint8_t)gx, (uint8_t)gy);
				m_cost[g.x][g.y] = 0;
				FloodQueue_Add(&queue, g);
			}
	}
	else
	{
		m_cost[target.x][target.y] = 0;
		FloodQueue_Add(&queue, target);
	}

	while (FloodQueue_Size(&queue) > 0)
	{
		Location here = FloodQueue_Pop(&queue);
		uint16_t newCost = m_cost[here.x][here.y] + 1;

		for (int h = HEADING_NORTH; h < HEADING_COUNT; h++)
		{
			Heading heading = (Heading)h;
			if (Maze_IsExit(here, heading))
			{
				Location next = Location_Neighbour(here, heading);
				if (m_cost[next.x][next.y] > newCost)
				{
					m_cost[next.x][next.y] = (uint8_t)newCost;
					if (!FloodQueue_Add(&queue, next))
						m_flood_queue_overflow = true;
				}
			}
		}
	}
}

bool Maze_FloodQueueOverflow(void) { return m_flood_queue_overflow; }

Heading Maze_HeadingToSmallest(Location cell, Heading start_heading)
{
	Heading best_heading = HEADING_BLOCKED;
	uint16_t best_cost = Maze_Cost(cell);
	uint16_t cost;

	Heading next_heading = start_heading;
	cost = Maze_NeighbourCost(cell, next_heading);
	if (cost < best_cost) { best_cost = cost; best_heading = next_heading; }

	next_heading = Heading_RightFrom(start_heading);
	cost = Maze_NeighbourCost(cell, next_heading);
	if (cost < best_cost) { best_cost = cost; best_heading = next_heading; }

	next_heading = Heading_LeftFrom(start_heading);
	cost = Maze_NeighbourCost(cell, next_heading);
	if (cost < best_cost) { best_cost = cost; best_heading = next_heading; }

	next_heading = Heading_BehindFrom(start_heading);
	cost = Maze_NeighbourCost(cell, next_heading);
	if (cost < best_cost) { best_cost = cost; best_heading = next_heading; }

	if (best_cost == MAX_COST) best_heading = HEADING_BLOCKED;

	return best_heading;
}
