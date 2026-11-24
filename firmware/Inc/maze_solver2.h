// maze_solver2.h
#ifndef MAZE_SOLVER2_H
#define MAZE_SOLVER2_H

#include <stdint.h>

// Continuous search (no stop between cells). Same idea as Maze_Run(), new motion.
void Maze_Run2(int maxMoves);

// Saved times (ms), valid after Maze_Run2 has run. 0 = not measured yet.
uint32_t Maze_GetSearchToCentreMs(void);   // search: start -> first entry into the centre (never changes)
uint32_t Maze_GetSearchReturnMs(void);     // search: centre -> back at the start (never changes)
uint32_t Maze_GetBestFastLapMs(void);      // fastest fast lap (start -> centre only)

#endif
