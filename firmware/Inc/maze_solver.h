// maze_solver.h
#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H

// Runs a real flood-fill search: explores from start to the maze goal
// cell, then floods back and drives home to (0,0). Stops early if the
// robot ever finds itself fully blocked. maxMoves is a hard safety cap
// on loop iterations (not physical cells) in case something goes wrong.
void Maze_Run(int maxMoves);

#endif /* MAZE_SOLVER_H */
