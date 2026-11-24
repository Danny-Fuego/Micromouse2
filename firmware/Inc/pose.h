#ifndef POSE_H
#define POSE_H

#include "maze.h"

void Pose_Init(void);

Location Pose_GetLocation(void);
Heading  Pose_GetHeading(void);

void Pose_SetLocation(Location loc);
void Pose_SetHeading(Heading h);

// Advances location one cell in the current heading's direction —
// call after a confirmed forward move completes.
void Pose_AdvanceOneCell(void);

#endif
