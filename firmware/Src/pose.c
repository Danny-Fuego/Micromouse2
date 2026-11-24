#include "pose.h"

static Location currentLoc;
static Heading  currentHeading;

void Pose_Init(void)
{
    currentLoc     = START_LOCATION;
    currentHeading = HEADING_NORTH;
}

Location Pose_GetLocation(void) { return currentLoc; }
Heading  Pose_GetHeading(void)  { return currentHeading; }

void Pose_SetLocation(Location loc) { currentLoc = loc; }
void Pose_SetHeading(Heading h)     { currentHeading = h; }

void Pose_AdvanceOneCell(void)
{
    currentLoc = Location_Neighbour(currentLoc, currentHeading);
}
