#include "motion_primitives.h"
#include "sensors.h"
#include "maze.h"
#include "pose.h"
#include "bt.h"
#include <stdlib.h>

static void PrintMaze(void)
{
    Location robotLoc = Pose_GetLocation();

    for (int y = MAZE_HEIGHT - 1; y >= 0; y--)
    {
        for (int x = 0; x < MAZE_WIDTH; x++)
        {
            WallInfo w = Maze_GetWalls(Location_Make(x, y));
            BT_Sendf("+%s", (w.north != WALL_EXIT) ? "---" : "   ");
        }
        BT_SendLine("+");

        for (int x = 0; x < MAZE_WIDTH; x++)
        {
            WallInfo w = Maze_GetWalls(Location_Make(x, y));
            char cellMark = ' ';

            if (robotLoc.x == x && robotLoc.y == y)
            {
                cellMark = 'X';
            }

            BT_Sendf("%s %c ", (w.west != WALL_EXIT) ? "|" : " ", cellMark);
        }

        WallInfo lastCell = Maze_GetWalls(Location_Make(MAZE_WIDTH - 1, y));
        BT_SendLine((lastCell.east != WALL_EXIT) ? "|" : " ");
    }

    for (int x = 0; x < MAZE_WIDTH; x++)
    {
        WallInfo w = Maze_GetWalls(Location_Make(x, 0));
        BT_Sendf("+%s", (w.south != WALL_EXIT) ? "---" : "   ");
    }
    BT_SendLine("+");
}

static void SenseAndStoreWalls(void)
{
    Location loc = Pose_GetLocation();
    Heading hd = Pose_GetHeading();

    uint8_t wF = IR_WallFront_BG();
    uint8_t wL = IR_WallLeft_BG();
    uint8_t wR = IR_WallRight_BG();

    Heading front = hd;
    Heading left  = Heading_LeftFrom(hd);
    Heading right = Heading_RightFrom(hd);

    Maze_UpdateWallState(loc, front, wF ? WALL_PRESENT : WALL_EXIT);
    Maze_UpdateWallState(loc, left,  wL ? WALL_PRESENT : WALL_EXIT);
    Maze_UpdateWallState(loc, right, wR ? WALL_PRESENT : WALL_EXIT);

    BT_Sendf("sense (%d,%d) hd=%d F=%d L=%d R=%d\r\n",
             loc.x, loc.y, hd, wF, wL, wR);
}

static Heading PickDirection(void)
{
    Location loc = Pose_GetLocation();
    Heading hd = Pose_GetHeading();
    Heading cameFrom = Heading_BehindFrom(hd);
    Heading options[4];
    int count = 0;

    for (int h = HEADING_NORTH; h < HEADING_COUNT; h++)
    {
        Heading candidate = (Heading)h;
        if (Maze_IsExit(loc, candidate) && candidate != cameFrom)
        {
            options[count++] = candidate;
        }
    }

    if (count == 0)
    {
        if (Maze_IsExit(loc, cameFrom))
        {
            return cameFrom;
        }
        return HEADING_BLOCKED;
    }

    return options[rand() % count];
}

static void TurnToward(Heading target, float driftCorrection_deg)
{
    Heading hd = Pose_GetHeading();
    if (target == hd) return;

    if (target == Heading_RightFrom(hd))       TurnRightCorrected(driftCorrection_deg);
    else if (target == Heading_LeftFrom(hd))   TurnLeftCorrected(driftCorrection_deg);
    else if (target == Heading_BehindFrom(hd)) Turn180Corrected(driftCorrection_deg);

    Pose_SetHeading(target);
}

void DummyMaze_Run(int maxMoves)
{
    Maze_Init();
    Pose_Init();

    BT_SendLine("===== DUMMY MAZE EXPLORER START =====");

    for (int i = 0; i < maxMoves; i++)
    {
        BT_Sendf("\r\n--- move %d ---\r\n", i + 1);

        SenseAndStoreWalls();

        // Resquare against whatever front wall we just sensed, BEFORE turning
        // away from it. Returns the signed heading drift picked up during the
        // reverse leg (0.0f automatically if there was no front wall to
        // resquare against, since Run_ResquareTest skips and returns 0 then).
        //
        // Skipped entirely on the very first move: there's no prior motion
        // yet to have drifted from, so there's nothing to correct against —
        // and if we happen to start right up against a corner wall, this
        // avoids driving straight into it as the robot's first action.
        // IR steering will still correct any small placement error during
        // normal forward travel from here on.
        float resquareDrift = 0.0f;
        if (i > 0)
        {
            resquareDrift = Resquare();
        }

        Heading next = PickDirection();

        if (next == HEADING_BLOCKED)
        {
            BT_SendLine("Fully blocked — stopping.");
            break;
        }

        TurnToward(next, resquareDrift);

        MoveForward();

        Pose_AdvanceOneCell();

        Location loc = Pose_GetLocation();
        BT_Sendf("now at (%d,%d) heading=%d\r\n",
                 loc.x, loc.y, Pose_GetHeading());

        PrintMaze();
    }

    BT_SendLine("===== DUMMY MAZE EXPLORER DONE =====");
}
