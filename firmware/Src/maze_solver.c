// maze_solver.c
#include "maze_solver.h"

#include "motion_primitives.h"
#include "sensors.h"
#include "maze.h"
#include "pose.h"
#include "bt.h"
#include "stm32f4xx.h"   // CHANGED: for DWT / CoreDebug / SystemCoreClock (timing)
// HAL_GetTick() comes from your HAL headers — usually already reachable via
// main.h or stm32f4xx_hal.h. Add whichever one your project uses if this
// doesn't resolve as-is.

// CHANGED: 1 = print the sense line, move header, "now at" line and the whole
// maze after every move (this is slow: blocking UART, ~440 bytes per move on
// a 6x6). 0 = silent search, only the start/center/finish messages and the
// timing report. Flip to 1 once to see exactly what the prints cost you in
// the "print" row of the timing report.
#define MAZE_PRINT_EACH_MOVE  0

// ───────────────────────────── CHANGED: stage timing ─────────────────────────
// Uses the Cortex-M4 cycle counter (the same one your SysTick_Handler uses),
// so results are wall-clock microseconds, including any interrupts that ran.
// Stats are collected silently and printed once at the end of Maze_Run().
typedef enum
{
    ST_SENSE,
    ST_RESQUARE,
    ST_FLOOD,
    ST_TURN,
    ST_MOVE,
    ST_PRINT,
    ST_COUNT
} Stage;

static const char *stageName[ST_COUNT] =
{
    "sense", "resquare", "flood", "turn", "move", "print"
};

static uint32_t stageTotal_us[ST_COUNT];
static uint32_t stageMax_us[ST_COUNT];
static uint32_t stageCount[ST_COUNT];

static void Timing_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // CYCCNT is deliberately NOT reset: SysTick_Handler measures with it too.

    for (int s = 0; s < ST_COUNT; s++)
    {
        stageTotal_us[s] = 0;
        stageMax_us[s] = 0;
        stageCount[s] = 0;
    }
}

static void Timing_Add(Stage s, uint32_t startCycles)
{
    uint32_t us = (DWT->CYCCNT - startCycles) / (SystemCoreClock / 1000000U);
    stageTotal_us[s] += us;
    stageCount[s]++;
    if (us > stageMax_us[s]) stageMax_us[s] = us;
}

// Wraps any statement(s) and records how long they took under `stage`.
#define TIMED(stage, ...) \
    do { uint32_t _t0 = DWT->CYCCNT; __VA_ARGS__; Timing_Add((stage), _t0); } while (0)

static void Timing_Report(void)
{
    BT_SendLine("--- stage timing (microseconds) ---");
    for (int s = 0; s < ST_COUNT; s++)
    {
        uint32_t avg = stageCount[s] ? (stageTotal_us[s] / stageCount[s]) : 0;
        BT_Sendf("%s n=%lu avg=%lu max=%lu total=%lu\r\n",
                 stageName[s],
                 (unsigned long)stageCount[s],
                 (unsigned long)avg,
                 (unsigned long)stageMax_us[s],
                 (unsigned long)stageTotal_us[s]);
    }

    // Time spent not moving between cells: everything except turn and move.
    // (Resquare is included even though part of it is physical motion.)
    uint32_t idle = stageTotal_us[ST_SENSE] + stageTotal_us[ST_RESQUARE]
                  + stageTotal_us[ST_FLOOD] + stageTotal_us[ST_PRINT];
    BT_Sendf("idle-ish total (sense+resquare+flood+print) = %lu us\r\n",
             (unsigned long)idle);
}
// ─────────────────────────────────────────────────────────────────────────────

typedef enum
{
    SEARCH_PHASE_TO_CENTER,
    SEARCH_PHASE_TO_START
} SearchPhase;

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

    if (MAZE_PRINT_EACH_MOVE)   // CHANGED: was unconditional (blocking UART)
    {
        BT_Sendf("sense (%d,%d) hd=%d F=%d L=%d R=%d\r\n",
                 loc.x, loc.y, hd, wF, wL, wR);
    }
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

static Location CurrentFloodTarget(SearchPhase phase)
{
    return (phase == SEARCH_PHASE_TO_CENTER) ? Maze_GetGoal() : START_LOCATION;
}

void Maze_Run(int maxMoves)
{
    Maze_Init();   // also runs an initial Maze_Flood() toward the goal
    Pose_Init();
    Timing_Init(); // CHANGED

    SearchPhase phase = SEARCH_PHASE_TO_CENTER;

    uint32_t searchStartTick = HAL_GetTick();
    uint32_t centerReachedTick = searchStartTick;

    BT_SendLine("===== FLOODFILL MAZE EXPLORER START =====");

    for (int i = 0; i < maxMoves; i++)
    {
        if (MAZE_PRINT_EACH_MOVE)   // CHANGED: was unconditional
        {
            TIMED(ST_PRINT,
                BT_Sendf("\r\n--- move %d (%s) ---\r\n",
                         i + 1, phase == SEARCH_PHASE_TO_CENTER ? "to-center" : "to-start");
            );
        }

        TIMED(ST_SENSE, SenseAndStoreWalls());   // CHANGED: timed

        Location loc = Pose_GetLocation();
        Location target = CurrentFloodTarget(phase);

        if (Location_Equals(loc, target))
        {
            if (phase == SEARCH_PHASE_TO_CENTER)
            {
                centerReachedTick = HAL_GetTick();
                uint32_t toCenter_ms = centerReachedTick - searchStartTick;
                BT_Sendf("Reached center — heading back to start. (%lu ms)\r\n",
                         (unsigned long)toCenter_ms);

                phase = SEARCH_PHASE_TO_START;
                continue;
            }
            else
            {
                BT_SendLine("Back at start — resquaring against the entrance wall before turning to face it.");
                float resquareDrift = Resquare();
                Turn180Corrected(resquareDrift);
                Pose_SetHeading(Heading_BehindFrom(Pose_GetHeading()));

                uint32_t doneTick = HAL_GetTick();
                uint32_t returnLeg_ms = doneTick - centerReachedTick;
                uint32_t total_ms = doneTick - searchStartTick;
                BT_Sendf("Search complete. Return leg: %lu ms. Total search: %lu ms.\r\n",
                         (unsigned long)returnLeg_ms, (unsigned long)total_ms);
                break;
            }
        }

        float resquareDrift = 0.0f;
        if (i > 0)
        {
            TIMED(ST_RESQUARE, resquareDrift = Resquare());   // CHANGED: timed
        }

        TIMED(ST_FLOOD, Maze_Flood(target));   // CHANGED: timed

        Heading next = Maze_HeadingToSmallest(loc, Pose_GetHeading());

        if (next == HEADING_BLOCKED)
        {
            BT_SendLine("Fully blocked — stopping.");
            break;
        }

        TIMED(ST_TURN, TurnToward(next, resquareDrift));   // CHANGED: timed
        TIMED(ST_MOVE, MoveForward());                     // CHANGED: timed
        Pose_AdvanceOneCell();

        loc = Pose_GetLocation();

        if (MAZE_PRINT_EACH_MOVE)   // CHANGED: was unconditional
        {
            TIMED(ST_PRINT,
                BT_Sendf("now at (%d,%d) heading=%d\r\n", loc.x, loc.y, Pose_GetHeading());
                PrintMaze();
            );
        }
    }

    Timing_Report();   // CHANGED

    BT_SendLine("===== FLOODFILL MAZE EXPLORER DONE =====");
}
