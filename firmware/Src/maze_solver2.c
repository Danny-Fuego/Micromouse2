// maze_solver2.c
// ============================================================================
// CONTINUOUS SEARCH (no stop between cells) at the slow 300 mm/s speed.
// This is a NEW file. maze_solver.c is untouched; call Maze_Run2() instead of
// Maze_Run() to try it. Same maze / pose / flood-fill / sensor / turn code.
//
// HOW IT MOVES
//   - One long forward profile keeps running (topped up with
//     RampProfile_AppendMove), so the mouse never stops between cells.
//   - Everything is measured in S = encoder path (mm) since the chain started.
//   - For each new cell B ahead we have its centre at S = sCenterNext.
//       decision point  = B centre - 127 + S2_DECISION_DEPTH_MM
//         -> Pose_AdvanceOneCell(), sense B's 3 walls, flood, pick next heading
//       STRAIGHT        -> sCenterNext += 254, keep going
//       LEFT / RIGHT    -> start the arc when S = B centre - S2_ARC_FWD_MM
//                          (+ S2_ARC_EXTRA_MM), wait for it to finish, then
//                          the next cell centre is arc end + (254 - ARC_SIDE)
//       BACK / blocked / target reached -> stop at B centre (abrupt stop, the
//                          same kind your MoveForward gives), return to the
//                          outer loop which resquares / spins / restarts.
//   - Arcs use the same call as the path test: RampProfile_SetMove on the
//     rotation profile, on top of the running forward profile.
//
// NUMBERS COME FROM YOUR LOGS (300 mm/s, rotation 2000/2000, max omega 300):
//   arc 98 fwd x 92 side at a true 90 deg, 155 mm of path to the true 90.
//
// THINGS I COULD NOT CHECK (see the notes in the reply):
//   - left arcs are assumed to be the mirror of right arcs
//   - S2_ARC_EXTRA_MM: the box test decides whether the arc must start later
//   - no re-sync inside long straights yet (drift only resets at stops)
// ============================================================================
#include "maze_solver2.h"
#include "maze_solver.h"

#include "motion_primitives.h"
#include "sensors.h"
#include "maze.h"
#include "pose.h"
#include "bt.h"
#include "stm32f4xx.h"
#include "main.h"            // BUTTON_GPIO_Port / BUTTON_Pin (button abort)
#include "ramp_profile.h"
#include "motion_controller.h"
#include "motion_state.h"
#include "encoder.h"
#include "motor.h"
#include "battery.h"

// the planner is a single header: the implementation is compiled in THIS file only
#define FAST_ROUTE_IMPLEMENTATION
#include "fast_route.h"
#include <math.h>
#include <stdint.h>

// defined elsewhere (your SysTick edit / calibration.c): 1 = IR steering off during arcs
extern volatile uint8_t g_irSteerOffInArc;

// ───────────────────────────── tunables ──────────────────────────────────────
#define S2_SPEED_MM_S           300.0f   // cruise speed (forward approachSpeed stays 300)
#define S2_MAX_OMEGA            300.0f   // peak turn rate in the arcs, deg/s
// Fast laps get their own speed and turn rate. Both start equal to the search values, so nothing changes until you raise them.
// (The arc constants below were measured at the search values: re-measure them when the fast values change.)
// DEMO SWITCH: 1 = fast laps run at the 300 search values (speed, turn rate, comp, arc geometry) for a before/after video. 0 = normal 400 fast laps.
#define S2_FAST_DEMO_300        0
#define S2_FINISH_COMP_DEG      3.0f     // same as PT_FINISH_COMP_DEG in the path test
#if S2_FAST_DEMO_300
#define S2_FAST_SPEED_MM_S      300.0f
#define S2_FAST_MAX_OMEGA       300.0f
#define S2_FAST_FINISH_COMP_DEG 3.0f
#else
#define S2_FAST_SPEED_MM_S      400.0f   // cruise speed on the fast laps
#define S2_FAST_MAX_OMEGA       400.0f   // peak arc turn rate on the fast laps, deg/s
#define S2_FAST_FINISH_COMP_DEG 1.5f     // the same setting for the fast laps (measured at 400 mm/s with PT_FINISH_COMP_DEG = 1.5)
#endif
#define S2_IR_OFF_IN_ARC        1        // same as IR_OFF_IN_ARC = 1 in your runs

#define S2_CELL_MM              254.0f
#define S2_ARC_FWD_MM           98.2f    // forward travel of the arc at a true 90 deg (measured)
#define S2_ARC_SIDE_MM          92.3f    // sideways travel at a true 90 deg (measured)
#define S2_ARC_PATH_TRUE_MM     155.3f   // encoder path from arc start to the true 90 deg
#define S2_ARC_EXTRA_MM         35.0f    // +x = start every arc x mm LATER than the geometric start (29 mm past the cell edge)
// Arc geometry for the FAST laps (S2_RunPathChain only). Same four numbers as above, measured at the fast speed and turn rate.
// Equal to the 300 mm/s values until you measure the fast arc (TEST_PATH in calibration.c) and paste the numbers here.
#if S2_FAST_DEMO_300
#define S2_FAST_ARC_FWD_MM       98.2f
#define S2_FAST_ARC_SIDE_MM      92.3f
#define S2_FAST_ARC_PATH_TRUE_MM 155.3f
#define S2_FAST_ARC_EXTRA_MM     35.0f
#else
#define S2_FAST_ARC_FWD_MM       111.0f  // forward travel of the fast arc at a true 90 deg
#define S2_FAST_ARC_SIDE_MM      94.5f   // sideways travel of the fast arc at a true 90 deg
#define S2_FAST_ARC_PATH_TRUE_MM 174.0f  // encoder path from arc start to the true 90 deg, fast arc
#define S2_FAST_ARC_EXTRA_MM     35.0f   // +x = start every fast arc x mm LATER than the geometric start
#endif

// how far past the cell edge (wheel centre) the mouse is when it senses + decides.
// - The front IR only flags a wall (about 6.5 in = 165 mm) once the wheels are ~42 mm into the cell.
// - The SIDE sensors keep reading the PREVIOUS cell's wall for about 40 mm past the boundary and need about
//   25 mm more to climb to "open" (log: irL 2.88 -> 5.08 -> 7.23 -> 11.04 in over S 1921..1968 at (4,0), and the
//   same at (7,1)). Deciding at 50 mm caught them mid-climb (5.0 in) and stored a false wall. At 75 mm both read
//   clearly open (9+ in). Was 50.0f.
// - The arc starts at max(the decision, 29 + S2_ARC_EXTRA_MM = 64 mm past the edge), so with 75 the arcs start
//   about 11 mm later than before.
// 75 mm did NOT fix the stale side reads (the lag is different every run: at (4,0) the left flag flipped 86 mm into
// the cell this run, 60 mm last run) and it starts every arc ~11 mm late, which pushes the path into inside corners.
// So it is back at 50. The wrong-wall problem is now caught at the cell CENTRE instead (see S2_CENTRE_RECHECK).
#define S2_DECISION_DEPTH_MM    50.0f

// safety: front IR closer than this while cruising / waiting for an arc = about to hit the wall. Stop.
#define S2_FRONT_ABORT_IN       1.3f

// FRONT WALL AT A DECISION (new): a wall counts as "ahead" only if the front IR reads this close or closer.
// Why: the front sensor's calibration ends at 10 in, where the raw value sits on the noise floor, so a wall TWO
// cells ahead already reads 8.6-9.9 in once the decision is at 75 mm. The old flag (ON at 10 in) called that a wall
// and the mouse stored a false front wall at (0,1). A real wall one cell ahead reads about 3.4-6.0 in at the decision.
// 7.5 sits between the two. Used only for the decision / centre sensing; the global IR_WallFront_BG() is untouched.
// 10 was too close: the wall two cells ahead (seen from the 2nd cell) reads 9.5-10.8 and tripped it. So it stays
// at 7.5. A real wall that reads above 7.5 (it happened at (7,1): 7.88) is caught by the CENTRE RE-CHECK below,
// which stops the mouse before it reaches the wall.
#define S2_FRONT_WALL_DECISION_IN   7.5f

// CENTRE RE-CHECK (new): when the mouse drives STRAIGHT through a cell, it reads the three walls again when it
// is at that cell's centre (the stale side reads are long gone by then, and a front wall is only ~127 mm ahead).
// If the corrected map makes the flood pick a different move than "straight", it stops right there (it still has
// ~110 mm to the wall), turns on the spot and carries on. 1 = on, 0 = off.
#define S2_CENTRE_RECHECK       1

// The stop code never stops in less than S2_STOP_MIN_MM (15 mm), so a re-check that FIRES at the centre stops the
// mouse 15+ mm PAST the centre, and the spin turn then starts off-centre (it clipped the wall at (6,6)).
// Fix: fire the re-check this many mm BEFORE the centre and ask to stop AT the centre. Must be >= S2_STOP_MIN_MM.
// (The stops at the end of a run land within ~2 mm of their target, so this should put the mouse on the centre.)
#define S2_RECHECK_LEAD_MM      20.0f

// (Used ONLY when there is NO wall in front at the re-check; with a front wall it stops on the centre and resquares.)
// Where the re-check stop should END, in mm PAST the cell centre as the odometry sees it. The mouse physically reaches
// the real centre LATER than S says (it turned early and clipped the wall at (7,7), about 8 runs in 10), so ask for
// the stop a little further on. Tune it: still turning early -> raise it, turning late -> lower it.
// The re-check line in the log prints where the stop actually ended (S) so you can see what it did.
#define S2_REPLAN_STOP_BIAS_MM  30.0f

// ── DIAGNOSTICS (new) ──
// 1 = when the mouse is stopped at a cell centre, the walls it reads there OVERWRITE what the map already holds
//     (the map normally keeps the FIRST reading of a wall forever, even a stale one from the decision point).
// 0 = old behaviour (first reading wins). The log still shows when the centre reading differs from the stored one.
#define S2_FORCE_CENTER_WALLS   1

// SIDE-IR HISTORY (new, diagnostic only): the decisions table gets two extra columns, irL and irR as they were
// S2_HIST_LOOKBACK_MM before the decision point. Compare them with the readings AT the decision: in a cell where a
// side wall just ended, a reading that is already climbing = the wall is ending; a flat wall-like reading = the
// sensor still really sees the wall. The history is recorded every S2_HIST_STEP_MM of travel.
#define S2_HIST_STEP_MM         5.0f
#define S2_HIST_LOOKBACK_MM     25.0f
#define S2_HIST_SIZE            16       // 16 x 5 mm = 80 mm of history

// WHERE THE MOUSE STANDS AT THE VERY START, measured BACKWARD from the centre of the start cell.
//   0.0   = you placed the mouse on the centre of the start cell
//   127.0 = you placed it against the back wall of the start cell (normal micromouse start)
// Every later restart (after a dead end / goal) is at a cell centre, so this only affects
// the first run down the chain.
#define S2_START_BEHIND_CENTER_MM  0.0f

#define S2_LOOKAHEAD_MM         600.0f   // length given to the forward profile
#define S2_MIN_REMAINING_MM     350.0f   // top it up when less than this is left
#define S2_STOP_MIN_MM          15.0f    // never ask to stop in less than this
#define S2_CHAIN_TIMEOUT_MS     60000U
#define S2_WAIT_TIMEOUT_MS      6000U

#define S2_MAX_EVENTS           80

// detailed motion log (RAM only, printed after the run, same idea as your path-test log)
#define S2_LOG_INTERVAL_MS      40       // one sample every 40 ms (search)
#define S2_FAST_LOG_INTERVAL_MS 20       // one sample every 20 ms on the fast laps: the ring then holds the last 300 x 20 ms = 6 s
#define S2_LOG_MAX_SAMPLES      300      // 300 x 40 ms = 12 s (about 27 KB of RAM; lower it if the linker complains)
#define S2_HAVE_IR_F            1        // 1 = log IR_GetDistanceF_BG(). Set 0 if the compiler says it doesn't exist

// safety: if the forward profile is asking for speed but the wheels have not moved this long, the mouse
// has hit something. Stop and print the log instead of pushing on a wall until the 60 s timeout.
#define S2_STALL_MS             600U

// ── BUTTON ABORT (new) ──
// Press the user button while the mouse is running: it brakes at once, then the program prints the
// decision table + motion log, so you never have to hit reset (which throws the log away).
// The logs are now RING buffers: they keep the LAST S2_MAX_EVENTS decisions / S2_LOG_MAX_SAMPLES samples,
// so the moment just before you press the button is always in the printout.
// If the abort fires by itself at the start, or never fires, the polarity is wrong: flip this to GPIO_PIN_SET.
#define S2_BUTTON_PRESSED_LEVEL GPIO_PIN_RESET   // RESET = button pulls the pin low (typical); SET = pulls high
#define S2_BUTTON_DEBOUNCE_MS   30U
// ─────────────────────────────────────────────────────────────────────────────

extern volatile float g_currentHeading;

typedef enum { S2_SEARCH_TO_CENTER, S2_SEARCH_TO_START } S2_Phase;

typedef enum
{
    S2_STOP_DEADEND,    // next move is "back": resquare + spin 180, then restart
    S2_STOP_TARGET,     // reached the flood target
    S2_STOP_BLOCKED,    // flood says fully blocked
    S2_STOP_MAXMOVES,   // move budget used up
    S2_STOP_TIMEOUT,    // safety abort
    S2_STOP_STALL,      // wheels stopped while the profile was still moving (hit a wall?)
    S2_STOP_REPLAN      // the centre re-check changed the best move: stopped at the centre to turn
} S2_StopReason;

typedef enum { S2_ACT_STRAIGHT, S2_ACT_LEFT, S2_ACT_RIGHT, S2_ACT_STOP, S2_ACT_CENTER, S2_ACT_REPLAN } S2_Action;

// state codes written into every motion sample
#define S2_ST_IDLE      0   // not in a chain
#define S2_ST_CRUISE    1   // running straight, waiting for the next decision point
#define S2_ST_ARCWAIT   2   // decided to turn, waiting for the arc start distance
#define S2_ST_ARC       3   // arc running
#define S2_ST_STOPPING  4   // stopping at a cell centre
#define S2_ST_STALLED   5   // stall detected

// ── event log (RAM only, printed after the run - no printing while moving) ──
typedef struct
{
    uint32_t  t_ms;           // 32 bit: a 16 bit time wraps after 65 s, which is shorter than the search
    int8_t    x, y;
    int8_t    heading;
    uint8_t   wF, wL, wR;
    uint8_t   action;
    int16_t   s_mm;
    int16_t   decide_us;
    float     irF;
    float     irL, irR;       // NEW: side readings at the moment of the decision
    float     irL_prev, irR_prev;   // NEW: side readings S2_HIST_LOOKBACK_MM of travel BEFORE the decision
} S2_Event;

static S2_Event s2_events[S2_MAX_EVENTS];
static int      s2_eventCount = 0;
static uint32_t s2_decideMax_us = 0;
static uint32_t s2_runStartTick = 0;

// set just before a decision is logged (see S2_HistLookup), cleared again by S2_LogEvent
static float    s2_evIrLprev = 0.0f;
static float    s2_evIrRprev = 0.0f;

static void S2_LogEvent(uint8_t wF, uint8_t wL, uint8_t wR, S2_Action act,
                        float s, uint32_t decide_us, float irF)
{
    Location loc = Pose_GetLocation();
    S2_Event *e = &s2_events[s2_eventCount % S2_MAX_EVENTS];   // ring buffer: keeps the last S2_MAX_EVENTS
    s2_eventCount++;
    e->t_ms      = (uint32_t)(HAL_GetTick() - s2_runStartTick);
    e->x         = (int8_t)loc.x;
    e->y         = (int8_t)loc.y;
    e->heading   = (int8_t)Pose_GetHeading();
    e->wF = wF; e->wL = wL; e->wR = wR;
    e->action    = (uint8_t)act;
    e->s_mm      = (int16_t)s;
    e->decide_us = (int16_t)((decide_us > 32000U) ? 32000U : decide_us);
    e->irF       = irF;
    e->irL       = IR_GetDistanceL_BG();
    e->irR       = IR_GetDistanceR_BG();
    e->irL_prev  = s2_evIrLprev;
    e->irR_prev  = s2_evIrRprev;
    s2_evIrLprev = 0.0f;           // only decisions fill these; the centre / re-check lines print 0.00
    s2_evIrRprev = 0.0f;
}

// ── arc start records (new): where each arc really started and what triggered it ──
#define S2_MAX_ARCS 40
typedef struct
{
    int8_t   x, y, heading;
    int16_t  sNominal;     // S where the wheel count wanted the arc
    int16_t  sActual;      // S where the arc really started
    float    irF;          // front reading at the moment the arc started
    float    irL, irR;     // side readings at the moment the arc started
    int16_t  toCentre;     // believed distance from the arc start to the cell centre (mm)
} S2_ArcRec;
static S2_ArcRec s2_arcs[S2_MAX_ARCS];
static int       s2_arcCount = 0;

// ── detailed motion samples ──
typedef struct
{
    uint32_t t_ms;           // 32 bit (see S2_Event)
    uint8_t  state;
    int8_t   cx, cy, hd;          // cell and heading the POSE thinks we are in
    uint8_t  wF, wL, wR;          // live IR wall flags
    float    S;                   // encoder path since the chain started
    float    sCenterNext;         // where the code thinks the next cell centre is
    float    sArcTarget;          // S at which the pending arc starts (0 if none)
    float    targetPos, targetSpeed;
    float    measSpeed;
    float    heading;             // gyro heading
    float    rotError, posOutput, rotOutput;
    float    leftV, rightV;
    float    irL, irR, irF;
    float    steerAdj, rotTarget;
} S2_Sample;

static float     s2_S;                // defined again below with the odometry (same variable)
static S2_Sample s2_samples[S2_LOG_MAX_SAMPLES];
static int       s2_sampleCount = 0;
static uint32_t  s2_lastSampleTick = 0;
static uint8_t   s2_state = S2_ST_IDLE;
static float     s2_sCenterNext = 0.0f;
static float     s2_sArcTarget = 0.0f;
static float     s2_measSpeed = 0.0f;
static uint32_t  s2_speedTick = 0;
static float     s2_speedS = 0.0f;
static uint32_t  s2_stallStart = 0;
static uint8_t   s2_frontDanger = 0;
static float     s2_headRef = 0.0f;       // gyro heading at the start of the run = 'north' (frame check)
static uint8_t   s2_replanFrontWall = 0;   // 1 = the re-check stop had a wall in front (square on it, no extra forward)
static float     s2_replanOff = 0.0f;     // how far past the centre the last re-check stop ended (mm)
static uint8_t   s2_userAbort = 0;        // set when the button was pressed during a run (or the run clock ran out)
// Speed and turn rate in use right now: set to the search values at the start of Maze_Run2, to the fast values at the start of S2_FastRun.
static float     s2_cruiseMmS = S2_SPEED_MM_S;
static float     s2_maxOmega  = S2_MAX_OMEGA;
static uint32_t  s2_logIntervalMs = S2_LOG_INTERVAL_MS;   // search value; the fast run switches it to S2_FAST_LOG_INTERVAL_MS
static float     s2_finishComp = S2_FINISH_COMP_DEG;   // search value; the fast run switches it to S2_FAST_FINISH_COMP_DEG
static uint8_t   s2_timeUp = 0;           // set when the run clock (main.c / SysTick) ran out: g_runExpired
static uint8_t   s2_btnEnabled = 0;       // 0 if the button polarity looks wrong (see Maze_Run2)
static uint32_t  s2_btnSince = 0;

// ── side-IR history (new): S, irL, irR recorded every S2_HIST_STEP_MM of travel ──
typedef struct { float s; float irL; float irR; } S2_HistRec;
static S2_HistRec s2_hist[S2_HIST_SIZE];
static int        s2_histCount = 0;
static float      s2_histLastS = 0.0f;

static void S2_HistReset(void)
{
    s2_histCount = 0;
    s2_histLastS = 0.0f;
}

static void S2_HistRecord(void)
{
    if ((s2_histCount == 0) || ((s2_S - s2_histLastS) >= S2_HIST_STEP_MM))
    {
        S2_HistRec *h = &s2_hist[s2_histCount % S2_HIST_SIZE];
        h->s   = s2_S;
        h->irL = IR_GetDistanceL_BG();
        h->irR = IR_GetDistanceR_BG();
        s2_histCount++;
        s2_histLastS = s2_S;
    }
}

// Finds the newest record taken at least S2_HIST_LOOKBACK_MM of travel before now and parks its readings in
// s2_evIrLprev / s2_evIrRprev for the next S2_LogEvent. If the history is not long enough yet, it uses the oldest record.
static void S2_HistLookup(void)
{
    int n = (s2_histCount < S2_HIST_SIZE) ? s2_histCount : S2_HIST_SIZE;
    if (n == 0) { s2_evIrLprev = 0.0f; s2_evIrRprev = 0.0f; return; }

    float want = s2_S - S2_HIST_LOOKBACK_MM;
    int   first = s2_histCount - n;               // index of the oldest record
    const S2_HistRec *pick = &s2_hist[first % S2_HIST_SIZE];
    for (int k = first; k < s2_histCount; k++)
    {
        const S2_HistRec *h = &s2_hist[k % S2_HIST_SIZE];
        if (h->s <= want) pick = h;               // records are in increasing S, so the last match is the newest
    }
    s2_evIrLprev = pick->irL;
    s2_evIrRprev = pick->irR;
}

static void S2_LogSample(void)
{
    Location loc = Pose_GetLocation();
    S2_Sample *p = &s2_samples[s2_sampleCount % S2_LOG_MAX_SAMPLES];   // ring buffer: keeps the last S2_LOG_MAX_SAMPLES
    s2_sampleCount++;

    p->t_ms        = (uint32_t)(HAL_GetTick() - s2_runStartTick);
    p->state       = s2_state;
    p->cx          = (int8_t)loc.x;
    p->cy          = (int8_t)loc.y;
    p->hd          = (int8_t)Pose_GetHeading();
    p->wF          = IR_WallFront_BG();
    p->wL          = IR_WallLeft_BG();
    p->wR          = IR_WallRight_BG();
    p->S           = s2_S;
    p->sCenterNext = s2_sCenterNext;
    p->sArcTarget  = s2_sArcTarget;
    p->targetPos   = RampProfile_GetPosition(&forwardProfile);
    p->targetSpeed = RampProfile_GetSpeed(&forwardProfile);
    p->measSpeed   = s2_measSpeed;
    p->heading     = g_currentHeading;
    p->rotError    = MotionController_GetLastRotError();
    p->posOutput   = MotionController_GetLastPosOutput();
    p->rotOutput   = MotionController_GetLastRotOutput();
    p->leftV       = MotionController_GetLastLeftVolts();
    p->rightV      = MotionController_GetLastRightVolts();
    p->irL         = IR_GetDistanceL_BG();
    p->irR         = IR_GetDistanceR_BG();
#if S2_HAVE_IR_F
    p->irF         = IR_GetDistanceF_BG();
#else
    p->irF         = 0.0f;
#endif
    p->steerAdj    = MotionController_GetLastSteeringAdjustment();
    p->rotTarget   = RampProfile_GetSpeed(&rotationProfile);
}

// ── wall-flag changes (new): every time a wall flag flips while driving, remember where (S) and the cell the
//    code believes it is in. A wall that ENDS is a physical cell boundary, so "mm_to_believed_centre" at that
//    moment should be the same number every time (about 127 minus the sensor lag). If it drifts, the odometry drifts. ──
#define S2_MAX_EDGES 80
typedef struct
{
    int8_t   x, y;
    uint8_t  which;        // 0 = left flag, 1 = right flag, 2 = front flag
    uint8_t  now;          // the flag after the change (1 = wall, 0 = open)
    int16_t  s_mm;
    int16_t  toCentre_mm;  // believed distance from here to the next cell centre
    float    ir;           // that sensor's distance reading (in)
} S2_EdgeRec;
static S2_EdgeRec s2_edges[S2_MAX_EDGES];
static int        s2_edgeCount = 0;
static int8_t     s2_lastFlag[3] = { -1, -1, -1 };

static void S2_EdgeWatch(void)
{
    uint8_t f[3];
    f[0] = IR_WallLeft_BG();
    f[1] = IR_WallRight_BG();
    f[2] = IR_WallFront_BG();
    for (int k = 0; k < 3; k++)
    {
        if ((s2_lastFlag[k] >= 0) && (s2_lastFlag[k] != (int8_t)f[k]))
        {
            Location loc = Pose_GetLocation();
            S2_EdgeRec *e = &s2_edges[s2_edgeCount % S2_MAX_EDGES];
            s2_edgeCount++;
            e->x = (int8_t)loc.x; e->y = (int8_t)loc.y;
            e->which = (uint8_t)k;
            e->now = f[k];
            e->s_mm = (int16_t)s2_S;
            e->toCentre_mm = (int16_t)(s2_sCenterNext - s2_S);
            e->ir = (k == 0) ? IR_GetDistanceL_BG() : (k == 1) ? IR_GetDistanceR_BG() : IR_GetDistanceF_BG();
        }
        s2_lastFlag[k] = (int8_t)f[k];
    }
}

static void S2_PrintResync(void);   // defined next to S2_RunPathChain

static void S2_PrintEdges(void)
{
    static const char *nm[] = { "left", "right", "front" };
    BT_SendLine("--- wall flag changes while driving (wall<->open) ---");
    BT_SendLine("cell_x,cell_y,sensor,becomes,S_mm,mm_to_believed_centre,ir_in");
    int first = (s2_edgeCount > S2_MAX_EDGES) ? (s2_edgeCount - S2_MAX_EDGES) : 0;
    for (int n = first; n < s2_edgeCount; n++)
    {
        S2_EdgeRec *e = &s2_edges[n % S2_MAX_EDGES];
        BT_Sendf("%d,%d,%s,%s,%d,%d,%.2f\r\n", e->x, e->y, nm[e->which], e->now ? "WALL" : "open",
                 e->s_mm, e->toCentre_mm, e->ir);
    }
}

// ── what the mouse BELIEVES the maze looks like ──
static uint8_t S2_StoredWall(Location loc, Heading h)
{
    WallInfo w = Maze_GetWalls(loc);
    switch (h)
    {
        case HEADING_NORTH: return (uint8_t)w.north;
        case HEADING_EAST:  return (uint8_t)w.east;
        case HEADING_SOUTH: return (uint8_t)w.south;
        case HEADING_WEST:  return (uint8_t)w.west;
        default:            return 2;
    }
}

static char S2_WallChar(uint8_t st)    // for the one-letter listings
{
    return (st == WALL_PRESENT) ? 'W' : (st == WALL_EXIT) ? '.' : (st == WALL_UNKNOWN) ? '?' : 'V';
}

static void S2_PrintMaze(const char *title)
{
    static const char arrow[4] = { '^', '>', 'v', '<' };
    Location me = Pose_GetLocation();
    Heading hd = Pose_GetHeading();
    char line[(MAZE_WIDTH * 4) + 8];

    BT_Sendf("--- maze as the mouse believes it (%s): robot at (%d,%d) heading %d ---\r\n", title, me.x, me.y, (int)hd);
    BT_SendLine("legend: --- and | = wall, ? = never sensed, blank = open, . = visited cell, arrow = robot. y=0 is the bottom row.");
    for (int y = MAZE_HEIGHT - 1; y >= 0; y--)
    {
        int n = 0;
        line[n++] = '+';
        for (int x = 0; x < MAZE_WIDTH; x++)
        {
            WallInfo w = Maze_GetWalls(Location_Make((uint8_t)x, (uint8_t)y));
            char c = (w.north == WALL_PRESENT) ? '-' : (w.north == WALL_UNKNOWN) ? '?' : ' ';
            line[n++] = (c == '?') ? ' ' : c;
            line[n++] = c;
            line[n++] = (c == '?') ? ' ' : c;
            line[n++] = '+';
        }
        line[n] = 0;
        BT_SendLine(line);

        n = 0;
        for (int x = 0; x < MAZE_WIDTH; x++)
        {
            Location cl = Location_Make((uint8_t)x, (uint8_t)y);
            WallInfo w = Maze_GetWalls(cl);
            line[n++] = (w.west == WALL_PRESENT) ? '|' : (w.west == WALL_UNKNOWN) ? '?' : ' ';
            line[n++] = ' ';
            line[n++] = ((me.x == x) && (me.y == y)) ? arrow[(int)hd & 3] : (Maze_CellIsVisited(cl) ? '.' : ' ');
            line[n++] = ' ';
        }
        {
            WallInfo w = Maze_GetWalls(Location_Make((uint8_t)(MAZE_WIDTH - 1), (uint8_t)y));
            line[n++] = (w.east == WALL_PRESENT) ? '|' : (w.east == WALL_UNKNOWN) ? '?' : ' ';
        }
        line[n] = 0;
        BT_SendLine(line);
    }
    {
        int n = 0;
        line[n++] = '+';
        for (int x = 0; x < MAZE_WIDTH; x++)
        {
            WallInfo w = Maze_GetWalls(Location_Make((uint8_t)x, 0));
            char c = (w.south == WALL_PRESENT) ? '-' : (w.south == WALL_UNKNOWN) ? '?' : ' ';
            line[n++] = (c == '?') ? ' ' : c;
            line[n++] = c;
            line[n++] = (c == '?') ? ' ' : c;
            line[n++] = '+';
        }
        line[n] = 0;
        BT_SendLine(line);
    }
}

static void S2_PrintSamples(void)
{
    int sFirst = (s2_sampleCount > S2_LOG_MAX_SAMPLES) ? (s2_sampleCount - S2_LOG_MAX_SAMPLES) : 0;
    BT_Sendf("--- search2 motion log (%d samples; %d ms apart in the search, %d ms apart on fast laps; samples %d..%d of the run) ---\r\n",
             s2_sampleCount - sFirst, S2_LOG_INTERVAL_MS, S2_FAST_LOG_INTERVAL_MS, sFirst, s2_sampleCount - 1);
    BT_SendLine("state: 1=cruise 2=arcwait 3=arc 4=stopping 5=stalled");
    BT_SendLine("t_ms,state,cx,cy,hd,wF,wL,wR,S,sCenterNext,sArcTarget,targetPos,targetSpeed,measSpeed,heading,rotError,posOutput,rotOutput,leftV,rightV,irL,irR,irF,steerAdj,rotTarget");
    for (int n = sFirst; n < s2_sampleCount; n++)
    {
        S2_Sample *p = &s2_samples[n % S2_LOG_MAX_SAMPLES];
        BT_Sendf("%lu,%u,%d,%d,%d,%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f\r\n",
                 (unsigned long)p->t_ms, p->state, p->cx, p->cy, p->hd, p->wF, p->wL, p->wR,
                 p->S, p->sCenterNext, p->sArcTarget, p->targetPos, p->targetSpeed, p->measSpeed,
                 p->heading, p->rotError, p->posOutput, p->rotOutput, p->leftV, p->rightV,
                 p->irL, p->irR, p->irF, p->steerAdj, p->rotTarget);
    }
}

static void S2_PrintLog(void)
{
    static const char *actName[] = { "straight", "LEFT", "RIGHT", "stop", "at-center", "REPLAN-at-centre" };
    BT_SendLine("--- search2 decisions ---");
    BT_Sendf("t_ms,x,y,heading,F,L,R,action,S_mm,decide_us,irF,irL,irR,irL_%.0fmm_before,irR_%.0fmm_before\r\n",
             S2_HIST_LOOKBACK_MM, S2_HIST_LOOKBACK_MM);
    int eFirst = (s2_eventCount > S2_MAX_EVENTS) ? (s2_eventCount - S2_MAX_EVENTS) : 0;
    if (eFirst > 0) BT_Sendf("(ring buffer: decisions 1..%d are not shown, these are decisions %d..%d)\r\n", eFirst, eFirst + 1, s2_eventCount);
    for (int n = eFirst; n < s2_eventCount; n++)
    {
        S2_Event *e = &s2_events[n % S2_MAX_EVENTS];
        BT_Sendf("%lu,%d,%d,%d,%u,%u,%u,%s,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                 (unsigned long)e->t_ms, e->x, e->y, e->heading,
                 e->wF, e->wL, e->wR, actName[e->action], e->s_mm, e->decide_us, e->irF, e->irL, e->irR,
                 e->irL_prev, e->irR_prev);
    }
    BT_Sendf("slowest decision (sense+flood+pick) = %lu us\r\n", (unsigned long)s2_decideMax_us);
    BT_SendLine("--- arc starts ---");
    BT_SendLine("x,y,heading,S_wheels_wanted,S_actual,shift_mm,irF_at_start,irL_at_start,irR_at_start,mm_to_believed_centre");
    {
        int aFirst = (s2_arcCount > S2_MAX_ARCS) ? (s2_arcCount - S2_MAX_ARCS) : 0;
        for (int n = aFirst; n < s2_arcCount; n++)
        {
            S2_ArcRec *a = &s2_arcs[n % S2_MAX_ARCS];
            BT_Sendf("%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%d\r\n", a->x, a->y, a->heading,
                     a->sNominal, a->sActual, (int)(a->sActual - a->sNominal), a->irF, a->irL, a->irR, a->toCentre);
        }
    }
    S2_PrintEdges();
    S2_PrintResync();
    S2_PrintMaze("final");
    S2_PrintSamples();
}

// ── odometry: S = mean encoder path since the chain started ──
static int32_t s2_prevL, s2_prevR;
static float   s2_S;

static void S2_OdoReset(void)
{
    s2_prevL = Encoder_GetLeftCount();
    s2_prevR = Encoder_GetRightCount();
    s2_S = 0.0f;
}

static void S2_OdoUpdate(void)
{
    int32_t l = Encoder_GetLeftCount();
    int32_t r = Encoder_GetRightCount();
    s2_S += 0.5f * (((float)(l - s2_prevL) * MM_PER_TICK_LEFT) +
                    ((float)(r - s2_prevR) * MM_PER_TICK_RIGHT));
    s2_prevL = l;
    s2_prevR = r;
}

static void S2_KeepForwardProfileFed(void);

// One call per pass of every wait loop: odometry, keep the profile fed, log a sample when due,
// and watch for a stall. Returns 1 if the mouse has stopped moving while the profile still asks for speed.
static uint8_t S2_Poll(uint8_t feed)
{
    S2_OdoUpdate();
    S2_EdgeWatch();
    S2_HistRecord();
    if (feed) S2_KeepForwardProfileFed();

    uint32_t now = HAL_GetTick();

    // BUTTON ABORT: pressed for S2_BUTTON_DEBOUNCE_MS in a row -> stop and print the logs
    if (s2_btnEnabled)
    {
        if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == S2_BUTTON_PRESSED_LEVEL)
        {
            if (s2_btnSince == 0) s2_btnSince = now;
            else if ((now - s2_btnSince) >= S2_BUTTON_DEBOUNCE_MS) s2_userAbort = 1;
        }
        else
        {
            s2_btnSince = 0;
        }
    }

    if ((now - s2_lastSampleTick) >= s2_logIntervalMs)
    {
        S2_LogSample();
        s2_lastSampleTick = now;
    }

    if ((now - s2_speedTick) >= 50)
    {
        s2_measSpeed = (s2_S - s2_speedS) * 1000.0f / (float)(now - s2_speedTick);
        s2_speedS = s2_S;
        s2_speedTick = now;

        if ((fabsf(RampProfile_GetSpeed(&forwardProfile)) > 100.0f) && (fabsf(s2_measSpeed) < 20.0f))
        {
            if (s2_stallStart == 0) s2_stallStart = now;
        }
        else
        {
            s2_stallStart = 0;
        }

#if S2_HAVE_IR_F
        // wheels can keep spinning while the nose is on a wall, so also watch the front IR directly
        if ((s2_state == S2_ST_CRUISE) || (s2_state == S2_ST_ARCWAIT))
        {
            if (IR_GetDistanceF_BG() < S2_FRONT_ABORT_IN) s2_frontDanger = 1;
        }
#endif
    }

    // RUN CLOCK: SysTick has already cut the motors. Treat it like the abort button so every loop unwinds.
    if (g_runExpired) { s2_timeUp = 1; s2_userAbort = 1; }

    if (s2_userAbort) return 1;
    if (s2_frontDanger) return 1;
    return (s2_stallStart != 0) && ((now - s2_stallStart) > S2_STALL_MS);
}

static uint8_t S2_FrontWallNow(void)
{
#if S2_HAVE_IR_F
    return (IR_GetDistanceF_BG() <= S2_FRONT_WALL_DECISION_IN) ? 1U : 0U;
#else
    return IR_WallFront_BG();
#endif
}

// ── same helpers as maze_solver.c (copied, so maze_solver.c stays untouched) ──
static void S2_SenseAndStoreWalls(uint8_t *oF, uint8_t *oL, uint8_t *oR)
{
    Location loc = Pose_GetLocation();
    Heading hd = Pose_GetHeading();

    uint8_t wF = S2_FrontWallNow();
    uint8_t wL = IR_WallLeft_BG();
    uint8_t wR = IR_WallRight_BG();

    Maze_UpdateWallState(loc, hd,                      wF ? WALL_PRESENT : WALL_EXIT);
    Maze_UpdateWallState(loc, Heading_LeftFrom(hd),    wL ? WALL_PRESENT : WALL_EXIT);
    Maze_UpdateWallState(loc, Heading_RightFrom(hd),   wR ? WALL_PRESENT : WALL_EXIT);

    if (oF) *oF = wF;
    if (oL) *oL = wL;
    if (oR) *oR = wR;
}

// NEW: same as S2_SenseAndStoreWalls, but OVERWRITES the stored walls. Only used when the mouse is stopped at a
// cell centre (reliable reads). The reads taken at the decision point while driving can be stale (they can still
// show the previous cell's wall), and the map keeps the first reading forever, so this corrects it.
static void S2_SenseAndForceWalls(uint8_t *oF, uint8_t *oL, uint8_t *oR)
{
    Location loc = Pose_GetLocation();
    Heading hd = Pose_GetHeading();

    uint8_t wF = S2_FrontWallNow();
    uint8_t wL = IR_WallLeft_BG();
    uint8_t wR = IR_WallRight_BG();

    Maze_ForceWallState(loc, hd,                      wF ? WALL_PRESENT : WALL_EXIT);
    Maze_ForceWallState(loc, Heading_LeftFrom(hd),    wL ? WALL_PRESENT : WALL_EXIT);
    Maze_ForceWallState(loc, Heading_RightFrom(hd),   wR ? WALL_PRESENT : WALL_EXIT);

    if (oF) *oF = wF;
    if (oL) *oL = wL;
    if (oR) *oR = wR;
}

// NEW: sense at a cell centre (mouse stopped): prints what it read vs what the map already held,
// then stores the walls (overwrites if S2_FORCE_CENTER_WALLS = 1), then prints the maze the mouse believes.
static void S2_SenseCentre(uint8_t *oF, uint8_t *oL, uint8_t *oR)
{
    Location loc = Pose_GetLocation();
    Heading hd = Pose_GetHeading();
    uint8_t rF = S2_FrontWallNow(), rL = IR_WallLeft_BG(), rR = IR_WallRight_BG();
    uint8_t sF = S2_StoredWall(loc, hd);
    uint8_t sL = S2_StoredWall(loc, Heading_LeftFrom(hd));
    uint8_t sR = S2_StoredWall(loc, Heading_RightFrom(hd));
    uint8_t diff = ((sF != WALL_UNKNOWN) && (sF != rF)) || ((sL != WALL_UNKNOWN) && (sL != rL)) ||
                   ((sR != WALL_UNKNOWN) && (sR != rR));

    BT_Sendf("CENTRE (%d,%d) heading=%d | read F/L/R = %u/%u/%u | map held F/L/R = %c/%c/%c (W=wall .=open ?=unknown) | irF=%.2f irL=%.2f irR=%.2f%s\r\n",
             loc.x, loc.y, (int)hd, rF, rL, rR, S2_WallChar(sF), S2_WallChar(sL), S2_WallChar(sR),
             IR_GetDistanceF_BG(), IR_GetDistanceL_BG(), IR_GetDistanceR_BG(),
             diff ? "  <-- DIFFERENT FROM THE MAP" : "");

#if S2_FORCE_CENTER_WALLS
    S2_SenseAndForceWalls(oF, oL, oR);
#else
    S2_SenseAndStoreWalls(oF, oL, oR);
#endif
    S2_PrintMaze("at centre");
}

static void S2_TurnToward(Heading target, float driftCorrection_deg)
{
    Heading hd = Pose_GetHeading();
    if (target == hd) return;

    if (target == Heading_RightFrom(hd))       TurnRightCorrected(driftCorrection_deg);
    else if (target == Heading_LeftFrom(hd))   TurnLeftCorrected(driftCorrection_deg);
    else if (target == Heading_BehindFrom(hd)) Turn180Corrected(driftCorrection_deg);

    Pose_SetHeading(target);
}

// NEW: "am I at the target" - on the way to the centre any cell of the goal block
// counts (all four on an even maze), on the way back it is the start cell.
static bool S2_AtTarget(Location loc, S2_Phase phase)
{
    return (phase == S2_SEARCH_TO_CENTER) ? Maze_IsGoal(loc)
                                          : Location_Equals(loc, START_LOCATION);
}

static Location S2_FloodTarget(S2_Phase phase)
{
    return (phase == S2_SEARCH_TO_CENTER) ? Maze_GetGoal() : START_LOCATION;
}

// ── motion pieces ──
static void S2_StartMotion(void)
{
    // same start-up sequence as Run_PathTestSteps
    g_rotationBraked = 0;
    g_spinTurnActive = 0;
    g_robotIsMoving  = 1;

    Encoder_ResetAll();
    Encoder_Update();
    S2_OdoReset();
    S2_HistReset();

    MotionController_Reset();
    MotionController_ResetSteering();

    RampProfile_SetMove(&forwardProfile, S2_LOOKAHEAD_MM, s2_cruiseMmS);
    g_motionControlEnabled = 1;

    s2_speedTick = HAL_GetTick();
    s2_speedS = 0.0f;
    s2_measSpeed = 0.0f;
    s2_stallStart = 0;
    s2_frontDanger = 0;
    s2_sArcTarget = 0.0f;
    s2_lastFlag[0] = s2_lastFlag[1] = s2_lastFlag[2] = -1;
    s2_state = S2_ST_CRUISE;
}

static void S2_KeepForwardProfileFed(void)
{
    if (RampProfile_GetRemaining(&forwardProfile) < S2_MIN_REMAINING_MM)
    {
        RampProfile_AppendMove(&forwardProfile, S2_LOOKAHEAD_MM, s2_cruiseMmS,
                               RampProfile_GetApproachSpeed(&forwardProfile));
    }
}

static void S2_StartArc(int turnRight)
{
    float deg = (float)(90.0f + s2_finishComp);
    float cmd = turnRight ? deg : -deg;       // positive angle = right turn

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    RampProfile_SetMove(&rotationProfile, cmd, s2_maxOmega);
    __set_PRIMASK(primask);
}

// Stop with the centre of the cell at S = sTarget. Blocks until the mouse is still.
static void S2_StopAt(float sTarget)
{
    float remaining = sTarget - s2_S;
    if (remaining < S2_STOP_MIN_MM) remaining = S2_STOP_MIN_MM;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    forwardProfile.targetDist = fabsf(forwardProfile.position) + remaining;
    __set_PRIMASK(primask);

    uint32_t t0 = HAL_GetTick();
    uint32_t lastCheck = t0;
    uint32_t stillChecks = 0;
    int32_t  pl = Encoder_GetLeftCount();
    int32_t  pr = Encoder_GetRightCount();

    s2_state = S2_ST_STOPPING;
    s2_sArcTarget = 0.0f;

    while ((HAL_GetTick() - t0) < S2_WAIT_TIMEOUT_MS)
    {
        (void)S2_Poll(0);
        uint32_t now = HAL_GetTick();

        if (RampProfile_IsFinished(&forwardProfile) && (now - lastCheck) >= 20)
        {
            int32_t l = Encoder_GetLeftCount();
            int32_t r = Encoder_GetRightCount();
            int32_t dl = l - pl; if (dl < 0) dl = -dl;
            int32_t dr = r - pr; if (dr < 0) dr = -dr;
            if (dl < 2 && dr < 2) stillChecks++; else stillChecks = 0;
            pl = l; pr = r; lastCheck = now;
            if (stillChecks >= 3) break;
        }
    }

    S2_LogSample();
    s2_state = S2_ST_IDLE;

    g_motionControlEnabled = 0;
    g_robotIsMoving = 0;
    MotionController_Reset();
    Motor_Stop(MOTOR_BRAKE);
}

static void S2_AbortStop(void)
{
    S2_LogSample();
    g_motionControlEnabled = 0;
    g_robotIsMoving = 0;
    MotionController_Reset();
    Motor_Stop(MOTOR_BRAKE);
}

// ── the continuous chain ─────────────────────────────────────────────────────
// Precondition: stopped at the centre of Pose_GetLocation(), already facing the
// way we want to go (heading == Pose heading, next cell is open).
// Returns when it has stopped at the centre of the cell Pose_GetLocation() now reports.
static S2_StopReason S2_RunChain(S2_Phase phase, int *moves, int maxMoves, float startBehindCentre)
{
    S2_StartMotion();

    float sCenterNext = S2_CELL_MM + startBehindCentre;     // centre of the cell ahead
    s2_sCenterNext = sCenterNext;
    uint32_t chainStart = HAL_GetTick();
    float   sCentreCheck = 0.0f;       // S of the centre of the cell we just decided to drive STRAIGHT through
    uint8_t centreCheckPending = 0;

    while (1)
    {
        if (S2_Poll(1))
        {
            s2_state = S2_ST_STALLED;
            S2_AbortStop();
            return S2_STOP_STALL;
        }

        if ((HAL_GetTick() - chainStart) > S2_CHAIN_TIMEOUT_MS)
        {
            S2_AbortStop();
            return S2_STOP_TIMEOUT;
        }

#if S2_CENTRE_RECHECK
        if (centreCheckPending && (s2_S >= (sCentreCheck - S2_RECHECK_LEAD_MM)))
        {
            centreCheckPending = 0;
            uint8_t cF = 0, cL = 0, cR = 0;
            S2_SenseAndForceWalls(&cF, &cL, &cR);        // centre reads overwrite the decision-point reads
            Location cLoc = Pose_GetLocation();
            Heading  cHd  = Pose_GetHeading();
            Maze_Flood(S2_FloodTarget(phase));
            Heading  cNext = Maze_HeadingToSmallest(cLoc, cHd);
            if (cNext != cHd)
            {
                S2_LogEvent(cF, cL, cR, S2_ACT_REPLAN, s2_S, 0, IR_GetDistanceF_BG());
                // wall in front: stop on the centre and square on that wall afterwards (outer loop).
                // NO wall in front (nothing to square on, e.g. (7,7)): stop a bit further on, see the bias note above.
                s2_replanFrontWall = cF ? 1U : 0U;
                S2_StopAt(cF ? sCentreCheck : (sCentreCheck + S2_REPLAN_STOP_BIAS_MM));
                s2_replanOff = s2_S - sCentreCheck;     // + = stopped past the centre, - = short of it
                return S2_STOP_REPLAN;
            }
        }
#endif

        float sDecision = sCenterNext - (0.5f * S2_CELL_MM) + S2_DECISION_DEPTH_MM;
        if (s2_S < sDecision) continue;

        // ---------------- decision point inside the next cell ----------------
        uint32_t c0 = DWT->CYCCNT;

        Pose_AdvanceOneCell();
        (*moves)++;

        uint8_t wF = 0, wL = 0, wR = 0;
        S2_SenseAndStoreWalls(&wF, &wL, &wR);
        float irF = IR_GetDistanceF_BG();
        S2_HistLookup();       // side readings S2_HIST_LOOKBACK_MM earlier, for the decisions table

        Location loc = Pose_GetLocation();
        Location target = S2_FloodTarget(phase);
        Heading  hd = Pose_GetHeading();

        S2_Action act = S2_ACT_STRAIGHT;
        S2_StopReason stopWhy = S2_STOP_DEADEND;
        Heading next = hd;

        if (S2_AtTarget(loc, phase))
        {
            act = S2_ACT_STOP; stopWhy = S2_STOP_TARGET;
        }
        else if (*moves >= maxMoves)
        {
            act = S2_ACT_STOP; stopWhy = S2_STOP_MAXMOVES;
        }
        else
        {
            Maze_Flood(target);
            next = Maze_HeadingToSmallest(loc, hd);

            if (next == HEADING_BLOCKED)
            {
                act = S2_ACT_STOP; stopWhy = S2_STOP_BLOCKED;
            }
            else if (next == hd)
            {
                act = S2_ACT_STRAIGHT;
            }
            else if (next == Heading_RightFrom(hd))
            {
                act = S2_ACT_RIGHT;
            }
            else if (next == Heading_LeftFrom(hd))
            {
                act = S2_ACT_LEFT;
            }
            else
            {
                act = S2_ACT_STOP; stopWhy = S2_STOP_DEADEND;   // behind us
            }
        }

        uint32_t decide_us = (DWT->CYCCNT - c0) / (SystemCoreClock / 1000000U);
        if (decide_us > s2_decideMax_us) s2_decideMax_us = decide_us;
        S2_LogEvent(wF, wL, wR, act, s2_S, decide_us, irF);
        S2_LogSample();   // a motion sample exactly at every decision

        // ---------------- act on it ----------------
        if (act == S2_ACT_STOP)
        {
            S2_StopAt(sCenterNext);
            return stopWhy;
        }

        if (act == S2_ACT_STRAIGHT)
        {
            sCentreCheck = sCenterNext;          // the centre of the cell we are in now
            centreCheckPending = 1;
            sCenterNext += S2_CELL_MM;
            s2_sCenterNext = sCenterNext;
            continue;
        }

        // LEFT or RIGHT: wait until the arc start, then run the arc
        float sArc = sCenterNext - S2_ARC_FWD_MM + S2_ARC_EXTRA_MM;
        s2_sArcTarget = sArc;
        s2_state = S2_ST_ARCWAIT;
        uint32_t w0 = HAL_GetTick();

        while (s2_S < sArc)
        {
            if (S2_Poll(1)) { s2_state = S2_ST_STALLED; S2_AbortStop(); return S2_STOP_STALL; }
            if ((HAL_GetTick() - w0) > S2_WAIT_TIMEOUT_MS) { S2_AbortStop(); return S2_STOP_TIMEOUT; }
        }

        float sArcStart = s2_S;
        {
            Location aLoc = Pose_GetLocation();
            S2_ArcRec *a = &s2_arcs[s2_arcCount % S2_MAX_ARCS];
            s2_arcCount++;
            a->x = (int8_t)aLoc.x; a->y = (int8_t)aLoc.y; a->heading = (int8_t)Pose_GetHeading();
            a->sNominal = (int16_t)sArc;
            a->sActual = (int16_t)sArcStart;
            a->irF = IR_GetDistanceF_BG();
            a->irL = IR_GetDistanceL_BG();
            a->irR = IR_GetDistanceR_BG();
            a->toCentre = (int16_t)(sCenterNext - sArcStart);
        }
        s2_state = S2_ST_ARC;
        S2_LogSample();
        S2_StartArc(act == S2_ACT_RIGHT);

        w0 = HAL_GetTick();
        while (!RampProfile_IsFinished(&rotationProfile))
        {
            if (S2_Poll(1)) { s2_state = S2_ST_STALLED; S2_AbortStop(); return S2_STOP_STALL; }
            if ((HAL_GetTick() - w0) > S2_WAIT_TIMEOUT_MS) { S2_AbortStop(); return S2_STOP_TIMEOUT; }
        }

        S2_LogSample();
        s2_state = S2_ST_CRUISE;
        s2_sArcTarget = 0.0f;
        Pose_SetHeading(next);

        // next cell centre: arc start + path to the true 90 + what is left of this cell's exit
        sCenterNext = sArcStart + S2_ARC_PATH_TRUE_MM + (S2_CELL_MM - S2_ARC_SIDE_MM);
        s2_sCenterNext = sCenterNext;
    }
}

// ════════════════════════════════════════════════════════════════════════════
// FAST RUN (new): after the search is finished and the mouse is back at the start, fast_route.h (FR_Plan, the same
// planner as the simulator) plans the cheapest route through the cells the mouse has SEEN. The mouse drives it to the
// centre (this is the FAST LAP, timed and saved), then drives back to the start the same way (NOT timed), squares on
// the back wall and turns to face the maze again. One lap only for now. Same speed as the search.
//   - Unknown walls count as closed, and the route only passes through visited cells.
//   - During the fast run the map is NOT updated from the side sensors (their stale reads would only damage a good
//     map). The only check is the front IR at each decision: if the route says "straight" but a wall is there, the
//     mouse stops at that centre, the wall goes into the map, it squares on it, and the route is planned again.
// ════════════════════════════════════════════════════════════════════════════
#define S2_PATH_MAX         128
#define S2_MAX_FAST_REPLANS 5         // give up if the fast run keeps hitting walls the map did not have

static Heading s2_path[S2_PATH_MAX];

// 1 if the map knows the edge from cell (x,y) in direction h is OPEN. Unknown counts as closed.
static uint8_t S2_KnownOpen(int x, int y, int h)
{
    static const int8_t dx[4] = { 0, 1, 0, -1 };
    static const int8_t dy[4] = { 1, 0, -1, 0 };
    int nx = x + dx[h];
    int ny = y + dy[h];
    if ((nx < 0) || (ny < 0) || (nx >= MAZE_WIDTH) || (ny >= MAZE_HEIGHT)) return 0;

    uint8_t a = S2_StoredWall(Location_Make((uint8_t)x,  (uint8_t)y),  (Heading)h);
    uint8_t b = S2_StoredWall(Location_Make((uint8_t)nx, (uint8_t)ny), (Heading)((h + 2) & 3));
    if ((a == WALL_PRESENT) || (b == WALL_PRESENT)) return 0;
    return ((a == WALL_EXIT) || (b == WALL_EXIT)) ? 1U : 0U;
}

// ════════════════════════════════════════════════════════════════════════════
// FAST-LAP RE-SYNC (new). The fast lap follows a route through cells whose walls the search already mapped, so the
// mouse knows WHERE each wall should start or end. When a real wall flag flips (or the front IR sees the wall in front
// of a turn), it compares where it thought it was with where the map says it must be, and moves sCenterNext
// (the S position of the next cell centre that the decisions, arcs and stops are all measured from).
// Only used inside S2_RunPathChain, so the search is not affected.
//   Side edges : wall <-> open flips on the straights, measured against the map (needs the cell it is in AND the
//                cell ahead to be known and to differ on that side).
//   Front wall : at the decision point of a turn/stop cell whose front wall is in the map, the front IR distance
//                gives the real distance to the cell centre.
// Each correction is only a fraction (gain) of the error and is ignored if it is bigger than the gate.
// Set S2_RESYNC_ENABLE to 0 to turn all of it off. Every correction is printed after the run in "fast re-sync".
// ════════════════════════════════════════════════════════════════════════════
#define S2_RESYNC_ENABLE         1
#define S2_RESYNC_SIDE_ON        1
#define S2_RESYNC_FRONT_ON       1
#define S2_RESYNC_SIDE_GAIN      0.5f     // fraction of a side-edge error that is corrected
#define S2_RESYNC_SIDE_GATE_MM   60.0f    // ignore side edges that are further off than this
#define S2_RESYNC_MAX_ROT_ERR    6.0f     // deg: ignore side edges while the heading is still settling
#define S2_RESYNC_FRONT_GAIN     0.7f
#define S2_RESYNC_FRONT_GATE_MM  150.0f
#define S2_RESYNC_FRONT_MAX_IN   8.6f     // only trust the front IR when it reads closer than this (inches)
#define S2_RESYNC_FRONT_OFFSET_MM 36.0f   // front IR distance minus real distance from the wheel axle to the wall

// where an edge shows up, in mm before the next cell centre (measured on the 400 mm/s fast run log)
//                                    left:  WALL  open     right: WALL  open
static const float s2_edgeToCentre[2][2] = { { 115.0f, 149.0f }, { 130.0f, 125.5f } };

#define S2_MAX_RESYNC 60
typedef struct { int8_t x, y; uint8_t kind; int16_t err_mm; int16_t applied_mm; } S2_ResyncRec;
static S2_ResyncRec s2_resync[S2_MAX_RESYNC];
static int          s2_resyncCount = 0;

static void S2_ResyncLog(int x, int y, uint8_t kind, float err, float applied)
{
    S2_ResyncRec *r = &s2_resync[s2_resyncCount % S2_MAX_RESYNC];
    s2_resyncCount++;
    r->x = (int8_t)x; r->y = (int8_t)y; r->kind = kind;
    r->err_mm = (int16_t)err; r->applied_mm = (int16_t)applied;
}

static void S2_PrintResync(void)
{
    static const char *nm[] = { "left", "right", "front" };
    BT_SendLine("--- fast re-sync (error = how far off S was, applied = how much sCenterNext moved) ---");
    BT_SendLine("cell_x,cell_y,source,error_mm,applied_mm");
    int first = (s2_resyncCount > S2_MAX_RESYNC) ? (s2_resyncCount - S2_MAX_RESYNC) : 0;
    for (int n = first; n < s2_resyncCount; n++)
    {
        S2_ResyncRec *r = &s2_resync[n % S2_MAX_RESYNC];
        BT_Sendf("%d,%d,%s,%d,%d\r\n", r->x, r->y, nm[r->kind], r->err_mm, r->applied_mm);
    }
}

// 1 = wall in the map, 0 = known open, -1 = unknown
static int8_t S2_CellSideState(int x, int y, int h)
{
    if (S2_StoredWall(Location_Make((uint8_t)x, (uint8_t)y), (Heading)h) == WALL_PRESENT) return 1;
    return S2_KnownOpen(x, y, h) ? 0 : -1;
}

static int8_t s2_rsLast[2]   = { -1, -1 };   // last left/right flag
static int8_t s2_rsFrom[2]   = { -1, -1 };   // what the map says the side is in the cell the mouse is in
static int8_t s2_rsExp[2]    = { -1, -1 };   // the state it should flip to at the next cell boundary (-1 = no flip expected)
static int    s2_rsCx = -1, s2_rsCy = -1, s2_rsH = -1;

static void S2_ResyncReset(void)
{
    s2_rsLast[0] = s2_rsLast[1] = -1;
    s2_rsCx = s2_rsCy = s2_rsH = -1;
}

// call every loop pass while driving straight inside S2_RunPathChain
static void S2_ResyncSide(float *sCenterNext)
{
#if S2_RESYNC_ENABLE && S2_RESYNC_SIDE_ON
    static const int8_t dx[4] = { 0, 1, 0, -1 };
    static const int8_t dy[4] = { 1, 0, -1, 0 };
    Location loc = Pose_GetLocation();
    int h = (int)Pose_GetHeading();

    if ((s2_rsCx != (int)loc.x) || (s2_rsCy != (int)loc.y) || (s2_rsH != h))
    {
        s2_rsCx = (int)loc.x; s2_rsCy = (int)loc.y; s2_rsH = h;
        int nx = s2_rsCx + dx[h], ny = s2_rsCy + dy[h];
        for (int s = 0; s < 2; s++)
        {
            int side = (s == 0) ? ((h + 3) & 3) : ((h + 1) & 3);
            int8_t cur = S2_CellSideState(s2_rsCx, s2_rsCy, side);
            int8_t ahd = ((nx >= 0) && (ny >= 0) && (nx < MAZE_WIDTH) && (ny < MAZE_HEIGHT)) ? S2_CellSideState(nx, ny, side) : (int8_t)-1;
            s2_rsFrom[s] = cur;
            s2_rsExp[s]  = ((cur >= 0) && (ahd >= 0) && (cur != ahd)) ? ahd : (int8_t)-1;
        }
    }

    int8_t now[2];
    now[0] = (int8_t)IR_WallLeft_BG();
    now[1] = (int8_t)IR_WallRight_BG();
    uint8_t ok = (s2_state == S2_ST_CRUISE) && (fabsf(MotionController_GetLastRotError()) < S2_RESYNC_MAX_ROT_ERR);

    for (int s = 0; s < 2; s++)
    {
        if (ok && (s2_rsLast[s] >= 0) && (s2_rsLast[s] != now[s]) &&
            (s2_rsExp[s] >= 0) && (now[s] == s2_rsExp[s]) && (s2_rsLast[s] == s2_rsFrom[s]))
        {
            float expected = s2_edgeToCentre[s][now[s] ? 0 : 1];
            float measured = *sCenterNext - s2_S;
            float err = measured - expected;
            if (fabsf(err) <= S2_RESYNC_SIDE_GATE_MM)
            {
                float adj = -S2_RESYNC_SIDE_GAIN * err;     // measured too small => the centre is further away
                *sCenterNext += adj;
                s2_sCenterNext = *sCenterNext;
                S2_ResyncLog(s2_rsCx, s2_rsCy, (uint8_t)s, err, adj);
            }
            else
            {
                S2_ResyncLog(s2_rsCx, s2_rsCy, (uint8_t)s, err, 0.0f);
            }
        }
        s2_rsLast[s] = now[s];
    }
#else
    (void)sCenterNext;
#endif
}

// call at the decision point of a turn or stop cell (after Pose_AdvanceOneCell, before sCenterNext is moved on)
static void S2_ResyncFront(float *sCenterNext, Location loc, Heading hd, float irF)
{
#if S2_RESYNC_ENABLE && S2_RESYNC_FRONT_ON
    if (S2_StoredWall(loc, hd) != WALL_PRESENT) return;
    if ((irF <= 0.0f) || (irF >= S2_RESYNC_FRONT_MAX_IN)) return;

    float est  = (irF * 25.4f) + S2_RESYNC_FRONT_OFFSET_MM - (0.5f * S2_CELL_MM);   // real distance to this cell's centre
    float corr = (s2_S + est) - *sCenterNext;                                       // + = the centre is further than thought
    if (fabsf(corr) <= S2_RESYNC_FRONT_GATE_MM)
    {
        float adj = S2_RESYNC_FRONT_GAIN * corr;
        *sCenterNext += adj;
        s2_sCenterNext = *sCenterNext;
        S2_ResyncLog((int)loc.x, (int)loc.y, 2, corr, adj);
    }
    else
    {
        S2_ResyncLog((int)loc.x, (int)loc.y, 2, corr, 0.0f);
    }
#else
    (void)sCenterNext; (void)loc; (void)hd; (void)irF;
#endif
}

// Fast-lap timing (new): the lap clock stops when the mouse ENTERS the first cell of the centre block, not when it has
// driven the extra cell deeper and stopped. s2_goalEnterIdx = the cell count (k) of that entrance cell, -1 = no deeper cell
// this lap (then the lap is timed to the stop, as before). s2_goalEnterTick = HAL_GetTick() when it got there (0 = not yet).
static int      s2_goalEnterIdx  = -1;
static uint32_t s2_goalEnterTick = 0;

// Drives a planned path as ONE continuous chain from a stop at the centre of the first cell, already facing path[0].
// Returns S2_STOP_TARGET when it has stopped at the centre of the last cell of the path,
// S2_STOP_REPLAN if the front IR saw a wall where the path wanted to go straight (the wall is now in the map),
// or STALL / TIMEOUT / DEADEND (the path wanted a U-turn mid-way, which a shortest path never does).
static S2_StopReason S2_RunPathChain(const Heading *path, int len)
{
    S2_StartMotion();

    float sCenterNext = S2_CELL_MM;          // centre of the cell ahead
    s2_sCenterNext = sCenterNext;
    uint32_t chainStart = HAL_GetTick();
    int k = 0;                               // cells entered so far on this path
    S2_ResyncReset();

    while (1)
    {
        if (S2_Poll(1))
        {
            s2_state = S2_ST_STALLED;
            S2_AbortStop();
            return S2_STOP_STALL;
        }
        if ((HAL_GetTick() - chainStart) > S2_CHAIN_TIMEOUT_MS)
        {
            S2_AbortStop();
            return S2_STOP_TIMEOUT;
        }

        S2_ResyncSide(&sCenterNext);
        float sDecision = sCenterNext - (0.5f * S2_CELL_MM) + S2_DECISION_DEPTH_MM;
        if (s2_S < sDecision) continue;

        // ---------------- decision point inside the next cell ----------------
        Pose_AdvanceOneCell();
        k++;
        if (k == s2_goalEnterIdx) s2_goalEnterTick = HAL_GetTick();      // entered the first centre cell: the fast lap is over

        Location loc = Pose_GetLocation();
        Heading  hd  = Pose_GetHeading();
        uint8_t  wF  = S2_FrontWallNow();
        uint8_t  wL  = IR_WallLeft_BG();
        uint8_t  wR  = IR_WallRight_BG();
        float    irF = IR_GetDistanceF_BG();
        S2_HistLookup();

        S2_Action act = S2_ACT_STRAIGHT;
        Heading next = hd;

        if (k >= len)
        {
            act = S2_ACT_STOP;                                   // this cell is the end of the path
        }
        else
        {
            next = path[k];
            if (next == hd)                         act = S2_ACT_STRAIGHT;
            else if (next == Heading_RightFrom(hd)) act = S2_ACT_RIGHT;
            else if (next == Heading_LeftFrom(hd))  act = S2_ACT_LEFT;
            else                                    act = S2_ACT_STOP;   // behind: should never happen
        }

        if (act != S2_ACT_STRAIGHT) S2_ResyncFront(&sCenterNext, loc, hd, irF);

        // the map said "open ahead" but the front IR sees a wall: stop at this centre, learn it, re-plan
        if ((act == S2_ACT_STRAIGHT) && wF)
        {
            S2_LogEvent(wF, wL, wR, S2_ACT_REPLAN, s2_S, 0, irF);
            Maze_ForceWallState(loc, hd, WALL_PRESENT);
            S2_StopAt(sCenterNext);
            return S2_STOP_REPLAN;
        }

        S2_LogEvent(wF, wL, wR, act, s2_S, 0, irF);
        S2_LogSample();

        if (act == S2_ACT_STOP)
        {
            S2_StopAt(sCenterNext);
            return (k >= len) ? S2_STOP_TARGET : S2_STOP_DEADEND;
        }

        if (act == S2_ACT_STRAIGHT)
        {
            sCenterNext += S2_CELL_MM;
            s2_sCenterNext = sCenterNext;
            continue;
        }

        // LEFT or RIGHT: wait until the arc start, then run the arc (same numbers as the search)
        float sArc = sCenterNext - S2_FAST_ARC_FWD_MM + S2_FAST_ARC_EXTRA_MM;
        s2_sArcTarget = sArc;
        s2_state = S2_ST_ARCWAIT;
        uint32_t w0 = HAL_GetTick();

        while (s2_S < sArc)
        {
            if (S2_Poll(1)) { s2_state = S2_ST_STALLED; S2_AbortStop(); return S2_STOP_STALL; }
            if ((HAL_GetTick() - w0) > S2_WAIT_TIMEOUT_MS) { S2_AbortStop(); return S2_STOP_TIMEOUT; }
        }

        float sArcStart = s2_S;
        {
            S2_ArcRec *a = &s2_arcs[s2_arcCount % S2_MAX_ARCS];
            s2_arcCount++;
            a->x = (int8_t)loc.x; a->y = (int8_t)loc.y; a->heading = (int8_t)hd;
            a->sNominal = (int16_t)sArc;
            a->sActual = (int16_t)sArcStart;
            a->irF = IR_GetDistanceF_BG();
            a->irL = IR_GetDistanceL_BG();
            a->irR = IR_GetDistanceR_BG();
            a->toCentre = (int16_t)(sCenterNext - sArcStart);
        }
        s2_state = S2_ST_ARC;
        S2_LogSample();
        S2_StartArc(act == S2_ACT_RIGHT);

        w0 = HAL_GetTick();
        while (!RampProfile_IsFinished(&rotationProfile))
        {
            if (S2_Poll(1)) { s2_state = S2_ST_STALLED; S2_AbortStop(); return S2_STOP_STALL; }
            if ((HAL_GetTick() - w0) > S2_WAIT_TIMEOUT_MS) { S2_AbortStop(); return S2_STOP_TIMEOUT; }
        }

        S2_LogSample();
        s2_state = S2_ST_CRUISE;
        s2_sArcTarget = 0.0f;
        Pose_SetHeading(next);
        S2_ResyncReset();

        sCenterNext = sArcStart + S2_FAST_ARC_PATH_TRUE_MM + (S2_CELL_MM - S2_FAST_ARC_SIDE_MM);
        s2_sCenterNext = sCenterNext;
    }
}

// ── saved times (new) ──
// searchToCentreMs / searchReturnMs are set once by the search and never compared with anything.
// bestFastLapMs is the fastest fast lap (start -> centre only); 0 = no fast lap has finished yet.
static uint32_t s2_searchToCentreMs = 0;
static uint32_t s2_searchReturnMs   = 0;
static uint32_t s2_bestFastLapMs    = 0;
uint32_t Maze_GetSearchToCentreMs(void) { return s2_searchToCentreMs; }
uint32_t Maze_GetSearchReturnMs(void)   { return s2_searchReturnMs; }
uint32_t Maze_GetBestFastLapMs(void)    { return s2_bestFastLapMs; }

static FR_Route      s2_route;
static unsigned char s2_frWalls[MAZE_WIDTH * MAZE_HEIGHT];
static unsigned char s2_frVisited[MAZE_WIDTH * MAZE_HEIGHT];
static int           s2_goalX1, s2_goalY1, s2_goalX2, s2_goalY2;

// Copies the mouse's map into the layout fast_route.h wants: walls[x*h+y] bit0=N bit1=E bit2=S bit3=W (set = wall).
// An edge is only "open" if the map KNOWS it is open (S2_KnownOpen); unknown counts as a wall.
// Also finds the bounding box of the goal cells (the centre block).
static void S2_BuildRouteMaps(void)
{
    s2_goalX1 = MAZE_WIDTH;  s2_goalY1 = MAZE_HEIGHT;
    s2_goalX2 = -1;          s2_goalY2 = -1;

    for (int x = 0; x < MAZE_WIDTH; x++)
    {
        for (int y = 0; y < MAZE_HEIGHT; y++)
        {
            Location c = Location_Make((uint8_t)x, (uint8_t)y);
            unsigned char bits = 0;
            for (int h = 0; h < 4; h++)
                if (!S2_KnownOpen(x, y, h)) bits |= (unsigned char)(1U << h);
            s2_frWalls[(x * MAZE_HEIGHT) + y]   = bits;
            s2_frVisited[(x * MAZE_HEIGHT) + y] = Maze_CellIsVisited(c) ? 1U : 0U;

            if (Maze_IsGoal(c))
            {
                if (x < s2_goalX1) s2_goalX1 = x;
                if (x > s2_goalX2) s2_goalX2 = x;
                if (y < s2_goalY1) s2_goalY1 = y;
                if (y > s2_goalY2) s2_goalY2 = y;
            }
        }
    }
}

// Turns the planner's moves (straight N cells / arc / spin) into one heading per cell, which is what
// S2_RunPathChain drives. An arc or spin just changes the heading used for the cells that follow.
// Returns the number of cells, or -1 if it does not fit.
static int S2_RouteToHeadings(const FR_Route *r, Heading startHd, Heading *out, int maxLen)
{
    int hd = (int)startHd & 3;
    int len = 0;
    for (int i = 0; i < r->count; i++)
    {
        switch (r->moves[i].type)
        {
            case FR_STRAIGHT:
                for (int k = 0; k < r->moves[i].cells; k++)
                {
                    if (len >= maxLen) return -1;
                    out[len++] = (Heading)hd;
                }
                break;
            case FR_ARC_LEFT:
            case FR_SPIN_LEFT:   hd = (hd + 3) & 3; break;
            case FR_ARC_RIGHT:
            case FR_SPIN_RIGHT:  hd = (hd + 1) & 3; break;
            default:             hd = (hd + 2) & 3; break;     // FR_SPIN_U
        }
    }
    return len;
}

// ── go one cell deeper into the centre on the fast laps (new) ──
// 1 = on a fast lap to the centre, do not stop in the first (entrance) cell of the centre block: drive one more cell
//     straight in, stop there, square on the wall in front of that cell (the normal Resquare), then come back out. The
//     lap is timed to that deeper stop. 0 = stop in the entrance cell like the search.
// Only used on the fast laps. The extra cell is only added if the map KNOWS the edge ahead of the last cell is open
// and that next cell is inside the centre block.
#define S2_FAST_GO_DEEPER   1

// The deeper cell is one the search never entered, so the planner (which only expands visited cells) would refuse to plan the
// way BACK out of it. Remember it here and mark it visited in S2_FastLeg. -1 = no deeper cell this lap.
static int s2_deepX = -1, s2_deepY = -1;

static void S2_FastGoDeeper(int *len)
{
#if S2_FAST_GO_DEEPER
    s2_deepX = -1; s2_deepY = -1;
    s2_goalEnterIdx = -1;
    static const int8_t dx[4] = { 0, 1, 0, -1 };
    static const int8_t dy[4] = { 1, 0, -1, 0 };
    if ((*len < 1) || (*len >= (S2_PATH_MAX - 1))) return;

    Location loc = Pose_GetLocation();
    int x = (int)loc.x, y = (int)loc.y;
    for (int i = 0; i < *len; i++)
    {
        int h = (int)s2_path[i] & 3;
        x += dx[h]; y += dy[h];
    }
    int h  = (int)s2_path[*len - 1] & 3;              // the heading the mouse arrives at the entrance cell with
    int nx = x + dx[h], ny = y + dy[h];

    if ((nx < s2_goalX1) || (nx > s2_goalX2) || (ny < s2_goalY1) || (ny > s2_goalY2)) return;   // next cell is not in the centre block
    if (!S2_KnownOpen(x, y, h)) return;                                                          // the map does not know it is open

    s2_goalEnterIdx = *len;                           // the cell count at which the entrance cell is entered
    s2_path[*len] = (Heading)h;
    (*len)++;
    s2_deepX = nx; s2_deepY = ny;
    BT_Sendf("Fast run: one cell deeper into the centre: stopping at (%d,%d) instead of (%d,%d).\r\n", nx, ny, x, y);
#else
    (void)len;
#endif
}

// One leg of the fast run: plan, drive, and if a wall turns up that the map did not have, learn it and plan again.
// toCentre = 1: from where the mouse is to the centre block.  toCentre = 0: from where it is back to the start cell.
// drift0 = the value Resquare() returned just before this leg (0 if none).
// Returns 1 if the mouse is now stopped at the target, 0 on any problem (the reason is printed).
static uint8_t S2_FastLeg(uint8_t toCentre, float drift0)
{
    int   replans = 0;
    float drift = drift0;      // heading correction left over from a Resquare() just before this leg

    while (!s2_userAbort)
    {
        Location loc = Pose_GetLocation();
        S2_BuildRouteMaps();
        if ((s2_deepX >= 0) && (s2_deepY >= 0)) s2_frVisited[(s2_deepX * MAZE_HEIGHT) + s2_deepY] = 1U;   // we are standing in it
        s2_frVisited[((int)loc.x * MAZE_HEIGHT) + (int)loc.y] = 1U;   // wherever the mouse stands counts as known (a cell the search never entered, such as the deeper cell)

        int ok;
        if (toCentre)
            ok = FR_Plan(s2_frWalls, s2_frVisited, MAZE_WIDTH, MAZE_HEIGHT,
                         loc.x, loc.y, (int)Pose_GetHeading(),
                         s2_goalX1, s2_goalY1, s2_goalX2, s2_goalY2, &s2_route);
        else
        {
            Location st = START_LOCATION;
            ok = FR_Plan(s2_frWalls, s2_frVisited, MAZE_WIDTH, MAZE_HEIGHT,
                         loc.x, loc.y, (int)Pose_GetHeading(),
                         st.x, st.y, st.x, st.y, &s2_route);
        }
        if (!ok)
        {
            BT_Sendf("Fast run: no plan from (%d,%d) heading %d, going to %s.\r\n", (int)loc.x, (int)loc.y, (int)Pose_GetHeading(), toCentre ? "the centre" : "the start");
            BT_SendLine("Fast run: the map has no fully known route for this leg. Stopping.");
            return 0;
        }
        if (s2_route.count == 0) return 1;                      // already at the target

        {
            char text[200];
            FR_ToString(&s2_route, text, (int)sizeof(text));
            BT_Sendf("[t=%lu ms] Fast run %s from (%d,%d): %s  (cells %d, turns %d, cost %d)\r\n",
                     (unsigned long)(HAL_GetTick() - s2_runStartTick), toCentre ? "to centre" : "to start", loc.x, loc.y, text,
                     s2_route.cells, s2_route.turns, s2_route.cost);
        }

        int len = S2_RouteToHeadings(&s2_route, Pose_GetHeading(), s2_path, S2_PATH_MAX);
        if (len <= 0)
        {
            BT_SendLine("Fast run: could not turn the route into moves. Stopping.");
            return 0;
        }

        if (toCentre) S2_FastGoDeeper(&len);                    // fast lap: end one cell deeper in the centre block

        S2_TurnToward(s2_path[0], drift);                       // the route's first spin, if it has one
        drift = 0.0f;

        S2_StopReason why = S2_RunPathChain(s2_path, len);

        if (s2_userAbort)
        {
            BT_SendLine(s2_timeUp ? "Fast run: TIME UP." : "Fast run: STOPPED BY BUTTON.");
            return 0;
        }
        if (why == S2_STOP_TARGET) return 1;

        if (why == S2_STOP_REPLAN)
        {
            replans++;
            BT_Sendf("Fast run: a front wall the map did not have at (%d,%d). Written into the map, planning again (%d/%d).\r\n",
                     Pose_GetLocation().x, Pose_GetLocation().y, replans, S2_MAX_FAST_REPLANS);
            if (replans > S2_MAX_FAST_REPLANS)
            {
                BT_SendLine("Fast run: too many surprises, stopping.");
                return 0;
            }
            drift = Resquare();
            continue;
        }

        BT_Sendf("Fast run: the leg ended early (reason %d). Stopping.\r\n", (int)why);
        return 0;
    }
    return 0;
}

// How many fast laps to run. 0 = keep lapping until the run clock runs out (or the button is pressed).
// N = do exactly N laps, then stop at the start facing the maze. Use 1 to test one lap, like before the clock existed.
#define S2_FAST_LAP_COUNT   0

// The fast run: repeat { start -> centre (the FAST LAP, timed and saved if it is the best), centre -> start (NOT timed),
// square on the back wall, turn to face the maze } until the clock runs out, the button is pressed, or a leg fails.
// A lap that is cut off by the clock never finishes, so it is never saved.
// Precondition: stopped at the start cell, facing into the maze (the end of the search leaves it like that).
static void S2_FastRun(void)
{
    s2_cruiseMmS = S2_FAST_SPEED_MM_S;     // the fast laps run at the fast values
    s2_maxOmega  = S2_FAST_MAX_OMEGA;
    s2_finishComp = S2_FAST_FINISH_COMP_DEG;
    s2_logIntervalMs = S2_FAST_LOG_INTERVAL_MS;
    BT_SendLine("===== FAST RUN START =====");
    BT_Sendf("fast finishComp=%.1f\r\n", s2_finishComp);
    BT_Sendf("fast speed=%.0f maxOmega=%.0f arcFwd=%.1f arcSide=%.1f arcPathTrue=%.1f arcExtra=%.1f\r\n", s2_cruiseMmS, s2_maxOmega,
             S2_FAST_ARC_FWD_MM, S2_FAST_ARC_SIDE_MM, S2_FAST_ARC_PATH_TRUE_MM, S2_FAST_ARC_EXTRA_MM);
#if S2_FAST_LAP_COUNT > 0
    BT_Sendf("fast laps: %d\r\n", (int)S2_FAST_LAP_COUNT);
#else
    BT_SendLine("fast laps: until the run clock runs out");
#endif

    uint32_t lapsDone = 0;
    uint32_t lastLapMs = 0;
    const char *endWhy = "stopped";

    while (1)
    {
        if (g_runExpired) { s2_timeUp = 1; s2_userAbort = 1; }
        if (s2_userAbort) { endWhy = s2_timeUp ? "TIME UP" : "button"; break; }
#if S2_FAST_LAP_COUNT > 0
        if (lapsDone >= (uint32_t)S2_FAST_LAP_COUNT) { endWhy = "lap count reached"; break; }
#endif

        // ---- the fast lap: start -> centre ----
        uint32_t t0 = HAL_GetTick();
        s2_goalEnterTick = 0;
        if (!S2_FastLeg(1U, 0.0f)) { endWhy = s2_userAbort ? (s2_timeUp ? "TIME UP" : "button") : "a leg failed"; break; }
        uint32_t stopMs = HAL_GetTick() - t0;                  // to the stop in the deeper cell
        uint32_t lapMs  = (s2_goalEnterTick != 0U) ? (s2_goalEnterTick - t0) : stopMs;   // the lap ends on entering the first centre cell

        lapsDone++;
        lastLapMs = lapMs;
        if ((s2_bestFastLapMs == 0U) || (lapMs < s2_bestFastLapMs)) s2_bestFastLapMs = lapMs;
        BT_Sendf("[t=%lu ms] Fast lap %lu (start -> centre): %lu ms. Best so far: %lu ms.\r\n",
                 (unsigned long)(HAL_GetTick() - s2_runStartTick), (unsigned long)lapsDone, (unsigned long)lapMs, (unsigned long)s2_bestFastLapMs);
        BT_Sendf("   (lap timed to entering the first centre cell; stopped deeper after %lu ms)\r\n", (unsigned long)stopMs);

        // ---- at the centre: square up on a front wall if there is one, same as the search does ----
        float drift = 0.0f;
        if (S2_FrontWallNow()) drift = Resquare();

        // ---- back to the start (not timed) ----
        if (!S2_FastLeg(0U, drift)) { endWhy = s2_userAbort ? (s2_timeUp ? "TIME UP" : "button") : "a leg failed"; break; }

        // ---- at the start: same ending as the search (square on the back wall, turn to face the maze) ----
        {
            float d = Resquare();
            Turn180Corrected(d);
            Pose_SetHeading(Heading_BehindFrom(Pose_GetHeading()));
        }
    }

    BT_Sendf("===== FAST RUN DONE (%s) =====\r\n", endWhy);
    BT_SendLine("--- times ---");
    BT_Sendf("search to centre : %lu ms\r\n", (unsigned long)s2_searchToCentreMs);
    BT_Sendf("search return    : %lu ms\r\n", (unsigned long)s2_searchReturnMs);
    BT_Sendf("fast laps done   : %lu (last %lu ms)\r\n", (unsigned long)lapsDone, (unsigned long)lastLapMs);
    BT_Sendf("best fast lap    : %lu ms\r\n", (unsigned long)s2_bestFastLapMs);
}

// ── public entry point ───────────────────────────────────────────────────────
void Maze_Run2(int maxMoves)
{
    Maze_Init();
    Pose_Init();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s2_eventCount = 0;
    s2_sampleCount = 0;
    s2_arcCount = 0;
    s2_edgeCount = 0;
    s2_userAbort = 0;
    s2_timeUp = 0;
    s2_cruiseMmS = S2_SPEED_MM_S;     // the search runs at the search values
    s2_logIntervalMs = S2_LOG_INTERVAL_MS;
    s2_maxOmega  = S2_MAX_OMEGA;
    s2_finishComp = S2_FINISH_COMP_DEG;
    s2_btnSince = 0;
    s2_headRef = (float)g_currentHeading;

    // arm the button abort: first wait (max 3 s) for the button to be released, because you just pressed it to start.
    // If it is STILL "pressed" after 3 s the polarity setting is probably wrong, so the abort is switched off
    // instead of firing at once.
    {
        uint32_t t0 = HAL_GetTick();
        while ((HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == S2_BUTTON_PRESSED_LEVEL) &&
               ((HAL_GetTick() - t0) < 3000U)) { }
        s2_btnEnabled = (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) != S2_BUTTON_PRESSED_LEVEL) ? 1U : 0U;
        HAL_Delay(50);
    }
    s2_lastSampleTick = 0;
    s2_state = S2_ST_IDLE;
    s2_S = 0.0f;
    s2_sCenterNext = 0.0f;
    s2_sArcTarget = 0.0f;
    s2_decideMax_us = 0;
    s2_searchToCentreMs = 0;
    s2_searchReturnMs = 0;
    s2_bestFastLapMs = 0;
    s2_runStartTick = HAL_GetTick();
    g_irSteerOffInArc = S2_IR_OFF_IN_ARC;

    S2_Phase phase = S2_SEARCH_TO_CENTER;
    int moves = 0;
    float drift = 0.0f;
    float startBehind = S2_START_BEHIND_CENTER_MM;

    uint32_t centerReachedTick = s2_runStartTick;
    uint8_t  searchDone = 0;         // 1 = the search went to the centre AND back to the start without a problem

    BT_SendLine("===== FLOODFILL SEARCH 2 (CONTINUOUS) START =====");
    BT_SendLine(s2_btnEnabled ? "Press the button at any time to stop the mouse and print the logs."
                              : "Button abort is OFF (the button read as pressed for 3 s: check S2_BUTTON_PRESSED_LEVEL).");
    BT_Sendf("config: speed=%.0f maxOmega=%.0f cell=%.0f arcFwd=%.1f arcSide=%.1f arcPathTrue=%.1f arcExtra=%.1f decisionDepth=%.1f startBehind=%.1f\r\n",
             S2_SPEED_MM_S, S2_MAX_OMEGA, S2_CELL_MM, S2_ARC_FWD_MM, S2_ARC_SIDE_MM,
             S2_ARC_PATH_TRUE_MM, S2_ARC_EXTRA_MM, S2_DECISION_DEPTH_MM, S2_START_BEHIND_CENTER_MM);

    BT_Sendf("centre re-check %s, fires %.0f mm before the centre, stop bias +%.0f mm past it\r\n", S2_CENTRE_RECHECK ? "ON" : "OFF", S2_RECHECK_LEAD_MM, S2_REPLAN_STOP_BIAS_MM);
    BT_Sendf("side-wall settle: decisions at %.0f mm into the cell; centre wall overwrite %s; front wall = irF <= %.1f in at a decision\r\n",
             S2_DECISION_DEPTH_MM, S2_FORCE_CENTER_WALLS ? "ON" : "OFF", S2_FRONT_WALL_DECISION_IN);

    while (moves < maxMoves)
    {
        // ---- we are stopped at a cell centre here ----
        uint8_t cF = 0, cL = 0, cR = 0;
        S2_SenseCentre(&cF, &cL, &cR);          // stopped at a centre: prints read-vs-map and the believed maze
        S2_LogEvent(cF, cL, cR, S2_ACT_CENTER, 0.0f, 0, 0.0f);

        Location loc = Pose_GetLocation();
        Location target = S2_FloodTarget(phase);

        if (S2_AtTarget(loc, phase))
        {
            if (phase == S2_SEARCH_TO_CENTER)
            {
                centerReachedTick = HAL_GetTick();
                s2_searchToCentreMs = centerReachedTick - s2_runStartTick;
                BT_Sendf("Reached center - heading back to start. (%lu ms)\r\n",
                         (unsigned long)(centerReachedTick - s2_runStartTick));
                phase = S2_SEARCH_TO_START;

                // Square up on the front wall before heading back so any heading
                // error from the trip in does not carry into the return leg. This
                // needs a front wall; with an open front there is nothing to square on.
                if (cF)
                {
                    BT_SendLine("At center - resquaring on the front wall before the return leg.");
                    drift = Resquare();
                }
                else
                {
                    BT_SendLine("At center - no front wall, skipping resquare.");
                }
                continue;
            }
            else
            {
                BT_SendLine("Back at start - resquaring against the entrance wall before turning to face it.");
                float d = Resquare();
                Turn180Corrected(d);
                Pose_SetHeading(Heading_BehindFrom(Pose_GetHeading()));

                uint32_t doneTick = HAL_GetTick();
                BT_Sendf("Search complete. Return leg: %lu ms. Total search: %lu ms.\r\n",
                         (unsigned long)(doneTick - centerReachedTick),
                         (unsigned long)(doneTick - s2_runStartTick));
                s2_searchReturnMs = doneTick - centerReachedTick;
                searchDone = 1;
                break;
            }
        }

        Maze_Flood(target);
        Heading next = Maze_HeadingToSmallest(loc, Pose_GetHeading());

        if (next == HEADING_BLOCKED)
        {
            BT_SendLine("Fully blocked - stopping.");
            break;
        }

        BT_Sendf("center (%d,%d) heading=%d walls F=%u L=%u R=%u -> next heading=%d\r\n",
                 loc.x, loc.y, (int)Pose_GetHeading(), cF, cL, cR, (int)next);

        if (s2_userAbort)
        {
            BT_SendLine(s2_timeUp ? "search2: TIME UP. Printing the logs." : "search2: STOPPED BY BUTTON. Printing the logs.");
            break;
        }

        S2_TurnToward(next, drift);
        drift = 0.0f;

        S2_StopReason why = S2_RunChain(phase, &moves, maxMoves, startBehind);
        startBehind = 0.0f;   // after the first run we always restart from a cell centre
        if (s2_userAbort) why = S2_STOP_STALL;   // button pressed: report it as a stop-by-button below

        if (why == S2_STOP_STALL)
        {
            if (s2_userAbort)
                BT_SendLine(s2_timeUp ? "search2: TIME UP. Printing the logs." : "search2: STOPPED BY BUTTON. Printing the logs.");
            else
                BT_SendLine("search2: STALL - wheels stopped while the profile was still moving (hit a wall?). Stopped.");
            break;
        }
        if (why == S2_STOP_TIMEOUT)
        {
            BT_SendLine("search2: timeout - stopped.");
            break;
        }
        if (why == S2_STOP_BLOCKED)
        {
            BT_SendLine("Fully blocked - stopping.");
            break;
        }
        if (why == S2_STOP_MAXMOVES)
        {
            BT_SendLine("search2: move budget used up.");
            break;
        }
        if (why == S2_STOP_REPLAN)
        {
            {
                // heading error against the grid: the run starts facing north, a right turn adds +90 to the gyro
                float expect = s2_headRef + 90.0f * (float)Pose_GetHeading();
                float err = (float)g_currentHeading - expect;
                while (err > 180.0f) err -= 360.0f;
                while (err < -180.0f) err += 360.0f;
                BT_Sendf("search2: centre re-check changed the map - stopped %.1f mm past the cell centre (negative = short), gyro heading %.1f, heading error vs the grid %.1f deg (+ = turned right of the corridor). Spin turn next.\r\n",
                         s2_replanOff, (float)g_currentHeading, err);
            }
        }
        if (why == S2_STOP_REPLAN && s2_replanFrontWall)
        {
            BT_SendLine("search2: wall in front at the re-check stop - squaring on it.");
            drift = Resquare();
        }
        if (why == S2_STOP_DEADEND)
        {
            // front wall is ahead of us at the centre: square up on it, the
            // outer loop then floods and spins toward the open side
            drift = Resquare();
        }
        // S2_STOP_TARGET: outer loop handles the phase change
    }

    if (searchDone && !s2_userAbort)
    {
        S2_FastRun();
    }

    S2_PrintLog();
    BT_SendLine("===== FLOODFILL SEARCH 2 DONE =====");
}
