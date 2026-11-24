// fast_route.h
// ============================================================================
// Fast-run route planner. Pure logic: no hardware, no simulator calls, so the
// same file works in the MMS simulator and in the micromouse firmware.
//
// WHAT IT DOES
//   Given the wall map from the search, it finds the cheapest route from the
//   start cell to the goal and returns it as a short list of moves:
//       straight N cells, arc left, arc right (and, only at the very start, a
//       spin if the mouse is not facing the way the route begins).
//   Consecutive cells in the same direction are merged into ONE straight move,
//   which is what lets the fast run use a long straight at full speed.
//
//   Cost = every cell step costs FR_STEP_COST, every turn adds FR_TURN_COST.
//   Turns are expensive because each arc takes a fixed time no matter how fast
//   the straights are, so the planner will happily take a slightly longer route
//   with fewer turns.
//
// HOW TO USE
//   In exactly ONE source file:   #define FAST_ROUTE_IMPLEMENTATION
//                                 #include "fast_route.h"
//   In every other file just:     #include "fast_route.h"
//
//   walls[x*h + y]   bit0 = wall on the NORTH side, bit1 = EAST, bit2 = SOUTH,
//                    bit3 = WEST (set = wall). Same layout as the MMS program.
//   visited[x*h + y] nonzero = the mouse has been in that cell, so its walls are
//                    known. Pass NULL to treat every cell as known.
//                    The route only passes THROUGH visited cells. The goal cell
//                    itself does not have to be visited.
//   headings         0 = north, 1 = east, 2 = south, 3 = west. Positive angle is
//                    a right turn, same as the rest of the code.
// ============================================================================
#ifndef FAST_ROUTE_H
#define FAST_ROUTE_H

#ifdef __cplusplus
extern "C" {
#endif

#define FR_MAX_W        16
#define FR_MAX_H        16
// A route is at most one straight plus one arc per cell, so about 2 * cells. A winding
// maze can need far more than 96 moves, which made the planner fail on those mazes.
#ifndef FR_MAX_MOVES
#define FR_MAX_MOVES    (FR_MAX_W * FR_MAX_H * 2 + 8)
#endif

#define FR_STEP_COST    10          // cost of moving one cell
#define FR_TURN_COST    6           // extra cost of one 90 degree turn (arc)

typedef enum
{
    FR_STRAIGHT = 0,                // 'cells' = how many cells to drive
    FR_ARC_LEFT,                    // 90 degree arc through the current cell
    FR_ARC_RIGHT,
    FR_SPIN_LEFT,                   // only ever the first move: face the route
    FR_SPIN_RIGHT,
    FR_SPIN_U
} FR_MoveType;

typedef struct
{
    unsigned char type;             // FR_MoveType
    unsigned char cells;            // used by FR_STRAIGHT only
} FR_Move;

typedef struct
{
    FR_Move moves[FR_MAX_MOVES];
    int     count;                  // number of moves
    int     cells;                  // total cells driven
    int     turns;                  // arcs in the route
    int     cost;                   // total planner cost
} FR_Route;

// Returns 1 and fills 'out' if a route exists, 0 if there is none.
int FR_Plan(const unsigned char *walls, const unsigned char *visited,
            int w, int h,
            int startX, int startY, int startHeading,
            int goalX1, int goalY1, int goalX2, int goalY2,
            FR_Route *out);

// Writes the route as text such as "S3 R S2 L S1" into buf (always terminated).
void FR_ToString(const FR_Route *r, char *buf, int bufSize);

#ifdef __cplusplus
}
#endif

// ───────────────────────────── implementation ───────────────────────────────
#ifdef FAST_ROUTE_IMPLEMENTATION
#ifndef FAST_ROUTE_IMPLEMENTATION_DONE
#define FAST_ROUTE_IMPLEMENTATION_DONE

#ifdef __cplusplus
extern "C" {
#endif

#define FR_STATES   (FR_MAX_W * FR_MAX_H * 4)
#define FR_INF      0x7FFF

static const int fr_dx[4] = { 0, 1, 0, -1 };
static const int fr_dy[4] = { 1, 0, -1, 0 };

static int fr_wall(const unsigned char *walls, int h, int x, int y, int dir)
{
    return (walls[x * h + y] >> dir) & 1;
}

int FR_Plan(const unsigned char *walls, const unsigned char *visited,
            int w, int h,
            int startX, int startY, int startHeading,
            int goalX1, int goalY1, int goalX2, int goalY2,
            FR_Route *out)
{
    static short         dist[FR_STATES];
    static short         prev[FR_STATES];
    static unsigned char done[FR_STATES];

    int total = w * h * 4;
    int i;

    out->count = 0; out->cells = 0; out->turns = 0; out->cost = 0;
    if (w > FR_MAX_W || h > FR_MAX_H) return 0;

    for (i = 0; i < total; i++) { dist[i] = FR_INF; prev[i] = -1; done[i] = 0; }

    int startState = (startX * h + startY) * 4 + startHeading;
    dist[startState] = 0;

    int goalState = -1;

    // plain O(n^2) Dijkstra: at most 16*16*4 = 1024 states, fast enough everywhere
    for (;;)
    {
        int best = -1;
        int bestD = FR_INF;
        for (i = 0; i < total; i++)
            if (!done[i] && dist[i] < bestD) { bestD = dist[i]; best = i; }
        if (best < 0) break;
        done[best] = 1;

        int hd = best & 3;
        int cell = best >> 2;
        int x = cell / h;
        int y = cell % h;

        if (x >= goalX1 && x <= goalX2 && y >= goalY1 && y <= goalY2)
        {
            goalState = best;
            break;
        }

        // only cells the mouse has been in have known walls, so only expand those
        if (visited && !visited[cell]) continue;

        int atStart = (best == startState);

        for (int nh = 0; nh < 4; nh++)
        {
            int diff = (nh - hd + 4) & 3;
            if (diff == 2 && !atStart) continue;           // no reversing, except the spin at the start
            if (fr_wall(walls, h, x, y, nh)) continue;

            int nx = x + fr_dx[nh];
            int ny = y + fr_dy[nh];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

            int c = FR_STEP_COST;
            if (diff == 1 || diff == 3) c += FR_TURN_COST;
            else if (diff == 2)         c += 2 * FR_TURN_COST;

            int ns = (nx * h + ny) * 4 + nh;
            int nd = dist[best] + c;
            if (nd < dist[ns]) { dist[ns] = (short)nd; prev[ns] = (short)best; }
        }
    }

    if (goalState < 0) return 0;

    // walk back from the goal to get the heading of every step
    int heads[FR_MAX_W * FR_MAX_H];
    int n = 0;
    int s = goalState;
    while (s != startState)
    {
        if (n >= FR_MAX_W * FR_MAX_H) return 0;
        heads[n++] = s & 3;
        s = prev[s];
        if (s < 0) return 0;
    }
    for (i = 0; i < n / 2; i++) { int t = heads[i]; heads[i] = heads[n - 1 - i]; heads[n - 1 - i] = t; }

    out->cost = dist[goalState];
    if (n == 0) return 1;                                  // already at the goal

    // turn the list of headings into moves
    int cnt = 0;
    if (heads[0] != startHeading)
    {
        int d0 = (heads[0] - startHeading + 4) & 3;
        out->moves[cnt].type  = (d0 == 1) ? FR_SPIN_RIGHT : (d0 == 3) ? FR_SPIN_LEFT : FR_SPIN_U;
        out->moves[cnt].cells = 0;
        cnt++;
    }

    int run = 0;
    for (i = 0; i < n; i++)
    {
        run++;
        int last = (i == n - 1);
        if (last || heads[i + 1] != heads[i])
        {
            if (cnt + 2 >= FR_MAX_MOVES) return 0;
            out->moves[cnt].type  = FR_STRAIGHT;
            out->moves[cnt].cells = (unsigned char)run;
            cnt++;
            out->cells += run;
            run = 0;
            if (!last)
            {
                int d = (heads[i + 1] - heads[i] + 4) & 3;
                out->moves[cnt].type  = (d == 1) ? FR_ARC_RIGHT : FR_ARC_LEFT;
                out->moves[cnt].cells = 0;
                cnt++;
                out->turns++;
            }
        }
    }
    out->count = cnt;
    return 1;
}

void FR_ToString(const FR_Route *r, char *buf, int bufSize)
{
    int pos = 0;
    if (bufSize <= 0) return;
    buf[0] = 0;
    for (int i = 0; i < r->count; i++)
    {
        char tok[8];
        int len = 0;
        switch (r->moves[i].type)
        {
            case FR_STRAIGHT:
                tok[len++] = 'S';
                if (r->moves[i].cells >= 10) tok[len++] = (char)('0' + r->moves[i].cells / 10);
                tok[len++] = (char)('0' + r->moves[i].cells % 10);
                break;
            case FR_ARC_LEFT:   tok[len++] = 'L'; break;
            case FR_ARC_RIGHT:  tok[len++] = 'R'; break;
            case FR_SPIN_LEFT:  tok[len++] = 'l'; break;
            case FR_SPIN_RIGHT: tok[len++] = 'r'; break;
            default:            tok[len++] = 'U'; break;
        }
        if (pos + len + 2 >= bufSize) break;
        if (pos > 0) buf[pos++] = ' ';
        for (int k = 0; k < len; k++) buf[pos++] = tok[k];
        buf[pos] = 0;
    }
}

#ifdef __cplusplus
}
#endif

#endif // FAST_ROUTE_IMPLEMENTATION_DONE
#endif // FAST_ROUTE_IMPLEMENTATION

#endif // FAST_ROUTE_H
