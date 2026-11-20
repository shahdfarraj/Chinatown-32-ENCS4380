/*
 * solver.c - Team T32 - robot name: chinatown
 * WEEK 2 - week 1 + flood fill (distances shown in the cells)
 */
#include "mouse_hal.h"
#include <stdio.h>

#define UNSEEN      0xFFFFu   /* "no distance yet" marker                    */
#define WANDER_STEPS 100      /* how many steps the test walk takes          */

/* ---- State -------------------------------------------------------------- */

static mm_maze_t maze;                                  /* size and goal      */
static uint8_t   walls[MM_MAX_SIZE][MM_MAX_SIZE];       /* walls we believe   */
static uint8_t   known[MM_MAX_SIZE][MM_MAX_SIZE];       /* sides we observed  */
static uint16_t  dist[MM_MAX_SIZE][MM_MAX_SIZE];        /* flood-fill numbers */
static int px, py, heading;                             /* where the mouse is */

/* One step in each absolute direction: N, E, S, W (y grows to the north). */
static const int DX[4] = { 0, 1, 0, -1 };
static const int DY[4] = { 1, 0, -1, 0 };

static bool in_maze(int x, int y)
{
    return x >= 0 && x < maze.width && y >= 0 && y < maze.height;
}

/* ---- Step 1: map and position ------------------------------------------- */

/* Record one side of a cell, and the matching side of its neighbour. */
static void set_side(int x, int y, int dir, bool is_wall)
{
    uint8_t b = MM_WALL(dir);
    known[x][y] |= b;
    if (is_wall) walls[x][y] |= b; else walls[x][y] &= (uint8_t)~b;

    int nx = x + DX[dir], ny = y + DY[dir];
    if (in_maze(nx, ny)) {
        uint8_t ob = MM_WALL((dir + 2) & 3);            /* opposite side      */
        known[nx][ny] |= ob;
        if (is_wall) walls[nx][ny] |= ob; else walls[nx][ny] &= (uint8_t)~ob;
    }
}

/* Start with an empty map, except the outer border, which always exists. */
static void init_map(void)
{
    for (int x = 0; x < MM_MAX_SIZE; x++)
        for (int y = 0; y < MM_MAX_SIZE; y++) {
            walls[x][y] = 0;
            known[x][y] = 0;
            dist[x][y]  = UNSEEN;
        }
    for (int x = 0; x < maze.width; x++) {
        set_side(x, 0, MM_SOUTH, true);
        set_side(x, maze.height - 1, MM_NORTH, true);
    }
    for (int y = 0; y < maze.height; y++) {
        set_side(0, y, MM_WEST, true);
        set_side(maze.width - 1, y, MM_EAST, true);
    }
}

/* Read the three sensors, turn left/front/right into absolute walls, and
 * return the raw relative reading so the caller can decide where to go. */
static uint8_t sense_here(void)
{
    uint8_t s = mm_sense();
    const uint8_t bits[3] = { MM_SEE_LEFT, MM_SEE_FRONT, MM_SEE_RIGHT };
    const int     dirs[3] = { (heading + 3) & 3, heading, (heading + 1) & 3 };

    for (int i = 0; i < 3; i++) {
        bool wall = (s & bits[i]) != 0;
        bool seen_before = (known[px][py] & MM_WALL(dirs[i])) != 0;
        set_side(px, py, dirs[i], wall);
        if (wall && !seen_before) mm_show_wall(px, py, dirs[i]);
    }
    return s;
}

/* Turn in place to face an absolute direction. */
static mm_status_t face(int dir)
{
    int diff = (dir - heading + 4) & 3;
    mm_status_t st = MM_OK;
    if      (diff == 1) st = mm_turn(+1);
    else if (diff == 2) st = mm_turn(2);
    else if (diff == 3) st = mm_turn(-1);
    if (st == MM_OK) heading = dir;
    return st;
}

/* Drive one cell forward and update x, y. If a wall stops us, learn it. */
static mm_status_t step(void)
{
    mm_status_t st = mm_forward(1);
    if (st == MM_OK) {
        set_side(px, py, heading, false);   /* we just drove through it      */
        px += DX[heading];
        py += DY[heading];
    } else if (st == MM_BLOCKED) {
        set_side(px, py, heading, true);    /* the map was wrong: fix it     */
        mm_show_wall(px, py, heading);
    }
    return st;
}

/* ---- Step 2: flood fill ------------------------------------------------- */

/*
 * Give every cell its distance to the target block (x0,y0)-(x1,y1) with a
 * queue (breadth-first search, no recursion).
 *   optimistic = true : sides we have not seen are treated as open
 *   optimistic = false: only sides we know are open are used ("known path")
 */
static void flood(int x0, int y0, int x1, int y1, bool optimistic)
{
    static uint8_t qx[MM_MAX_SIZE * MM_MAX_SIZE];
    static uint8_t qy[MM_MAX_SIZE * MM_MAX_SIZE];
    int head = 0, tail = 0;

    for (int x = 0; x < maze.width; x++)
        for (int y = 0; y < maze.height; y++) dist[x][y] = UNSEEN;

    for (int x = x0; x <= x1; x++)
        for (int y = y0; y <= y1; y++) {
            dist[x][y] = 0;
            qx[tail] = (uint8_t)x;
            qy[tail] = (uint8_t)y;
            tail++;
        }

    while (head < tail) {
        int cx = qx[head], cy = qy[head];
        head++;
        for (int d = 0; d < 4; d++) {
            uint8_t b = MM_WALL(d);
            if (walls[cx][cy] & b) continue;                 /* wall          */
            if (!optimistic && !(known[cx][cy] & b)) continue;
            int nx = cx + DX[d], ny = cy + DY[d];
            if (!in_maze(nx, ny)) continue;
            if (dist[nx][ny] != UNSEEN) continue;            /* already done  */
            dist[nx][ny] = (uint16_t)(dist[cx][cy] + 1);
            qx[tail] = (uint8_t)nx;
            qy[tail] = (uint8_t)ny;
            tail++;
        }
    }
}

/* Write the distance of every reached cell on the simulator maze. */
static void show_distances(void)
{
    char text[12];
    for (int x = 0; x < maze.width; x++)
        for (int y = 0; y < maze.height; y++)
            if (dist[x][y] != UNSEEN) {
                snprintf(text, sizeof text, "%u", (unsigned)dist[x][y]);
                mm_label(x, y, text);
            }
}

/* ---- Test walk (weeks 1-2 only) ------------------------------------------ */

/* Left-hand wall follower. It only moves the mouse around so the map fills
 * up. After every step the distances to the goal are recalculated and shown,
 * so you can watch the numbers change when a new wall is found. */
static void wander(int steps)
{
    for (int n = 0; n < steps; n++) {
        uint8_t s = sense_here();

        /* Week 2: flood fill from the goal, show the numbers */
        flood(maze.goal_x0, maze.goal_y0, maze.goal_x1, maze.goal_y1, true);
        show_distances();

        int d;
        if      (!(s & MM_SEE_LEFT))  d = (heading + 3) & 3;
        else if (!(s & MM_SEE_FRONT)) d = heading;
        else if (!(s & MM_SEE_RIGHT)) d = (heading + 1) & 3;
        else                          d = (heading + 2) & 3;

        if (face(d) != MM_OK) return;
        if (step() != MM_OK) return;
        mm_mark(px, py, MM_COLOR_GREEN);
    }
}

/* ---- Entry point -------------------------------------------------------- */

void solver_run(void)
{
    mm_begin("T32");

    maze = mm_maze();
    mm_log("solver started: maze %dx%d", maze.width, maze.height);

    init_map();
    px = 0; py = 0; heading = MM_NORTH;    /* start: (0,0) facing north      */

    wander(WANDER_STEPS);
    mm_log("tracked position: (%d,%d) heading %d", px, py, heading);
}
