# Micromouse II firmware

STM32F411CEUx, STM32CubeIDE project. The control loop runs at 1 kHz.

## Building

The ST HAL drivers are not stored here. To build:

1. In STM32CubeIDE, create a project from `Micromouse II.ioc` (STM32F411CEUx) and generate the code. That regenerates `Drivers/` and the HAL setup.
2. Copy `Inc/` and `Src/` from this folder into the project's `Core/Inc` and `Core/Src`, and `Startup/` into `Core/Startup`.
3. Use the linker scripts `STM32F411CEUX_FLASH.ld` or `STM32F411CEUX_RAM.ld` from this folder.

## Choosing what runs

In `Src/main.c`:

| Setting | Meaning |
| --- | --- |
| `CURRENT_MODE` | `0` runs the calibration and tests in `calibration.c`, `1` runs the maze |
| `TEST_MAZE_SOLVER2` | `1` runs the continuous search and the fast laps (`Maze_Run2`) |

The tests in `Src/calibration.c` are switched on one at a time with the `TEST_*` flags at the top of the file (encoders, IR sensors, turn calibration, battery, position control, and the path test that measures the arcs).

## Code layout

| Area | Files |
| --- | --- |
| Maze and pose | `maze.c`, `pose.c` |
| Original stop-and-go solver | `maze_solver.c` |
| Continuous search and fast laps | `maze_solver2.c` |
| Fast route planner (pure logic) | `fast_route.h` |
| Motion profiles | `ramp_profile.c` |
| Controllers | `motion_controller.c` (rotation), `speed_controller.c`, `straight_controller.c` |
| Motion helpers | `motion_actions.c`, `motion_primitives.c`, `motion_guard.c`, `motion_state.c`, `turn.c` |
| Sensors | `sensors.c` (IR walls and distances), `bno055.c` (IMU), `encoder.c`, `battery.c` |
| Hardware | `motor.c`, `button.c`, `bt.c` (Bluetooth log output) |
| Calibration and tests | `calibration.c` |

## Settings in `maze_solver2.c`

| Setting | What it does |
| --- | --- |
| `S2_FAST_SPEED_MM_S`, `S2_FAST_MAX_OMEGA` | Fast-lap speed (400 mm/s) and arc turn rate |
| `S2_FAST_ARC_*` | Fast arc geometry, measured at 400 mm/s with the path test |
| `S2_FAST_FINISH_COMP_DEG` | Heading compensation added to the 90 degrees of a fast arc |
| `S2_FAST_DEMO_300` | `1` runs the fast laps at the 300 mm/s search values, for comparison videos |
| `S2_RESYNC_ENABLE`, `S2_RESYNC_SIDE_ON`, `S2_RESYNC_FRONT_ON` | Turn the map-based position re-sync on or off |
| `S2_RESYNC_FRONT_OFFSET_MM` | Distance from the front IR to the wheel axle |
| `S2_FAST_GO_DEEPER` | `1` drives one cell deeper into the centre block on fast laps and resquares |
| `S2_FAST_LAP_COUNT` | Number of fast laps, `0` means lap until the time limit |

`RUN_TIME_LIMIT_MS` in `Inc/main.h` is the run clock (600000 ms, 10 minutes). The clock starts after the second button press and stops the run in the SysTick handler.

## Logs

The mouse keeps ring buffers of decisions, arc starts, wall-flag changes, re-sync corrections and motion samples. They print over Bluetooth after the run ends (time limit or button press). Each fast lap also prints its lap time as it finishes.
