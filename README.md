# Micromouse II

A micromouse built around an STM32F411 that searches a 10x10 foam maze, then runs the shortest route back and forth at speed. It does continuous-floodfill search without stopping between cells, plans a fast route from the map it built, and corrects its own position from the walls while it drives.

Competition target: April 2027, NRC-style maze (10x10, 254 mm cells).

## Results so far

| Run | Speed | Time to the centre |
| --- | --- | --- |
| Search (continuous floodfill) | 300 mm/s | 33.8 s |
| Fast lap | 400 mm/s | 14.5 s |

- That is about 2.3x faster than the search run.
- In a full 10-minute run the mouse did 15 fast laps back to back, every one between 14.50 and 14.62 s, then stopped itself at the 10-minute mark.
- Search speed stays at 300 mm/s. Only the fast laps go faster.

## How it works

**Search (300 mm/s).** One long forward profile keeps running, topped up cell by cell, so the mouse never stops between cells. At a decision point inside each new cell it senses the walls, floods the maze and picks the next heading. Turns are arcs started at a measured point, not stop-and-spin turns.

**Fast laps (400 mm/s).**
1. `fast_route.h` plans the cheapest route through the mapped cells and merges cells in the same direction into one straight move.
2. The route runs as one continuous chain: straights at full speed, arcs through the turns, with its own arc geometry measured at 400 mm/s.
3. **Map-based re-sync.** The fast lap knows where every wall should start and end. When a side-wall flag flips, or the front IR sees the wall ahead of a turn, the mouse compares that with where it thought it was and corrects its position along the route.
4. **Go deeper, then resquare.** On a fast lap the mouse drives one cell further into the centre block, squares on the wall in front, then turns around. Squaring is the only true reset of position and heading, so every leg starts from zero error.
5. The fast lap is timed from the start to the first cell of the centre block.

A 10-minute run clock stops everything at the time limit.

## Hardware

- STM32F411CEU6
- BNO055 IMU for heading
- Magnetic encoders on micro metal gearmotors
- DRV8833 motor drivers
- IR sensors (front, left, right)
- Bluetooth serial link for logs
- Battery voltage monitoring

PCB and CAD are in [`hardware/`](hardware/).

## Repository layout

```
firmware/    STM32 firmware (Inc, Src, Startup, .ioc, linker scripts)
hardware/    KiCad PCB and CAD files
```

See [`firmware/README.md`](firmware/README.md) for the code layout, how to build, and the settings you can change.

## Where it is going next

- Higher fast-lap speed. 400 mm/s is solid, 450 and 500 need the arc geometry measured at those speeds.
- A speed ladder for race day: search at 300, then fast laps that step up speed after clean laps and step back down after a bad one.
- An integral term (or trim) in the rotation controller to remove the steady heading error after arcs, and a look at the steering gain on straights.
- Watching battery sag: at higher speed the motor voltage limit is reached during arcs.
