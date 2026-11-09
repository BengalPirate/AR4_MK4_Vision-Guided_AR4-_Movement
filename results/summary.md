# Test results

## Positioning error (camera target -> pen tip)

- Trials: 13
- Mean 6.1 mm, RMS 8.2 mm, max 16.4 mm
- Mean offset (bias): dX +1.2 mm, dY +3.2 mm

| # | Time | Target X | Target Y | Correction dX | dY | Error (mm) |
|---|---|---|---|---|---|---|
| 1 | 2026-11-09 12:00:52 | 478.9 | -73.5 | +4.5 | +11.5 | 12.3 |
| 2 | 2026-11-09 12:02:43 | 439.7 | -244.2 | +4.1 | +6.5 | 7.6 |
| 3 | 2026-11-09 13:04:45 | 457.8 | -25.9 | +3.4 | -0.5 | 3.4 |
| 4 | 2026-11-09 13:05:48 | 419.8 | -197.2 | -0.1 | -3.0 | 3.0 |
| 5 | 2026-11-09 13:06:30 | 349.4 | -98.5 | -0.0 | +0.0 | 0.0 |
| 6 | 2026-11-09 13:07:13 | 355.2 | -249.7 | -3.9 | -8.6 | 9.5 |
| 7 | 2026-11-09 13:08:31 | 342.8 | -41.4 | +3.7 | -2.4 | 4.4 |
| 8 | 2026-11-09 13:09:19 | 403.2 | -74.8 | -0.2 | +1.8 | 1.8 |
| 9 | 2026-11-09 13:09:55 | 403.2 | -74.8 | -0.2 | +0.6 | 0.6 |
| 10 | 2026-11-09 13:10:38 | 403.2 | -74.8 | +0.9 | +16.4 | 16.4 |
| 11 | 2026-11-09 13:12:38 | 403.2 | -74.8 | +1.8 | +15.5 | 15.7 |
| 12 | 2026-11-09 13:13:17 | 403.2 | -74.8 | -0.1 | +1.6 | 1.6 |
| 13 | 2026-11-09 13:14:06 | 403.2 | -74.8 | +1.8 | +2.1 | 2.8 |

## Repeatability (same target location, repeated trials)

| Location (X, Y) | Trials | Commanded spread (mm) | Corrected-position spread (mm) |
|---|---|---|---|
| 403, -75 | 6 | 0.1 | 20.1 |

## Controller tracking (commanded vs reported)

- Hover moves: 14, max difference 0.97 mm
- Detection spread at the moment of each move: max 0.5 mm

## Rejected / refused commands

| Time | Event | Reason |
|---|---|---|
| 2026-11-09 11:53:22 | limit_test | unreachable 900,0 |
| 2026-11-09 11:54:24 | limit_test | unreachable 900,0 |
| 2026-11-09 11:55:08 | limit_test | marker 7: wrist singularity, J5 5 deg |
| 2026-11-09 12:03:02 | go | Unreachable (wrist singularity, J5 11 deg) |
| 2026-11-09 12:03:09 | go | Unreachable (wrist singularity, J5 11 deg) |
| 2026-11-09 13:05:18 | go | No target (size 18 mm2) |
| 2026-11-09 13:05:23 | go | No target (size 28 mm2) |
| 2026-11-09 13:07:38 | go | Unreachable (joint limit) |
| 2026-11-09 13:07:41 | go | Unreachable (joint limit) |
| 2026-11-09 13:14:35 | go | Board markers not visible: [4] |
| 2026-11-09 13:15:06 | go | Ambiguous: 2 targets of this colour |
| 2026-11-09 13:15:23 | go | No target (size 77 mm2) |
| 2026-11-09 13:15:32 | go | No target (size 77 mm2) |
