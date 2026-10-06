# Mobile Manipulator: Navigation, Pick-and-Place and Live Mapping

A MATLAB simulation of a four-wheel mobile base carrying a six-axis robotic arm. The mission combines waypoint-constrained A* navigation, scripted AGV crossing stops, two-block stacking, geometric wrist-camera inspection, and live LiDAR occupancy mapping.

**Measured in MATLAB R2024a on Windows.** This repository includes the supplied run data, numerical checks, a technical report and reproducible controller experiments.

[Read the technical report](docs/Mobile_Manipulator_Technical_Report.pdf) · [Model and control](docs/MODEL_AND_CONTROL.md) · [Evidence provenance](docs/EVIDENCE.md)

## Simulation demo

[![Mobile manipulator simulation](assets/mobile_manipulator_demo.gif)](assets/mobile_manipulator_demo.mp4)

[Watch or download the MP4](assets/mobile_manipulator_demo.mp4) · [Animated GIF](assets/mobile_manipulator_demo.gif)

The supplied edited clip is 51.63 seconds at 1280 × 720, 30 FPS. The GIF preserves its sequence at 8 FPS. Reported mission duration remains 413.05 simulation seconds.

![Mission performance](results/mission_analysis.png)

## Mission

1. Approach the pickup station and grasp the blue block.
2. Follow the extended curved route, stopping while the AGV crosses.
3. Place the first block and return for the red block.
4. Stack the red block, hold a six-second synthetic inspection sweep, then return to the dock.

Pickup and placement pads are protected from base travel, including departure and turning. The 3D simulation opens in a separate window; the second window contains the live map, speed, clearance and status.

## Measured results

| Metric | Supplied MATLAB result |
|---|---:|
| Mission completed | Yes |
| Simulated mission duration | 413.05 s |
| Base travel | 113.134 m |
| Modeled collision samples | 0 |
| Minimum modeled clearance | 0.060003 m |
| Navigation nearest-path RMS | 0.020103 m |
| Dock position error | 0.039092 m |
| AGV crossings | 8 |
| Final scan hold | 6 s |
| Processed mapping scans | 2066 |

The two placement errors are approximately 0.58 and 0.61 micrometres in an idealized rigid-attachment simulation. They are not hardware accuracy estimates.

## Controller comparison

The separate torque-driven arm benchmark compares PD + gravity compensation against computed torque, using identical references, torque limits and matched noise seeds.

![Controller comparison](assets/controller_summary.png)

| Scenario | PD RMS (rad) | Computed-torque RMS (rad) |
|---|---:|---:|
| Nominal, 0 kg | 0.00162835 | 0.00021985 |
| Nominal, 1 kg | 0.00255107 | 0.00021985 |
| Nominal, 2 kg | 0.00373531 | 0.00021985 |
| 2 kg; 20% mass mismatch; noise + disturbance | 0.01688337 | 0.04747052 |

Computed torque tracks better in the three nominal cases. PD + gravity has lower pooled RMS in the combined stress case and all 20 sampled uncertainty trials. The Monte Carlo mean RMS is **0.007494 rad for PD** versus **0.013617 rad for computed torque**. This supports a trade-off at the supplied gains and scenarios, not a universal ranking.

## Live map

![Live occupancy map](results/live_slam_map.png)

The map starts unknown and updates at 5 Hz on a 0.10 m grid. Incremental simulation odometry predicts pose; a local correlative scan matcher aligns range returns with the accumulated map. Log-odds beam updates mark free and occupied cells. The display includes estimated pose, heading, trajectory and scan endpoints.

This is local scan matching and incremental mapping, **without loop closure or pose-graph optimization**. Replay uses ideal simulation odometry and noise-free LiDAR. Near-zero measured pose error therefore does not establish robustness to real sensor noise or drift. The navigation planner uses known obstacle geometry; it does not plan from the live occupancy map.

## Run

MATLAB R2024a on Windows is the verified user environment. The project uses base MATLAB functions; video export uses VideoWriter. Open this repository folder as MATLAB Current Folder.

Replay the included measured mission:

```matlab
clear functions
replay_simulation
```

Recompute the main suite (archives any previous results folder):

```matlab
run_all_results
```

Optional uncertainty study and simulation-only video, after the previous command finishes:

```matlab
run_monte_carlo(20)
export_video(30,2)
package_results
```

`run_all_results` runs numerical model tests, the full mission, station/AGV checks, SLAM checks and eight controller trials. Monte Carlo is separate. `resume_results` is available to rerun controller trials while keeping an existing mission.

## Repository contents

- `src/`: kinematics, dynamics, planning, sensing, mapping and rendering.
- `tests/`: model, mission, station-clearance, mapping and renderer checks.
- `results/`: supplied MAT/CSV data, figures and MATLAB execution log.
- `assets/`: comparison plots derived from the supplied measurements.
- `docs/`: technical report, mathematical model and evidence provenance.

## Scope and limitations

The mission arm uses an acceleration-command servo with inverse-dynamics torque checks; the independent controller experiments integrate torque-driven dynamics. Do not treat them as identical validation tasks. AGV stopping uses scripted lane occupancy plus geometric safeguards. Collision checks are simplified and do not cover full self-collision or physical contact. Grasping uses rigid attachment. The wrist-camera scan is a synthetic sweep and geometric projection of known block centroids, not image-based recognition. There is no hardware deployment or safety certification.

The supplied simulation MP4 and a derived animated GIF are included in assets/. The media is an edited demonstration; numerical timing and performance values come from the measured MATLAB logs.
