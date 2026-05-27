# Autonomous Navigation and Tracking

[中文版](README.md)

A MATLAB-based mobile robot autonomous navigation experiment project covering localization, mapping, path planning, and path tracking. The project integrates EKF landmark-based localization, particle filter SLAM, RRT global path planning, shortcut path optimization, Dubins curve generation, and Carrot-Chasing path tracking control.

Full report: [Autonomy_482348_Longlong Wang.pdf](Results/Autonomy_482348_Longlong%20Wang.pdf)

## Demos and Results

The following GIFs show the running results of each module. Full videos and the report are stored in the `Results/` directory.

| Module | GIF Preview | Full Video | Description |
| --- | --- | --- | --- |
| EKF Localization | [GIF](Results/ekf_localization.gif) | [MP4](Results/ekf_localization.mp4) | Robot pose estimation using landmark range-bearing observations |
| Particle Filter SLAM | [GIF](Results/particle_slam.gif) | [MP4](Results/Particle_slam.mp4) | Particle filter SLAM, landmark mapping, weight update, and resampling |
| RRT + Dubins Planning | [GIF](Results/rrt_dubins_planning.gif) | [MP4](Results/rrt_dubins_cca.mp4) | RRT obstacle-avoiding path search, shortcut optimization, and Dubins refinement |
| Dubins Tracking | [GIF](Results/dubins_tracking.gif) | [MP4](Results/dubins_path_tracking_doublespeed.mp4) | Dubins path generation and Carrot-Chasing path tracking |

### EKF Localization

This module uses range-bearing observations of known landmarks to estimate the robot pose `[x, y, theta]` with an EKF. It also evaluates filter performance through innovation, NIS, and state covariance analysis.

![EKF Localization](Results/ekf_localization.gif)

### Particle Filter SLAM

This module implements a FastSLAM-style particle filter mapping workflow. Each particle stores its own pose, weight, and landmark map. Particle weights are updated using measurement likelihoods, and resampling is triggered when the effective particle count drops.

![Particle Filter SLAM](Results/particle_slam.gif)

### RRT + Dubins Planning

This module uses RRT to search for a feasible path in a polygonal obstacle environment, removes redundant nodes through shortcut optimization, and then converts the polyline path into a trackable Dubins trajectory that satisfies a minimum turning radius constraint.

![RRT + Dubins Planning](Results/rrt_dubins_planning.gif)

### Dubins Tracking

This module generates a Dubins path from waypoint poses and simulates vehicle path tracking with the Carrot-Chasing method.

![Dubins Tracking](Results/dubins_tracking.gif)

## Features

- EKF pose estimation based on landmark range-bearing observations.
- Comparison between single-landmark and multi-landmark update modes.
- Filter stability analysis using innovation statistics, NIS consistency, and state uncertainty.
- FastSLAM-style particle filter mapping workflow.
- Landmark initialization, data association, weight update, `N_eff` monitoring, and resampling.
- RRT global path search in polygonal obstacle environments.
- Shortcut optimization to remove redundant nodes from RRT paths.
- Dubins path generation under minimum turning radius constraints.
- Carrot-Chasing path tracking simulation for a nonholonomic vehicle.

## Tech Stack

- MATLAB
- Extended Kalman Filter
- Particle Filter / FastSLAM-style SLAM
- Landmark-based Localization
- Range-Bearing Measurement Model
- RRT Path Planning
- Dubins Path
- Carrot-Chasing Path Following

## Project Structure

```text
.
|-- ekf_localization/
|   |-- main.m
|   |-- analysis_innovation.m
|   |-- analysis_NIS.m
|   |-- analysis_P.m
|   |-- animate_ekf.m
|   `-- data/
|-- particle_slam/
|   |-- main.m
|   |-- +particles/
|   |-- +perception/
|   |-- +map_manage/
|   |-- +control/
|   |-- +calculate/
|   |-- +visualise/
|   `-- +common/
|-- rrt_dubins_planning/
|   |-- main.m
|   |-- +path_planning/
|   |   |-- +rrt/
|   |   `-- +dubins/
|   |-- +path_following/
|   |-- +common/
|   `-- +data/
|-- dubins_tracking/
|   |-- main.m
|   |-- +path_planning/
|   |-- +path_following/
|   |-- +data/
|   `-- debug/
|-- Results/
`-- README.md
```

## Module Details

### EKF Localization

Path:

```text
ekf_localization/
```

This module estimates robot pose with a known landmark map. Inputs are control commands and landmark range-bearing measurements. Outputs include the estimated robot trajectory and covariance evolution.

Main contents:

- Nonlinear motion model prediction.
- Range-bearing observation model update.
- Single-landmark / multi-landmark update mode comparison.
- Kalman gain and Joseph-form covariance update.
- Innovation mean and standard deviation analysis.
- NIS consistency test.
- State uncertainty curves and covariance ellipse visualization.

Main files:

```text
ekf_localization/main.m
ekf_localization/analysis_innovation.m
ekf_localization/analysis_NIS.m
ekf_localization/analysis_P.m
ekf_localization/animate_ekf.m
```

### Particle Filter SLAM

Path:

```text
particle_slam/
```

This module implements a FastSLAM-style particle filter SLAM demo. Each particle maintains its own robot pose, weight, and landmark map. The system performs landmark initialization, data association, local EKF updates, and particle weight updates based on sensor observations.

Main contents:

- Particle state prediction and process noise injection.
- Range-bearing landmark measurement simulation.
- Landmark initialization and covariance calculation.
- Data association based on Mahalanobis distance.
- EKF-style update for matched landmarks.
- Particle weight normalization and measurement likelihood update.
- `N_eff` monitoring and resampling when the effective particle count is low.
- Landmark washing for merging or cleaning duplicate landmarks.
- Statistical analysis of particle error, effective particle count, and landmark association.

Main files:

```text
particle_slam/main.m
particle_slam/+particles/state_prediction.m
particle_slam/+particles/initialize_landmark.m
particle_slam/+perception/sensor_measurement.m
particle_slam/+map_manage/recheck_new_landmark.m
particle_slam/+map_manage/landmark_washing.m
particle_slam/+visualise/N_eff_particle.m
particle_slam/+visualise/particle_error.m
particle_slam/+visualise/landmark_association_anaysis.m
```

### RRT + Dubins Planning

Path:

```text
rrt_dubins_planning/
```

This module implements the workflow from global path search to trackable trajectory generation. It first uses RRT to search for a path in an obstacle environment, then reduces intermediate nodes with shortcut optimization, and finally converts the path into Dubins segments and tracks it with Carrot-Chasing.

Workflow:

```text
RRT search -> path backtracking -> shortcut optimization -> Dubins refinement -> Carrot-Chasing tracking
```

Main contents:

- Polygonal obstacle map construction.
- RRT random sampling, nearest-node search, and fixed-step expansion.
- Collision detection between line segments and polygonal obstacles.
- Raw path generation through backtracking from the RRT tree.
- Shortcut path optimization.
- Dubins refinement to satisfy the minimum turning radius constraint.
- Carrot-Chasing path tracking simulation.

Main files:

```text
rrt_dubins_planning/main.m
rrt_dubins_planning/+path_planning/+rrt/rrt.m
rrt_dubins_planning/+path_planning/+rrt/chk_collision.m
rrt_dubins_planning/+path_planning/+rrt/rrt_path_build.m
rrt_dubins_planning/+path_planning/+rrt/rrt_shortcut_opt.m
rrt_dubins_planning/+path_planning/+dubins/dubins_path.m
rrt_dubins_planning/+path_following/carrot_chasing.m
```

### Dubins Tracking

Path:

```text
dubins_tracking/
```

This module implements Dubins path generation and path tracking. The input is a set of waypoint poses, where each pose contains position and heading. The system computes left and right turning circle centers, generates CSC candidate paths, selects the shortest path, and tracks it with Carrot-Chasing.

Supported Dubins candidate types:

```text
LSL, RSR, LSR, RSL
```

Main contents:

- Left/right turning circle center calculation.
- CSC-type Dubins candidate path generation.
- Tangent point, arc, and straight segment calculation.
- Shortest Dubins path selection.
- Dubins path visualization.
- Carrot-Chasing path tracking.

Main files:

```text
dubins_tracking/main.m
dubins_tracking/+path_planning/calc_circle_centre.m
dubins_tracking/+path_planning/CSC.m
dubins_tracking/+path_planning/dubins_path_planning.m
dubins_tracking/+path_planning/draw_dubins_path.m
dubins_tracking/+path_following/carrot_chasing.m
```

## How to Run

### 1. Open MATLAB

Enter the root project directory:

```matlab
cd('D:\Workspace\autonomous-navigation-matlab')
```

### 2. Run EKF Localization

```matlab
cd('D:\Workspace\autonomous-navigation-matlab\ekf_localization')
main
```

### 3. Run Particle Filter SLAM

```matlab
cd('D:\Workspace\autonomous-navigation-matlab\particle_slam')
main
```

### 4. Run RRT + Dubins Planning

```matlab
cd('D:\Workspace\autonomous-navigation-matlab\rrt_dubins_planning')
main
```

### 5. Run Dubins Tracking

```matlab
cd('D:\Workspace\autonomous-navigation-matlab\dubins_tracking')
main
```

## Resume Project Mapping

This repository corresponds to the following resume project:

```text
Autonomous Navigation and Tracking | Autonomy
```

The project covers several core modules in mobile robot autonomous navigation:

```text
Localization -> SLAM -> Global Planning -> Path Refinement -> Path Following
```

It can be described as:

- Used EKF to estimate robot pose from landmark observations and compared single-landmark and multi-landmark update performance.
- Evaluated filter stability through innovation statistics, NIS consistency, and state uncertainty.
- Implemented a FastSLAM-style particle filter mapping workflow, including landmark initialization, data association, weight update, `N_eff` monitoring, and resampling.
- Used RRT for global path search in polygonal obstacle environments and applied shortcut optimization to remove redundant nodes.
- Converted paths into trackable trajectories satisfying minimum turning radius constraints with Dubins curves, and completed path tracking simulation with Carrot-Chasing.

## Notes

- This repository is an organized collection of MATLAB autonomy experiments, not a complete production-grade autonomous driving system.
- `ekf_localization` assumes known landmark positions, so it is more accurately described as landmark-based localization rather than SLAM.
- In `particle_slam`, each particle maintains its own landmark map, making it closer to a FastSLAM-style demo.
- The current Dubins path implementation mainly supports CSC candidate types: `LSL`, `RSR`, `LSR`, and `RSL`.
- RRT is implemented as a basic RRT, not RRT*.
- Some modules were originally independent experiments and have now been organized into a unified repository.