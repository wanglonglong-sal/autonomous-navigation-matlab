# Dubins Path Project

This project implements Dubins-path-based planning and path-following simulation.  
It is organized into three main parts: data parsing, path planning, and path following.

Note: This document covers core workflow files only. Files under `debug/` are intentionally excluded.

## Directory Structure

```text
dubins_path/
├─ main.m
├─ +data/
│  └─ parse_D_list.m
├─ +path_planning/
│  ├─ calc_circle_centre.m
│  ├─ CSC.m
│  ├─ dubins_path_planning.m
│  └─ draw_dubins_path.m
└─ +path_following/
   └─ carrot_chasing.m
```

## File Responsibilities

### Root

- `main.m`  
  Entry script of the project. It sets scenario parameters, calls the path-planning module to generate a Dubins path, and then runs path-following simulation and visualization.

### `+data/`

- `+data/parse_D_list.m`  
  Parses a Dubins path record `D` (1x22) into named fields such as start/end states, circle centers, tangent points, angles, and turn directions.

### `+path_planning/`

- `+path_planning/calc_circle_centre.m`  
  Computes left/right turning circle centers from pose and turning radius. This is a core geometric utility for Dubins construction.

- `+path_planning/CSC.m`  
  Computes geometry for CSC-type candidates (e.g., LSL, RSR, LSR, RSL), including tangent-related quantities.

- `+path_planning/dubins_path_planning.m`  
  Main planning function. It evaluates candidate Dubins paths, compares total length, and outputs the best path description (`D_list`).

- `+path_planning/draw_dubins_path.m`  
  Visualizes Dubins segments (arc and line) based on `D_list` for inspection and validation.

### `+path_following/`

- `+path_following/carrot_chasing.m`  
  Main tracking function. It follows sub-segments in sequence (start arc -> tangent line -> final arc) using carrot-chasing logic, computes desired heading, and updates vehicle states.

## Typical Workflow

1. `main.m` defines scenario and parameters.
2. `+path_planning/` generates and draws the Dubins path.
3. `+path_following/carrot_chasing.m` performs trajectory tracking simulation.
4. `+data/parse_D_list.m` provides a common parsing interface for planning and tracking modules.

