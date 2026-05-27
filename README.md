# Autonomous Navigation and Tracking

一个基于 MATLAB 的移动机器人自主导航实验项目，覆盖定位、建图、路径规划与路径跟踪流程。项目整合了 EKF landmark-based localization、粒子滤波 SLAM、RRT 全局路径规划、shortcut 路径优化、Dubins 曲线路径生成和 Carrot-Chasing 路径跟踪控制等模块。

## 演示与结果

以下 GIF 用于展示各模块的运行效果。高清或完整结果可通过运行对应模块重新生成。

| 模块 | 预览 | 内容 |
| --- | --- | --- |
| EKF Localization | [GIF](media/ekf_localization.gif) | 基于 landmark range-bearing 观测的机器人位姿估计 |
| Particle Filter SLAM | [GIF](media/particle_slam.gif) | 粒子滤波 SLAM、landmark 建图、权重更新与重采样 |
| RRT + Dubins Planning | [GIF](media/rrt_dubins_planning.gif) | RRT 避障路径搜索、shortcut 优化与 Dubins refinement |
| Dubins Tracking | [GIF](media/dubins_tracking.gif) | Dubins path 生成与 Carrot-Chasing 路径跟踪 |

### EKF Localization

该模块使用已知 landmark 的 range-bearing 观测对机器人位姿 `[x, y, theta]` 进行 EKF 估计，并通过 innovation、NIS 和状态协方差分析滤波表现。

![EKF Localization](media/ekf_localization.gif)

### Particle Filter SLAM

该模块实现 FastSLAM-style 粒子滤波建图流程。每个 particle 保存自身位姿、权重和 landmark map，并根据观测似然更新权重，在有效粒子数下降时触发重采样。

![Particle Filter SLAM](media/particle_slam.gif)

### RRT + Dubins Planning

该模块在多边形障碍环境中使用 RRT 搜索可行路径，通过 shortcut 删除冗余节点，再使用 Dubins 曲线将折线路径转换为满足最小转弯半径约束的可跟踪轨迹。

![RRT + Dubins Planning](media/rrt_dubins_planning.gif)

### Dubins Tracking

该模块基于 waypoint pose 生成 Dubins path，并使用 Carrot-Chasing 方法进行车辆路径跟踪仿真。

![Dubins Tracking](media/dubins_tracking.gif)

## 功能特性

- 基于 landmark range-bearing 观测的 EKF 位姿估计。
- 支持 single-landmark 与 multi-landmark 更新模式对比。
- 使用 innovation statistics、NIS consistency 和 state uncertainty 分析滤波稳定性。
- 实现 FastSLAM-style 粒子滤波建图流程。
- 支持 landmark 初始化、数据关联、权重更新、`N_eff` 监控和重采样。
- 使用 RRT 在多边形障碍环境中进行全局路径搜索。
- 通过 shortcut 优化删除 RRT 路径中的冗余节点。
- 使用 Dubins 曲线生成满足最小转弯半径约束的路径。
- 使用 Carrot-Chasing 方法完成非完整约束车辆的路径跟踪仿真。

## 技术栈

- MATLAB
- Extended Kalman Filter
- Particle Filter / FastSLAM-style SLAM
- Landmark-based Localization
- Range-Bearing Measurement Model
- RRT Path Planning
- Dubins Path
- Carrot-Chasing Path Following

## 项目结构

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
|-- media/
`-- README.md