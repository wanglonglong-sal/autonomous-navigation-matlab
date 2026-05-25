# Dubins Path Project

本项目实现了基于 Dubins 曲线的路径规划与路径跟随仿真，主要包含三部分：数据解析、路径规划、路径跟随。

说明：本文档仅介绍主流程相关文件，`debug/` 目录内文件已排除。

## 目录结构

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

## 文件功能说明

### 根目录

- `main.m`  
  项目入口脚本。负责设置起终点与参数，调用路径规划模块生成 Dubins 路径，并调用路径跟随模块进行车辆运动仿真与可视化。

### `+data/`

- `+data/parse_D_list.m`  
  对 Dubins 规划输出的 `D`（1x22）向量进行字段解析，拆分为起终点、圆心、切点、角度、转向类型等命名变量，供绘图和跟随模块使用。

### `+path_planning/`

- `+path_planning/calc_circle_centre.m`  
  根据位姿与最小转弯半径计算左/右转圆心坐标，是 Dubins 组合构造的基础几何函数。

- `+path_planning/CSC.m`  
  计算 CSC 类型路径（如 LSL、RSR、LSR、RSL）中与切线相关的几何量，生成候选路径参数。

- `+path_planning/dubins_path_planning.m`  
  路径规划主函数。综合候选 Dubins 路径，比较总长度并输出最优路径描述（`D_list`）。

- `+path_planning/draw_dubins_path.m`  
  根据 `D_list` 绘制 Dubins 路径（圆弧段与直线段），用于结果可视化和验证。

### `+path_following/`

- `+path_following/carrot_chasing.m`  
  路径跟随主函数。按“起始圆弧 -> 切线 -> 终止圆弧”的子段状态机执行 carrot-chasing 跟随，计算期望航向并更新车辆运动状态。

## 典型流程

1. `main.m` 设置场景与参数。
2. 调用 `+path_planning/` 生成并绘制 Dubins 路径。
3. 调用 `+path_following/carrot_chasing.m` 执行轨迹跟随仿真。
4. `+data/parse_D_list.m` 在规划与跟随阶段提供统一数据解析接口。

