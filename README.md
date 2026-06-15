# RTK-Navigation-System

`RTK-Navigation-System` 是一个基于 C++17 的机器人导航系统项目，支持 RTK / GNSS、CASS DAT 和 DXF 地图输入。

项目将测量得到的地理坐标或工程地图转换为机器人可使用的局部占据栅格，使用 A* 生成全局参考路径，再由 DWA 局部规划器根据机器人运动学、速度约束和障碍物净空实时选择短时轨迹。

## 项目流程

```text
RTK / GNSS 测量数据
        -> 坐标转换
        -> 局部地图生成
        -> 占据栅格地图
        -> A* 全局路径规划
        -> DWA 局部轨迹规划
        -> 机器人导航可视化
```

项目从 WGS84 经纬高、CASS 平面坐标或 DXF 地物数据开始，生成 Occupancy Grid Map，通过 OpenCV 显示 A* 全局路径、DWA 局部预测轨迹和机器人实际运动轨迹。

## 效果展示

### WGS84 示例数据导航

程序读取带有道路、障碍物、边界、起点和终点标签的示例 RTK 数据，完成 ENU 坐标转换、栅格建图、A* 全局规划和 DWA 局部导航。

![WGS84 DWA 机器人导航结果](docs/images/dwa_navigation_result.png)

图中：

- 橙黄色折线：A* 全局参考路径
- 青色曲线：DWA 连续重规划后形成的机器人实际轨迹
- 淡紫色曲线：当前动态窗口内采样的 DWA 候选轨迹
- 绿色粗线：候选轨迹中得分最高的最优局部轨迹
- 三角形：机器人当前位置和朝向

DWA 不会机械地沿 A* 栅格折线移动，而是在保持安全净空的同时生成满足速度和角速度约束的平滑轨迹。淡紫色候选中可能包含碰撞轨迹，它们仅用于展示采样空间，规划时会被剔除。

效果截图保存在：

```text
docs/images/dwa_navigation_result.png
```

### 华测/CASS 实测数据导航

程序读取华测 RTK 导出的 CASS DAT 平面坐标，排除远处基站记录，并基于测区地物轮廓生成近似占据地图。图中深色区域为建筑、水域和绿化障碍，黄色为 A* 路径，青色为机器人运动轨迹。

![华测 CASS 实测数据导航结果](docs/images/cass_navigation_result.png)

### 通用 DXF 导航

程序读取 ASCII DXF 中的图层和几何实体，根据 YAML 图层规则识别建筑、道路、水域、植被和围墙，再自动生成占据地图和导航路径。当前效果图启用了道路约束模式：只有道路区域可通行，机器人不会从普通空地穿过。

![通用 DXF 导航结果](docs/images/dxf_navigation_result.png)

## 技术栈

- C++17
- CMake
- OpenCV
- Eigen



## 构建项目

```bash
cmake -S . -B build
cmake --build build
```

## 运行项目

打开 OpenCV 动画窗口：

```bash
./build/RTK-Navigation-System
```

如果当前环境看不到 OpenCV 窗口，可以使用无窗口模式，只保存结果图：

```bash
./build/RTK-Navigation-System --no-gui
```

所有输入模式都会自动执行：

```text
GridMap -> A* 全局路径 -> DWA 局部规划 -> OpenCV 可视化
```

DWA 默认参数定义在 `src/planner/DWAPlanner.h`，包括最大速度、最大角速度、加速度、预测时域、机器人半径、采样分辨率和四项评价权重。无需增加命令行参数即可运行。

生成包含 A*、DWA 候选轨迹和最优局部轨迹的截图：

```bash
./build/RTK-Navigation-System \
  --no-gui \
  --output output/dwa_navigation_result.png
```

默认输出图片：

```text
output/navigation_result.png
```

指定自定义 CSV 输入和输出图片：

```bash
./build/RTK-Navigation-System --csv data/rtk_points.csv --output output/navigation_result.png
```

### 运行华测/CASS DAT 实测数据

项目支持直接读取华测 RTK 导出的 CASS 平面坐标 DAT：

```bash
./build/RTK-Navigation-System \
  --cass 海资.dat \
  --start-id I51 \
  --goal-id I73 \
  --no-gui
```

默认输出：

```text
output/cass_navigation_result.png
```

DAT 格式：

```text
点号,编码,平面坐标1,平面坐标2,高程
I51,,394058.561,3418949.061,18.236
```

真实数据模式会：

- 自动排除距离测区过远的基站记录
- 直接使用已有平面坐标，不再执行 WGS84 到 ENU 转换
- 将测区最小坐标作为局部地图原点
- 根据指定点号设置导航起点和终点
- 生成占据栅格、A* 路径和 DWA 机器人轨迹

CASS 模式会为 A* 启用障碍物净空代价，避免全局参考路径紧贴大型建筑边缘，为 DWA 的机器人足迹和制动距离留出空间。

当前对 `海资.dat` 的试验性分类规则：

- `B` 点组：海资楼建筑轮廓，填充为障碍物
- `R` 点组：水域轮廓，填充为障碍物
- `F` 点组：左侧绿化或小型构筑物，按测点附近障碍处理
- 其他点组：作为测量参考点显示，不直接判断为障碍物

这套分类依据现有 CASS 地形图人工确认，属于近似地图。后续如果提供 DXF 图层，可以替换为按 CASS 图层自动识别地物。

### 通用 DXF 导航架构

项目已经加入与具体测区无关的 DXF 地图处理层：

```text
DXF geometry
    -> MapFeature
    -> LayerConfig
    -> feature classification
    -> FeatureMapBuilder
    -> Occupancy Grid
    -> A*
    -> DWA
    -> robot navigation
```

核心模块：

- `MapFeature`：统一表示点、折线和闭合多边形
- `LayerConfig`：通过 YAML 配置图层语义和占据规则
- `FeatureMapBuilder`：将通用地物生成占据栅格
- `DxfReader`：读取 ASCII DXF 几何和图层

图层配置文件：

```text
config/dxf_layers.yaml
```

配置示例：

```yaml
map:
  resolution: 0.25
  padding: 5.0
  default_state: "occupied"
  endpoint_clearance: 0.5
  require_endpoints_on_free: 1
  clearance_cost_radius: 2.0
  clearance_cost_weight: 2.5

rules:
  - layers: [ "BUILDING", "JZW", "房屋", "建筑" ]
    match: "exact"
    semantic: "building"
    occupancy: "occupied"
    force_closed: 1
    inflation: 1.5
    line_width: 0.3
```

道路约束配置的含义：

- `default_state: occupied`：地图默认不可通行
- `road + occupancy: free`：只将道路图层释放为自由区域
- `resolution: 0.25`：DXF 道路按 0.25 米栅格生成，使斜线和曲线边缘更平滑
- `line_width`：仅用于开放道路中心线，对应实际道路宽度
- 闭合道路多段线：严格按照多边形边界填充为可通行道路面，不再使用 `line_width` 向外扩张
- `require_endpoints_on_free`：起点和终点必须位于道路内，否则拒绝规划
- `clearance_cost_radius/weight`：对靠近道路边缘的栅格增加代价，使路径倾向道路中部

道路中心线规则示例：

```yaml
- layers: [ "ROAD", "ROADS", "DL", "道路", "车行道" ]
  match: "exact"
  semantic: "road"
  occupancy: "free"
  force_closed: 0
  inflation: 0.0
  line_width: 6.0
```

对于只有道路中心线的 DXF，`line_width` 应设置为真实路宽。对于包含闭合道路边界的 DXF，应将每个道路面绘制为闭合 `LWPOLYLINE` 或闭合 `POLYLINE` 并放入道路图层。多个道路多边形可以相交或重叠，程序会将它们合并为连续可通行区域，例如十字路口或 T 形路口。

未来通用运行接口已经固定为：

```bash
./build/RTK-Navigation-System \
  --dxf site.dxf \
  --layer-config config/dxf_layers.yaml \
  --start-x 100 \
  --start-y 50 \
  --goal-x 300 \
  --goal-y 200 \
  --no-gui
```

当前内置解析器支持：

- `LINE`
- `POINT`
- `LWPOLYLINE`
- 2D/3D `POLYLINE`、`VERTEX`、`SEQEND`
- 闭合多段线
- LWPOLYLINE/POLYLINE bulge 圆弧展开
- 实体图层名称和三维顶点坐标

当前限制：

- 仅支持 ASCII DXF，Binary DXF 需要先导出为 ASCII DXF
- 尚未展开 `INSERT` 块引用
- 尚未处理 `SPLINE`、`ELLIPSE`、`HATCH` 等复杂实体
- 暂不处理非默认 OCS/挤出方向

建议从 CASS/AutoCAD 导出 `AutoCAD R12 ASCII DXF` 或 `AutoCAD 2007 ASCII DXF`。后续如需完整 Binary DXF 和复杂实体支持，可将 `DxfReader` 后端替换为
[libdxfrw](https://github.com/LibreCAD/libdxfrw)，地图、规划和导航模块无需修改。

通用图层分类和栅格构建已经通过独立测试验证，因此后续接入解析库时只需要让 `DxfReader` 输出 `MapFeature`，A*、导航和可视化模块无需修改。

## RTK 数据格式

默认输入文件为：

```text
data/rtk_points.csv
```

CSV 格式如下：

```csv
id,latitude,longitude,height,type
start,31.2304000,121.4737000,8.0,start
road_00,31.2304000,121.4737000,8.0,road
goal,31.2304719,121.4739521,8.0,goal
```

字段含义：

- `id`：测量点编号
- `latitude`：WGS84 纬度
- `longitude`：WGS84 经度
- `height`：高程
- `type`：点类型

支持的点类型：

- `road`：道路或可通行区域测量点
- `obstacle`：障碍物点
- `boundary`：边界点
- `start`：导航起点
- `goal`：导航目标点

## 项目结构

```text
src/
  common/
    Types.h
  sensor/
    RTKReader.h
    RTKReader.cpp
  coordinate/
    CoordinateTransformer.h
    CoordinateTransformer.cpp
  map/
    GridMap.h
    GridMap.cpp
    MapBuilder.h
    MapBuilder.cpp
  planner/
    AStar.h
    AStar.cpp
    DWAPlanner.h
    DWAPlanner.cpp
  navigation/
    Navigator.h
    Navigator.cpp
  visualization/
    Viewer.h
    Viewer.cpp
  main.cpp
```

`main.cpp` 只负责组织整体流程，具体功能分别放在独立模块中。

## 模块说明

### 1. RTK 数据读取

`RTKReader` 负责读取 `data/rtk_points.csv`。

主要功能：

- 打开 CSV 文件
- 跳过表头
- 解析 `id, latitude, longitude, height, type`
- 校验字段数量、数值格式和点类型
- 将测量点保存为 `RTKPoint`

### 2. 坐标转换

`CoordinateTransformer` 负责将 WGS84 经纬高转换为本地 ENU 坐标。

转换流程：

```text
latitude / longitude / height
        -> ECEF
        -> ENU
        -> x / y / z
```

实现细节：

- 使用 WGS84 椭球参数
- 使用 `start` 点作为 ENU 坐标原点
- 使用 Eigen 完成矩阵旋转计算
- 输出单位为米

### 3. 地图生成

`MapBuilder` 和 `GridMap` 负责生成占据栅格地图。

默认参数：

- 栅格分辨率：`0.5 m/cell`
- 地图边距：`5 m`
- 道路点扩展为可通行走廊
- 障碍物点扩展为占据区域
- 边界点连接为占据边界
- 起点和终点强制设为可通行

### 4. A* 路径规划

`AStar` 在占据栅格地图上进行全局路径规划。

特点：

- 使用 8 邻域搜索
- 支持横向、纵向和斜向移动
- 通过检查相邻侧边格子避免斜向穿越障碍角点
- 输出从起点到终点的 `GridCell` 路径

### 5. DWA 局部规划

`DWAPlanner` 以 A* 路径为全局参考，在每个控制周期内完成以下步骤：

规划接口显式接收：

- 当前机器人状态：`x, y, yaw, v, w`
- 目标点：`goal_x, goal_y`
- A* 全局路径
- `GridMap` 占据栅格

1. 根据当前速度、角速度和加速度约束计算动态窗口。
2. 在窗口内采样线速度 `v` 和角速度 `w`。
3. 使用独轮车运动学模型预测短时轨迹：

```text
x(k+1)   = x(k) + v * cos(yaw) * dt
y(k+1)   = y(k) + v * sin(yaw) * dt
yaw(k+1) = yaw(k) + w * dt
```

4. 从 `GridMap` 查询附近占用栅格，计算机器人圆形足迹到障碍栅格边界的净空。
5. 标记发生碰撞或净空小于制动距离的候选轨迹。
6. 对安全轨迹计算归一化加权得分并选择最高分轨迹：

```text
Score = heading_weight  * 目标方向得分
      + path_weight     * A* 路径跟随得分
      + obstacle_weight * 障碍物净空得分
      + speed_weight    * 速度得分

best_trajectory = argmax(Score)
```

规划器每轮只执行选中轨迹的第一个控制步，然后基于新状态重新规划。`DWAPlanResult` 输出所有采样候选的 `v、w、score、clearance、collision_free` 以及最优局部轨迹，`DWANavigationResult` 保存机器人实际轨迹、每轮候选集合和每轮最优轨迹。

默认机器人参数：

- 最大线速度：`1.2 m/s`
- 最大角速度：`1.2 rad/s`
- 预测时域：`2.0 s`
- 控制周期：`0.1 s`
- 机器人半径：`0.2 m`
- 目标容差：`0.4 m`

### 6. 兼容导航仿真

`Navigator` 将 A* 路径转换为车辆运动轨迹。

该模块保留原有轻量 Pure Pursuit 风格控制器接口，避免破坏已有代码；当前主流程已经切换为 DWA：

- 固定前视距离
- 固定速度
- 根据路径目标点更新车辆航向
- 输出车辆状态序列：`x, y, yaw, velocity`

### 7. 可视化

`Viewer` 使用 OpenCV 显示导航结果。

显示内容：

- RTK 测量点
- 道路区域
- 障碍物
- 边界
- A* 全局规划路径
- DWA 采样候选轨迹
- DWA 得分最高的最优局部轨迹
- 机器人实际运动轨迹
- 当前机器人姿态

## 简历描述

可直接用于简历的项目描述：

> 基于 C++17、OpenCV 和占据栅格地图实现机器人导航系统：使用 A* 生成全局路径，设计 DWA 局部规划器在动态窗口内采样线速度与角速度，基于差速运动学预测候选轨迹，并融合目标方向、路径偏差、障碍物净空和速度偏好进行评分；实现机器人圆形足迹碰撞检测、制动距离约束及候选/最优轨迹可视化，支持 RTK/GNSS、CASS DAT 和 DXF 地图输入。

## 示例运行输出

使用默认示例数据运行后，控制台会输出类似信息：

```text
Loaded RTK points: 17
ENU origin: start lat=31.2304 lon=121.474 h=8
Grid map: 82 x 49 cells, resolution=0.5 m/cell
Start cell: row=15 col=16
Goal cell: row=31 col=64
A* path cells: 49
DWA local plans: 326
DWA trajectory states: 327
DWA reached goal: yes
DWA trajectory collision states: 0
Saved visualization snapshot: output/navigation_result.png
```

## 当前版本说明

- 当前版本是仿真与可视化系统。
- 当前版本不连接真实 RTK / GNSS 硬件。
- 当前版本不依赖 ROS。
- 当前主流程使用 A* 全局规划和 DWA 局部规划。
- DWA 支持 `v/w` 动态窗口采样、短时运动学预测、机器人圆形足迹碰撞检测和制动距离检查。
- 原有 Pure Pursuit 风格 `Navigator` 仍保留，便于兼容和对比。
- 当前版本可以读取华测/CASS 平面坐标 DAT，但地物分类仍采用针对样例测区的人工规则。
- 已支持通用 ASCII DXF 图层解析、地物分类、栅格建图和导航；复杂实体与 Binary DXF 可在后续接入 `libdxfrw`。
- DXF 模式支持道路约束建图、起终点道路校验和道路边缘代价。

## 后续可扩展方向

- 读取真实 RTK 采集数据
- 支持更多坐标系统和投影方式
- 增加地图滤波与点云预处理
- 增加动态障碍物预测和速度障碍模型
- 加入 MPC 局部规划器并与 DWA 对比
- 接入 ROS 2
- 增加实时 GNSS 数据输入
- 增加差速、阿克曼等可配置机器人模型
