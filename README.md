# RTK-Navigation-System

`RTK-Navigation-System` is a C++17 simulation project for turning surveying / RTK / GNSS-style measured geographic data into a local navigation map for a robot or autonomous vehicle.

The pipeline is:

```text
RTK / GNSS measured data
        -> Coordinate transformation
        -> Local map generation
        -> Occupancy grid map
        -> A* path planning
        -> Vehicle navigation visualization
```

This is not a simple path-planning-only demo. The project starts from WGS84 latitude, longitude, and height, converts the data into a local ENU frame, builds a grid map, plans a route, and visualizes vehicle motion.

## Dependencies

- C++17 compiler
- CMake 3.16+
- OpenCV
- Eigen

On macOS with Homebrew:

```bash
brew install cmake opencv eigen
```

## Build

```bash
cmake -S . -B build
cmake --build build
```

## Run

Open the animated OpenCV window:

```bash
./build/RTK-Navigation-System
```

Run without a GUI and save a snapshot:

```bash
./build/RTK-Navigation-System --no-gui
```

Use a custom CSV file:

```bash
./build/RTK-Navigation-System --csv data/rtk_points.csv --output output/navigation_result.png
```

## CSV Format

The default input file is `data/rtk_points.csv`.

```csv
id,latitude,longitude,height,type
start,31.2304000,121.4737000,8.0,start
road_00,31.2304000,121.4737000,8.0,road
goal,31.2304719,121.4739521,8.0,goal
```

Supported point types:

- `road`: measured drivable or navigable road point
- `obstacle`: occupied obstacle point
- `boundary`: survey boundary point
- `start`: navigation start
- `goal`: navigation target

## Project Structure

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
  navigation/
    Navigator.h
    Navigator.cpp
  visualization/
    Viewer.h
    Viewer.cpp
  main.cpp
```

## Implementation Steps

1. RTK data loading
   - `RTKReader` opens `data/rtk_points.csv`.
   - It skips the header, parses five columns, validates numeric values, and validates the point type.
   - The result is a vector of `RTKPoint` objects containing geographic coordinates.

2. Coordinate conversion
   - `CoordinateTransformer` uses WGS84 ellipsoid constants.
   - It converts latitude, longitude, and height to ECEF.
   - It then rotates ECEF differences into a local ENU coordinate frame using Eigen.
   - The `start` point is the default ENU origin.

3. Map generation
   - `MapBuilder` converts local XY points into an occupancy grid.
   - The default resolution is `0.5 m/cell`.
   - Road points become free corridors.
   - Obstacle points become occupied disks.
   - Boundary points become occupied boundary lines.
   - Start and goal cells are forced free.

4. A* path planning
   - `AStar` searches the occupancy grid using 8-connected neighbors.
   - Diagonal corner cutting is prevented by checking the side cells.
   - The output is a list of `GridCell` path cells from start to goal.

5. Vehicle navigation simulation
   - `Navigator` converts the path into a simple vehicle trajectory.
   - It uses a lightweight Pure Pursuit-style lookahead target.
   - The simulated state contains `x`, `y`, `yaw`, and velocity.

6. Visualization
   - `Viewer` uses OpenCV to render the occupancy grid, RTK points, obstacles, boundaries, planned path, trajectory, and vehicle body.
   - By default it opens an animation window.
   - With `--no-gui`, it saves `output/navigation_result.png`.

## Notes

- This first version is a simulation and visualization system.
- It does not connect to live GNSS hardware.
- It does not use ROS.
- DWA is not included in v1; the navigation module uses Pure Pursuit-style path tracking.
