# points

Interactive ROS 2 node for recording navigation points and checking LiDAR localization repeatability.

Repository map layout:

```bash
~/lidar/lio/maps/map/      # PCD maps
~/lidar/lio/maps/points/   # navigation points and point check files
```

Default input topic:

```bash
/Odometry
```

Run:

```bash
ros2 run points points
```

Keys:

- `r`: record the current localization pose as a reference point
- `c`, space, or enter: compare the current pose with the next reference point
- `1` to `9`: compare the current pose with a specific reference point
- `u`: undo the last recorded reference point
- `p`: print status and current error summary
- `s`: save navigation points and check results
- `q`: save and quit

By default, navigation points are saved to:

```bash
~/lidar/lio/maps/points/navigation_points.json
```

Point check results are saved to:

```bash
~/lidar/lio/maps/points/point_checks.json
```

The point file is JSON and uses this navigation-facing shape:

```json
{
  "schema": "lidar.navigation.points.v1",
  "kind": "navigation_points",
  "version": 1,
  "points": [
    {
      "id": 1,
      "name": "point_001",
      "frame_id": "camera_init",
      "position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
      "yaw_rad": 0.0
    }
  ]
}
```

Useful parameters:

```bash
ros2 run points points --ros-args -p odom_topic:=/Odometry
ros2 run points points --ros-args -p mode:=record
ros2 run points points --ros-args -p mode:=check
ros2 run points points --ros-args -p points_file:=/home/linkchen/lidar/lio/maps/points/navigation_points.json
ros2 run points points --ros-args -p checks_file:=/home/linkchen/lidar/lio/maps/points/point_checks.json
```

Create a report table after checking points:

```bash
cd ~/lidar/navigation
python3 tools/points_report.py
python3 tools/points_report.py --points 2,3,4
python3 tools/points_report.py --points 2,3,4 --format markdown --output ~/lidar/navigation/report.md
python3 tools/points_report.py --format csv --output ~/lidar/navigation/report.csv
```
