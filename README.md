# elevation_map
The cpp code for creating the elevation mapping with point cloud

## Inputs 

**cloud_in**: sensor_msgs::msg::PointCloud2 : Raw 3‑D points from camera or from depth projection.

**tf2**: geometry_msgs::msg::TransformStamped : Transform from cloud.header.frame_id → base_link(robot).

## Outputs

**elevation_map_pts**: sensor_msgs::msg::PointCloud : A dense grid of (x, y, z) where z is the fused height for that cell (–1 if unseen).

**elevation_map.txt**: One floating‑point number per cell (row‑major order).

## Key parameters

**map_center_** [m]: Default(0.5, 0.5) : Centre of the map in world_frame_id_ (base_link).

**map_widths_** [m]:	Default(1.0, 1.0) : Physical size of the map (X × Y).

**resolution_** [m]:	Default(0.005) : Cell size (both axes).

**grid_dim_** [cells]:	Default(round(width/res) + 1 → 201 × 201) : Number of cells per axis.

**sigma_meas_**: Default(0.1) :	Fixed measurement noise (used for fusion).

## Run
- Build the ros2 workspace:

`
colcon build --symlink-install
source install/setup.bash
`
- Then run with:

`
ros2 launch elevation_mapping elevation_mapping.launch.py
`

**Note**: make sure you run the robot package to publish the base_link before running the elevation_mapping.