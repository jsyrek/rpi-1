# Laser Pointer Manual Localization Configuration

## Problem Description

Your robot has a laser pointer mounted with following offset from robot center:
- **-30 cm (backward/rear)** along X-axis
- **-30 cm (left)** along Y-axis

When you place the robot on a table such that the laser pointer beam points at a specific location (e.g., bottom-left corner = map origin 0,0), the robot's **actual center** is offset by exactly the mounting offset.

**Example**: If laser points to (0,0), robot center is at (0.30, 0.30)

## Solution Architecture

### 1. Coordinate Transform (TF2)

```
map (global frame)
├─ base_link (robot center) ← This is where robot thinks it is
│  ├─ laser_pointer_link (laser physical location)
│  ├─ lidar_link (LiDAR sensor)
│  └─ wheels
└─ odom (odometry frame)
   └─ base_link
```

The URDF file defines static transformation:
```xml
<joint name="base_link_to_laser_pointer" type="fixed">
  <parent link="base_link"/>
  <child link="laser_pointer_link"/>
  <origin xyz="-0.30 -0.30 0" rpy="0 0 0"/>
</joint>
```

### 2. AMCL Localization

AMCL (Adaptive Monte Carlo Localization) is used to:
1. Accept manual initial pose from user
2. Track robot position using LiDAR scan matching
3. Fuse odometry with LiDAR measurements

### 3. Initial Pose Setting

Two methods to set initial pose:

#### Method A: RViz Interactive Marker (Easiest)
1. In RViz, click "2D Pose Estimate" button
2. Click on map where robot center should be
3. Drag to set orientation

#### Method B: ROS Service Call
```bash
ros2 service call /amcl_pose_service geometry_msgs/PoseWithCovarianceStamped \
  "pose: {pose: {position: {x: 0.30, y: 0.30, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

#### Method C: Initial Pose Setter Node
Automatically publishes initial pose on startup:
```bash
ros2 launch mks_motor_control nav2_with_amcl_manual_init.launch.py \
  map_yaml_file:=/path/to/map.yaml \
  initial_pose_x:=0.30 \
  initial_pose_y:=0.30 \
  initial_pose_theta:=0.0
```

## Implementation Steps

### Step 1: Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select mks_motor_control
source install/setup.bash
```

### Step 2: Create/Get a Map

You need a pre-created map in ROS format:

```bash
# Method A: Create using SLAM while teleoperating robot
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false

# Method B: If you already have a map.png file
ros2 param set /map_server map_name /path/to/map.yaml
```

Map file structure (map.yaml):
```yaml
image: map.pgm              # PGM or PNG file of map
resolution: 0.05            # Meters per pixel (adjust to your resolution)
origin: [-10.0, -10.0, 0.0] # Bottom-left corner position in world frame
negate: 0
occ_th: 0.65
free_th: 0.196
```

### Step 3: Launch Navigation with AMCL

```bash
# Terminal 1: Robot (on Raspberry Pi)
ros2 launch mks_motor_control nav2_with_amcl_manual_init.launch.py \
  map_yaml_file:=/home/pi/maps/map.yaml \
  initial_pose_x:=0.30 \
  initial_pose_y:=0.30

# Terminal 2: RViz (on remote computer)
ros2 launch nav2_bringup rviz_launch.py
```

### Step 4: Set Initial Pose in RViz

1. In RViz, click the "2D Pose Estimate" button (toolbar, third button)
2. Click on the map where the robot's **center** should be
3. Drag to set the robot's heading/orientation
4. Release the mouse

**IMPORTANT**: Remember the offset!
- If your laser pointer is at corner (0,0), click at (0.30, 0.30) in RViz

### Step 5: Send Navigation Goal

1. Click "Nav2 Goal" button in RViz
2. Click on destination on map
3. Drag to set goal orientation
4. Robot will navigate autonomously

## File Structure

New/Modified files in this feature branch:

```
src/mks_motor_control/
├── mks_motor_control/
│   ├── URDF/
│   │   └── two_wheel_robot.urdf          [NEW] - Robot model with laser offset
│   ├── config/
│   │   ├── amcl_params.yaml              [NEW] - AMCL localization parameters
│   │   └── nav2_params.yaml              [MODIFIED] - Updated if needed
│   └── initial_pose_setter.py            [NEW] - Node for setting initial pose
├── launch/
│   └── nav2_with_amcl_manual_init.launch.py [NEW] - Complete nav2 launch
└── setup.py                               [MODIFIED] - Added entry point
```

## Key Parameters to Adjust

### Laser Pointer Offset
In `two_wheel_robot.urdf`:
```xml
<joint name="base_link_to_laser_pointer" type="fixed">
  <origin xyz="-0.30 -0.30 0" rpy="0 0 0"/>
  <!-- ^^ Change these values if offset is different -->
</joint>
```

### AMCL Configuration
In `amcl_params.yaml`:
- `max_particles`: Increase if localization unstable (default 2000)
- `min_particles`: Decrease for faster computation (default 500)
- `update_min_d`: Distance moved before update (default 0.25m)
- `update_min_a`: Angle rotated before update (default 0.2 rad)
- `laser_model_type`: "likelihood_field_prob" for best results

### Initial Pose Uncertainty
In `initial_pose_setter.py` or launch file:
```yaml
initial_cov_xx: 0.25      # X position variance (m²)
initial_cov_yy: 0.25      # Y position variance (m²)
initial_cov_aa: 0.068     # Rotation variance (rad²)
```

## Troubleshooting

### Issue: Robot position drifts in RViz
**Cause**: AMCL not matching laser scans properly
**Solution**:
- Ensure map accuracy and matches physical environment
- Check LiDAR is properly publishing `/scan` topic
- Increase `max_particles` and reduce `update_min_d`
- Move robot around to help AMCL converge

### Issue: Orientation is wrong (robot rotated 180°)
**Cause**: Laser pointer reference frame mismatch
**Solution**:
- When setting initial pose in RViz, rotate indicator opposite direction
- Or adjust URDF orientation offset by adding 180° rotation

### Issue: "No map received" error
**Cause**: Map server not running or incorrect path
**Solution**:
```bash
# Check if map_server is running
ros2 node list | grep map

# Verify map file exists
ls -la /path/to/map.yaml

# Check ROS parameters
ros2 param get /map_server map_name
```

### Issue: AMCL not converging
**Cause**: Initial pose too far from actual position
**Solution**:
- Reduce initial covariance values (smaller uncertainty = tighter belief)
- Increase particles: `max_particles: 3000`
- Reduce update threshold: `update_min_d: 0.1`

## Monitoring and Debugging

### Check Transforms
```bash
# See all active transforms
ros2 run tf2_tools view_frames.py
# Opens frames.pdf showing transform tree

# Check specific transform
ros2 run tf2_ros tf_echo map base_link
ros2 run tf2_ros tf_echo base_link laser_pointer_link
```

### View Robot Pose
```bash
# AMCL pose estimate
ros2 topic echo /amcl_pose

# Odometry from motor driver
ros2 topic echo /odom

# Combined map pose
ros2 topic echo /tf | grep -A 5 'map'
```

### LiDAR Diagnostic
```bash
# Check LiDAR is publishing
ros2 topic echo /scan --once

# View scan in RViz:
# - Add LaserScan display
# - Set topic to `/scan`
# - Set frame to `map` or `base_link`
```

## Performance Tuning

For better localization on Raspberry Pi:

1. **Reduce computational load**:
   ```yaml
   max_particles: 1000        # Fewer particles
   update_min_d: 0.5          # Update less frequently
   ```

2. **Increase robustness**:
   ```yaml
   laser_model_type: "likelihood_field"  # Faster computation
   beam_skip_distance: 1.0    # Skip distant beams
   ```

3. **Manual loop closure** (optional):
   - Use RViz "2D Pose Estimate" to correct drift
   - AMCL will update position

## Integration with Nav2

The navigation stack includes:
- **Planner**: Navfn (fast grid-based planning)
- **Controller**: Regulated Pure Pursuit (smooth trajectory following)
- **Recovery**: Spin, Backup, Drive on Heading behaviors
- **Costmap**: Dynamic obstacle detection using LiDAR

These work together to:
1. Plan path from current pose to goal
2. Execute planned path while avoiding obstacles
3. Recover if stuck (spin, back up)

## Next Steps

1. ✅ Build package with new files
2. ✅ Create/obtain map in ROS format
3. ✅ Launch nav2 with AMCL
4. ✅ Set initial pose (laser pointer location)
5. ✅ Send navigation goals
6. 🔄 Tune parameters based on real-world performance
7. 🔄 Implement loop closure for long-term operation

## References

- [Nav2 Documentation](https://navigation.ros.org/)
- [AMCL Package](http://wiki.ros.org/amcl)
- [TF2 Introduction](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [ROS2 RViz](https://github.com/ros2/rviz)
