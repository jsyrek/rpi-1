# Laser Pointer Localization - Quick Start

## TL;DR - 5 Minute Setup

### Your Setup
- **Laser pointer offset**: 30cm backward (rear), 30cm left
- **Robot platform**: Diff-drive with encoders, ROS2, Nav2
- **LiDAR**: Unitree L2
- **Goal**: Manual initial pose setting using laser pointer reference

### What This Does
1. Adds laser pointer reference point to robot model (URDF)
2. Enables AMCL-based localization with manual pose initialization
3. Allows you to point laser at any location and tell robot "you are here"
4. Robot then navigates autonomously using LiDAR scan matching

---

## Quick Start Steps

### Step 1: Merge Branch
```bash
cd ~/ros2_ws/src/rpi-1
git fetch origin feature/laser-pointer-localization
git merge feature/laser-pointer-localization
cd ~/ros2_ws
colcon build --packages-select mks_motor_control
source install/setup.bash
```

### Step 2: Prepare Map

You need a pre-generated map. Two options:

**Option A: Already have map files**
```bash
# Verify map exists
ls -la /path/to/your/map.yaml
ls -la /path/to/your/map.pgm
```

**Option B: Create map now** (takes ~5 minutes)
```bash
# Terminal 1 - Robot (on RPi)
ros2 launch mks_motor_control nav2_bringup.launch.py  # Use old launch, no AMCL

# Terminal 2 - Workstation
ros2 launch slam_toolbox online_async_launch.py

# Manually drive robot around to map area, then
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: /home/pi/maps/my_table"
```

### Step 3: Launch Navigation

**On Raspberry Pi:**
```bash
ros2 launch mks_motor_control nav2_with_amcl_manual_init.launch.py \
  map_yaml_file:=/home/pi/maps/my_table.yaml \
  initial_pose_x:=0.30 \
  initial_pose_y:=0.30 \
  initial_pose_theta:=0.0
```

**On Workstation:**
```bash
ros2 launch nav2_bringup rviz_launch.py
```

### Step 4: Set Pose Using Laser Pointer

1. **Physical Setup**:
   - Place robot on table
   - Point laser at any reference point (e.g., corner)
   - Note the coordinates where laser points

2. **In RViz**:
   - Click "2D Pose Estimate" button (toolbar)
   - Click on map at the position **offset by your laser offset**
     - If laser points at (0, 0) on table
     - Click at (0.30, 0.30) in RViz
   - Drag to set robot heading
   - Release

3. **Verify**:
   - Green robot arrow should match physical robot orientation
   - Position should match with offset applied

### Step 5: Send Goals

1. Click "Nav2 Goal" button in RViz
2. Click destination on map
3. Drag to set goal heading
4. Robot navigates automatically!

---

## Key Files Added/Modified

| File | Purpose |
|------|----------|
| `mks_motor_control/URDF/two_wheel_robot.urdf` | Robot model with laser offset |
| `mks_motor_control/config/amcl_params.yaml` | AMCL localization tuning |
| `mks_motor_control/initial_pose_setter.py` | Auto-publish initial pose |
| `launch/nav2_with_amcl_manual_init.launch.py` | Complete navigation stack |

---

## Common Adjustments

### Your Laser Offset is Different

If your laser pointer has a different offset, edit `two_wheel_robot.urdf`:
```xml
<!-- Find this section:
<joint name="base_link_to_laser_pointer" type="fixed">
  <parent link="base_link"/>
  <child link="laser_pointer_link"/>
  <origin xyz="-0.30 -0.30 0" rpy="0 0 0"/>
                     ^^^^  ^^^^ <- Change these
</joint>
```

Change values:
- First number: **forward/backward** (negative = backward)
- Second number: **left/right** (negative = left)
- Third number: **up/down** (usually 0)

Example: If laser is 20cm forward and 10cm right:
```xml
<origin xyz="0.20 0.10 0" rpy="0 0 0"/>
```

### Localization is Drifting

Increase particles in launch command:
```bash
ros2 param set /amcl max_particles 3000
```

Or edit `amcl_params.yaml` before launch.

### Robot Rotated 180°

When setting pose:
- Rotate the indicator in opposite direction
- Or add 180° to your laser's orientation

---

## Monitoring

### Check Robot Knows Where It Is
```bash
# Print current estimated pose
ros2 topic echo /amcl_pose

# Should show x, y, theta matching your setup
```

### Monitor LiDAR
```bash
# Check LiDAR is scanning
ros2 topic echo /scan --once

# In RViz: Add "LaserScan" display, set topic to /scan
```

### Check Transforms
```bash
# View robot transforms
ros2 run tf2_ros tf_echo map base_link
```

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| "No map received" | Map file not found | Check file path exists |
| Robot position wrong | Initial pose set incorrectly | Remember: laser + offset = robot center |
| Navigation fails | AMCL not confident in position | Move robot around to let it localize |
| Orientation flipped | Laser offset sign wrong | Check URDF origin xyz values |
| Localization drifts | Not enough particles | Increase max_particles to 3000 |

---

## What Happens Behind the Scenes

1. **URDF defines geometry**: Robot shape + laser pointer location
2. **Odometry publishes**: Wheel encoder data → `/odom` topic
3. **LiDAR scans**: Publishes 360° laser data → `/scan` topic
4. **AMCL runs**: Matches LiDAR scans to map, estimates position
5. **Nav2 plans**: Creates path from robot position to goal
6. **Motor driver executes**: Sends wheel commands via CAN
7. **Cycle repeats**: 10-50 Hz localization + navigation

---

## Next: Fine Tuning

For production use, see `LASER_POINTER_LOCALIZATION.md` for:
- AMCL parameter tuning
- Performance optimization for RPi4
- Advanced troubleshooting
- Transform debugging

---

## Still Stuck?

Check these:
1. ✅ All nodes running: `ros2 node list`
2. ✅ RViz can connect to robot
3. ✅ Map file loads without error
4. ✅ `/scan` topic has data: `ros2 topic echo /scan --once`
5. ✅ Initial pose service exists: `ros2 service list | grep amcl`

Then run:
```bash
ros2 param get /amcl max_particles
ros2 topic echo /amcl_pose --once
ros2 run tf2_tools view_frames.py  # Opens frames.pdf
```
