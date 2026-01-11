# Diagnostic Tests: Message Filter "Queue is Full" Error

## Status: Current Situation

From output analysis:
- ✅ `scan_throttle` node IS running and publishing to `/scan_throttled` at 5 Hz
- ✅ Config file shows `scan_topic: /scan_throttled` (matches scan_throttle output)
- ❌ Error STILL occurs: "Message Filter dropping message: frame 'unilidar_lidar' at time ... queue is full"
- ❌ Collision monitor error: "parameter 'polygons' is not initialized" (separate issue)

## Diagnostic Test Plan

### Test 4: Verify Topic Subscriptions (CRITICAL)
**Purpose**: Confirm which topic SLAM Toolbox is actually subscribing to and verify the topic chain

**Commands** (run while launch is active in another terminal):
```bash
# Terminal 2: Check which topics exist and their subscribers
ros2 topic info /scan
ros2 topic info /scan_throttled

# Check publication rates
ros2 topic hz /scan
ros2 topic hz /scan_throttled
```

**Expected Results**:
- `/scan` should have publisher: `pointcloud_to_laserscan` 
- `/scan` should have subscriber: `scan_throttle` (and possibly others)
- `/scan_throttled` should have publisher: `scan_throttle`
- `/scan_throttled` should have subscriber: `slam_toolbox` (if working correctly)
- `/scan` rate: ~10-20 Hz (from pointcloud_to_laserscan)
- `/scan_throttled` rate: ~5 Hz (throttled)

**Analysis**:
- If SLAM Toolbox subscribes to `/scan` instead of `/scan_throttled`: Config mismatch
- If `/scan_throttled` has no subscribers: SLAM Toolbox not configured correctly
- If rates are wrong: Throttle node may not be working

---

### Test 5: Verify TF Transform Availability
**Purpose**: Check if TF transforms are actually available when SLAM Toolbox needs them

**Commands** (run while launch is active):
```bash
# Terminal 2: Test TF transforms
ros2 run tf2_ros tf2_echo odom base_link
# Wait 5 seconds, check if transforms are published regularly
# Press Ctrl+C after seeing several transforms

ros2 run tf2_ros tf2_echo base_link unilidar_lidar
# Should show static transform (should work immediately)

ros2 run tf2_ros tf2_echo map odom
# May not exist until SLAM initializes - this is OK if it doesn't exist yet
```

**Expected Results**:
- `odom -> base_link`: Should publish at ~50 Hz (every ~20ms)
- `base_link -> unilidar_lidar`: Should publish static transform immediately
- `map -> odom`: May not exist initially (SLAM creates this)

**Analysis**:
- If `odom -> base_link` is slow/missing: TF publishing issue (explains queue full)
- If `base_link -> unilidar_lidar` missing: Static TF publisher issue
- If transforms exist: TF is NOT the issue

---

### Test 6: Monitor Message Timestamps During Error
**Purpose**: Compare timestamps of scans vs transforms to find timing issues

**Commands** (run while launch is active):
```bash
# Terminal 2: Monitor scan timestamps (first 20 messages)
timeout 10 ros2 topic echo /scan_throttled --no-arr | grep -E "(sec:|nanosec:|frame_id:)" | head -60

# Terminal 3: Monitor odom timestamps (first 20 messages)
timeout 10 ros2 topic echo /odom --no-arr | grep -E "(sec:|nanosec:)" | head -40

# Terminal 4: Check TF tree structure
ros2 run tf2_tools view_frames
# Wait 5 seconds, then Ctrl+C
# Check the generated frames.pdf file
```

**Analysis**:
- Compare timestamps: Do scans arrive before corresponding TF transforms?
- Check frame_id: Is it consistently `unilidar_lidar`?
- Check TF tree: Is the complete chain present?

---

### Test 7: Check SLAM Toolbox Configuration Parameters
**Purpose**: Verify what parameters SLAM Toolbox is actually using

**Commands** (run while launch is active):
```bash
# Terminal 2: Check SLAM Toolbox parameters
ros2 param list /slam_toolbox | grep -E "(scan_topic|scan_buffer|transform_timeout|tf_buffer)"

ros2 param get /slam_toolbox scan_topic
ros2 param get /slam_toolbox scan_buffer_size
ros2 param get /slam_toolbox transform_timeout
ros2 param get /slam_toolbox tf_buffer_duration
```

**Expected Results**:
- `scan_topic` should be `/scan_throttled` (matching config file)
- `scan_buffer_size` should be `5000`
- `transform_timeout` should be `10.0`
- `tf_buffer_duration` should be `300.0`

**Analysis**:
- If parameters don't match config file: Config not loaded correctly
- If scan_topic is `/scan`: Mismatch with scan_throttle output
- If buffer sizes are small: Explains queue saturation

---

### Test 8: Monitor Message Filter Queue Status (If Possible)
**Purpose**: Check if we can see queue status or diagnostic info

**Commands** (run while launch is active):
```bash
# Terminal 2: Check for diagnostic topics
ros2 topic list | grep -E "(diagnostic|slam)"

# Check SLAM Toolbox node info
ros2 node info /slam_toolbox

# Monitor SLAM Toolbox feedback (if exists)
ros2 topic echo /slam_toolbox/feedback --once 2>/dev/null || echo "No feedback topic"
```

**Analysis**:
- Look for diagnostic or status topics
- Check node subscriptions to confirm topic name
- Look for queue metrics if available

---

### Test 9: Timeline Analysis - Error vs System State
**Purpose**: Correlate error timing with system events

**From launch output, note**:
1. When does scan_throttle publish first scan?
   - Output: `[scan_throttle-6] [INFO] [1768131890.988495777] [scan_throttle]: ✓ Received first scan`
   - Time: ~1768131890.988

2. When does scan_throttle publish first throttled scan?
   - Output: `[scan_throttle-6] [INFO] [1768131891.142321565] [scan_throttle]: ✓ Published first throttled scan`
   - Time: ~1768131891.142

3. When does first "Message Filter dropping" error occur?
   - Output: `[async_slam_toolbox_node-7] [INFO] [1768131891.337331634] [slam_toolbox]: Message Filter dropping message: frame 'unilidar_lidar' at time 1768131891.137`
   - Time: ~1768131891.337 (message timestamp: 1768131891.137)

**Analysis**:
- Error occurs ~195ms after first throttled scan is published
- Message timestamp (1768131891.137) is close to first scan publish time (1768131891.142)
- This suggests SLAM Toolbox receives the scan but cannot process it immediately

---

## Recommended Test Execution Order

1. **Test 4** (Topic subscriptions) - HIGHEST PRIORITY
   - Will confirm if topic mismatch is the issue
   - Quick to execute
   
2. **Test 7** (SLAM Toolbox parameters) - HIGH PRIORITY
   - Will confirm config is loaded correctly
   - Quick to execute
   
3. **Test 5** (TF transforms) - MEDIUM PRIORITY
   - Will confirm TF chain availability
   - Takes a bit longer

4. **Test 6** (Timestamp monitoring) - MEDIUM PRIORITY
   - Will show timing relationships
   - Provides detailed data

5. **Test 8** (Diagnostics) - LOW PRIORITY
   - May not provide useful info
   - Quick to check

6. **Test 9** (Timeline analysis) - ALREADY DONE
   - Already analyzed from output
   - Shows error occurs soon after scans start

---

## Expected Findings Based on Current Output

From the launch output, we already see:
1. ✅ `scan_throttle` is running and publishing to `/scan_throttled`
2. ✅ Config shows `scan_topic: /scan_throttled`
3. ❌ Error still occurs
4. ❌ Error timing: ~195ms after first throttled scan

**Hypothesis**: 
- Topic configuration seems correct (scan_throttle publishes, config expects it)
- But error still occurs, suggesting TF transform unavailability OR timing issue
- Since odometry publishes at 50 Hz, TF should be available
- BUT: Maybe the specific transform needed at scan timestamp is missing/delayed?

**Next tests should confirm**:
- Is SLAM Toolbox actually subscribed to `/scan_throttled`? (Test 4)
- Are TF transforms available when scans arrive? (Test 5)
- What are the actual SLAM Toolbox parameters? (Test 7)
