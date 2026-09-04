# LIO-SAM Tuning Guide

This document covers tuning for LIO-SAM with:
- **IMU:** ZED2i (BMI055 MEMS) at 400 Hz
- **LiDAR:** LSLidar C32 at 10 Hz
- **GPS:** RTK (optional, currently disabled)

---

## IMU Tuning

### Root cause: "Large velocity, reset IMU-preintegration!"

`imuPreintegration.cpp` triggers a reset when the GTSAM-optimized velocity
exceeds **30 m/s**. This means the optimizer diverged — not that the robot
is actually moving fast.

The `failureDetection()` check:
```cpp
// imuPreintegration.cpp:478
if (vel.norm() > 30) {
    RCLCPP_WARN(get_logger(), "Large velocity, reset IMU-preintegration!");
    return true;
}
```

### Why it diverges

IMU noise params that are too **low** make GTSAM over-trust the IMU. When
the IMU and LiDAR disagree (always the case to some degree due to vibration,
bias drift, timing), the optimizer resolves the conflict by computing large
velocities — eventually hitting the 30 m/s threshold and resetting.

### ZED2i IMU rate

The ZED2i IMU (BMI055) runs internally at 400 Hz. The ZED ROS2 wrapper was
throttling it to 100 Hz due to `sensors_image_sync: true`. This was fixed by:

```yaml
# jetson/params/common.yaml
sensors:
    sensors_image_sync: false   # was: true  — was tying IMU rate to image rate
    sensors_pub_rate: 400.      # was: 200   — use hardware maximum
```

LIO-SAM hardcodes a fallback dt of `1/500 s`. At 100 Hz the actual dt is
`0.01 s` (5× difference for the first integration step). At 400 Hz it is
`0.0025 s`, which is much closer to the fallback and gives 40 IMU samples
per LiDAR scan instead of 10.

### Extrinsic calibration (top LiDAR mount)

The active vehicle TF captured on 2026-09-02 uses the top LiDAR mount:

```
laser_link in base_link:             xyz = (2.650000,  0.000000,  1.539000)
zed2i_left_camera_frame in base_link xyz = (3.043000,  0.060000,  0.981000)
zed2i_imu_link in left-camera frame: xyz = (-0.002000, -0.023061, 0.000217)

extrinsicTrans (laser_link -> zed2i_imu_link)
               = ( 0.391000,  0.036939, -0.557783)
```

That value is configured in `lio_sam_params.yaml`. It replaces the obsolete
`(-0.1726, 0, 0.5011)` value, which was derived for the old low LiDAR mount.

The factory transform measured a small camera-to-IMU rotation. The matching
non-identity `extrinsicRot` and `extrinsicRPY` matrices are configured in
`lio_sam_params.yaml`: the first rotates IMU acceleration/gyro into the LiDAR
frame and the second converts IMU orientation into the LiDAR frame. LIO-SAM
uses these parameters, not a TF lookup, for its IMU conversion.

### IMU noise parameter changes

File: `autopilot_nav2_bringup/params/lio_sam_params.yaml`

| Parameter | Old | New | Reason |
|---|---|---|---|
| `imuAccNoise` | 0.005 | **0.01** | 7× BMI055 spec — accounts for vehicle vibration and mounting noise |
| `imuGyrNoise` | 0.0003 | **0.001** | 8× BMI055 spec — accounts for chassis/motor vibration |
| `imuAccBiasN` | 0.0001 | **0.0004** | Was 4× too tight; matches BMI055 actual bias instability (~40 μg) |
| `imuGyrBiasN` | 0.000022 | **0.0002** | Was ~10× too tight; allows optimizer to track real bias drift with temperature |

**`imuAccBiasN` and `imuGyrBiasN` are the most impactful.**
These control how fast bias is allowed to drift. If too tight, GTSAM holds
bias frozen while the real bias drifts (temperature, vibration), causing the
velocity estimate to accumulate error until the 30 m/s reset triggers.

```yaml
# IMU Settings
imuAccNoise: 0.01
imuGyrNoise: 0.001
imuAccBiasN: 0.0004
imuGyrBiasN: 0.0002
```

---

## Loop Closure Tuning

### Background: "Large velocity" after ~5 minutes at real-time

The system runs fine at startup but triggers "Large velocity, reset IMU-preintegration!"
after ~5 minutes. This is **not** a thermal or noise issue — confirmed by the fact that
replaying the same bag at low rate works without errors.

**Root cause: loop closure blocking the mapOptimization thread**

When a loop closure fires, `mapOptimization` blocks on:
1. 6× `isam->update()` — cost scales with total keyframe count
2. `correctPoses()` — iterates over every keyframe ever added (`mapOptmization.cpp:1597`)

After 5 minutes at 1 m/s with keyframes every 1 m, ~300 keyframes have accumulated.
The blocking window grows longer with each loop closure detection. During the block,
no pose correction is published to `imuPreintegration` → IMU integrates without
correction → velocity diverges → reset.

At low bag rate, there is enough wall-clock slack between scans to absorb the block.
At real-time there is not.

### Parameter changes

| Parameter | Old | New | Status | Reason |
|---|---|---|---|---|
| `loopClosureFrequency` | 1.0 | **1.0** | keep / revert if changed | Do not increase — more frequent closures = more frequent blocking |
| `historyKeyframeSearchRadius` | 15.0 | **20.0** | applied | Wider outdoor loop closure search |
| `surroundingkeyframeAddingDistThreshold` | 1.0 | **2.0** | applied | Keyframe every 2 m instead of 1 m — halves accumulation rate, `correctPoses()` stays fast |
| `historyKeyframeSearchNum` | 25 | **15** | applied | Smaller ICP submap per closure detection — faster per-closure ICP |

```yaml
loopClosureFrequency:                    1.0   # do NOT increase
historyKeyframeSearchRadius:             20.0
surroundingkeyframeAddingDistThreshold:  2.0
historyKeyframeSearchNum:                15
```

### Complete param status (all changes across this session)

| Parameter | Original | Current | Pending | Reason |
|---|---|---|---|---|
| `imuAccNoise` | 0.005 | 0.01 | — | ZED2i vibration margin |
| `imuGyrNoise` | 0.0003 | 0.001 | — | Real-world gyro noise |
| `imuAccBiasN` | 0.0001 | 0.0004 | — | Allow GTSAM to track bias drift |
| `imuGyrBiasN` | 0.000022 | 0.0002 | — | Allow GTSAM to track bias drift |
| `loopClosureFrequency` | 1.0 | 2.0 | revert to 1.0 | Frequent closures block main thread |
| `historyKeyframeSearchRadius` | 15.0 | 20.0 | — | Wider outdoor search |
| `surroundingkeyframeAddingDistThreshold` | 1.0 | 2.0 | — | Reduce keyframe accumulation rate |
| `historyKeyframeSearchNum` | 25 | 15 | — | Smaller ICP submap |

---

## RTK GPS Tuning (when GPS is enabled)

GPS is currently disabled. When re-enabling, apply these changes.

### How GPS works in LIO-SAM

GPS only lives in `mapOptmization.cpp`. The other nodes do not process GPS.

```
[RTK GPS → navsat_transform → nav_msgs/Odometry]
            ↓  gpsTopic
       gpsHandler()  →  gpsQueue (deque)
            ↓  (called every LiDAR scan)
       addGPSFactor()
         ├─ Gate 1: robot traveled ≥ 5 m total
         ├─ Gate 2: LiDAR pose covariance > poseCovThreshold
         ├─ Gate 3: GPS timestamp within ±0.2 s of LiDAR scan
         ├─ Gate 4: GPS covariance < gpsCovThreshold
         └─ Creates gtsam::GPSFactor (position-only, no rotation)
            ↓
       ISAM2 optimizer  →  correctPoses() backtracks all keyframes
```

GPS provides **position-only (X, Y, Z) constraints** — it never constrains
orientation. It acts as a long-range drift corrector, not a primary sensor.

### Hardcoded noise floor (code change required)

`src/mapOptmization.cpp:1468` floors all GPS noise at 1.0 m², so even RTK
fixed solutions (0.01 m²) are treated the same as single-point GPS:

```cpp
// Current — wastes RTK precision
Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);

// Fix — let RTK fixed accuracy reach GTSAM (~100x more weight)
Vector3 << max(noise_x, 0.01f), max(noise_y, 0.01f), max(noise_z, 0.01f);
```

### RTK covariance by fix type

| Fix type | Typical covariance |
|---|---|
| RTK Fixed | 0.001 – 0.02 m² |
| RTK Float | 0.05 – 0.5 m² |
| Single point | 1 – 10+ m² |

### Recommended GPS params

```yaml
useImuHeadingInitialization: true    # IMU provides heading on cold start
useGpsElevation:             true    # RTK vertical is ~2-3 cm, worth using
gpsCovThreshold:             0.05    # only accept RTK fixed; float/single rejected automatically
poseCovThreshold:            2.0     # GPS corrects earlier, less drift accumulation
```

### Additional code changes for RTK

**Spatial decimation** (`src/mapOptmization.cpp:1462`) — reduce from 5 m to 1 m:
```cpp
if (pointDistance(curGPSPoint, lastGPSPoint) < 1.0)  // was 5.0
```

**Multipath rejection** — add after line 1465:
```cpp
static double lastGPSTime = 0;
double curGPSTime = stamp2Sec(thisGPS.header.stamp);
float jumpDist = pointDistance(curGPSPoint, lastGPSPoint);
double dt = curGPSTime - lastGPSTime;
if (lastGPSTime > 0 && dt > 0 && jumpDist / dt > 20.0)
    continue;
lastGPSTime = curGPSTime;
```

### GPS change summary

| # | Change | File | Impact |
|---|---|---|---|
| 1 | Noise floor: `1.0f` → `0.01f` | `src/mapOptmization.cpp:1468` | Highest — unlocks RTK precision |
| 2 | `gpsCovThreshold: 0.05` | params yaml | Gates out float/single automatically |
| 3 | `poseCovThreshold: 2.0` | params yaml | GPS corrects earlier |
| 4 | `useGpsElevation: true` | params yaml | Uses RTK vertical accuracy |
| 5 | Spatial decimation: `5.0` → `1.0` | `src/mapOptmization.cpp:1462` | Denser corrections |
| 6 | Velocity jump check | `src/mapOptmization.cpp:1465` | Rejects multipath outliers |

---

## Notes

- GPS provides position-only constraints. Heading always comes from LiDAR/IMU.
- GPS input must be Cartesian ENU odometry (from `robot_localization`'s
  `navsat_transform_node`), not raw lat/lon.
- When RTK is unavailable, the system falls back to pure LiDAR odometry
  transparently — no code changes needed.
- ZED positional tracking (`pos_tracking_enabled`) should be **disabled** when
  LIO-SAM is running to avoid TF conflicts and save Jetson GPU.
