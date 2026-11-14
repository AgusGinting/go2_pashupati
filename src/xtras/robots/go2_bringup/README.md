# go2_bringup: the Unitree Go2 EDU for Pashupati

The robot package ([contract](../../README.md)) for a **Go2 EDU** with a Livox **Mid-360** and a
RealSense **D435i**, running on the Go2's own Jetson Orin NX (Ubuntu 20.04, ROS 2 Foxy).

| Part                            | What                                                                                  |
| ------------------------------- | ------------------------------------------------------------------------------------- |
| `go2_driver_node`               | the contract over [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) |
| `launch/bringup.launch.py`      | driver, `robot_state_publisher`, Livox driver (`lidar:=`), RealSense (`camera:=`, off for now) |
| `urdf/go2.urdf`                 | `base_link`, `livox_frame`, `livox_imu_frame`, `camera_link`                          |
| `config/robot.yaml`             | the robot profile (`ROBOT_PARAMS`)                                                    |
| `config/go2_driver.yaml`        | driver parameters: interface, watchdog, speed ceiling                                 |
| `config/mid360.json`            | Livox network: host 192.168.123.18 (the Jetson), LiDAR 192.168.123.20                 |
| `config/realsense.yaml`         | colour 640x480 at 30 Hz only                                                          |
| `scripts/setup.sh`              | `ROBOT_SETUP`: installs and checks the SDK and the link to the robot                  |

## How it talks to the robot

The Go2's sport service and state are DDS topics on **domain 0** of the robot network
(`eth0`, 192.168.123.0/24). The driver reaches them through the SDK's own Cyclone DDS 0.10, not
through ROS, so the Pashupati stack keeps `rmw_fastrtps_cpp` and `ROS_LOCALHOST_ONLY=1`: nothing
of the robot network appears in `ros2 topic list`, nothing on it can publish to the stack.

| Contract (`/go2/...`)       | Go2                                                                                           |
| --------------------------- | --------------------------------------------------------------------------------------------- |
| `cmd_vel` subscribe         | `SportClient.Move(vx, vy, wz)` at 50 Hz while commands come; `StopMove` once after 0.5 s or on a zero command |
| `odom` publish, ~40 Hz      | `rt/lf/sportmodestate`: `velocity` (Unitree's odometry frame) turned into `base_link` by the IMU yaw, `yaw_speed` |
| `battery` publish, 1 Hz     | `rt/lf/lowstate`: `bms_state.soc`, `power_v`, `bms_state.current`                             |
| `diagnostics` publish, 1 Hz | sport `mode`, `gait_type`, `error_code`, `body_height`; ERROR without sport state for 1 s     |
| `stand_up`                  | `StandUp`, 1.5 s, `BalanceStand` (the mode in which `Move` walks)                             |
| `sit_down`                  | `StopMove`, `StandDown`                                                                       |
| `emergency_stop`            | `Damp`: the motors go soft and the robot lies down, like the remote's e-stop                  |

No TF from the driver: Pashupati owns `odom → base_link`.

## Install on the Go2

```bash
# /etc/pashupati/pashupati.env (scripts/pashupati.env.example for the rest)
ROBOT_ID=go2-01
ROBOT_DRIVER=go2_bringup
ROBOT_NS=go2
ROBOT_PARAMS=/home/unitree/Pashupati/install/go2_bringup/share/go2_bringup/config/robot.yaml
ROBOT_SETUP=/home/unitree/Pashupati/src/xtras/robots/go2_bringup/scripts/setup.sh
ROBOT_IP=192.168.123.161
LIVOX_IP=192.168.123.20
```

```bash
scripts/setup.sh install robot   # unitree_sdk2py (already there on this Go2), cyclonedds 0.10.2
scripts/setup.sh check
```

The SDK needs Cyclone DDS 0.10 (C) at `CYCLONEDDS_HOME`, default `~/cyclonedds/install`: the
launch puts its `lib` first in the driver's `LD_LIBRARY_PATH`, because Foxy's own `libddsc` (0.7)
breaks `import cyclonedds` (`undefined symbol: ddsi_sertype_v0`).

## Before the first patrol

1. **Mounts:** measure the Mid-360 on the robot and set `livox_joint` in `urdf/go2.urdf`;
   measure the footprint with the sensors on (`config/robot.yaml`). The site file's LiDAR
   height band (`filter.height_max`, Tuning Guide) was set for the taller L1W.
   The camera is off (`camera:=false`, no AprilTags, VLM inspection or people). Before turning
   it on, calibrate it again: the FAST-Calib result on this Go2
   (`~/calib_results/board0_success_*`) used 1280x720 intrinsics on 640x480 images and puts the
   camera 1.3 m from the LiDAR. Then set `camera_joint` from it.
2. **The robot package alone** (robot lifted or in an open area, the vendor remote at hand):
   ```bash
   ros2 launch go2_bringup bringup.launch.py
   ros2 topic hz /go2/odom                       # ~40 Hz
   ros2 topic echo /go2/battery --once
   ros2 service call /go2/stand_up std_srvs/srv/Trigger
   ros2 topic pub -r 20 /go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ros2 topic echo /go2/odom --field twist.twist  # linear.x ~0.1, linear.y ~0 at any heading
   ```
   Stop the publisher: the robot must stop within 0.5 s. Then stop the driver itself (Ctrl-C)
   while it walks: it sends `StopMove` on the way out. **Also kill it hard** (`kill -9`) while it
   walks and note what the Go2 does without commands; if it keeps walking, the e-stop on the
   vendor remote is the only stop for a crashed driver.
3. **Braking:** measure, then set `robot.limits.max_decel` and `reaction_time` (Tuning Guide 3.2).
4. **Obstacle avoidance of the Go2** (the app's) off: it changes the commanded speeds.

## Develop

```bash
colcon build --packages-select go2_bringup && colcon test --packages-select go2_bringup
```

The tests run the node against a stand-in sport service; `test_sdk_contract.py` checks the pinned
SDK still has what the driver calls (skipped when the SDK is not importable). CI:
`.github/workflows/go2.yml`, on Foxy and Humble.
