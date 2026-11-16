# NEW
```text
激光＋视觉＋IMU → 局部里程计与局部地图（odom）
                         ↓ 关键帧、相对运动
RTK → 质量筛选 → 增量全局后端 → map→odom
                         ↓
                 全局轨迹与子地图
```
- 整理时间、坐标与数据接口
- 输入数据异常检测
- 动态过滤

## M3DGR
- 测试鲁棒性
- LVIO-RTK，没有 wheel

图片在各组表格下方以 360 像素宽度显示，点击图片可查看原图。

|Sequence Name|indoor/outdoor|Features|Challenge|TEST|
|--|--|--|--|--|
|Outdoor01|Outdoor|Standard|None|✅|
|Outdoor04|Outdoor|Standard|None|✅|

**Outdoor01**

[<img src="doc/m3dgr_outdoor01.png" width="360" alt="Outdoor01">](doc/m3dgr_outdoor01.png)

**Outdoor04**

[<img src="doc/m3dgr_outdoor04.png" width="360" alt="Outdoor04">](doc/m3dgr_outdoor04.png)

|Sequence Name|indoor/outdoor|Features|Challenge|TEST|
|--|--|--|--|--|
|Dark01|Outdoor|Night|Visual Challenge|✅|
|Dark02|Outdoor|Night|Visual Challenge|✅|
|Dark03|Indoor|Dark Room|Visual Challenge|✅|
|Dark04|Indoor|Dark Room|Visual Challenge|✅|
|Dynamic01|Indoor|Dynamic Person|Visual Challenge|✅|
|Dynamic02|Indoor|Dynamic Person|Visual Challenge|✅|
|Dynamic03|Outdoor|Dynamic Person|Visual Challenge|✅|
|Dynamic04|Outdoor|Dynamic Person|Visual Challenge|✅|
|Occlusion01|Indoor|Full Occlusion|Visual Challenge|✅|
|Occlusion02|Indoor|Full Occlusion|Visual Challenge|✅|
|Varying-illu01|Indoor|Varying Illumination|Visual Challenge|✅|
|Varying-illu02|Indoor|Varying Illumination|Visual Challenge|✅|

**Dark01**

[<img src="doc/m3dgr_visual_Dark01.png" width="360" alt="Dark01">](doc/m3dgr_visual_Dark01.png)

**Dark02**

[<img src="doc/m3dgr_visual_Dark02.png" width="360" alt="Dark02">](doc/m3dgr_visual_Dark02.png)

**Dark03**

[<img src="doc/m3dgr_visual_Dark03.png" width="360" alt="Dark03">](doc/m3dgr_visual_Dark03.png)

**Dark04**

[<img src="doc/m3dgr_visual_Dark04.png" width="360" alt="Dark04">](doc/m3dgr_visual_Dark04.png)

**Dynamic01**

[<img src="doc/m3dgr_visual_Dynamic01.png" width="360" alt="Dynamic01">](doc/m3dgr_visual_Dynamic01.png)

**Dynamic02**

[<img src="doc/m3dgr_visual_Dynamic02.png" width="360" alt="Dynamic02">](doc/m3dgr_visual_Dynamic02.png)

**Dynamic03**

[<img src="doc/m3dgr_visual_Dynamic03.png" width="360" alt="Dynamic03">](doc/m3dgr_visual_Dynamic03.png)

**Dynamic04**

[<img src="doc/m3dgr_visual_Dynamic04_01.png" width="360" alt="Dynamic04 视图 1">](doc/m3dgr_visual_Dynamic04_01.png)

[<img src="doc/m3dgr_visual_Dynamic04.png" width="360" alt="Dynamic04 视图 2">](doc/m3dgr_visual_Dynamic04.png)

**Occlusion01**

[<img src="doc/m3dgr_visual_Occlusion01.png" width="360" alt="Occlusion01">](doc/m3dgr_visual_Occlusion01.png)

**Occlusion02**

[<img src="doc/m3dgr_visual_Occlusion02.png" width="360" alt="Occlusion02">](doc/m3dgr_visual_Occlusion02.png)

**Varying-illu01**

[<img src="doc/m3dgr_visual_Varying-illu01.png" width="360" alt="Varying-illu01">](doc/m3dgr_visual_Varying-illu01.png)

**Varying-illu02**

[<img src="doc/m3dgr_visual_Varying-illu02.png" width="360" alt="Varying-illu02">](doc/m3dgr_visual_Varying-illu02.png)

|Sequence Name|indoor/outdoor|Features|Challenge|TEST|
|--|--|--|--|--|
|Grass01|Outdoor|Wheel Float|Wheel Challenge|✅|
|Grass02|Outdoor|Wheel Float|Wheel Challenge|✅|
|Sha-turn01|Indoor|Sha-turn|Wheel Challenge|✅|
|Sha-turn02|Indoor|Sha-turn|Wheel Challenge|✅|
|Wheel-float01|Indoor|Wheel Float|Wheel Challenge|✅|
|Wheel-float02|Indoor|Wheel Float|Wheel Challenge|✅|
|Z-Rough-Road01|Outdoor|Rough Road|Wheel Challenge|✅|

**Grass01**

[<img src="doc/m3dgr_wheel_Grass01.png" width="360" alt="Grass01">](doc/m3dgr_wheel_Grass01.png)

**Grass02**

[<img src="doc/m3dgr_wheel_Grass02.png" width="360" alt="Grass02">](doc/m3dgr_wheel_Grass02.png)

**Sha-turn01**

[<img src="doc/m3dgr_wheel_sha_turn01.png" width="360" alt="Sha-turn01">](doc/m3dgr_wheel_sha_turn01.png)

**Sha-turn02**

[<img src="doc/m3dgr_wheel_sha_turn02.png" width="360" alt="Sha-turn02">](doc/m3dgr_wheel_sha_turn02.png)

**Wheel-float01**

[<img src="doc/m3dgr_wheel_wheel_float01.png" width="360" alt="Wheel-float01">](doc/m3dgr_wheel_wheel_float01.png)

**Wheel-float02**

[<img src="doc/m3dgr_wheel_wheel_float02.png" width="360" alt="Wheel-float02">](doc/m3dgr_wheel_wheel_float02.png)

**Z-Rough-Road01**

[<img src="doc/m3dgr_wheel_Z.png" width="360" alt="Z-Rough-Road01">](doc/m3dgr_wheel_Z.png)

|Sequence Name|indoor/outdoor|Features|Challenge|TEST|
|--|--|--|--|--|
|Corridor01|Indoor|Long Corridor|Lidar Challenge|❌|
|Corridor02|Indoor|Long Corridor|Lidar Challenge|❌|
|Elevator01|Indoor|Long Corridor,Elevator|Lidar Challenge|❌|

**[todo] 检测 Hessian 退化，限制弱方向更新，用 IMU/VIO 保持短时连续，用结构平面和子地图提供弱先验。**       
- Hessian 退化: 在 LIO/VIO 更新后记录最小特征值、条件数、弱方向向量
- 退化方向保护更新: 当检测到 Long Corridor 退化时，不直接信任完整 6DoF LiDAR 更新；对弱方向做降权或冻结，只保留可信方向的修正。
- 特征增强:提取地面、左右墙面、天花板平面，估计走廊主方向；用主方向稳定 yaw 和横向位置，但不要过度约束沿走廊位移。
- 关键帧/子地图策略:长走廊中减少无限累积局部地图，按段构建子地图；在拐角、门口、明显结构处建立更强关键帧锚点。
- 加入轮速计。

|Sequence Name|indoor/outdoor|Features|Challenge|TEST|
|--|--|--|--|--|
|GNSS-denial01|outdoor|GNSS Denial|GNSS Challenge|✅|
|GNSS-denial02|outdoor|GNSS Denial|GNSS Challenge|✅|

**GNSS-denial01**

[<img src="doc/m3dgr_gnss_denial01.png" width="360" alt="GNSS-denial01">](doc/m3dgr_gnss_denial01.png)

**GNSS-denial02**

[<img src="doc/m3dgr_gnss_denial02.png" width="360" alt="GNSS-denial02">](doc/m3dgr_gnss_denial02.png)




# FAST-LIVO2-RTK

FAST-LIVO2-RTK extends FAST-LIVO2 with RTK/GNSS-constrained global optimization for long-term LiDAR-visual mapping.

**Key highlights include:**

1. Fully Reproducible: Complete open-source software and hardware setup guaranteeing fully reproducible LIVO-RTK experiments.
2. Robust Initialization Module: Built-in initialization featuring cross-correlation-based time offset estimation and hand-eye extrinsic calibration.
3. LIV-RTK Fusion Paradigm: A simple example paradigm for fusing LIVO trajectories with RTK observations.

📬 For further assistance or inquiries, please feel free to contact Chunran Zheng at [zhengcr@connect.hku.hk](mailto:zhengcr@connect.hku.hk).

<div align="center">
  <img src="pics/compare.jpg" width="100%" alt="FAST-LIVO2 and FAST-LIVO2-RTK comparison in a degraded LiDAR-visual scene">
  <br>
  <sub>Collected in a challenging scene with degraded geometry and texture.
<b>a2, b2, c2</b>: FAST-LIVO2 baseline. <b>a1, b1, c1</b>: results after RTK fusion.</sub>
</div>

## 1. Prerequisited

### 1.1 Ubuntu and ROS

Ubuntu 18.04~20.04. See [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 1.2 PCL, Eigen, and OpenCV

PCL>=1.8, Eigen>=3.3.4, OpenCV>=4.2.

### 1.3 Sophus

Install the non-templated/double-only version of Sophus.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

### 1.4 Vikit

Vikit provides the camera models and math utilities required by this project. Put it in your catkin workspace source folder.

```bash
cd ~/catkin_ws/src
git clone https://github.com/xuankuzcr/rpg_vikit.git
```

### 1.5 RTK Dependencies

The RTK branch depends on the GNSS ROS message package used by the u-blox/GVINS toolchain. Put it in the same catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git
```

GTSAM is used for factor graph-based post-processing optimization.

```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j$(nproc)
sudo make install
```

GeographicLib is used for converting geographic coordinates to local Cartesian coordinates.

```bash
sudo apt-get install libgeographic-dev ros-${ROS_DISTRO}-eigen-conversions
```

## 2. Run our examples

Download the provided RTK test rosbag file: [RTK-Dataset](https://drive.google.com/drive/folders/1xun-YL78cbMtyV60uN3FQFsHkkLUWvtJ?usp=sharing).

1. Launch the system and load the configuration file:

```bash
roslaunch fast_livo HH.launch
```

2. Play the rosbag. Once the sequence is finished, press `Enter` in the terminal running the launch file to trigger the backend optimizer.

```bash
rosbag play HH-LVGO-01.bag
```

### Data Interface Contract

The frontend now submits atomic keyframe and GNSS events directly to the backend.
`/synced_cloud` is in `imu_link`, while local poses and registered clouds use `odom`.
Original local IMU poses are retained separately from optimized global IMU poses.
`gps/gps_en=false` disables GNSS input without enabling or changing local RTK fusion.
RViz defaults to `odom`; no identity `map -> odom` transform is assumed.

See [the frame, time and replay contract](doc/data_contract.md) for parameter directions,
output migration, diagnostic exports and regression-test commands.

Input, GNSS and frontend health diagnostics are available on `/diagnostics`.
See [health diagnostics and fault injection](doc/health_diagnostics.md) for thresholds,
noise floors, CSV metrics and replay checks. `config/health.yaml` defaults to advisory
mode; trusted FIX and update rejection gates must be enabled explicitly.

An independent [iSAM2 online backend](doc/online_backend.md) is now available.
`HH.launch` and `AGV.launch` default to `backend_mode:=online`; use
`backend_mode:=batch` for the preserved Enter-triggered offline comparison.
Online GNSS constraints always require trusted FIX, regardless of input advisory mode.
The backend publishes `map -> odom`, `/odometry/global`, `/path/global` and a cached
optimized keyframe map on `/map/global`, without feeding global corrections into
local frontend state. A pre-existing uninitialized visual patch mean read was fixed
in both modes; numerical regression uses a freshly generated, corrected batch baseline.
Opt-in [geometric dynamic filtering](doc/dynamic_filter.md) now shares confidence
gates across LIO, permanent/temporary maps, visual patches and colored keyframes.
Enable it with `dynamic_filter:=true`; it defaults off pending real dynamic-scene
validation. Semantic detection is not included.

Opt-in [localization safety and recovery](doc/localization_safety.md) adds five
states, trusted output/map gates, bounded IMU propagation and nearby trusted-submap
matching. Enable it with `safety:=true`; consumers must check `/localization/status`
validity, measurement time, segment ID and timeout. Operator-confirmed stationary
reinitialization creates a new odometry segment in online mode. This is not global
place recognition or a safety certification; validation limits are documented.

Opt-in [resource limits and regression measurements](doc/resource_regression.md)
adds sliding map/visual history budgets, bounded queues and PCD chunks, graph
budget fail-stop, RSS sampling and P95/P99/recovery reports. Use
`safety:=true resources:=true` in online mode. Graph budget exhaustion invalidates
global output; fixed-lag marginalization and permanent historical submap graphs
are not implemented. Missing labelled scene clips are reported as SKIPPED.

## 3. Appendix
**Time Synchronization:**

The diagram illustrates the time synchronization scheme among GNSS, LiDAR, and image data.

<div align="center">
  <img src="pics/sync2.jpg" width="100%" alt="Time synchronization diagram">
</div>

**Hardware Platform:**

The table below summarizes the main devices used by the platform.

<div align="center">
  <table align="center">
    <tr>
      <th>Device</th>
      <th>Image</th>
      <th>Model Description</th>
    </tr>
    <tr>
      <td align="center">LiDAR</td>
      <td align="center"><img src="pics/LiDAR.jpg" width="180" alt="Livox Mid-360 LiDAR"></td>
      <td align="center">Model: Livox Mid-360</td>
    </tr>
    <tr>
      <td align="center">Camera</td>
      <td align="center"><img src="pics/camera.jpg" width="180" alt="MV-CB016-10GC-S-W camera"></td>
      <td align="center">Model: MV-CB016-10GC-S-W</td>
    </tr>
    <tr>
      <td align="center">GNSS Receiver</td>
      <td align="center"><img src="pics/ublox.jpg" width="180" alt="u-blox ZED-F9P GNSS receiver"></td>
      <td align="center">Model: u-blox ZED-F9P</td>
    </tr>
    <tr>
      <td align="center">Computing Unit</td>
      <td align="center"><img src="pics/n100.jpg" width="180" alt="N100 mini PC"></td>
      <td align="center">Model: N100 mini PC</td>
    </tr>
    <tr>
      <td align="center">Synchronization Controller</td>
      <td align="center"><img src="pics/stm32.jpg" width="180" alt="STM32 synchronization controller"></td>
      <td align="center">Model: STM32</td>
    </tr>
  </table>
</div>

## 4. Acknowledgements

This repository is built on top of [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) and uses several open-source libraries and packages, including [GTSAM](https://github.com/borglab/gtsam), [GeographicLib](https://geographiclib.sourceforge.io/), and [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm).
