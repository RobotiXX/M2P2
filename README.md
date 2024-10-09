# M2P2: Multi-Modal Passive Perception Dataset

![Preview](https://raw.githubusercontent.com/RobotiXX/M2P2/main/images/m2p2.png)

 M2P2 Dataset and Open-Source Build are available on

[![Dataset Access](https://raw.githubusercontent.com/RobotiXX/M2P2/main/images/dataverse.png '')](https://dataverse.orc.gmu.edu/dataset.xhtml?persistentId=doi:10.13021/orc2020/SP577T)

M2P2 is a Multi-Modal Passive Perception dataset designed for off-road mobility in extreme low-light conditions. This repository contains the code and resources related to the M2P2 dataset, which enables off-road mobility using passive perception in low-light to no-light conditions.

## Dataset Overview

- **Total Compressed Size**: 1138.63 GB
- **Total Distance**: >32 km
- **Total Time**: ~10.15 hours
- **Environments**: Paved trails, non-paved off-road paths, and unprepared off-trail areas
- **Lighting Conditions**: 20 lx to complete darkness (0 lx)

## Sensor Suite
![Sensor suite](https://raw.githubusercontent.com/RobotiXX/M2P2/main/images/suite.jpg)

The M2P2 dataset was collected using a multi-modal sensor suite mounted on a Clearpath Husky A200 robot, including:

- Xenics Ceres V 1280 thermal camera @ 1280x1024, 10 Hz
- Prophesee Metavision EVK4 event camera @ 1280x720
- Two FLIR Blackfly S RGB cameras @ 1616x1240, 10 Hz
- Yahboom 10-DoF IMU @ 200 Hz
- 3D Ouster OS1-128 LiDAR @ 1024x10, 10 Hz (with embedded IMU @ 125 Hz) for ground truth

## Dataset Information

The dataset consists of the following runs, with each of the runs consisting of several 45s, 60s or 90s chunks due to file size constraints on GMU Dataverse

1. Burke Lake (60s chunks)
2. Lake Braddock (90s chunks)
3. Lake Royal (90s chunks)
4. GMU Main Campus (45s chunks)
5. GMU West Campus - day 1 (90s chunks)
6. GMU West Campus - day 2 (60s chunks)
7. GMU West Campus - day 3 (90s chunks)

### Dataverse directory information

| Dataverse Directory | Number of Files | Total Size (GB) | Chunk Size (seconds) |
|-----------|-----------------|-----------------|----------------------|
| m2p2_rosbags/burke_lake | 136 | 297.49 | 60 |
| m2p2_rosbags/lake_braddock | 59 | 134.08 | 90 |
| m2p2_rosbags/lake_royal | 103 | 223.91 | 90 |
| m2p2_rosbags/main_campus | 56 | 139.87 | 45 |
| m2p2_rosbags/west_campus_1 | 28 | 66.78 | 90 |
| m2p2_rosbags/west_campus_2 | 72 | 137.77 | 60 |
| m2p2_rosbags/west_campus_3 | 60 | 138.73 | 90 |


## Rosbag information

Each rosbag chunk contains the following topics on a best effort basis:

### Camera topics
#### RGB cameras
- `/sensor_suite/left_camera_optical/image_color/compressed : sensor_msgs/CompressedImage`
- `/sensor_suite/left_camera_optical/camera_info : sensor_msgs/CameraInfo`
- `/sensor_suite/right_camera_optical/image_color/compressed : sensor_msgs/CompressedImage`
- `/sensor_suite/right_camera_optical/camera_info : sensor_msgs/CameraInfo`

#### Thermal camera
- `/sensor_suite/lwir/lwir/image_raw/compressed : sensor_msgs/CompressedImage`
- `/sensor_suite/lwir/lwir/camera_info : sensor_msgs/CameraInfo`

#### Event camera
- `/sensor_suite/event_camera/events : event_camera_msgs/EventPacket`
- `/sensor_suite/event_camera/camera_info : sensor_msgs/CameraInfo`

### LiDAR topics
#### Points
- `/sensor_suite/ouster/points : sensor_msgs/PointCloud2`

#### IMU
- `/sensor_suite/ouster/imu : sensor_msgs/Imu`

### GNSS topics
- `/sensor_suite/f9p_rover/fix : sensor_msgs/NavSatFix`
- `/sensor_suite/f9p_rover/fix_velocity : geometry_msgs/TwistWithCovarianceStamped`
- `/sensor_suite/f9p_rover/navposecef : ublox_msgs/NavPOSECEF`
- `/sensor_suite/f9p_rover/navpvt : ublox_msgs/NavPVT`
- `/sensor_suite/f9p_rover/navsat : ublox_msgs/NavSAT`
- `/sensor_suite/f9p_rover/navstatus : ublox_msgs/NavSTATUS`

### IMU topics
- `/sensor_suite/witmotion_imu/imu : sensor_msgs/Imu`
- `/sensor_suite/witmotion_imu/magnetometer : sensor_msgs/MagneticField`
- `/sensor_suite/witmotion_imu/temperature : sensor_msgs/Temperature`

### Odometry topics
- `/husky_velocity_controller/odom : nav_msgs/Odometry`
- `/odometry/filtered : nav_msgs/Odometry`

### Control topics
- `/husky_velocity_controller/cmd_vel : geometry_msgs/Twist`
- `/husky_velocity_controller/cmd_vel_out : geometry_msgs/TwistStamped`
- `/joy_teleop/cmd_vel : geometry_msgs/Twist`
- `/joy_teleop/joy : sensor_msgs/Joy`

### Status topics
- `/status : husky_msgs/HuskyStatus`

### Transform topics
- `/tf : tf2_msgs/TFMessage`
- `/tf_static : tf2_msgs/TFMessage`


## Links

- [M2P2 Dataset](https://dataverse.orc.gmu.edu/dataset.xhtml?persistentId=doi:10.13021/orc2020/SP577T)
- [M2P2 Paper](https://cs.gmu.edu/~xiao/papers/m2p2.pdf)
- [M2P2 Video](https://youtu.be/ZA0CDbuL_4s)

## License

This project is licensed under the MIT License.

