# 整体分析
## 通信消息
### 点云消息， 用于接收bag中Livox
位于third_party中。
```msg
# Livox publish pointcloud msg format.

Header header             # ROS standard message header time(s,ns) id
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data

# Livox costom pointcloud format.

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
```
### 位姿
```msg
# the preintegrated Lidar states at the time of IMU measurements in a frame
float64  offset_time # the offset time of IMU measurement w.r.t the first lidar point
float64[3] acc       # the preintegrated total acceleration (global frame) at the Lidar origin
float64[3] gyr       # the unbiased angular velocity (body frame) at the Lidar origin
float64[3] vel       # the preintegrated velocity (global frame) at the Lidar origin
float64[3] pos       # the preintegrated position (global frame) at the Lidar origin
float64[9] rot       # the preintegrated rotation (global frame) at the Lidar origin
```
## 离线app逻辑
文件 app/run_mapping_online.cc
```cpp
 for (const rosbag::MessageInstance &m : rosbag::View(bag)) {
        auto livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_msg) {
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &livox_msg]() {
                    laser_mapping->LivoxPCLCallBack(livox_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
            continue;
        }

        auto point_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (point_cloud_msg) {
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &point_cloud_msg]() {
                    laser_mapping->StandardPCLCallBack(point_cloud_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
            continue;
        }

        auto imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg) {
            laser_mapping->IMUCallBack(imu_msg);
            continue;
        }

        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
    }
```
## 源码
真正的实现位于include和src.对源码关键部分进行注释。