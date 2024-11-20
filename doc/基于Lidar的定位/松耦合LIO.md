# 基于松耦合的LIO
LIO = Lidar-IMU-Odometry。即基于激光雷达和IMU的里程计。此外还可以考虑频率较低的RTK数据。
一次要耦合的数据
```c++
/// IMU 数据与雷达同步
struct MeasureGroup {
    MeasureGroup() { this->lidar_.reset(new FullPointCloudType()); };

    double lidar_begin_time_ = 0;   // 雷达包的起始时间
    double lidar_end_time_ = 0;     // 雷达的终止时间
    FullCloudPtr lidar_ = nullptr;  // 雷达点云，单个，但点云时间不同，需要处理畸变
    std::deque<IMUPtr> imu_;        // 上一时时刻到现在的IMU读数,多个IMU数据
};
// 整体流程
void LooselyLIO::ProcessMeasurements(const MeasureGroup &meas) {
    LOG(INFO) << "call meas, imu: " << meas.imu_.size() << ", lidar pts: " << meas.lidar_->size();
    measures_ = meas;

    if (imu_need_init_) {
        // 初始化IMU系统
        TryInitIMU();
        return;
    }

    // 利用IMU数据进行状态预测
    Predict();

    // 对点云去畸变
    Undistort();

    // 配准
    Align();
}
```
## 初始化IMU
IMU初始化只用涉及3个变量，两个方差
1. bg ba g
2. $\Sigma_{ba} |\Sigma_{bg}$
```c++
bool StaticIMUInit::TryInit() {
    if (init_imu_deque_.size() < 10) {
        return false;
    }

    // 计算均值和方差
    Vec3d mean_gyro, mean_acce;
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_, [](const IMU& imu) { return imu.gyro_; });
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU& imu) { return imu.acce_; });

    // 以acce均值为方向，取9.8长度为重力
    LOG(INFO) << "mean acce: " << mean_acce.transpose();
    gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;

    // 重新计算加计的协方差
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                [this](const IMU& imu) { return imu.acce_ + gravity_; });

    // 检查IMU噪声
    if (cov_gyro_.norm() > options_.max_static_gyro_var) {
        LOG(ERROR) << "陀螺仪测量噪声太大" << cov_gyro_.norm() << " > " << options_.max_static_gyro_var;
        return false;
    }

    if (cov_acce_.norm() > options_.max_static_acce_var) {
        LOG(ERROR) << "加计测量噪声太大" << cov_acce_.norm() << " > " << options_.max_static_acce_var;
        return false;
    }

    // 估计测量噪声和零偏
    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    LOG(INFO) << "IMU 初始化成功，初始化时间= " << current_time_ - init_start_time_ << ", bg = " << init_bg_.transpose()
              << ", ba = " << init_ba_.transpose() << ", gyro sq = " << cov_gyro_.transpose()
              << ", acce sq = " << cov_acce_.transpose() << ", grav = " << gravity_.transpose()
              << ", norm: " << gravity_.norm();
    LOG(INFO) << "mean gyro: " << mean_gyro.transpose() << " acce: " << mean_acce.transpose();
    init_success_ = true;
    return true;
}
```
初始化数据来初始化ESKF
```cpp
void LooselyLIO::TryInitIMU() {
    for (auto imu : measures_.imu_) {
        imu_init_.AddIMU(*imu);
    }

    if (imu_init_.InitSuccess()) {
        // 读取初始零偏，设置ESKF
        sad::ESKFD::Options options;
        // 噪声由初始化器估计
        options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
        options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
        eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
        imu_need_init_ = false;

        LOG(INFO) << "IMU初始化成功";
    }
}
```
使用的方法为利用数据中前面的IMU去初始化。
## 根据IMU计算位姿
```cpp
void LooselyLIO::Predict() {
    imu_states_.clear();
    imu_states_.emplace_back(eskf_.GetNominalState());

    /// 对IMU状态进行预测
    for (auto &imu : measures_.imu_) {
        eskf_.Predict(*imu);
        imu_states_.emplace_back(eskf_.GetNominalState());
    }
}
```
## 点云去畸变
点云畸变：激光雷达在产生数据不是在同一个时刻，这个过程中车辆发生了移动，导致点云会漂移
点云去畸变：利用IMU数据，将点云转移到某一时刻（通常 是最后时刻）
```c++
    /// 将所有点转到最后时刻状态上
    std::for_each(std::execution::par_unseq, cloud->points.begin(), cloud->points.end(), [&](auto &pt) {
        SE3 Ti = T_end; // 第i个点云的状态 T_wi 表示IMU坐标到世界坐标的转换
        NavStated match;

        // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
        math::PoseInterp<NavStated>(
            measures_.lidar_begin_time_ + pt.time * 1e-3, imu_states_, [](const NavStated &s) { return s.timestamp_; },
            [](const NavStated &s) { return s.GetSE3(); }, Ti, match);

        Vec3d pi = ToVec3d(pt);
        // 此处T表示T_wi  从右到左 点云坐标-> IMU坐标->世界坐标->IMU坐标->点云坐标
        Vec3d p_compensate = TIL_.inverse() *T_end.inverse() * Ti * TIL_ * pi;

        pt.x = p_compensate(0);
        pt.y = p_compensate(1);
        pt.z = p_compensate(2);
    });
```
## 组合导航
```cpp
void LooselyLIO::Align() {
    FullCloudPtr scan_undistort_trans(new FullPointCloudType);
    pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans, TIL_.matrix());
    scan_undistort_ = scan_undistort_trans;

    auto current_scan = ConvertToCloud<FullPointType>(scan_undistort_);

    // voxel 之
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(0.5, 0.5, 0.5);
    voxel.setInputCloud(current_scan);

    CloudPtr current_scan_filter(new PointCloudType);
    voxel.filter(*current_scan_filter);

    /// 处理首帧雷达数据
    if (flg_first_scan_) {
        SE3 pose;
        inc_ndt_lo_->AddCloud(current_scan_filter, pose);
        flg_first_scan_ = false;
        return;
    }

    /// 从EKF中获取预测pose，放入LO，获取LO位姿，最后合入EKF
    // 这儿用了IMU的数据，不能完全称之为松耦合，但可IEKF确实有区别
    SE3 pose_predict = eskf_.GetNominalSE3();
    inc_ndt_lo_->AddCloud(current_scan_filter, pose_predict, true);
    pose_of_lo_ = pose_predict;
    // 由误差卡尔曼滤波进行融合， 这儿两个系统计算过程中相对独立，为松耦合
    eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2);

    if (options_.with_ui_) {
        // 放入UI
        ui_->UpdateScan(current_scan, eskf_.GetNominalSE3());  // 转成Lidar Pose传给UI
        ui_->UpdateNavState(eskf_.GetNominalState());
    }
    frame_num_++;
}
```

## 迭代更新
下一次迭代的初始值，就是上一次迭代的输出值。
这个状态有ESKF维护