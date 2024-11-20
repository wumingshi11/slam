# 摘要
本文是阅读高翔博士 《自动驾驶与机器人中SLAM技术》的笔记。
基本原理见 doc/ICP.md。本文主要看实现。试图实现以NDT为例，举例整个LO的流程。

## NDT实现
NDT的ICP方法基于的迭代优化实现。也可之间计算数值点，但异常点不好处理。
### 计算单个残差和雅可比
重点关注其残差的计算和雅可比。这儿换成其他方法计算残差可一样可行
```c++
    for (int i = 0; i < nearby_grids_.size(); ++i) {
                auto key_off = key + nearby_grids_[i];
                auto it = grids_.find(key_off);
                int real_idx = idx * num_residual_per_point + i;
                if (it != grids_.end()) {
                    auto& v = it->second;  // voxel
                    Vec3d e = qs - v.mu_;

                    // check chi2 th
                    double res = e.transpose() * v.info_ * e;
                    // 这儿是由于数值解法的地方。排除异常点
                    if (std::isnan(res) || res > options_.res_outlier_th_) {
                        effect_pts[real_idx] = false;
                        continue;
                    }

                    // build residual
                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(q);
                    J.block<3, 3>(0, 3) = Mat3d::Identity();

                    jacobians[real_idx] = J;
                    errors[real_idx] = e;
                    infos[real_idx] = v.info_;
                    effect_pts[real_idx] = true;
                } else {
                    effect_pts[real_idx] = false;
                }
            }
```
### 整合残差和雅可比
这部分其他ICP类似
```cpp
        // 累加Hessian和error,计算dx
        // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
        double total_res = 0;
        int effective_num = 0;

        Mat6d H = Mat6d::Zero();
        Vec6d err = Vec6d::Zero();

        for (int idx = 0; idx < effect_pts.size(); ++idx) {
            if (!effect_pts[idx]) {
                continue;
            }

            total_res += errors[idx].transpose() * infos[idx] * errors[idx];
            // chi2.emplace_back(errors[idx].transpose() * infos[idx] * errors[idx]);
            effective_num++;

            H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
            err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
        }

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();
```
### 总结
NDT通过构建残差和雅可比（与其他方法的区别），通过同非线性优化方法不断迭代，达到收敛的效果。

## LO
整体流程如下
``` c++
void DirectNDTLO::AddCloud(CloudPtr scan, SE3& pose) {
    if (local_map_ == nullptr) {
        // 第一个帧，直接加入local map
        local_map_.reset(new PointCloudType);
        // operator += 用来拼接点云
        *local_map_ += *scan;
        pose = SE3();
        last_kf_pose_ = pose;

        if (options_.use_pcl_ndt_) {
            ndt_pcl_.setInputTarget(local_map_);
        } else {
            ndt_.SetTarget(local_map_);
        }

        return;
    }

    // 计算scan相对于local map的位姿
    pose = AlignWithLocalMap(scan);
    CloudPtr scan_world(new PointCloudType);
    pcl::transformPointCloud(*scan, *scan_world, pose.matrix().cast<float>());

    if (IsKeyframe(pose)) {
        last_kf_pose_ = pose;

        // 重建local map
        scans_in_local_map_.emplace_back(scan_world);
        if (scans_in_local_map_.size() > options_.num_kfs_in_local_map_) {
            scans_in_local_map_.pop_front();
        }

        local_map_.reset(new PointCloudType);
        for (auto& scan : scans_in_local_map_) {
            *local_map_ += *scan;
        }

        if (options_.use_pcl_ndt_) {
            ndt_pcl_.setInputTarget(local_map_);
        } else {
            ndt_.SetTarget(local_map_);
        }
    }

    if (viewer_ != nullptr) {
        viewer_->SetPoseAndCloud(pose, scan_world);
    }
}
```
配准流程
```c++
SE3 DirectNDTLO::AlignWithLocalMap(CloudPtr scan) {
    if (options_.use_pcl_ndt_) {
        ndt_pcl_.setInputSource(scan);
    } else {
        ndt_.SetSource(scan);
    }

    CloudPtr output(new PointCloudType());

    SE3 guess;
    bool align_success = true;
    if (estimated_poses_.size() < 2) {
        if (options_.use_pcl_ndt_) {
            ndt_pcl_.align(*output, guess.matrix().cast<float>());
            guess = Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
        } else {
            align_success = ndt_.AlignNdt(guess);
        }
    } else {
        // 从最近两个pose来推断
        SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
        SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
        guess = T1 * (T2.inverse() * T1);

        if (options_.use_pcl_ndt_) {
            ndt_pcl_.align(*output, guess.matrix().cast<float>());
            guess = Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
        } else {
            align_success = ndt_.AlignNdt(guess);
        }
    }

    LOG(INFO) << "pose: " << guess.translation().transpose() << ", "
              << guess.so3().unit_quaternion().coeffs().transpose();

    if (options_.use_pcl_ndt_) {
        LOG(INFO) << "trans prob: " << ndt_pcl_.getTransformationProbability();
    }

    estimated_poses_.emplace_back(guess);
    return guess;
}
```