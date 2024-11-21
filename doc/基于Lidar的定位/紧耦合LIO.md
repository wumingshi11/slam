# 紧耦合LIO
## IEKF 可迭代的基于误差的卡尔曼滤波
其原理见doc/从卡尔曼滤波到组合导航.md
### 雅可比和残差
```cpp
    // gauss-newton 迭代
    // 最近邻，可以并发
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
        auto q = ToVec3d(source_->points[idx]);
        Vec3d qs = pose * q;  // 转换之后的q

        // 计算qs所在的栅格以及它的最近邻栅格
        Vec3i key = CastToInt(Vec3d(qs * options_.inv_voxel_size_));

        for (int i = 0; i < nearby_grids_.size(); ++i) {
            Vec3i real_key = key + nearby_grids_[i];
            auto it = grids_.find(real_key);
            int real_idx = idx * num_residual_per_point + i;
            /// 这里要检查高斯分布是否已经估计
            if (it != grids_.end() && it->second->second.ndt_estimated_) {
                auto& v = it->second->second;  // voxel
                Vec3d e = qs - v.mu_;

                // check chi2 th
                double res = e.transpose() * v.info_ * e;
                if (std::isnan(res) || res > options_.res_outlier_th_) {
                    effect_pts[real_idx] = false;
                    continue;
                }

                // build residual
                Eigen::Matrix<double, 3, 18> J;
                J.setZero();
                J.block<3, 3>(0, 0) = Mat3d::Identity();                   // 对p
                J.block<3, 3>(0, 6) = -pose.so3().matrix() * SO3::hat(q);  // 对R

                jacobians[real_idx] = J;
                errors[real_idx] = e;
                infos[real_idx] = v.info_;
                effect_pts[real_idx] = true;
            } else {
                effect_pts[real_idx] = false;
            }
        }
    });
```
```cpp
    // 累加Hessian和error,计算dx
    double total_res = 0;
    int effective_num = 0;

    HTVH.setZero();
    HTVr.setZero();

    const double info_ratio = 0.01;  // 每个点反馈的info因子

    for (int idx = 0; idx < effect_pts.size(); ++idx) {
        if (!effect_pts[idx]) {
            continue;
        }

        total_res += errors[idx].transpose() * infos[idx] * errors[idx];
        effective_num++;

        HTVH += jacobians[idx].transpose() * infos[idx] * jacobians[idx] * info_ratio;
        HTVr += -jacobians[idx].transpose() * infos[idx] * errors[idx] * info_ratio;
    }

    LOG(INFO) << "effective: " << effective_num;
```
### 更新
上述雅可比和残差只考虑了纯激光信息，应该还要考虑IMU约束，这是紧耦合的关键所在
```cpp
    SO3 start_R = R_;
    Eigen::Matrix<S, 18, 1> HTVr;
    Eigen::Matrix<S, 18, 18> HTVH;
    Eigen::Matrix<S, 18, Eigen::Dynamic> K;
    Mat18T Pk, Qk;

    for (int iter = 0; iter < options_.num_iterations_; ++iter) {
        // 调用obs function
        // HTVr 中 r表示残差
        obs(GetNominalSE3(), HTVH, HTVr);

        // 投影P 
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat((R_.inverse() * start_R).log());
        Pk = J * cov_ * J.transpose();

        // 卡尔曼更新 这儿考虑了IMU的约束
        Qk = (Pk.inverse() + HTVH).inverse();  // 这个记作中间变量，最后更新时可以用
        dx_ = Qk * HTVr; // 原理见SMW 恒等式。

        // LOG(INFO) << "iter " << iter << " dx = " << dx_.transpose() << ", dxn: " << dx_.norm();

        // dx合入名义变量
        Update();

        if (dx_.norm() < options_.quit_eps_) {
            break;
        }
    }

```
### 运动方程协方差跟新
因为迭代过程中R不断乘以一个$\delta R$,所以R的协方差需要改变
1. 更新过程中：
```cpp
        // 投影P 
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat((R_.inverse() * start_R).log());
        Pk = J * cov_ * J.transpose();
```
2. 程序收敛后:
```cpp
    // update P 
    cov_ = (Mat18T::Identity() - Qk * HTVH) * Pk;

    // project P
    Mat18T J = Mat18T::Identity();
    Vec3d dtheta = (R_.inverse() * start_R).log();
    J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dtheta);
    cov_ = J * cov_ * J.inverse();
```


