#include <glog/logging.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <sophus/se3.hpp>

TEST(sophus, transform) {
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Quaterniond q(R);

  // 转为李代数
  Sophus::SO3d SO3_R(R);
  Sophus::SO3d SO3_Q(q);
  auto r = SO3_R.log();
  LOG(INFO) << SO3_R.matrix();
  LOG(INFO) << SO3_R.log().transpose();
  LOG(INFO) << Sophus::SO3d::hat(r);

  auto rr = SO3_R * SO3_Q;
  LOG(INFO) << rr.log().transpose();

  rr *= SO3_R;
  LOG(INFO) << rr.log().transpose();
}

TEST(EigenTest, pose) {
  Eigen::Matrix3d R;
  R << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03,
      -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01,
      9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03;
  Eigen::Vector3d t;
  t << -4.784029760483e-03, -7.337429464231e-02, -3.339968064433e-01;
  
  Eigen::Quaterniond q(R);
  q.normalize();
  R = q.toRotationMatrix();
  LOG(INFO) << R * R.transpose();
  Sophus::SE3d T_cw = Sophus::SE3d(R, t);
  
  LOG(INFO) << T_cw.matrix();
  LOG(INFO) << T_cw.inverse().matrix();
}

int main(int argc, char const *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}
