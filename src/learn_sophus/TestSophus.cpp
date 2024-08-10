#include <glog/logging.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <sophus/se3.hpp>

TEST(sophus, transform){
   Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1)).toRotationMatrix();
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

int main(int argc, char const *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}
