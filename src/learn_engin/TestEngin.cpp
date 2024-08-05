#include <glog/logging.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

TEST(EigenTest, baseType) {
  // Eigen最重要类型。矩阵
  Eigen::Matrix<double, 3, 3> m;
  m << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  EXPECT_EQ(m(2, 2), 9);
  // 向量是矩阵的一个using 限制模板后的类型
  Eigen::Vector3d v;
  v << 1, 3, 4;
  EXPECT_EQ(v(1), 3);

  // 只是数据副本
  auto m_sub = m.block<2,2>(1,1);
  m(1,1) = 11;
  LOG(INFO) << m_sub; 
  m_sub(1,1) = 11;
  LOG(INFO) << m;
}

// 四元数， R， aixs 三者直接的关系，以及相互转换
TEST(EigenTest, quaterniond) {
  // w,x,y,z
  Eigen::Quaternion<double> q(1, 2, 3, 4);
  q = q.normalized();
  LOG(INFO) << q.coeffs().transpose();
  auto R = q.toRotationMatrix();
  LOG(INFO) << R;
  // 计算特征值
  Eigen::EigenSolver<Eigen::Matrix3d> solver(R);
  auto featureVec = solver.eigenvectors();
  auto featureValue = solver.eigenvalues();
  LOG(INFO) << featureValue;
  LOG(INFO) << featureVec;

  Eigen::AngleAxisd axis(R);
  LOG(INFO) << axis.axis().transpose();

  Eigen::Quaternion<double> q2(R);
  LOG(INFO) << q2.coeffs().transpose();
}

TEST(EigenTest, compute) {
  // 向量运算
  Eigen::Vector3d v;
  v << 1, 3, 4;
  EXPECT_EQ(v.transpose() * v, 26);
  Eigen::Vector3d res;
  res << 0,0,0;
  EXPECT_EQ(v.cross(v), res);
}

TEST(EigenTest, compute2) {
Eigen::Vector3d v;
  v << 1, 3, 4;
  Eigen::Matrix<double, 3, 3> m;
  m << 1, 2, 11, 4, 5, 6, 7, 8, 9;
  m = m.transpose() * m; // 对称矩阵
  auto dert = m.determinant();
  LOG(INFO) << m;
  ASSERT_NE(dert,0);
  // 直接法
  Eigen::Vector3d x1 = m.inverse() * v;
  // lu分解
  Eigen::Vector3d x2 = m.lu().solve(v);
  // qr分解
  Eigen::Vector3d x3 = m.colPivHouseholderQr().solve(v);
  // clolesky分解
  Eigen::Vector3d x4 = m.ldlt().solve(v); // 适合对称 半正定对称矩阵
  Eigen::Vector3d x5 = m.llt().solve(v); // 蛇和对称，正定矩阵

  LOG(INFO) << x1.transpose();
  LOG(INFO) << x2.transpose();
  LOG(INFO) << x3.transpose();
  LOG(INFO) << x4.transpose();
  LOG(INFO) << x5.transpose();
  // 浮点不玩笑相同

  // EXPECT_EQ(x1,x2);
  // EXPECT_EQ(x3,x2);
  // EXPECT_EQ(x3,x4);
  // EXPECT_EQ(x4,x5);
}

int main(int argc, char const *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}
