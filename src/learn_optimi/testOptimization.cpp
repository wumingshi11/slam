
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <stdlib.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <sophus/se3.hpp>
#include <sstream>
// #include <g2o/solvers/levenberg_marquardt/optimization_algorithm_levenberg.h>

// 通义千问的示例
TEST(optimization, g2o_zyqw) {
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(true);  // 设置为 verbose 模式,显示优化信息

  // 使用 Eigen 线性求解器, linearSolver负责最后的计算
  using blockType = g2o::BlockSolverX;
  auto linearSolver =
      std::make_unique<g2o::LinearSolverEigen<blockType::PoseMatrixType>>();
  // 负责分块，然后交由线性求解器
  auto blockSolver = std::make_unique<blockType>(std::move(linearSolver));
  // 迭代方法
  auto* solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
  optimizer.setAlgorithm(solver);

  // 添加顶点
  g2o::VertexSE2* v1 = new g2o::VertexSE2();
  v1->setId(0);
  v1->setEstimate(g2o::SE2(0, 0, 0));  // 设置初始估计
  optimizer.addVertex(v1);

  g2o::VertexSE2* v2 = new g2o::VertexSE2();
  v2->setId(1);
  v2->setEstimate(g2o::SE2(1, 0, M_PI));  // 设置初始估计
  optimizer.addVertex(v2);

  // 添加边
  g2o::EdgeSE2* edge = new g2o::EdgeSE2();
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  edge->setMeasurement(g2o::SE2(0, 0, 0));  // 设置测量值
  edge->setInformation(
      g2o::EdgeSE2::InformationType::Identity());  // 设置信息矩阵
  optimizer.addEdge(edge);

  // 开始优化
  optimizer.initializeOptimization();
  optimizer.optimize(10);  // 最多进行 10 次迭代

  // 输出结果
  std::cout << "Vertex 1 estimate: " << v1->estimate().translation().transpose()
            << std::endl;
  std::cout << "Vertex 2 estimate: " << v2->estimate().translation().transpose()
            << std::endl;
}

// 定义顶点
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 重置
  virtual void setToOriginImpl() override { _estimate << 0, 0, 0; }
  // 更新
  virtual void oplusImpl(const double* update) override {
    LOG(INFO) << Eigen::Vector3d(update).transpose();
    _estimate += Eigen::Vector3d(update);
  }
  // 读存
  virtual bool read(std::istream& in) override {}
  virtual bool write(std::ostream& out) const override {}
};

// 定义边
// tparm : 观测（err）维度, 类型，顶点类型
class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}
  virtual void computeError() override {
    const CurveFittingVertex* v =
        static_cast<CurveFittingVertex*>(_vertices[0]);
    const Eigen::Vector3d params = v->estimate();
    _error(0, 0) = _measurement - std::exp(params(0, 0) * _x * _x +
                                           params(1, 0) * _x + params(2, 0));
  }
  virtual void linearizeOplus() override {
    const CurveFittingVertex* v =
        static_cast<CurveFittingVertex*>(_vertices[0]);
    const Eigen::Vector3d params = v->estimate();
    double y =
        std::exp(params(0, 0) * _x * _x + params(1, 0) * _x + params(2, 0));
    _jacobianOplusXi[0] = -_x * _x * y;
    _jacobianOplusXi[1] = -_x * y;
    _jacobianOplusXi[2] = -y;
  }
  // 读存
  virtual bool read(std::istream& in) override {}
  virtual bool write(std::ostream& out) const override {}

  double _x{0};
};
// 曲线拟合
TEST(optimization, g2o_curze) {
  // 数据准备
  std::vector<double> data_(100);
  std::vector<double> y_(100);
  constexpr int a = 1, b = 2, c = 3;
  for (size_t i = 0; i < data_.size(); i++) {
    auto tmp = drand48();
    data_[i] = tmp;
    y_[i] = std::exp(a * tmp * tmp + b * tmp + c) + (drand48() - 0.5);
  }
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(true);
  // 顶点维度，误差维度
  using blockType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
  // 使用 Eigen 线性求解器, linearSolver负责最后的计算
  auto linearSolver =
      std::make_unique<g2o::LinearSolverEigen<blockType::PoseMatrixType>>();
  // 负责分块，然后交由线性求解器
  auto blockSolver = std::make_unique<blockType>(std::move(linearSolver));
  // 迭代方法
  auto* solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
  optimizer.setAlgorithm(solver);
  auto v1 = new CurveFittingVertex();
  v1->setId(0);
  v1->setEstimate(Eigen::Vector3d(1, 0, 0));
  optimizer.addVertex(v1);
  for (size_t i = 0; i < data_.size(); i++) {
    auto* edge = new CurveFittingEdge(data_[i]);
    edge->setId(i);
    edge->setVertex(0, v1);
    edge->setMeasurement(y_[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    optimizer.addEdge(edge);
  }
  LOG(INFO) << "start optimization!";

  auto startTime = std::chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  auto endTime = std::chrono::steady_clock::now();
  auto du = std::chrono::duration_cast<std::chrono::duration<double>>(
      endTime - startTime);
  LOG(WARNING) << "optimize cost " << du.count() << " s";
  LOG(WARNING) << v1->estimate().transpose();
}

// 重投影误差
TEST(optimization, reprojection) {

}

// // 位姿图
// TEST(optimazation, pose) {}

int main(int argc, char const* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}
