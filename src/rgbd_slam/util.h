#include <gflags/gflags.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
using namespace std;

using PointT = pcl::PointXYZRGBA;
using PointCloud = pcl::PointCloud<PointT>;

struct camera {
  camera(double fx, double fy, double cx, double cy)
      : fx(fx), fy(fy), cx(cx), cy(cy) {}
  camera() {}
  // 焦距
  double fx, fy;  // 焦距
  double cx, cy;  // 平移
};

struct rgbd_camera : public camera {
  rgbd_camera(double fx, double fy, double cx, double cy, double factor)
      : camera(fx, fy, cx, cy), factor(factor) {}
  double factor{1.0};  // 深度缩放因子
};

PointCloud::Ptr generateCloud(const std::string& img_path,
                              const std::string& depth_path,
                              const rgbd_camera& cam,
                              const std::string& save_path = "");

/// @brief 根据深度图，将像素坐标转为xyz(相机坐标系)
/// @param uv 像素位置
/// @param result  xyz
/// @param depth 深度图
/// @param factor 深度缩放因子
/// @return 是否成功
bool point2dTo3d(const Eigen::Vector2d& uv, Eigen::Vector3d& result,
                 const cv::Mat& depth, const rgbd_camera& cam);
