#include "util.h"

#include "glog/logging.h"

PointCloud::Ptr generateCloud(const std::string& img_path,
                              const std::string& depth_path,const rgbd_camera& cam,
                              const std::string& save_path) {
  /* code */
  cv::Mat img = cv::imread(img_path);
  cv::Mat depth = cv::imread(depth_path);

  PointCloud::Ptr cloud{new PointCloud};
  for (size_t m = 0; m < depth.rows; m++) {
    for (size_t n = 0; n < depth.cols; n++) {
      uint16_t d = depth.ptr<uint16_t>(m)[n];
      if (d == 0) continue;
      PointT p;
      p.z = double(d) / cam.factor;
      p.x = (n - cam.cx) / cam.fx * p.z;
      p.y = (m - cam.cy) / cam.fy * p.z;
      // bgr
      p.b = img.ptr<uint8_t>(m)[n * 3];
      p.g = img.ptr<uint8_t>(m)[n * 3 + 1];
      p.r = img.ptr<uint8_t>(m)[n * 3 + 2];
      cloud->points.push_back(p);
    }
  }
  // 查看pcd标准
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;
  if (!save_path.empty()) {
    pcl::io::savePCDFile(save_path, *cloud);
  }

  // cloud->clear();
  return cloud;
}


bool point2dTo3d(const Eigen::Vector2d& uv, Eigen::Vector3d& result,
                 const cv::Mat& depth, const rgbd_camera& cam) {
  int m = uv(0, 0), n = uv(1, 0);
  if (depth.rows >= m || depth.cols >= n) return false;

  uint16_t d = depth.ptr<uint16_t>(m)[n];
  if (d == 0) return false;
  auto z = double(d) / cam.factor;
  result(2) = z;
  result(0) = (n - cam.cx) / cam.fx * z;
  result(1) = (m - cam.cy) / cam.fy * z;
  return true;
}