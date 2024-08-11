#include <gflags/gflags.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <iostream>
#include <map>
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
      : camera(fx, fy, cx, cy), scale(factor) {}
  double scale{1.0};  // 深度缩放因子
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
bool point2dTo3d(const Eigen::Vector3d& uvd, Eigen::Vector3d& result,
                 const rgbd_camera& cam);
cv::Point3f point2dTo3d(cv::Point3f& point, rgbd_camera& camera);

// 计算图像orb特征和描述子
void orb_feature(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors);

// 帧结构
struct FRAME {
  cv::Mat rgb, depth;       //该帧对应的彩色图与深度图
  cv::Mat desp;             //特征描述子
  vector<cv::KeyPoint> kp;  //关键点
};

// PnP 结果
struct RESULT_OF_PNP {
  cv::Mat rvec, tvec;
  int inliers;
};

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp(FRAME& frame, const std::string& detector,
                             const std::string& descriptor);

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, rgbd_camera& camera);

// 参数读取类
class ParameterReader {
 public:
  ParameterReader(std::string filename = "./parameters.txt") {
    std::ifstream fin(filename.c_str());
    if (!fin) {
      cerr << "parameter file does not exist." << std::endl;
      return;
    }
    while (!fin.eof()) {
      std::string str;
      std::getline(fin, str);
      if (str[0] == '#') {
        // 以‘＃’开头的是注释
        continue;
      }

      int pos = str.find("=");
      if (pos == -1) continue;
      string key = str.substr(0, pos);
      string value = str.substr(pos + 1, str.length());
      data[key] = value;

      if (!fin.good()) break;
    }
  }
  string getData(string key) {
    std::map<std::string, std::string>::iterator iter = data.find(key);
    if (iter == data.end()) {
      std::cerr << "Parameter name " << key << " not found!" << std::endl;
      return string("NOT_FOUND");
    }
    return iter->second;
  }

 public:
  std::map<string, string> data;
};
