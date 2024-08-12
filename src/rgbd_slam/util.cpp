#include "util.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "glog/logging.h"

PointCloud::Ptr generateCloud(const std::string& img_path,
                              const std::string& depth_path,
                              const rgbd_camera& cam,
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
      p.z = double(d) / cam.scale;
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

bool point2dTo3d(const Eigen::Vector3d& uvd, Eigen::Vector3d& result,
                 const rgbd_camera& cam) {
  int m = uvd(0, 0), n = uvd(1, 0);
  auto z = uvd(3, 0) / cam.scale;
  result(2) = z;
  result(0) = (n - cam.cx) / cam.fx * z;
  result(1) = (m - cam.cy) / cam.fy * z;
  return true;
}

cv::Point3f point2dTo3d(cv::Point3f& point, rgbd_camera& camera) {
  cv::Point3f p;  // 3D 点
  p.z = double(point.z) / camera.scale;
  p.x = (point.x - camera.cx) * p.z / camera.fx;
  p.y = (point.y - camera.cy) * p.z / camera.fy;
  return p;
}

void orb_feature(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors) {
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  // cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  detector->detect(img, keypoints);
  detector->compute(img, keypoints, descriptors);

  LOG(INFO) << "detect " << keypoints.size() << " points";
  LOG(INFO) << "descriptors size " << descriptors.size();
}

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp(FRAME& frame, string detector1,
                             string descriptor1) {
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  if (detector.empty()) {
    LOG(ERROR) << "detector is empty!";
    return;
  }
  detector->detect(frame.rgb, frame.kp);
  detector->compute(frame.rgb, frame.kp, frame.desp);
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2,
                             rgbd_camera& camera) {
  static ParameterReader pd;
  vector<cv::DMatch> matches;
  cv::BFMatcher matcher;
  matcher.match(frame1.desp, frame2.desp, matches);

  cout << "find total " << matches.size() << " matches." << endl;
  vector<cv::DMatch> goodMatches;
  double minDis = 9999;
  double good_match_threshold =
      atof(pd.getData("good_match_threshold").c_str());
  for (size_t i = 0; i < matches.size(); i++) {
    if (matches[i].distance < minDis) minDis = matches[i].distance;
  }

  for (size_t i = 0; i < matches.size(); i++) {
    if (matches[i].distance < good_match_threshold * minDis)
      goodMatches.push_back(matches[i]);
  }

  cout << "good matches: " << goodMatches.size() << endl;
  // 第一个帧的三维点
  vector<cv::Point3f> pts_obj;
  // 第二个帧的图像点
  vector<cv::Point2f> pts_img;

  // 相机内参
  for (size_t i = 0; i < goodMatches.size(); i++) {
    // query 是第一个, train 是第二个
    cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
    // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
    ushort d = frame1.depth.ptr<ushort>(int(p.y))[int(p.x)];
    if (d == 0) continue;
    pts_img.push_back(cv::Point2f(frame2.kp[goodMatches[i].trainIdx].pt));

    // 将(u,v,d)转成(x,y,z)
    cv::Point3f pt(p.x, p.y, d);
    cv::Point3f pd = point2dTo3d(pt, camera);
    pts_obj.push_back(pd);
  }

  double camera_matrix_data[3][3] = {
      {camera.fx, 0, camera.cx}, {0, camera.fy, camera.cy}, {0, 0, 1}};

  LOG(INFO) << "solving pnp";
  // 构建相机矩阵
  cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
  cv::Mat rvec, tvec, inliers;
  // 求解pnp
  cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec,
                     false, 100, 1.0, 100, inliers);

  RESULT_OF_PNP result;
  result.rvec = rvec;
  result.tvec = tvec;
  result.inliers = inliers.rows;

  return result;
}

Eigen::Isometry3d cvMat2Eigen(const cv::Mat& rvec, const cv::Mat& tvec) {
  auto result = Eigen::Isometry3d::Identity();
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d R_eigen;
  cv::cv2eigen(R, R_eigen);
  Eigen::Vector3d t_eigen;
  cv::cv2eigen(tvec, t_eigen);
  result.rotate(R_eigen);
  result.pretranslate(t_eigen);
  return result;
}

PointCloud::Ptr mergePointCloud(PointCloud::Ptr cloud1, PointCloud::Ptr cloud2,
                                Eigen::Isometry3d T) {
  PointCloud::Ptr result{new PointCloud};
  pcl::transformPointCloud(*cloud1, *result, T.matrix());
  *result += *cloud2;
  // 使用体素法降采样点
  pcl::VoxelGrid<PointT> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(0.01, 0.01, 0.01);
  PointCloud::Ptr filtered_cloud;
  voxel_grid_filter.setInputCloud(result);
  voxel_grid_filter.filter(*filtered_cloud);
  return filtered_cloud;
}

PointCloud::Ptr joinPointCloud(PointCloud::Ptr cloud1, FRAME& frame2,
                               Eigen::Isometry3d T, rgbd_camera& camera) {
  PointCloud::Ptr newCloud =
      image2PointCloud(newFrame.rgb, newFrame.depth, camera);
  return mergePointCloud(cloud1, newCloud, T);
}
