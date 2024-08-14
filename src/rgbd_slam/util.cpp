#include "util.h"

#include <pcl/filters/voxel_grid.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "glog/logging.h"

PointCloud::Ptr generateCloud(const std::string& img_path,
                              const std::string& depth_path,
                              const rgbd_camera& cam,
                              const std::string& save_path) {
  /* code */
  cv::Mat img = cv::imread(img_path);
  cv::Mat depth = cv::imread(depth_path);
  auto cloud = generateCloud(img, depth, cam);
  if (!save_path.empty()) {
    pcl::io::savePCDFile(save_path, *cloud);
  }
  // cloud->clear();
  return cloud;
}

PointCloud::Ptr generateCloud(const cv::Mat& img, const cv::Mat& depth,
                              const rgbd_camera& cam) {
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
void computeKeyPointsAndDesp(FRAME& frame, const std::string& detector1,
                             const std::string& descriptor1) {
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
  RESULT_OF_PNP result;
  result.inliers = 0;
  static ParameterReader pd;
  vector<cv::DMatch> matches;
  cv::BFMatcher matcher;
  matcher.match(frame1.desp, frame2.desp, matches);

  cout << "find total " << matches.size() << " matches." << endl;
  vector<cv::DMatch> goodMatches;
  double minDis = 9999, min2 = 9999;
  double good_match_threshold = atof(pd.getData("min_good_match").c_str());
  for (size_t i = 0; i < matches.size(); i++)
    if (matches[i].distance < minDis) {
      min2 = minDis;
      minDis = matches[i].distance;
    }
  minDis = min2;
  if (minDis < 10) minDis = 10;  // 防止特别小的点

  for (size_t i = 0; i < matches.size(); i++) {
    if (matches[i].distance < 6 * minDis) goodMatches.push_back(matches[i]);
  }

  cout << "good matches: " << goodMatches.size() << endl;
  if (goodMatches.size() < good_match_threshold) {
    result.inliers = 0;
    return result;
  }
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
  if (pts_img.size() < 4) return result;

  double camera_matrix_data[3][3] = {
      {camera.fx, 0, camera.cx}, {0, camera.fy, camera.cy}, {0, 0, 1}};

  LOG(INFO) << "solving pnp";
  // 构建相机矩阵
  cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
  cv::Mat rvec, tvec, inliers;
  // 求解pnp
  cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec,
                     false, 100, 5.0, 0.9, inliers);

  // RESULT_OF_PNP result;
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
  voxel_grid_filter.setLeafSize(0.05, 0.05, 0.05);
  PointCloud::Ptr filtered_cloud{new PointCloud};
  voxel_grid_filter.setInputCloud(result);
  voxel_grid_filter.filter(*filtered_cloud);
  return filtered_cloud;
}

PointCloud::Ptr joinPointCloud(PointCloud::Ptr cloud1, FRAME& frame2,
                               Eigen::Isometry3d T, rgbd_camera& camera) {
  PointCloud::Ptr newCloud = generateCloud(frame2.rgb, frame2.depth, camera);
  return mergePointCloud(cloud1, newCloud, T);
}

FRAME readFrame(int index, const std::string& rgb_path,
                const std::string& depth_file_path, const std::string& rgb_type,
                const std::string& depth_type) {
  // TODO 路径检测
  FRAME result;
  result.rgb = cv::imread(rgb_path + "/" + std::to_string(index) + rgb_type);
  result.depth =
      cv::imread(depth_file_path + "/" + std::to_string(index) + depth_type,
                 cv::IMREAD_UNCHANGED);
  result.frameID = index;

  return result;
}

CHECK_RESULT checkKeyframes(FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti,
                            rgbd_camera& camera, bool is_loops) {
  ParameterReader& pd = ParameterReader::getInstance();
  static int min_inliers = atoi(pd.getData("min_inliers").c_str());
  static double max_norm = atof(pd.getData("max_norm").c_str());
  static double keyframe_threshold =
      atof(pd.getData("keyframe_threshold").c_str());
  static double max_norm_lp = atof(pd.getData("max_norm_lp").c_str());
  // static CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
  //  比较f1 和 f2
  RESULT_OF_PNP result = estimateMotion(f1, f2, camera);
  if (result.inliers < min_inliers)  // inliers不够，放弃该帧
    return NOT_MATCHED;
  // 计算运动范围是否太大
  double norm = normofTransform(result.rvec, result.tvec);
  if (is_loops == false) {
    if (norm >= max_norm) return TOO_FAR_AWAY;  // too far away, may be error
  } else {
    if (norm >= max_norm_lp) return TOO_FAR_AWAY;
  }

  if (norm <= keyframe_threshold) return TOO_CLOSE;  // too adjacent frame
  // 向g2o中增加这个顶点与上一帧联系的边
  // 顶点部分
  // 顶点只需设定id即可
  if (is_loops == false) {
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(f2.frameID);
    v->setEstimate(Eigen::Isometry3d::Identity());
    opti.addVertex(v);
  }
  // 边部分
  g2o::EdgeSE3* edge = new g2o::EdgeSE3();
  // 连接此边的两个顶点id
  edge->setVertex(0, opti.vertex(f1.frameID));
  edge->setVertex(1, opti.vertex(f2.frameID));
  edge->setRobustKernel(new g2o::RobustKernelHuber());
  // 信息矩阵
  Eigen::Matrix<double, 6, 6> information =
      Eigen::Matrix<double, 6, 6>::Identity();
  // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
  // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
  // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
  information(0, 0) = information(1, 1) = information(2, 2) = 100;
  information(3, 3) = information(4, 4) = information(5, 5) = 100;
  // 也可以将角度设大一些，表示对角度的估计更加准确
  edge->setInformation(information);
  // 边的估计即是pnp求解之结果
  Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
  // edge->setMeasurement( T );
  edge->setMeasurement(T.inverse());
  // 将此边加入图中
  opti.addEdge(edge);
  return KEYFRAME;
}

void checkNearbyLoops(vector<FRAME>& frames, FRAME& currFrame,
                      g2o::SparseOptimizer& opti, rgbd_camera& camera) {
  static ParameterReader pd;
  static int nearby_loops = atoi(pd.getData("nearby_loops").c_str());

  // 就是把currFrame和 frames里末尾几个测一遍
  if (frames.size() <= nearby_loops) {
    // no enough keyframes, check everyone
    for (size_t i = 0; i < frames.size(); i++) {
      checkKeyframes(frames[i], currFrame, opti, camera, true);
    }
  } else {
    // check the nearest ones
    for (size_t i = frames.size() - nearby_loops; i < frames.size(); i++) {
      checkKeyframes(frames[i], currFrame, opti, camera, true);
    }
  }
}

void checkRandomLoops(vector<FRAME>& frames, FRAME& currFrame,
                      g2o::SparseOptimizer& opti, rgbd_camera& camera) {
  static ParameterReader pd;
  static int random_loops = atoi(pd.getData("random_loops").c_str());
  // srand((unsigned int)time(NULL));
  //  随机取一些帧进行检测

  if (frames.size() <= random_loops) {
    // no enough keyframes, check everyone
    for (size_t i = 0; i < frames.size(); i++) {
      checkKeyframes(frames[i], currFrame, opti, camera, true);
    }
  } else {
    // randomly check loops
    for (int i = 0; i < random_loops; i++) {
      int index = rand() % frames.size();
      checkKeyframes(frames[index], currFrame, opti, camera, true);
    }
  }
}
