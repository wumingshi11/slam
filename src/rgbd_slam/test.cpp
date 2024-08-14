#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include "util.h"

DEFINE_double(camera_factor, 100, "深度缩放因子");
DEFINE_double(camera_cx, 325.5, "x平移像素");
DEFINE_double(camera_cy, 325.5, "y平移像素");
DEFINE_double(camera_fx, 518.0, "x焦距");
DEFINE_double(camera_fy, 518.0, "y焦距");

DEFINE_string(rgb_image_path, "../../data/rgbd_slam/data2/rgb.png", "");
DEFINE_string(depth_image_path, "../../data/rgbd_slam/data2/depth.png", "");
DEFINE_string(result_path, "../../data/rgbd_slam/data2/cloud.pcd", "");

TEST(rgbd_slam, generatePoint) {
  rgbd_camera cam{FLAGS_camera_fx, FLAGS_camera_fy, FLAGS_camera_cx,
                  FLAGS_camera_cy, FLAGS_camera_factor};

  auto cloud_ptr =
      generateCloud(FLAGS_rgb_image_path, FLAGS_depth_image_path, cam);
  pcl::io::savePCDFile(FLAGS_result_path, *cloud_ptr);
  cloud_ptr->clear();
}

DEFINE_string(featureAndMatch_image1, "../../data/rgbd_slam/data3/rgb1.png",
              "");
DEFINE_string(featureAndMatch_image2, "../../data/rgbd_slam/data3/rgb2.png",
              "");
DEFINE_string(featureAndMatch_depth1, "../../data/rgbd_slam/data3/depth1.png",
              "");
DEFINE_string(featureAndMatch_depth2, "../../data/rgbd_slam/data3/depth2.png",
              "");
DEFINE_string(featureAndMatch_key, "../../data/rgbd_slam/data3/keypoints.png",
              "");
DEFINE_string(featureAndMatch_match, "../../data/rgbd_slam/data3/match.png",
              "");
DEFINE_string(featureAndMatch_good_match,
              "../../data/rgbd_slam/data3/good_match.png", "");
DEFINE_string(featureAndMatch_inlier_match,
              "../../data/rgbd_slam/data3/inlier_match.png", "");
DEFINE_bool(featureAndMatch_show, false, "是否显示");
TEST(rgbd_slam, featureAndMatch) {
  LOG(INFO) << "curr path : " << std::filesystem::current_path().string();
  cv::Mat img1 = cv::imread(FLAGS_featureAndMatch_image1);
  cv::Mat img2 = cv::imread(FLAGS_featureAndMatch_image2);
  cv::Mat depth1 = cv::imread(FLAGS_featureAndMatch_depth1, -1);
  cv::Mat depth2 = cv::imread(FLAGS_featureAndMatch_depth2, -1);

  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  cv::Mat descriptors1, descriptors2;
  orb_feature(img1, keypoints1, descriptors1);
  orb_feature(img2, keypoints2, descriptors2);
  LOG(INFO) << "keypoints1: " << keypoints1.size();
  LOG(INFO) << "keypoints2: " << keypoints2.size();
  LOG(INFO) << "descriptors1: " << descriptors1.rows << " "
            << descriptors1.cols;
  LOG(INFO) << "descriptors2: " << descriptors2.rows << " "
            << descriptors2.cols;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<cv::DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);
  LOG(INFO) << "matches: " << matches.size();
  cv::Mat img_match;
  cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_match);
  cv::imwrite(FLAGS_featureAndMatch_match, img_match);
  cv::Mat img_good_match;
  // 过滤，最大匹配距离为最小匹配的10倍
  std::vector<cv::DMatch> good_matches;
  double min_dist = 10000;
  for (int i = 0; i < matches.size(); i++) {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
  }
  for (int i = 0; i < matches.size(); i++) {
    if (matches[i].distance <= 10 * min_dist) {
      good_matches.push_back(matches[i]);
    }
  }
  cv::drawMatches(img1, keypoints1, img2, keypoints2, good_matches,
                  img_good_match);
  cv::imwrite(FLAGS_featureAndMatch_good_match, img_good_match);
  if (FLAGS_featureAndMatch_show) {
    cv::imshow("img_match", img_match);
    cv::waitKey(0);
    cv::imshow("img_good_match", img_good_match);
    cv::waitKey(0);
  }

  // 配准
  rgbd_camera cam{FLAGS_camera_fx, FLAGS_camera_fy, FLAGS_camera_cx,
                  FLAGS_camera_cy, FLAGS_camera_factor};
  // 第一个图像的三维点
  std::vector<cv::Point3f> pts1;
  // 第二个图像的二维点
  std::vector<cv::Point2f> pts2_img;
  // 匹配点信息
  LOG(INFO) << "good_matches: " << good_matches.size();
  std::vector<size_t> inliers_idx;
  for (int i = 0; i < good_matches.size(); i++) {
    cv::Point2f p1 = keypoints1[good_matches[i].queryIdx].pt;
    cv::Point3f p1_3d;
    p1_3d.x = p1.x;
    p1_3d.y = p1.y;
    p1_3d.z = depth1.at<ushort>(p1.y, p1.x);
    if (p1_3d.z == 0) continue;
    cv::Point2f p2 = keypoints2[good_matches[i].trainIdx].pt;
    pts1.push_back(point2dTo3d(p1_3d, cam));
    pts2_img.push_back(p2);
    inliers_idx.push_back(i);
  }
  LOG(INFO) << "feature num: " << pts1.size();
  // 求解pnp
  LOG(INFO) << "start pnp";
  double camera_matrix_data[3][3] = {
      {cam.fx, 0, cam.cx}, {0, cam.fy, cam.cy}, {0, 0, 1}};
  cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_data);
  double r[3] = {0, 0, 0};
  cv::Mat rvec = cv::Mat(3, 1, CV_64F, r);
  double t[3] = {0, 0, 0};
  cv::Mat tvec = cv::Mat{3, 1, CV_64F, t};
  cv::solvePnPRefineLM(pts1, pts2_img, camera_matrix, cv::Mat(), rvec, tvec);

  LOG(INFO) << "rvec: " << rvec;
  LOG(INFO) << "tvec: " << tvec;
  cv::Mat inliers, rvec2, tvec2;
  cv::solvePnPRansac(pts1, pts2_img, camera_matrix, cv::Mat(), rvec2, tvec2,
                     false, 100, 1.0, 0.95, inliers);
  LOG(INFO) << "inliers: " << inliers.rows;
  LOG(INFO) << "rvec2: " << rvec2;
  LOG(INFO) << "tvec2: " << tvec2;
  // 画出inliers匹配
  vector<cv::DMatch> matchesShow;
  for (size_t i = 0; i < inliers.rows; i++) {
    matchesShow.push_back(good_matches[inliers_idx[inliers.ptr<int>(i)[0]]]);
  }
  cv::Mat imgMatches;
  cv::drawMatches(img1, keypoints1, img2, keypoints2, matchesShow, imgMatches);

  cv::imwrite(FLAGS_featureAndMatch_inlier_match, imgMatches);

  if (FLAGS_featureAndMatch_show) {
    cv::imshow("inlier matches", imgMatches);
    cv::waitKey(0);
  }
}

DEFINE_string(merge_result_path, "../../data/rgbd_slam/data4/cloud.pcd", "");
// 合并点云测试
TEST(rgbd_slam, joinPointCloud) {
  rgbd_camera cam{FLAGS_camera_fx, FLAGS_camera_fy, FLAGS_camera_cx,
                  FLAGS_camera_cy, FLAGS_camera_factor};
  auto pc1 = generateCloud(FLAGS_featureAndMatch_image1,
                           FLAGS_featureAndMatch_depth1, cam);
  FRAME frame1;
  frame1.rgb = cv::imread(FLAGS_featureAndMatch_image1);
  frame1.depth = cv::imread(FLAGS_featureAndMatch_depth1, -1);
  computeKeyPointsAndDesp(frame1, "ORB", "ORB");
  FRAME frame2;
  frame2.rgb = cv::imread(FLAGS_featureAndMatch_image2);
  frame2.depth = cv::imread(FLAGS_featureAndMatch_depth2, -1);
  computeKeyPointsAndDesp(frame2, "ORB", "ORB");
  RESULT_OF_PNP result = estimateMotion(frame1, frame2, cam);
  Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
  auto pc2 = joinPointCloud(pc1, frame2, T, cam);
  pcl::io::savePCDFile(FLAGS_merge_result_path, *pc2);
}

// 多张图像点云合并
DEFINE_string(merge_mult_img_result_path,
              "../../data/rgbd_slam/data5/cloud.pcd", "");
TEST(rbgd_slam, merge_mult_img) {
  LOG(INFO) << "merge_mult_img start";
  auto& params = ParameterReader::getInstance();
  rgbd_camera cam{FLAGS_camera_fx, FLAGS_camera_fy, FLAGS_camera_cx,
                  FLAGS_camera_cy, FLAGS_camera_factor};
  auto start_index = std::atoi(params.getData("start_index").c_str());
  auto end_index = std::atoi(params.getData("end_index").c_str());
  auto current_index = start_index;
  auto rgb_path = params.getData("rgb_path");
  auto depth_path = params.getData("depth_path");
  auto rgb_type = params.getData("rgb_extension");
  auto depth_type = params.getData("depth_extension");
  auto max_norm = std::atof(params.getData("max_norm").c_str());
  auto last_frame =
      readFrame(current_index, rgb_path, depth_path, rgb_type, depth_type);
  computeKeyPointsAndDesp(last_frame, "ORB", "ORB");
  auto point_cloud = generateCloud(last_frame.rgb, last_frame.depth, cam);
  size_t inlier_small_fram_num = 0, t_big_fram_num = 0;
  for (++current_index; current_index <= end_index; current_index++) {
    auto start_time = std::chrono::system_clock::now();
    auto curr_frame =
        readFrame(current_index, rgb_path, depth_path, rgb_type, depth_type);
    computeKeyPointsAndDesp(curr_frame, "ORB", "ORB");
    auto result = estimateMotion(curr_frame, last_frame, cam);
    if (result.inliers < 5) {
      LOG(INFO) << "Not enough inliers: " << result.inliers;
      inlier_small_fram_num++;
      continue;
    }
    if (normofTransform(result.rvec, result.tvec) > max_norm) {
      LOG(INFO) << "Transfor too big: "
                << normofTransform(result.rvec, result.tvec);
      t_big_fram_num++;
      continue;
    }

    auto T = cvMat2Eigen(result.rvec, result.tvec);
    point_cloud = joinPointCloud(point_cloud, curr_frame, T, cam);
    last_frame = curr_frame;
    auto end_time = std::chrono::system_clock::now();
    LOG(INFO) << current_index << " time cost "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     end_time - start_time)
                     .count();
    LOG(INFO) << inlier_small_fram_num << " " << t_big_fram_num;
  }
  pcl::io::savePCDFile(FLAGS_merge_mult_img_result_path, *point_cloud);
}

// 测试离线回环检测
TEST(rgbd_slam, loop_closure) {
  // 前面部分和vo是一样的
  ParameterReader& params = ParameterReader::getInstance();
  int startIndex = atoi(params.getData("start_index").c_str());
  int endIndex = atoi(params.getData("end_index").c_str());
  auto rgb_path = params.getData("rgb_path");
  auto depth_path = params.getData("depth_path");
  auto rgb_type = params.getData("rgb_extension");
  auto depth_type = params.getData("depth_extension");
  // 所有的关键帧都放在了这里
  vector<FRAME> keyframes;
  // initialize
  LOG(INFO) << "Initializing ...";
  int currIndex = startIndex;  // 当前索引为currIndex
  FRAME currFrame = readFrame(currIndex, rgb_path, depth_path, rgb_type,
                              depth_type);  // 上一帧数据

  string detector = "ORB";
  string descriptor = "ORB";
  rgbd_camera camera{FLAGS_camera_fx, FLAGS_camera_fy, FLAGS_camera_cx,
                     FLAGS_camera_cy, FLAGS_camera_factor};
  computeKeyPointsAndDesp(currFrame, detector, descriptor);
  PointCloud::Ptr cloud = generateCloud(currFrame.rgb, currFrame.depth, camera);

  /*******************************
  // 新增:有关g2o的初始化
  *******************************/
  // 初始化求解器
  // 把g2o的定义放到前面
  typedef g2o::BlockSolver_6_3 SlamBlockSolver;
  typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>
      SlamLinearSolver;
  std::unique_ptr<SlamLinearSolver> linearSolver =
      std::make_unique<g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>>();
  linearSolver->setBlockOrdering(false);
  std::unique_ptr<SlamBlockSolver> blockSolver =
      std::unique_ptr<SlamBlockSolver>(new SlamBlockSolver(std::move(linearSolver)));
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
  globalOptimizer.setAlgorithm(solver);
  // 输出调试信息
  globalOptimizer.setVerbose(true);

  // 向globalOptimizer增加第一个顶点
  g2o::VertexSE3* v = new g2o::VertexSE3();
  v->setId(currIndex);
  v->setEstimate(Eigen::Isometry3d::Identity());  //估计为单位矩阵
  v->setFixed(true);  //第一个顶点固定，不用优化
  globalOptimizer.addVertex(v);

  keyframes.push_back(currFrame);

  double keyframe_threshold = atof(params.getData("keyframe_threshold").c_str());
  bool check_loop_closure = params.getData("check_loop_closure") == string("yes");

  for (currIndex = startIndex + 1; currIndex < endIndex; currIndex++) {
    cout << "Reading files " << currIndex << endl;
    FRAME currFrame = readFrame(currIndex, rgb_path, depth_path, rgb_type,
                                depth_type);  // 读取currFrame
    computeKeyPointsAndDesp(currFrame, detector, descriptor);  //提取特征
    CHECK_RESULT result =
        checkKeyframes(keyframes.back(), currFrame, globalOptimizer,
                       camera);  //匹配该帧与keyframes里最后一帧
    switch (result)              // 根据匹配结果不同采取不同策略
    {
      case NOT_MATCHED:
        //没匹配上，直接跳过
        cout << "Not enough inliers." << endl;
        break;
      case TOO_FAR_AWAY:
        // 太近了，也直接跳
        cout << "Too far away, may be an error." << endl;
        break;
      case TOO_CLOSE:
        // 太远了，可能出错了
        cout << "Too close, not a keyframe" << endl;
        break;
      case KEYFRAME:
        cout << "This is a new keyframe" << endl;
        // 不远不近，刚好
        /**
         * This is important!!
         * This is important!!
         * This is important!!
         * (very important so I've said three times!)
         */
        // 检测回环
        if (check_loop_closure) {
          checkNearbyLoops(keyframes, currFrame, globalOptimizer, camera);
          checkRandomLoops(keyframes, currFrame, globalOptimizer, camera);
        }

        keyframes.push_back(currFrame);

        break;
      default:
        break;
    }
  }

  // 优化
  cout << "optimizing pose graph, vertices: "
       << globalOptimizer.vertices().size() << endl;
  globalOptimizer.save("./result_before.g2o");
  globalOptimizer.initializeOptimization();
  globalOptimizer.optimize(100);  //可以指定优化步数
  globalOptimizer.save("./result_after.g2o");
  cout << "Optimization done." << endl;

  // 拼接点云地图
  cout << "saving the point cloud map..." << endl;
  PointCloud::Ptr output(new PointCloud());  //全局地图
  PointCloud::Ptr tmp(new PointCloud());

  pcl::VoxelGrid<PointT> voxel;  // 网格滤波器，调整地图分辨率
  pcl::PassThrough<PointT>
      pass;  // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 4.0);  // 4m以上就不要了

  double gridsize =
      atof(params.getData("voxel_grid").c_str());  //分辨图可以在parameters.txt里调
  voxel.setLeafSize(gridsize, gridsize, gridsize);

  for (size_t i = 0; i < keyframes.size(); i++) {
    // 从g2o里取出一帧
    g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(
        globalOptimizer.vertex(keyframes[i].frameID));
    Eigen::Isometry3d pose = vertex->estimate();  //该帧优化后的位姿
    PointCloud::Ptr newCloud =
        generateCloud(keyframes[i].rgb, keyframes[i].depth, camera);  //转成点云
    // 以下是滤波
    voxel.setInputCloud(newCloud);
    voxel.filter(*tmp);
    pass.setInputCloud(tmp);
    pass.filter(*newCloud);
    // 把点云变换后加入全局地图中
    pcl::transformPointCloud(*newCloud, *tmp, pose.matrix());
    *output += *tmp;
    tmp->clear();
    newCloud->clear();
  }

  voxel.setInputCloud(output);
  voxel.filter(*tmp);
  //存储
  pcl::io::savePCDFile("./result.pcd", *tmp);

  cout << "Final map is saved." << endl;
}

int main(int argc, char const* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}
