#include <glog/logging.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

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

int main(int argc, char const *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}
