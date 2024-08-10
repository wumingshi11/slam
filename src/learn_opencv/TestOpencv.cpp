#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
//#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/se3.hpp>

DEFINE_string(IMAGE_PATH, "./data/learn_opencv/image.png", "image path");
DEFINE_string(IMAGE_LEFT, "./data/learn_opencv/left.png",
              "depth estimate left image path");
DEFINE_string(IMAGE_RIGHT, "./data/learn_opencv/right.png",
              "depth estimate right image path");

TEST(opencv, read) {
  cv::Mat imags;
  imags = cv::imread(FLAGS_IMAGE_PATH);
  if (imags.data == nullptr) {
    LOG(ERROR) << FLAGS_IMAGE_PATH << " is null data";
  }
  LOG(INFO) << "col_num,row_num,chan_num : " << imags.cols << "," << imags.rows
            << "," << imags.channels();
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  uint8_t tmp = 0;
  for (size_t y = 0; y < imags.rows; y++) {
    unsigned char* row_ptr = imags.ptr<unsigned char>(y);
    for (size_t x = 0; x < imags.cols; x++) {
      for (size_t i = 0; i < imags.channels(); i++) {
        tmp = row_ptr[x * imags.channels() + i];
      }
    }
  }
  auto now2 = std::chrono::steady_clock::now();
  auto du =
      std::chrono::duration_cast<std::chrono::duration<double>>(now2 - now);
  LOG(INFO) << du.count();
  // 最后一个像素相同
  EXPECT_EQ(tmp, imags.at<cv::Vec3b>(imags.rows - 1, imags.cols - 1)[2]);
}

TEST(opencv, depth) {
  constexpr double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
  constexpr double b = 0.573;
  cv::Mat img_left = cv::imread(FLAGS_IMAGE_LEFT);
  cv::Mat img_right = cv::imread(FLAGS_IMAGE_RIGHT);
  cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
      0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
  cv::Mat disparity_sgbm, disparity;
  sgbm->compute(img_left, img_right, disparity_sgbm);
  disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

  // 生成点云
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      pointcould;
  for (size_t v = 0; v < img_left.rows; v++) {
    for (size_t u = 0; u < img_left.cols; u++) {
      if (disparity.at<float>(u, v) <= 10.0 ||
          disparity.at<float>(u, v) >= 96.0) {
        continue;
      }
      Eigen::Vector4d point(0,0,0,img_left.at<uchar>(v,u)/255.0);
      // 反算x,y,z
      double x = (u - cx)/fx;
      double y = (v - cy)/fy;
      double depth = fx * b/(disparity.at<float>(v,u));
      point[0] = x * depth;
      point[1] = y * depth;
      point[2] = depth;
      pointcould.push_back(point);
    }
  }
}

int main(int argc, char const* argv[]) {
  /* code */
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}
