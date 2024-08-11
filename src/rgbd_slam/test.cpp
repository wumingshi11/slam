#include <glog/logging.h>
#include <gtest/gtest.h>

#include "util.h"

DEFINE_double(camera_factor, 100, "深度缩放因子");
DEFINE_double(camera_cx, 325.5, "x平移像素");
DEFINE_double(camera_cy, 325.5, "y平移像素");
DEFINE_double(camera_fx, 518.0, "x焦距");
DEFINE_double(camera_fy, 518.0, "y焦距");

DEFINE_string(rgb_image_path, "./data/rgbd_slam/data2/rgb.png", "");
DEFINE_string(depth_image_path, "./data/rgbd_slam/data2/depth.png", "");
DEFINE_string(result_path, "./data/rgbd_slam/data2/cloud.pcd", "");

TEST(rgbd_slam, generatePoint) {
  rgbd_camera cam{FLAGS_camera_fx, FLAGS_camera_fy, FLAGS_camera_cx,
                  FLAGS_camera_cy, FLAGS_camera_factor};

  auto cloud_ptr =
      generateCloud(FLAGS_rgb_image_path, FLAGS_depth_image_path, cam);
  pcl::io::savePCDFile(FLAGS_result_path, *cloud_ptr);
  cloud_ptr->clear();
}

int main(int argc, char const *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}
