add_library(slam_base SHARED util.cpp)
target_link_libraries(slam_base glog::glog ${g2o_libs} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
add_executable(rgbdslamTest test.cpp )
target_link_libraries(rgbdslamTest GTest::GTest slam_base)