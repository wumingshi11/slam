add_executable(testopencv TestOpencv.cpp)
target_link_libraries(testopencv GTest::GTest glog::glog ${OpenCV_LIBS})