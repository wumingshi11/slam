add_executable(testOptimization testOptimization.cpp)
target_link_libraries(testOptimization GTest::GTest glog::glog ${g2o_libs})