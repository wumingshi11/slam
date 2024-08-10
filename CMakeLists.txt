cmake_minimum_required(VERSION 3.15)
project(slam)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR})

# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
message(STATUS ${EIGEN3_INCLUDE_DIRS})

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# cholmod
# find_package(Cholmod REQUIRED)
# include_directories(${CHOLMOD_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(GTest REQUIRED)
include_directories(${GTest_INCLUDE_DIRS})

find_package(GLOG REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# # 查找 g2o 库
# find_package(g2o REQUIRED)
# include_directories(${G2O_INCLUDE_DIRS})

# # 检查 g2o 是否找到
# if(g2o_FOUND)
#     message(STATUS "g2o library found")
#     message(STATUS "g2o header : ${g2o_INCLUDE_DIRS}")
# else()
#     message(FATAL_ERROR "g2o library not found")
# endif()

# g2o 使用thirdparty中的
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/g2o/)
set(g2o_libs
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_stuff.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_core.so
	# ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_cholmod.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_dense.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_csparse.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_csparse_extension.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_types_sba.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_types_slam2d.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_types_slam3d.so
        ${CSPARSE_LIBRARY}
        ${CHOLMOD_LIBRARY}
        )


add_subdirectory(src)