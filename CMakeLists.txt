# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 FATAL_ERROR )

# 声明一个 cmake 工程
PROJECT(SLAM)

# 设置编译模式
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")

find_package( OpenCV REQUIRED )

#导入PCL库
find_package( PCL 1.7 REQUIRED )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS} include)

# #导入Sophus
# find_package( Sophus REQUIRED )
# include_directories( ${Sophus_INCLUDE_DIRS} )

# add find modules rules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/lib/cmake_modules)

# 导入g2o
set(G2O_ROOT "/usr/local/include/g2o")
FIND_PACKAGE(G2O REQUIRED)
IF(G2O_FOUND)
  MESSAGE(STATUS ${G2O_INCLUDE_DIR})
ELSE(G2O_FOUND)
  MESSAGE(FATAL_ERROR "g2o not FOUND.")
ENDIF(G2O_FOUND)

# Eigen3
FIND_PACKAGE(Eigen3)
IF(EIGEN3_FOUND)
  MESSAGE(STATUS ${EIGEN3_INCLUDE_DIR})
ELSE(EIGEN3_FOUND)
  MESSAGE(FATAL_ERROR "EIGEN3 not FOUND.")
ENDIF(EIGEN3_FOUND)

# CSparse
FIND_PACKAGE(CSparse)
IF(CSPARSE_FOUND)
  SET(BUILD_CSPARSE OFF CACHE BOOL "Build local CSparse library")
ELSE(CSPARSE_FOUND)
  SET(BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")
  IF(BUILD_CSPARSE)
    SET(CSPARSE_FOUND TRUE)
  ENDIF()
ENDIF(CSPARSE_FOUND)

#添加包含文件
include_directories(mylib)
include_directories(include)

INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

FILE(GLOB LIB_SRCS src/Lidar3dIndoor/*.cpp)
add_subdirectory(lib)

FILE(GLOB SRCS src/*.cpp)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") #不加在某些pcl环境下编译时可能会报错
add_executable( ../mapping ${SRCS})

#动态链接库
link_directories(${PROJECT_SOURCE_DIR}/lib)
target_link_libraries( ../mapping Lidar3dIndoor )
target_link_libraries( ../mapping ${PCL_LIBRARIES} ${LIBS} -lpcap)
target_link_libraries( ../mapping
  ${OpenCV_LIBS} ${Sophus_LIBRARIES} g2o_core pthread ${PCL_LIBRARIES} g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY}  -lpcap
)

