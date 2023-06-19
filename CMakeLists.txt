cmake_minimum_required(VERSION 3.10)
project(intensity_feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE " -O3 -Wall -g -pthread -DROSCONSOLE_MIN_SEVERITY=5")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  cv_bridge
  roscpp
  roslib
  sensor_msgs
  std_msgs
  pcl_conversions
  tf
  tf2 
  tf2_ros
  message_filters
)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib message_runtime geometry_msgs nav_msgs std_msgs
  DEPENDS PCL EIGEN3
  INCLUDE_DIRS include
)
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)
link_directories(
    inlcude
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
)

file(GLOB source_files
    src/intensity_feature_tracker_node.cpp
    src/intensity_feature_tracker.cpp
    src/loop_closure_handler.cpp
    src/Scancontext.cpp
    src/parameters.h
)

add_executable(${PROJECT_NAME}_node ${source_files})
target_compile_options(${PROJECT_NAME}_node PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_node ${CERES_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${DBoW3_LIBS} Eigen3::Eigen gtsam)

file(GLOB map_source_files
    src/mapOptimizationNode.cpp
    src/mapOptimization.cpp    
    src/image_handler.h
    src/ikd-Tree/ikd_Tree.cpp
    ikd-Tree/ikd_Tree.h
    src/parameters.h

)
add_executable(${PROJECT_NAME}_mapping_node ${map_source_files})
target_compile_options(${PROJECT_NAME}_mapping_node PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapping_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} Eigen3::Eigen gtsam ${CERES_LIBRARIES} ${OpenMP_LIBS})

add_executable(ascanRegistration src/scanRegistration.cpp src/intensity_feature_tracker.cpp
src/loop_closure_handler.cpp
src/Scancontext.cpp
src/parameters.h)
target_compile_options(ascanRegistration PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} Eigen3::Eigen ${DBoW3_LIBS} gtsam )

add_executable(alaserMapping_node src/laserMapping.cpp)
target_link_libraries(alaserMapping_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserOdometry_node src/laserOdometry.cpp)
target_link_libraries(alaserOdometry_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(odomHandler_node src/odom_handler_node.cpp)
target_link_libraries(odomHandler_node ${catkin_LIBRARIES})

