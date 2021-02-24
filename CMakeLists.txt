cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_tools)

add_definitions(-std=c++11)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  tf
  tf_conversions
)


add_service_files(
  DIRECTORY srv
  FILES
  MeshToCloudPose.srv
  )


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  )


catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)

add_library(transformation_analyzation src/transformation_analyzation.cpp)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ICPPoseDemo src/ICPPoseDemo.cpp)
target_link_libraries(ICPPoseDemo ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ICPMeshtoCloudSrv src/icp_mesh_to_cloud_pose_server.cpp)
target_link_libraries(ICPMeshtoCloudSrv transformation_analyzation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
