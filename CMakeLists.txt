cmake_minimum_required(VERSION 3.0.2)
project(ros_icp_tutorial)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf 
  tf2 geometry_msgs
)
catkin_package()
add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(example_voxelgrid src/example_voxelgrid.cpp)
target_link_libraries(example_voxelgrid ${catkin_LIBRARIES})

add_executable(pcl_shptr_1 src/pcl_shptr_1.cpp)
target_link_libraries(pcl_shptr_1 ${catkin_LIBRARIES})

add_executable(pcl_shptr_2 src/pcl_shptr_2.cpp)
target_link_libraries(pcl_shptr_2 ${catkin_LIBRARIES})

add_executable(pcl_shptr_3 src/pcl_shptr_3.cpp)
target_link_libraries(pcl_shptr_3 ${catkin_LIBRARIES})

add_executable(transform src/transform.cpp)
target_link_libraries(transform ${catkin_LIBRARIES})

add_executable(pcd_read src/pcd_read.cpp)
target_link_libraries(pcd_read ${catkin_LIBRARIES})

add_executable(pcd_write src/pcd_write.cpp)
target_link_libraries(pcd_write ${catkin_LIBRARIES})

add_executable(downsampling src/downsampling.cpp)
target_link_libraries(downsampling ${catkin_LIBRARIES})

add_executable(uniformsampling src/uniformsampling.cpp)
target_link_libraries(uniformsampling ${catkin_LIBRARIES})

add_executable(upsampling src/upsampling.cpp)
target_link_libraries(upsampling ${catkin_LIBRARIES})

add_executable(passthrough src/passthrough.cpp)
target_link_libraries(passthrough ${catkin_LIBRARIES})

add_executable(noise src/noise.cpp)
target_link_libraries(noise ${catkin_LIBRARIES})

add_executable(ransac src/ransac.cpp)
target_link_libraries(ransac ${catkin_LIBRARIES})

add_executable(kdtree src/kdtree.cpp)
target_link_libraries(kdtree ${catkin_LIBRARIES})

add_executable(icp src/icp.cpp)
target_link_libraries(icp ${catkin_LIBRARIES})

add_executable(test_icp src/test_icp.cpp)
target_link_libraries(test_icp ${catkin_LIBRARIES})

add_executable(test_gicp src/test_gicp.cpp)
target_link_libraries(test_gicp ${catkin_LIBRARIES})

include_directories(
  ${catkin_INCLUDE_DIRS}
)
