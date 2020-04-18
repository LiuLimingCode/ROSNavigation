cmake_minimum_required(VERSION 2.8)
project(pure_mapping)

find_package(catkin REQUIRED nav_msgs openslam_gmapping roscpp tf rosbag_storage)

find_package(Boost REQUIRED signals)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(src)

catkin_package()

add_executable(pure_mapping src/pure_mapping.cpp src/main.cpp)
target_link_libraries(pure_mapping ${Boost_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(pure_mapping ${catkin_EXPORTED_TARGETS})
endif()
