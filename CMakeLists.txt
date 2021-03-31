cmake_minimum_required(VERSION 2.8)
project(mie443_contest3)

add_compile_options(-std=c++11)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	std_msgs
	image_transport
	sensor_msgs
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  roslib
  std_msgs
  tf
  visualization_msgs
	kobuki_msgs)




add_message_files(
  FILES
  EmotionMsg.msg
  EmotionFaceMsg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  kobuki_msgs
)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS message_runtime roscpp std_msgs roslib)

# catkin_package(
# 	CATKIN_DEPENDS message_runtime
# )

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(contest3
    src/contest3.cpp
    src/costmap_client.cpp
    src/explore.cpp
    src/frontier_search.cpp
    )
target_link_libraries(contest3
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
