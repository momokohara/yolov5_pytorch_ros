cmake_minimum_required(VERSION 2.8.12)
project(yolov5_pytorch_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    std_msgs
    message_generation
    actionlib_msgs
    actionlib
    image_transport
    cv_bridge
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxes.msg
)

add_action_files(
   FILES
   BoundingBoxes.action
)


generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    actionlib_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(push_image_action src/push_image_action.cpp)
add_dependencies(push_image_action ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(push_image_action ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
