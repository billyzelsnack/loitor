cmake_minimum_required(VERSION 2.8.3)
project(loitor_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  message_generation
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  image_transport
  message_runtime
  sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})

add_library(loitorusbcam src/loitorusb.cpp src/loitorcam.cpp)
target_link_libraries(loitorusbcam loitorimu)

add_library(loitorimu src/loitorimu.cpp)

add_executable(camtest src/camtest.cpp)
target_link_libraries(camtest loitorusbcam loitorimu usb-1.0 ${OpenCV_LIBS})

add_executable(loitor_node src/loitornode.cpp) 
target_link_libraries(loitor_node loitorusbcam loitorimu ${catkin_LIBRARIES} ${OpenCV_LIBS} usb-1.0)

