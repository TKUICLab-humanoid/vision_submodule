cmake_minimum_required(VERSION 2.8.3)
project(vision)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  genmsg
  roscpp
  sensor_msgs
  std_msgs
  roslib
  message_generation
  tku_libs
  tku_msgs
)

find_package(OpenCV REQUIRED)
set(TKU_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libtku_libs.so)
# add the resized image message
#add_message_files(DIRECTORY msg
#   FILES 
#)


#add_service_files(DIRECTORY srv FILES )

#generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES imageprocess
   CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add the publisher example
add_executable(vision
    src/VisionBase/VisionBase.cpp
    src/main/imagemain.cpp
    src/ObjectDetected/ObjectDetected.cpp
    src/FeatureDistance/FeatureDistance.cpp
    src/LineDetected/LineDetected.cpp
    src/ModelBase/ModelBase.cpp
)
target_link_libraries(vision ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${TKU_LIBRARIES})

install(FILES
  launch/image.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

