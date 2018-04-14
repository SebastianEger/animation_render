set(CMAKE_PREFIX_PATH /opt/ros/kinetic)
add_compile_options(-std=c++11)
cmake_minimum_required(VERSION 2.8.3)
project(animation_render)

option(BUILD_BLENDER_MODULE "Build the blender python module" OFF)

find_package(catkin REQUIRED roscpp rospy std_msgs image_transport camera_info_manager cv_bridge)
find_package(OpenCV REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs image_transport camera_info_manager cv_bridge
)

if(BUILD_BLENDER_MODULE)
  add_subdirectory(external)
endif(BUILD_BLENDER_MODULE)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} /usr/include/libxml2 src)



add_executable(${PROJECT_NAME}_node src/animationrender.cpp src/pythoncaller.cpp src/videostream.cpp src/imageevaluation.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_add_gtest(utest test/utest.cpp)


# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
