set(CMAKE_PREFIX_PATH /opt/ros/kinetic)
add_compile_options(-std=c++11)
cmake_minimum_required(VERSION 2.8.3)
project(animation_render)

option(BUILD_BLENDER_MODULE "Build the blender python module" ON)

find_package(catkin REQUIRED roscpp rospy std_msgs image_transport camera_info_manager cv_bridge)
find_package(OpenCV REQUIRED)
find_package(rostest REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs image_transport camera_info_manager cv_bridge
)

if(BUILD_BLENDER_MODULE AND NOT EXISTS "/usr/lib/python3.5/site-packages/bpy.so")
  add_subdirectory(external)
endif(BUILD_BLENDER_MODULE AND NOT EXISTS "/usr/lib/python3.5/site-packages/bpy.so")

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS} /usr/include/libxml2 src)

add_executable(${PROJECT_NAME}_node src/main.cpp src/animationrender.cpp src/pythoncaller.cpp src/videostream.cpp src/templateevaluation.cpp)
add_executable(evaluation_tool tool/image_evaluation.cpp  src/templateevaluation.cpp)
add_executable(image_creator tool/image_creator.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(evaluation_tool ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(image_creator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
