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

add_executable(${PROJECT_NAME}_node src/main.cpp src/animationrender.cpp src/pythoncaller.cpp src/videostream.cpp src/imageevaluation.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(utest test/utest.cpp src/videostream.cpp)
  target_link_libraries(utest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)

if(CATKIN_ENABLE_TESTING)
  add_rostest_gtest(videostream_test test/videostream_test.test test/videostream_test.cpp src/videostream.cpp)
  target_link_libraries(videostream_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)

if(CATKIN_ENABLE_TESTING)
  add_rostest_gtest(pythoncaller_test test/pythoncaller_test.test test/pythoncaller_test.cpp src/pythoncaller.cpp)
  target_link_libraries(pythoncaller_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)

if(CATKIN_ENABLE_TESTING)
  add_rostest_gtest(animationrender_test test/animationrender_test.test test/animationrender_test.cpp src/animationrender.cpp src/pythoncaller.cpp src/videostream.cpp src/imageevaluation.cpp)
  target_link_libraries(animationrender_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
