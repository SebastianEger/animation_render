cmake_minimum_required(VERSION 3.0)
include(ExternalProject)
message(${CMAKE_CURRENT_SOURCE_DIR}/blender/)

ExternalProject_Add(blender
  GIT_REPOSITORY "https://git.blender.org/blender.git"
  GIT_TAG "v2.78c"
  PATCH_COMMAND patch ${CMAKE_CURRENT_BINARY_DIR}/blender-prefix/src/blender/CMakeLists.txt ${CMAKE_CURRENT_SOURCE_DIR}/blender_cmake.patch
  COMMAND patch ${CMAKE_CURRENT_BINARY_DIR}/blender-prefix/src/blender/build_files/build_environment/install_deps.sh ${CMAKE_CURRENT_SOURCE_DIR}/blender_install.patch
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/blender-prefix
  BUILD_IN_SOURCE 0
  CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/blender-prefix/src/blender/build_files/build_environment/install_deps.sh
  BUILD_COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/blender-prefix/src/blender
  INSTALL_COMMAND ""
)
