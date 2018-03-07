cmake_minimum_required(VERSION 3.0)
include(ExternalProject)

ExternalProject_Add(blender
  GIT_REPOSITORY "https://git.blender.org/blender.git"
  GIT_TAG "v2.78c"
  PATCH_COMMAND patch ${CMAKE_CURRENT_BINARY_DIR}/blender-prefix/src/blender/CMakeLists.txt ${CMAKE_CURRENT_SOURCE_DIR}/blender_cmake.patch
  COMMAND patch ${CMAKE_CURRENT_BINARY_DIR}/blender-prefix/src/blender/build_files/build_environment/install_deps.sh ${CMAKE_CURRENT_SOURCE_DIR}/blender_install.patch
  BUILD_IN_SOURCE 0
  CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/blender-prefix/src/blender/build_files/build_environment/install_deps.sh
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/setup_python.sh
  BUILD_COMMAND make BUILD_CMAKE_ARGS="-U *SNDFILE* -U *PYTHON* -U *BOOST* -U *Boost* -U *OPENCOLORIO* -U *OPENEXR* -U *OPENIMAGEIO* -U *LLVM* -U *CYCLES* -U *OPENSUBDIV* -U *OPENVDB* -U *COLLADA* -U *FFMPEG* -U *ALEMBIC* -D WITH_CODEC_SNDFILE=ON -D PYTHON_VERSION=3.5 -D WITH_OPENCOLORIO=ON -D OPENCOLORIO_ROOT_DIR=/opt/lib/ocio -D WITH_OPENIMAGEIO=ON -D OPENIMAGEIO_ROOT_DIR=/opt/lib/oiio -D WITH_CYCLES_OSL=ON -D WITH_LLVM=ON -D LLVM_VERSION=3.4 -D CYCLES_OSL=/opt/lib/osl -D LLVM_ROOT_DIR=/opt/lib/llvm -D LLVM_STATIC=ON -D WITH_OPENSUBDIV=ON -D OPENSUBDIV_ROOT_DIR=/opt/lib/osd -D WITH_OPENVDB=ON -D WITH_OPENVDB_BLOSC=ON -D WITH_ALEMBIC=OFF -D ALEMBIC_ROOT_DIR=/opt/lib/alembic -D WITH_CODEC_FFMPEG=ON" -C ${CMAKE_CURRENT_BINARY_DIR}/blender-prefix/src/blender
  INSTALL_COMMAND ""
)
