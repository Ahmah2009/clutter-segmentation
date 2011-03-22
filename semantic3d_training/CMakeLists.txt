cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

file(GLOB hdrs include/*.h*)
file(GLOB srcs src/*.cpp)


#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} ${srcs} ${hdrs} )

rosbuild_add_gtest(utest test/test.cpp test/test_filesystem.cpp test/test_image_io.cpp test/test_misc.cpp test/test_pcl.cpp test/test_yaml.cpp)
target_link_libraries(utest ${PROJECT_NAME})

#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(preparetraining apps/preparetraining.cpp)
target_link_libraries(preparetraining ${PROJECT_NAME})

rosbuild_add_executable(xyz_to_xyzrgb apps/xyz_to_xyzrgb.cpp)
target_link_libraries(xyz_to_xyzrgb ${PROJECT_NAME})
