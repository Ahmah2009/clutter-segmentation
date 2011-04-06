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
file(GLOB tsts test/*.cpp)

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} ${srcs} ${hdrs} )

rosbuild_add_gtest(utest ${tsts})
target_link_libraries(utest ${PROJECT_NAME})


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
# rosbuild_link_boost(${PROJECT_NAME})
rosbuild_add_executable(blackbox_recognizer apps/blackbox_recognizer.cpp)
target_link_libraries(blackbox_recognizer tod_detecting ${PROJECT_NAME})

rosbuild_add_executable(poseviewer apps/poseviewer.cpp)
target_link_libraries(poseviewer ${PROJECT_NAME})

rosbuild_add_executable(poserandomizer apps/poserandomizer.cpp)
target_link_libraries(poserandomizer ${PROJECT_NAME})

rosbuild_add_executable(xyz_to_xyzrgb apps/xyz_to_xyzrgb.cpp)
target_link_libraries(xyz_to_xyzrgb ${PROJECT_NAME})

rosbuild_add_executable(poseinfo apps/poseinfo.cpp)
target_link_libraries(poseinfo ${PROJECT_NAME})

