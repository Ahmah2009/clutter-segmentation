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

#for pcl visualization... TODO remove this.
set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} " -Wno-deprecated")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_add_boost_directories()
rosbuild_add_library(${PROJECT_NAME} src/tod_stub.cpp )
rosbuild_add_library(tod_stub_impl src/tod_stub_impl.cpp)



rosbuild_add_executable(image_pcl_recorder src/image_pcl_recorder.cpp)
rosbuild_add_executable(bag_processor src/bag_processor.cpp)
rosbuild_add_executable(trainer src/trainer.cpp)
rosbuild_add_executable(detector src/detector.cpp)


rosbuild_link_boost(bag_processor program_options)
rosbuild_link_boost(trainer program_options)

target_link_libraries(bag_processor ${PROJECT_NAME})

target_link_libraries(tod_stub_impl ${PROJECT_NAME})
target_link_libraries(trainer ${PROJECT_NAME} tod_stub_impl)
target_link_libraries(detector ${PROJECT_NAME} tod_stub_impl)



