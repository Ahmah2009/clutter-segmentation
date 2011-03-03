cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug )

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()

#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

file(GLOB tod_headers include/tod/training/*.h*)
file(GLOB tod_srcs src/*.cpp)

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} ${tod_srcs} ${tod_headers} )
rosbuild_add_library(${PROJECT_NAME}_ros src/ros/msgs.cpp include/tod/training/ros/msgs.h )

rosbuild_add_executable(pose_estimator apps/pose_estimator.cpp)
target_link_libraries(pose_estimator ${PROJECT_NAME})
rosbuild_link_boost(pose_estimator thread program_options)

rosbuild_add_executable(checkerboard_tests apps/checkerboard_tests.cpp)
target_link_libraries(checkerboard_tests ${PROJECT_NAME})
rosbuild_link_boost(checkerboard_tests thread program_options)

rosbuild_add_executable(gen_fiducial apps/gen_fiducial.cpp)
target_link_libraries(gen_fiducial ${PROJECT_NAME})

rosbuild_add_executable(masker apps/masker.cpp)
target_link_libraries(masker ${PROJECT_NAME})
rosbuild_link_boost(masker thread program_options)

rosbuild_add_executable(detector apps/detector.cpp)
target_link_libraries(detector ${PROJECT_NAME})
rosbuild_link_boost(detector thread program_options)

rosbuild_add_executable(f3d_creator apps/f3d_creator.cpp)
target_link_libraries(f3d_creator ${PROJECT_NAME})
rosbuild_link_boost(f3d_creator thread program_options)

rosbuild_add_executable(fiducial apps/fiducial.cpp)
target_link_libraries(fiducial ${PROJECT_NAME})
rosbuild_link_boost(fiducial program_options)

rosbuild_add_executable(image_pcl_recorder apps/image_pcl_recorder.cpp)

rosbuild_add_executable(bag_dumper apps/bag_dumper.cpp)
target_link_libraries(bag_dumper ${PROJECT_NAME}_ros ${PROJECT_NAME} )
rosbuild_link_boost(bag_dumper thread program_options filesystem)

rosbuild_add_executable(pcd_segmenter apps/pcd_segmenter.cpp)
target_link_libraries(pcd_segmenter ${PROJECT_NAME} )
#rosbuild_link_boost(pcd_segmenter thread program_options filesystem)

