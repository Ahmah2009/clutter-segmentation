if(NOT NON_ROSBUILD)
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)
rosbuild_init()

# patch
# include_directories("/home/julius/Desktop/vision_opencv/opencv2/opencv/include")
# include_directories("/opt/ros/diamondback/stacks/vision_opencv/opencv2/lib")
	
set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} " -Wno-deprecated -Wno-long-long")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOPENCV_R5024")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lgomp")
rosbuild_add_library(${PROJECT_NAME} 
	src/RBrief.cpp 
	src/lsh.cpp 
	src/StopWatch.cpp
	src/testing.cpp
	src/detectors.cpp
  src/find_homography.cpp
  src/quaternion.cpp
  src/pose_estimation.cpp
  src/artificial_warping.cpp
	)

rosbuild_link_boost(${PROJECT_NAME} filesystem program_options system)
	
rosbuild_add_executable(example apps/example.cpp)
target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(rotation_test experiments/rotation_test.cpp )
target_link_libraries(rotation_test ${PROJECT_NAME})
rosbuild_link_boost(rotation_test program_options)

rosbuild_add_executable(test_gen experiments/test_gen.cpp )
target_link_libraries(test_gen ${PROJECT_NAME})
rosbuild_link_boost(test_gen program_options)

rosbuild_add_executable(keypoint_tester experiments/keypoint_tester.cpp )
target_link_libraries(keypoint_tester ${PROJECT_NAME})
rosbuild_link_boost(keypoint_tester program_options)

rosbuild_add_executable(matching_stresstests experiments/matching_stresstests.cpp)
target_link_libraries(matching_stresstests ${PROJECT_NAME})

rosbuild_add_executable(scale_study experiments/scale_study.cpp)
target_link_libraries(scale_study ${PROJECT_NAME})
rosbuild_link_boost(scale_study program_options filesystem system)

rosbuild_add_executable(test_lsh experiments/test_lsh.cpp )
target_link_libraries(test_lsh ${PROJECT_NAME})
rosbuild_link_boost(test_lsh program_options)
else(NOT NON_ROSBUILD)
set(NON_ROSBUILD TRUE CACHE BOOL "Not ros" FORCE)
add_subdirectory(nonros_cmake)
endif(NOT NON_ROSBUILD)
