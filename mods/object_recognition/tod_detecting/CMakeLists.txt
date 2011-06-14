cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-DOPENCV_R5024=OPENCV_R5024")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

file(GLOB tod_headers include/tod/detecting/*.h* include/tod/matching/*.h*)
file(GLOB tod_srcs src/*.cpp src/matching/*.cpp)

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} ${tod_srcs} ${tod_headers} )

rosbuild_add_executable(recognizer apps/recognizer.cpp)
target_link_libraries(recognizer ${PROJECT_NAME})
rosbuild_link_boost(recognizer program_options)

rosbuild_add_executable(frecognizer apps/folder_recognizer.cpp)
target_link_libraries(frecognizer ${PROJECT_NAME})
rosbuild_link_boost(frecognizer filesystem program_options system)

rosbuild_add_executable(recognition_node apps/recognition_node.cpp)
target_link_libraries(recognition_node ${PROJECT_NAME})
rosbuild_link_boost(recognition_node program_options system thread)

rosbuild_add_executable(test_recognition_service apps/test_recognition_service.cpp)
target_link_libraries(test_recognition_service ${PROJECT_NAME})
rosbuild_link_boost(test_recognition_service program_options system thread)

rosbuild_add_executable(cluster_sample apps/cluster_sample.cpp)
target_link_libraries(cluster_sample ${PROJECT_NAME})
rosbuild_link_boost(cluster_sample program_options)

rosbuild_add_executable(lsh_kd_tree_speed apps/lsh_kd_tree_speed.cpp)
target_link_libraries(lsh_kd_tree_speed ${PROJECT_NAME})
rosbuild_link_boost(lsh_kd_tree_speed program_options filesystem system thread)

