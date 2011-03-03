cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#rosbuild_gensrv()

add_definitions(-DTEST_TO_TRAIN)
add_definitions(-DCHESSBOARD)

rosbuild_add_library(${PROJECT_NAME} src/training_set.cpp src/object.cpp src/ts_parameters.cpp src/shared_functions.cpp)
#rosbuild_add_library(stereo src/training_set.cpp src/training_set3d.cpp src/object.cpp src/ts_parameters.cpp)

rosbuild_add_library(shared_functions src/shared_functions.cpp src/camera_info.cpp)

rosbuild_add_executable(rs src/recognition_sample.cpp)
target_link_libraries(rs ${PROJECT_NAME})

rosbuild_add_executable(stat src/statistics.cpp)
target_link_libraries(stat ${PROJECT_NAME})
target_link_libraries(stat shared_functions)

rosbuild_add_executable(log src/log_analyzer.cpp)
target_link_libraries(log ${PROJECT_NAME})

rosbuild_add_executable(log2 src/log_analyzer2.cpp)
target_link_libraries(log2 ${PROJECT_NAME})

rosbuild_add_executable(rt src/recognition_test.cpp)
target_link_libraries(rt ${PROJECT_NAME})

#rosbuild_add_executable(ms src/matching_sample.cpp)
#target_link_libraries(ms ${PROJECT_NAME})

rosbuild_add_executable(rtc src/recognition_test_clutter.cpp)
target_link_libraries(rtc ${PROJECT_NAME})

#rosbuild_add_executable(ransac_test src/ransac_test.cpp)
#target_link_libraries(ransac_test ${PROJECT_NAME})

#rosbuild_add_executable(ransac3d_test src/ransac3d_test.cpp)
rosbuild_add_executable(is src/image_subscriber.cpp)
target_link_libraries(is shared_functions)

rosbuild_add_executable(converter src/convert_camera_info.cpp)
target_link_libraries(converter shared_functions)

rosbuild_add_executable(find_RT src/find_RT.cpp)
target_link_libraries(find_RT shared_functions)

rosbuild_add_executable(tod src/node.cpp)
target_link_libraries(tod ${PROJECT_NAME})

#rosbuild_add_executable(rsnode3d src/node3d.cpp)
#target_link_libraries(rsnode3d stereo)
#target_link_libraries(rsnode3d shared_functions)

rosbuild_add_executable(cis src/ci_saver.cpp)
target_link_libraries(cis shared_functions)

rosbuild_add_executable(crop src/crop_object.cpp)
target_link_libraries(crop ${PROJECT_NAME})
target_link_libraries(crop shared_functions)

rosbuild_add_executable(crop_ros src/crop_ros.cpp)
target_link_libraries(crop_ros ${PROJECT_NAME})
target_link_libraries(crop_ros shared_functions)

rosbuild_add_executable(crop_auto src/crop_object_auto.cpp)
target_link_libraries(crop_auto ${PROJECT_NAME})
target_link_libraries(crop_auto shared_functions)

#rosbuild_add_executable(error src/error_sample.cpp)
