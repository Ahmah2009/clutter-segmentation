cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (ROS_BUILD_TYPE RelWithDebInfo)
#set (ROS_BUILD_TYPE Debug)
rosbuild_init ()
rosbuild_add_boost_directories ()
add_definitions (-Wall -O3 -msse3 -mssse3 -funroll-loops)

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})

# Enable dynamic_reconfigure
rosbuild_find_ros_package (dynamic_reconfigure)
include (${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg ()

rosbuild_add_executable (build_tree src/build_tree.cpp)
rosbuild_link_boost (build_tree filesystem system)
rosbuild_add_executable (nearest_neighbors src/nearest_neighbors.cpp)
rosbuild_link_boost (nearest_neighbors filesystem system)

# Demo datasets
rosbuild_download_test_data (http://pr.willowgarage.com/data/pcl/vfh_cluster_classifier_demo.tbz demos/vfh_cluster_classifier_demo.tbz b840a14fc353b43aa14f8c72db9f1eb2)
rosbuild_untar_file (demos/vfh_cluster_classifier_demo.tbz demos/data/ ALL)



rosbuild_add_library (vfh_classifier 
    src/vfh_classifier.cpp
    src/vfh_classifier_nodelet.cpp)
