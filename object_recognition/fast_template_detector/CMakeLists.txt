cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Openmp
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF()


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library (pcl_processing src/ftd/pcl_processing.cpp)
#rosbuild_add_library (ftd_discretizers src/ftd/discretizers/gradient_discretizer.cpp src/ftd/discretizers/surface_normal_discretizer.cpp)
rosbuild_add_library (ftd_discretizers src/ftd/discretizers/gradient_discretizer.cpp)
rosbuild_add_library (ftd src/ftd/fast_template_detector_vs.cpp)
rosbuild_add_library (util src/util/util.cpp src/util/mouse.cpp)

include_directories (./include ./include/fast_template_detector)


rosbuild_add_executable(combine src/combine.cpp)
target_link_libraries (combine ftd util)

rosbuild_add_executable(object_database src/object_database.cpp)
target_link_libraries (object_database ftd util)

# discretizers ----------------------------------------------------------------
rosbuild_add_executable(gradient_discretizer src/discretizers/gradient_discretizer.cpp)
target_link_libraries (gradient_discretizer ftd_discretizers util)

#rosbuild_add_executable(surface_normal_discretizer src/discretizers/surface_normal_discretizer.cpp)
#target_link_libraries (surface_normal_discretizer ftd_discretizers util)


# learners --------------------------------------------------------------------
rosbuild_add_executable(adaptive_template_learner src/learners/adaptive_template_learner.cpp)
target_link_libraries (adaptive_template_learner ftd ftd_discretizers util)

rosbuild_add_executable(gradient_based_adaptive_template_learner src/learners/gradient_based_adaptive_template_learner.cpp)
target_link_libraries (gradient_based_adaptive_template_learner ftd ftd_discretizers util)


# detectors -------------------------------------------------------------------
rosbuild_add_executable(multi_class_candidate_detector src/detectors/multi_class_candidate_detector.cpp)
target_link_libraries (multi_class_candidate_detector ftd ftd_discretizers util)



# filters ---------------------------------------------------------------------
rosbuild_add_executable(multi_class_best_candidate_selector src/filters/multi_class_best_candidate_selector.cpp)
target_link_libraries (multi_class_best_candidate_selector ftd ftd_discretizers util)

rosbuild_add_executable(overlap_remover src/filters/overlap_remover.cpp)
target_link_libraries (overlap_remover ftd ftd_discretizers util)


# visualizers -----------------------------------------------------------------
rosbuild_add_executable(candidate_visualizer src/visualizers/candidate_visualizer.cpp)
target_link_libraries (candidate_visualizer ftd ftd_discretizers util)


# bag-file utils --------------------------------------------------------------
rosbuild_add_executable(dummy_trigger src/dummy_trigger.cpp)
rosbuild_add_executable(triggered_bag_player src/bag/triggered_bag_player.cpp src/ftd/bag/time_publisher.cpp src/ftd/bag/triggered_bag_player.cpp)











