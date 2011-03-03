cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
add_definitions (-Wall -O3 -funroll-loops)
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg ()
rosbuild_gensrv ()

# ---[ Should we check for libpq with rosdep?
find_program (PG_CONFIG NAMES pg_config DOC "libpq config tool")
if (NOT PG_CONFIG)
  message (FATAL_ERROR "Couldn't find pg_config. Is libpq installed?")
endif (NOT PG_CONFIG)
find_package (PkgConfig)
pkg_check_modules (SQLITE3 REQUIRED sqlite3)
include_directories (${SQLITE3_INCLUDE_DIRS})
link_directories (${SQLITE3_LIBRARY_DIRS})
execute_process( 
  COMMAND pg_config --includedir
  ERROR_VARIABLE IGNORE_VAR 
  OUTPUT_VARIABLE PQ_INCLUDE_DIR 
  OUTPUT_STRIP_TRAILING_WHITESPACE)
include (CheckIncludeFiles)
set (CMAKE_REQUIRED_INCLUDES ${PQ_INCLUDE_DIR})
check_include_files (libpq-fe.h HAVE_LIBPQ)
if (NOT HAVE_LIBPQ)
  message (FATAL_ERROR "Error: PostgreSQL implementation cannot find libpq-fe.h")
endif (NOT HAVE_LIBPQ)
include (CheckIncludeFileCXX)
set (SOCI_INCLUDE_DIR /usr/include/soci)
set (CMAKE_REQUIRED_INCLUDES ${SOCI_INCLUDE_DIR})
check_include_file_cxx (soci.h HAVE_SOCI)
if (NOT HAVE_SOCI)
    set (SOCI_INCLUDE_DIR /opt/ros/include/soci)
    set (SOCI_LIB_DIR /opt/ros/lib)
    set (CMAKE_REQUIRED_INCLUDES ${SOCI_INCLUDE_DIR})
    check_include_file_cxx (soci.h HAVE_SOCI2)
    if (NOT HAVE_SOCI2)
        message (FATAL_ERROR "Error: cannot find SOCI header file: soci.h")
    endif ()
endif ()
include_directories (${PQ_INCLUDE_DIR} ${SOCI_INCLUDE_DIR})
link_directories (${SOCI_LIB_DIR})
# ]---

# ---[ ReIn library
rosbuild_add_library (${PROJECT_NAME} 
    #    src/algorithms/tabletop_attention.cpp
    src/io/db_model_storage.cpp
    src/io/fs_model_storage.cpp
)
rosbuild_add_compile_flags (${PROJECT_NAME} ${SSE_FLAGS})
target_link_libraries (${PROJECT_NAME}
    #soci
    soci_core-gcc-3_0 soci_postgresql-gcc-3_0 soci_sqlite3-gcc-3_0 
)
rosbuild_link_boost (${PROJECT_NAME} serialization)

# ---[ ReIn library, ROS interface
rosbuild_add_library (${PROJECT_NAME}_ros
    #detector nodelets
    src/nodelets/detector.cpp
    #attention nodelets
    src/nodelets/attention_operator.cpp
    #    src/nodelets/tabletop_attention_nodelet.cpp
    #pose estimator nodelets
    src/nodelets/pose_estimator.cpp

    src/nodelets/trainer_server.cpp
    src/nodelets/type_conversions.cpp
)
rosbuild_add_compile_flags (${PROJECT_NAME}_ros ${SSE_FLAGS})
target_link_libraries (${PROJECT_NAME}_ros ${PROJECT_NAME})


rosbuild_add_executable (detection_displayer src/tools/detection_displayer.cpp)
rosbuild_add_executable (select3dobj src/tools/select3dobj.cpp)
rosbuild_add_library (${PROJECT_NAME}_tools
    src/tools/pc_projector.cpp
    src/tools/point_cloud_resync.cpp
    src/tools/grabcut.cpp
    src/tools/clean_segmentation.cpp
    src/tools/save_bag_data.cpp
)
rosbuild_add_compile_flags (${PROJECT_NAME}_tools ${SSE_FLAGS})

