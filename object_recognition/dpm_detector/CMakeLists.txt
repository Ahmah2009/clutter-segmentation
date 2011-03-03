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

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()


find_program(OCTAVE octave)
if(OCTAVE)
	message(STATUS "octave found: ${MKOCTFILE}")
	file(GLOB OCTAVE_LIB /usr/lib/octave-[0-9]*.[0-9]*.[0-9]*)
	message("	Octave libs are in: ${OCTAVE_LIB}")
	file(GLOB OCTAVE_INCLUDE /usr/include/octave-[0-9]*.[0-9]*.[0-9]*)
	message("	Octave includes are in: ${OCTAVE_INCLUDE}")
else(OCTAVE)
	message(FATAL_ERROR "error: mkoctfile not found. This package requires octave package to be installed.")
endif(OCTAVE)


include_directories( ${OCTAVE_INCLUDE})
link_directories(${OCTAVE_LIB})

# Octave libs have to be first in the link list for some reason,
# using add_library, instead of rosbuild_add_library, and linking the octave libs first
# and ROS dependencies at the end
add_library(dpm_wrapper SHARED
    src/dpm.cpp
    src/dpm_nodelet.cpp
)
add_dependencies(dpm_wrapper rosbuild_precompile)
target_link_libraries(dpm_wrapper octinterp octave cruft lapackgf-3 blas-3gf fftw3 fftw3f readline ncurses dl hdf5 z m gfortranbegin gfortran ${${PROJECT_NAME}_LIBRARIES})


add_custom_target(voc_release ALL make -f Makefile.voc
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                COMMENT "Downloading and building VOC source code")
