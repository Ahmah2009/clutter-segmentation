cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
add_definitions (-Wno-missing-field-initializers -O4 -DNDEBUG -mfpmath=sse -msse4 -fopenmp)
#add_definitions (-Wno-missing-field-initializers -g -O2 -DNDEBUG -mfpmath=sse -msse3)

rosbuild_init(object_recognition)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library (stereo_object_recognition
   src/Coordinate3DProviderDenseStereo.cpp
   src/ObjectDatabase3D.cpp
   src/SVDRigidEstimator.cpp
   src/RANSACRigidTransformationEstimator.cpp
   src/GeometricValidator3D.cpp
   src/Semaphore.cpp
)

#common commands for building c++ executables and libraries
rosbuild_add_boost_directories()

#rosbuild_add_executable(evaluate_nister_mt examples/evaluate_nister_mt.cpp)
#target_link_libraries (evaluate_nister_mt stereo_object_recognition)

rosbuild_add_executable(generate_nister_video examples/generate_nister_video.cpp)
target_link_libraries (generate_nister_video stereo_object_recognition)
