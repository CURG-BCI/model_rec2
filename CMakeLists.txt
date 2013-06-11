
cmake_minimum_required(VERSION 2.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(CMAKE_VERBOSE_MAKEFILE ON)
set (ROS_BUILD_TYPE Release)
#set (ROS_BUILD_TYPE RelWithDebInfo)
add_definitions(-DEIGEN2_SUPPORT)
rosbuild_init ()
rosbuild_add_boost_directories ()
rosbuild_gensrv()

find_package(VTK)

IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)

find_package(OpenCV)

IF(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found. OpenCV is not installed or cmake can not find it. Install OpenCV first and then try again.
If OpenCV is installed but cmake can not find it, set the OPENCV_DIR entry to the folder which contains the file OpenCVConfig.cmake")
ENDIF(NOT OpenCV_FOUND)

add_subdirectory(./Recognition)

include_directories(
	${VTK_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	./src
	./Recognition/src
)

link_directories(
	${VTK_LIBRARY_DIRS}
	${OpenCV_LIB_DIR}
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()

rosbuild_add_library(ModelRec src/model_rec.cpp)

rosbuild_add_executable(ObjRecRANSACRun src/model_rec_server.cpp)
target_link_libraries(ObjRecRANSACRun ModelRec ObjRecRANSAC BasicToolsL1 BasicTools VtkBasics vtkCommon vtkRendering ${OpenCV_LIBS})


