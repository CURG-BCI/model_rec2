
cmake_minimum_required(VERSION 2.6)
project(model_rec2)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_definitions(-DEIGEN2_SUPPORT)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs sensor_msgs message_generation roscpp roslib)
find_package(Boost REQUIRED COMPONENTS thread)


find_package(VTK)
find_package(PCL 1.7 REQUIRED)
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
	${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ./srv
	./src
	./Recognition/src
)


message("PCL include dirs ${PCL_INCLUDE_DIRS}")


add_service_files(
	FILES FindObjects.srv
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	sensor_msgs
)

catkin_package(DEPENDS Boost
    CATKIN_DEPENDS message_runtime roscpp message_generation roslib
)



link_directories(
	${VTK_LIBRARY_DIRS}
	${OpenCV_LIB_DIR}
)


if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()
#Wrapper around library
add_library(ModelRec src/model_rec.cpp)
add_dependencies(ModelRec model_rec2_generate_messages_cpp roscpp)

#executable with ros loop
add_executable(ObjRecRANSACRun src/model_rec_server.cpp)

target_link_libraries(ObjRecRANSACRun ModelRec ObjRecRANSAC BasicToolsL1 BasicTools VtkBasics vtkCommon vtkRendering ${OpenCV_LIBS} ${PCL_LIBS} ${catkin_LIBRARIES})




