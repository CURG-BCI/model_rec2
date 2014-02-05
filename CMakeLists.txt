cmake_minimum_required(VERSION 2.8.3)
project(model_rec2)
set(CMAKE_VERBOSE_MAKEFILE ON)


add_definitions(-DEIGEN2_SUPPORT)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	std_msgs 
	geometry_msgs 
	sensor_msgs 
	message_generation 
	roscpp 
	pcl_ros
	eigen_conversions
	roslib)



## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

find_package(VTK)
find_package(PCL REQUIRED)
IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)

find_package(OpenCV)

IF(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found. OpenCV is not installed or cmake can not find it. Install OpenCV first and then try again.
If OpenCV is installed but cmake can not find it, set the OPENCV_DIR entry to the folder which contains the file OpenCVConfig.cmake")
ENDIF(NOT OpenCV_FOUND)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See <a href="http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html">http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html</a>
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
	FILES 
	FindObjects.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	geometry_msgs
	sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#   CATKIN_DEPENDS message_runtime roscpp message_generation roslib pcl geometry_msgs eigen_conversions
#   CATKIN_DEPENDS message_runtime roscpp message_generation pcl geometry_msgs eigen_conversions
#  DEPENDS system_lib
)


###########
## Build ##
###########

add_subdirectory(./Recognition)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${VTK_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ./srv
	./src
	./Recognition/src
)


if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()


## Declare a cpp library
add_library(ModelRec src/model_rec.cpp)

## Declare a cpp executable
# add_executable(my_pcl_tutorial_node src/my_pcl_tutorial_node.cpp)
add_executable(ObjRecRANSACRun src/model_rec_server.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(ModelRec model_rec2_generate_messages_cpp roscpp eigen_conversions)

## Specify libraries to link a library or executable target against
target_link_libraries(ModelRec ${PCL_LIBS} ${catkin_LIBRARIES})
target_link_libraries(ObjRecRANSACRun ModelRec ObjRecRANSAC BasicToolsL1 BasicTools VtkBasics vtkCommon vtkRendering ${OpenCV_LIBS} ${PCL_LIBS} ${catkin_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See <a href="http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html">http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html</a>

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS my_pcl_tutorial my_pcl_tutorial_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_pcl_tutorial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



