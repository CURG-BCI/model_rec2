################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ComputationalGeometry/DataStructures/Octree/Octree.cpp \
../ComputationalGeometry/DataStructures/Octree/OctreeNode.cpp 

OBJS += \
./ComputationalGeometry/DataStructures/Octree/Octree.o \
./ComputationalGeometry/DataStructures/Octree/OctreeNode.o 

CPP_DEPS += \
./ComputationalGeometry/DataStructures/Octree/Octree.d \
./ComputationalGeometry/DataStructures/Octree/OctreeNode.d 


# Each subdirectory must supply rules for building sources it contributes
ComputationalGeometry/DataStructures/Octree/%.o: ../ComputationalGeometry/DataStructures/Octree/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I../.. -I/usr/local/include/vtk-5.6 -O3 -Wall -c -fmessage-length=0 -Wno-deprecated -msse4.1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


