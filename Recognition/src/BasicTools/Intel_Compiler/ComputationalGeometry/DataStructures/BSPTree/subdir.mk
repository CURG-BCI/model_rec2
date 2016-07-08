################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ComputationalGeometry/DataStructures/BSPTree/BSPTree.cpp \
../ComputationalGeometry/DataStructures/BSPTree/BSPTreeNode.cpp \
../ComputationalGeometry/DataStructures/BSPTree/BSPTreeNodeData.cpp 

OBJS += \
./ComputationalGeometry/DataStructures/BSPTree/BSPTree.o \
./ComputationalGeometry/DataStructures/BSPTree/BSPTreeNode.o \
./ComputationalGeometry/DataStructures/BSPTree/BSPTreeNodeData.o 

CPP_DEPS += \
./ComputationalGeometry/DataStructures/BSPTree/BSPTree.d \
./ComputationalGeometry/DataStructures/BSPTree/BSPTreeNode.d \
./ComputationalGeometry/DataStructures/BSPTree/BSPTreeNodeData.d 


# Each subdirectory must supply rules for building sources it contributes
ComputationalGeometry/DataStructures/BSPTree/%.o: ../ComputationalGeometry/DataStructures/BSPTree/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I../.. -I/usr/local/include/vtk-5.6 -O3 -Wall -c -fmessage-length=0 -Wno-deprecated -msse4.1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


