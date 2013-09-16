################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ComputationalGeometry/DataStructures/KdTree/KdTree.cpp \
../ComputationalGeometry/DataStructures/KdTree/KdTreeNode.cpp 

OBJS += \
./ComputationalGeometry/DataStructures/KdTree/KdTree.o \
./ComputationalGeometry/DataStructures/KdTree/KdTreeNode.o 

CPP_DEPS += \
./ComputationalGeometry/DataStructures/KdTree/KdTree.d \
./ComputationalGeometry/DataStructures/KdTree/KdTreeNode.d 


# Each subdirectory must supply rules for building sources it contributes
ComputationalGeometry/DataStructures/KdTree/%.o: ../ComputationalGeometry/DataStructures/KdTree/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I../.. -I/usr/local/include/vtk-5.6 -O3 -Wall -c -fmessage-length=0 -Wno-deprecated -msse4.1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


