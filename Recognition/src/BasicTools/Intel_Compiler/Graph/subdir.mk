################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Graph/GraphNode.cpp \
../Graph/GraphNodeData.cpp \
../Graph/UndirectedGraph.cpp 

OBJS += \
./Graph/GraphNode.o \
./Graph/GraphNodeData.o \
./Graph/UndirectedGraph.o 

CPP_DEPS += \
./Graph/GraphNode.d \
./Graph/GraphNodeData.d \
./Graph/UndirectedGraph.d 


# Each subdirectory must supply rules for building sources it contributes
Graph/%.o: ../Graph/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I../.. -I/usr/local/include/vtk-5.6 -O3 -Wall -c -fmessage-length=0 -Wno-deprecated -msse4.1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


