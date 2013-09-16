################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../DataStructures/Box.cpp \
../DataStructures/BoxStructure3d.cpp \
../DataStructures/LayeredPointSet.cpp \
../DataStructures/NdBoxStructure.cpp 

OBJS += \
./DataStructures/Box.o \
./DataStructures/BoxStructure3d.o \
./DataStructures/LayeredPointSet.o \
./DataStructures/NdBoxStructure.o 

CPP_DEPS += \
./DataStructures/Box.d \
./DataStructures/BoxStructure3d.d \
./DataStructures/LayeredPointSet.d \
./DataStructures/NdBoxStructure.d 


# Each subdirectory must supply rules for building sources it contributes
DataStructures/%.o: ../DataStructures/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I../.. -I/usr/local/include/vtk-5.6 -O3 -Wall -c -fmessage-length=0 -Wno-deprecated -msse4.1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


