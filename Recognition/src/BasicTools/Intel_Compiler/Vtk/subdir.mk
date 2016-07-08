################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Vtk/VtkConvexHull2d.cpp \
../Vtk/VtkCopy.cpp \
../Vtk/VtkMeshSampler.cpp \
../Vtk/VtkRMS.cpp \
../Vtk/VtkTransform.cpp 

OBJS += \
./Vtk/VtkConvexHull2d.o \
./Vtk/VtkCopy.o \
./Vtk/VtkMeshSampler.o \
./Vtk/VtkRMS.o \
./Vtk/VtkTransform.o 

CPP_DEPS += \
./Vtk/VtkConvexHull2d.d \
./Vtk/VtkCopy.d \
./Vtk/VtkMeshSampler.d \
./Vtk/VtkRMS.d \
./Vtk/VtkTransform.d 


# Each subdirectory must supply rules for building sources it contributes
Vtk/%.o: ../Vtk/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I../.. -I/usr/local/include/vtk-5.6 -O3 -Wall -c -fmessage-length=0 -Wno-deprecated -msse4.1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


