################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Stochastics/Noise.cpp \
../Stochastics/PointSampler.cpp \
../Stochastics/RandomGenerator.cpp \
../Stochastics/RigidTransformSampler.cpp \
../Stochastics/SphericalBoxSampler.cpp \
../Stochastics/SphericalPatchSampler.cpp 

OBJS += \
./Stochastics/Noise.o \
./Stochastics/PointSampler.o \
./Stochastics/RandomGenerator.o \
./Stochastics/RigidTransformSampler.o \
./Stochastics/SphericalBoxSampler.o \
./Stochastics/SphericalPatchSampler.o 

CPP_DEPS += \
./Stochastics/Noise.d \
./Stochastics/PointSampler.d \
./Stochastics/RandomGenerator.d \
./Stochastics/RigidTransformSampler.d \
./Stochastics/SphericalBoxSampler.d \
./Stochastics/SphericalPatchSampler.d 


# Each subdirectory must supply rules for building sources it contributes
Stochastics/%.o: ../Stochastics/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I../.. -I/usr/local/include/vtk-5.6 -O3 -Wall -c -fmessage-length=0 -Wno-deprecated -msse4.1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


