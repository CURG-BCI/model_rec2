################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ComputationalGeometry/Algorithms/AuxCompGeom.cpp \
../ComputationalGeometry/Algorithms/OptimalTransformation.cpp \
../ComputationalGeometry/Algorithms/PCA.cpp \
../ComputationalGeometry/Algorithms/PCANormalEstimator.cpp \
../ComputationalGeometry/Algorithms/RANSACPlaneDetector.cpp 

OBJS += \
./ComputationalGeometry/Algorithms/AuxCompGeom.o \
./ComputationalGeometry/Algorithms/OptimalTransformation.o \
./ComputationalGeometry/Algorithms/PCA.o \
./ComputationalGeometry/Algorithms/PCANormalEstimator.o \
./ComputationalGeometry/Algorithms/RANSACPlaneDetector.o 

CPP_DEPS += \
./ComputationalGeometry/Algorithms/AuxCompGeom.d \
./ComputationalGeometry/Algorithms/OptimalTransformation.d \
./ComputationalGeometry/Algorithms/PCA.d \
./ComputationalGeometry/Algorithms/PCANormalEstimator.d \
./ComputationalGeometry/Algorithms/RANSACPlaneDetector.d 


# Each subdirectory must supply rules for building sources it contributes
ComputationalGeometry/Algorithms/%.o: ../ComputationalGeometry/Algorithms/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I../.. -I/usr/local/include/vtk-5.6 -O3 -Wall -c -fmessage-length=0 -Wno-deprecated -msse4.1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


