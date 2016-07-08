################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Aux/NumberUtils.cpp \
../Aux/Stopwatch.cpp \
../Aux/StringUtils.cpp 

OBJS += \
./Aux/NumberUtils.o \
./Aux/Stopwatch.o \
./Aux/StringUtils.o 

CPP_DEPS += \
./Aux/NumberUtils.d \
./Aux/Stopwatch.d \
./Aux/StringUtils.d 


# Each subdirectory must supply rules for building sources it contributes
Aux/%.o: ../Aux/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I../.. -I/usr/local/include/vtk-5.6 -O3 -Wall -c -fmessage-length=0 -Wno-deprecated -msse4.1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


