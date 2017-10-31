################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Source/Main.cpp \
../Source/SelidarApplication.cpp 

OBJS += \
./Source/Main.o \
./Source/SelidarApplication.o 

CPP_DEPS += \
./Source/Main.d \
./Source/SelidarApplication.d 


# Each subdirectory must supply rules for building sources it contributes
Source/%.o: ../Source/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I"/home/cybernik/Development/Projects/SeNaviCommon/Source" -I/usr/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


