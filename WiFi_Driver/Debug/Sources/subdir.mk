################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/esp8266_at_command.c \
../Sources/main.c \
../Sources/port.c \
../Sources/timer.c \
../Sources/uart.c 

OBJS += \
./Sources/esp8266_at_command.o \
./Sources/main.o \
./Sources/port.o \
./Sources/timer.o \
./Sources/uart.o 

C_DEPS += \
./Sources/esp8266_at_command.d \
./Sources/main.d \
./Sources/port.d \
./Sources/timer.d \
./Sources/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"../Sources" -I"../Includes" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


