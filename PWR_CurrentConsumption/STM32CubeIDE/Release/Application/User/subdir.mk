################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/xiaolonghuang/STM32CubeIDE/workspace_1.19.0/PWR_CurrentConsumption/Src/main.c \
/Users/xiaolonghuang/STM32CubeIDE/workspace_1.19.0/PWR_CurrentConsumption/Src/stm32g4xx_hal_msp.c \
/Users/xiaolonghuang/STM32CubeIDE/workspace_1.19.0/PWR_CurrentConsumption/Src/stm32g4xx_it.c \
/Users/xiaolonghuang/STM32CubeIDE/workspace_1.19.0/PWR_CurrentConsumption/Src/stm32g4xx_lp_modes.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c 

OBJS += \
./Application/User/main.o \
./Application/User/stm32g4xx_hal_msp.o \
./Application/User/stm32g4xx_it.o \
./Application/User/stm32g4xx_lp_modes.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/stm32g4xx_hal_msp.d \
./Application/User/stm32g4xx_it.d \
./Application/User/stm32g4xx_lp_modes.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: /Users/xiaolonghuang/STM32CubeIDE/workspace_1.19.0/PWR_CurrentConsumption/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G491xx -DUSE_NUCLEO_64 -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32G4xx_Nucleo -O0 -ffunction-sections -fdata-sections -Wall -fopt-info-vec-optimized=vec-opt.txt -fopt-info-vec-missed=vec-missed.txt -fopt-info-loop=loop.txt -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32g4xx_hal_msp.o: /Users/xiaolonghuang/STM32CubeIDE/workspace_1.19.0/PWR_CurrentConsumption/Src/stm32g4xx_hal_msp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G491xx -DUSE_NUCLEO_64 -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32G4xx_Nucleo -O0 -ffunction-sections -fdata-sections -Wall -fopt-info-vec-optimized=vec-opt.txt -fopt-info-vec-missed=vec-missed.txt -fopt-info-loop=loop.txt -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32g4xx_it.o: /Users/xiaolonghuang/STM32CubeIDE/workspace_1.19.0/PWR_CurrentConsumption/Src/stm32g4xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G491xx -DUSE_NUCLEO_64 -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32G4xx_Nucleo -O0 -ffunction-sections -fdata-sections -Wall -fopt-info-vec-optimized=vec-opt.txt -fopt-info-vec-missed=vec-missed.txt -fopt-info-loop=loop.txt -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32g4xx_lp_modes.o: /Users/xiaolonghuang/STM32CubeIDE/workspace_1.19.0/PWR_CurrentConsumption/Src/stm32g4xx_lp_modes.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G491xx -DUSE_NUCLEO_64 -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32G4xx_Nucleo -O0 -ffunction-sections -fdata-sections -Wall -fopt-info-vec-optimized=vec-opt.txt -fopt-info-vec-missed=vec-missed.txt -fopt-info-loop=loop.txt -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/%.o Application/User/%.su Application/User/%.cyclo: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G491xx -DUSE_NUCLEO_64 -c -I../../Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/BSP/STM32G4xx_Nucleo -O0 -ffunction-sections -fdata-sections -Wall -fopt-info-vec-optimized=vec-opt.txt -fopt-info-vec-missed=vec-missed.txt -fopt-info-loop=loop.txt -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/main.cyclo ./Application/User/main.d ./Application/User/main.o ./Application/User/main.su ./Application/User/stm32g4xx_hal_msp.cyclo ./Application/User/stm32g4xx_hal_msp.d ./Application/User/stm32g4xx_hal_msp.o ./Application/User/stm32g4xx_hal_msp.su ./Application/User/stm32g4xx_it.cyclo ./Application/User/stm32g4xx_it.d ./Application/User/stm32g4xx_it.o ./Application/User/stm32g4xx_it.su ./Application/User/stm32g4xx_lp_modes.cyclo ./Application/User/stm32g4xx_lp_modes.d ./Application/User/stm32g4xx_lp_modes.o ./Application/User/stm32g4xx_lp_modes.su ./Application/User/syscalls.cyclo ./Application/User/syscalls.d ./Application/User/syscalls.o ./Application/User/syscalls.su ./Application/User/sysmem.cyclo ./Application/User/sysmem.d ./Application/User/sysmem.o ./Application/User/sysmem.su

.PHONY: clean-Application-2f-User

