/**
  @page PWR_STOP1 PWR Example
  
  @verbatim
  ******************************************************************************
  * @file    Examples_MIX/PWR/PWR_STOP1/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the PWR_STOP1 example.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @endverbatim

@par Example Description 
 
How to enter the STOP 1 mode and wake up from this mode by using external 
reset or wakeup interrupt (all the RCC function calls use RCC LL API 
for minimizing footprint and maximizing performance).

In the associated software, the system clock is set to 170 MHz, an EXTI line
is connected to the user button through PC.13 and configured to generate an 
interrupt on falling edge upon key press.
The SysTick is programmed to generate an interrupt each 1 ms and in the SysTick 
interrupt handler, LED2 is toggled in order to indicate whether the MCU is in STOP 1 mode 
or RUN mode.

5 seconds after start-up, the system automatically enters STOP 1 mode and 
LED2 stops toggling.

The User push-button can be pressed at any time to wake-up the system. 
The software then comes back in RUN mode for 5 sec. before automatically entering STOP 1 mode again. 

LED2 is used to monitor the system state as follows:
 - LED2 toggling: system in RUN mode
 - LED2 off : system in STOP 1 mode

These steps are repeated in an infinite loop.

@note To measure MCU current consumption on board STM32WB Nucleo,
      board configuration must be applied:
      - remove all jumpers on connector JP5 to avoid leakages between ST-Link circuitry and STM32G4 device.
      - remove jumper JP6 and connect an amperemeter to measure current between the 2 connectors of JP6.

@note This example can not be used in DEBUG mode due to the fact 
      that the Cortex-M4 core is no longer clocked during low power mode 
      so debugging features are disabled.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.
  
@par Keywords

Power, PWR, stop mode, wake-up, external reset, Interrupt, low power mode    

@par Directory contents 

  - Examples_MIX/PWR/PWR_STOP1/Inc/stm32g4xx_conf.h         HAL Configuration file
  - Examples_MIX/PWR/PWR_STOP1/Inc/stm32g4xx_it.h           Header for stm32g4xx_it.c
  - Examples_MIX/PWR/PWR_STOP1/Inc/main.h                   Header file for main.c
  - Examples_MIX/PWR/PWR_STOP1/Src/system_stm32g4xx.c       STM32G4xx system clock configuration file
  - Examples_MIX/PWR/PWR_STOP1/Src/stm32g4xx_it.c           Interrupt handlers
  - Examples_MIX/PWR/PWR_STOP1/Src/main.c                   Main program
  - Examples_MIX/PWR/PWR_STOP1/Src/stm32g4xx_hal_msp.c      HAL MSP module

@par Hardware and Software environment

  - This example runs on STM32G4xx devices
    

  - This example has been tested with STMicroelectronics NUCLEO-G474RE RevC
    board and can be easily tailored to any other supported device 
    and development board.

  - NUCLEO-G474RE RevC set-up:
    - LED2 connected to PA.05 pin
    - User push-button connected to pin PC.13 (External line 13)
    
@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example


 */
