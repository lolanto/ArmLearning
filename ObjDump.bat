@echo off
Set GCC_ARM_PATH=C:\LC\Workplace\STM32\gcc-arm-none-eabi-10.3-2021.10-win32\gcc-arm-none-eabi-10.3-2021.10\bin
Set GCC_ARM_PREFIX=%GCC_ARM_PATH%\arm-none-eabi-

%GCC_ARM_PREFIX%objdump.exe %~1 %~2