# -*- coding: UTF-8 -*- 
import os
# 编译器基础配置
GCC_BIN_PATH='C:/LC\Workplace/STM32/gcc-arm-none-eabi-10.3-2021.10-win32/gcc-arm-none-eabi-10.3-2021.10/bin/'
GCC_PREFIX='arm-none-eabi-'
GCC='gcc.exe'
GPP='g++.exe'
OBJDUMP='objdump.exe'
OBJCOPY='objcopy.exe'

# 输出环境配置
INTERMEDIATE_PATH='./intermediate'
OBJECT_FILE_DIR=os.path.join(INTERMEDIATE_PATH, 'objects')
OUT_PATH='./build'

COMPILATION_MODE_DEFAULT = 0
COMPILATION_MODE_FORCE_THUMB = 1

# 编译使用的文件配置
LINK_SCRIPT='./memory.ld' # 配置链接脚本
COMPILE_FILES=[('main.cpp', COMPILATION_MODE_FORCE_THUMB), 'App.cpp']


# 烧写相关的配置
STM32_PROGRAMMER_BIN='C:/LC/Workplace/STM32/CubeProgrammer/bin/STM32_Programmer_CLI.exe'