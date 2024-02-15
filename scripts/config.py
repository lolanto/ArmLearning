# -*- coding: UTF-8 -*- 
import os
import platform
# 编译器基础配置
GCC_BIN_PATH='/Applications/ARM/bin/'
GCC_PREFIX='arm-none-eabi-'
GCC='gcc'
GPP='g++'
AS='as'
OBJDUMP='objdump'
OBJCOPY='objcopy'
READELF='readelf'
GDB='gdb'
if platform.system() == "Windows":
    OBJDUMP += '.exe'
    OBJCOPY += '.exe'
    READELF += '.exe'
    GDB += '.exe'

# 输出环境配置
INTERMEDIATE_PATH='./intermediate'
OBJECT_FILE_DIR=os.path.join(INTERMEDIATE_PATH, 'objects')
OUT_PATH='./build'

COMPILATION_MODE_DEFAULT = 0
COMPILATION_MODE_FORCE_THUMB = 1

# 编译使用的文件配置
LINK_SCRIPT='./memory.ld' # 配置链接脚本
COMPILE_FILES=[('main.cpp', COMPILATION_MODE_FORCE_THUMB), 'App.cpp', 'MyHL.cpp', 'MyLibrary.cpp', 'Start.S']
FILEs_TO_LINK=[]

# 烧写相关的配置
STM32_PROGRAMMER_BIN='C:/LC/Workplace/STM32/CubeProgrammer/bin/STM32_Programmer_CLI.exe'

