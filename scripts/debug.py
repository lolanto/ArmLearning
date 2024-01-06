# -*- coding: UTF-8 -*- 

import os
import shutil
import argparse

from config import *

def readelf(file_name):
    readelf_cmd = GCC_BIN_PATH + GCC_PREFIX + READELF
    cmd = '{0} --all {1}'.format(readelf_cmd, file_name)
    return_code = os.system(cmd)
    return True if return_code == 0 else False

def dump(file_name):
    objdump_cmd = GCC_BIN_PATH + GCC_PREFIX + OBJDUMP
    cmd = '{0} -S {1}'.format(objdump_cmd, file_name)
    return_code = os.system(cmd)
    return True if return_code == 0 else False

def run_gdb(file_name):
    gdb_cmd = GCC_BIN_PATH + GCC_PREFIX + GDB
    cmd = '{0} {1}'.format(gdb_cmd, file_name)
    os.system(cmd)
    return

def main():
    parser = argparse.ArgumentParser(description='这是一个示例脚本，演示如何定义自定义指令。')
    # parser.add_argument('-c', action='store_true', help='执行编译，有任意文件编译错误将直接停止执行')
    # parser.add_argument('-cf', action='store_true', help='执行编译，会强制将所有文件编译完成，即便有个别文件编译失败')
    # parser.add_argument('-ct', type=str, default='', help='仅执行编译，以默认模式编译指定文件')
    # parser.add_argument('-l', action='store_true', help='执行链接，链接失败会直接停止执行')
    # parser.add_argument('-lf', action='store_true', help='执行链接，即便链接失败也会继续执行')
    # parser.add_argument('-b', action='store_true', help='进行构建并strip')
    # parser.add_argument('-o', type=str, default='application', help='设置输出文件名称')
    parser.add_argument('-r', type=str, default='', help='readelf文件')
    parser.add_argument('-d', type=str, default='', help='objdump指定文件')
    parser.add_argument('-gdb', type=str, default='', help='启动gdb并加载指定的elf符号文件')
    # parser.add_argument('-clean', action='store_true', help='清空所有中间文件')

    args = parser.parse_args()

    if args.r != '':
        readelf(args.r)
        return
    
    if args.d != '':
        dump(args.d)
        return

    if args.gdb != '':
        run_gdb(args.gdb)
        return
        

if __name__ == '__main__':
    main()
