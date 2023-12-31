# -*- coding: UTF-8 -*- 

import os
import shutil
import argparse

from config import *


def compilation(file, mode):
    force_thumb_cmd = '-mthumb' if mode == COMPILATION_MODE_FORCE_THUMB else ''
    gpp_cmd = GCC_BIN_PATH + GCC_PREFIX + GPP
    if not os.path.exists('{0}.cpp'.format(file)):
        print('File {0}.cpp is not exists'.format(file))
        return False
    if not os.path.exists(OBJECT_FILE_DIR):
        os.makedirs(OBJECT_FILE_DIR)
    output_file_path = os.path.join(OBJECT_FILE_DIR, '{0}.o'.format(file))

    cmd = '{0} -o {1} -c -g -std=c++17 -fno-exceptions -mcpu=cortex-m3 {2} {3}.cpp'.format(gpp_cmd, output_file_path, force_thumb_cmd, file)
    return_code = os.system(cmd)
    print('Compile file: {0}.cpp {1} {2}'.format(file, 'with Thumb mode' if mode == COMPILATION_MODE_FORCE_THUMB else '', 'Succeed' if return_code == 0 else 'Failed'))
    return True if return_code == 0 else False

def link(files_to_link, output_file_name):
    if not os.path.exists(LINK_SCRIPT):
        print('Target Link Script: {0} is not exists'.format(LINK_SCRIPT))
        return False
    link_file_str = ''
    for file in files_to_link:
        object_file_path = os.path.join(OBJECT_FILE_DIR, file)
        if not os.path.exists(object_file_path):
            print('Object File: {0} is not exists'.format(file))
            return False
        link_file_str = link_file_str + ' ' + object_file_path
    gpp_cmd = GCC_BIN_PATH + GCC_PREFIX + GPP
    cmd = '{0} -o {1} -specs=nosys.specs -specs=nano.specs -Wl,--no-eh-frame-hdr,-T{2} -nostartfiles {3}'.format(gpp_cmd, os.path.join(INTERMEDIATE_PATH, output_file_name), LINK_SCRIPT, link_file_str)
    return_code = os.system(cmd)
    return True if return_code == 0 else False

def build(output_file_name):
    if not os.path.exists(OUT_PATH):
        os.makedirs(OUT_PATH)
    objcopy_cmd = GCC_BIN_PATH + GCC_PREFIX + OBJCOPY
    cmd = '{0} -O binary {1} {2}'.format(objcopy_cmd, os.path.join(INTERMEDIATE_PATH, '{0}.elf'.format(output_file_name)), os.path.join(OUT_PATH, '{0}.bin'.format(output_file_name)))
    return_code = os.system(cmd)
    return True if return_code == 0 else False

def main():
    parser = argparse.ArgumentParser(description='这是一个示例脚本，演示如何定义自定义指令。')
    parser.add_argument('-c', action='store_true', help='执行编译，有任意文件编译错误将直接停止执行')
    parser.add_argument('-cf', action='store_true', help='执行编译，会强制将所有文件编译完成，即便有个别文件编译失败')
    parser.add_argument('-ct', type=str, default='', help='仅执行编译，以默认模式编译指定文件')
    parser.add_argument('-l', action='store_true', help='执行链接，链接失败会直接停止执行')
    parser.add_argument('-lf', action='store_true', help='执行链接，即便链接失败也会继续执行')
    parser.add_argument('-b', action='store_true', help='进行构建并strip')
    parser.add_argument('-o', type=str, default='application', help='设置输出文件名称')
    parser.add_argument('-clean', action='store_true', help='清空所有中间文件')

    args = parser.parse_args()
    if args.ct != '':
        compile_mode = COMPILATION_MODE_DEFAULT
        file_name = args.ct
        compilation(file_name, compile_mode)
        return

    if args.clean:
        if os.path.exists(INTERMEDIATE_PATH):
            print('cleaned up {0}'.format(INTERMEDIATE_PATH))
            shutil.rmtree(INTERMEDIATE_PATH)
        if os.path.exists(OUT_PATH):
            print('cleaned up {0}'.format(OUT_PATH))
            shutil.rmtree(OUT_PATH)
        return

    any_failed = False
    
    if args.c or args.cf:
        go_through = args.cf
        print('Begin File Compilation')
        for target_file in COMPILE_FILES:
            compile_mode = COMPILATION_MODE_DEFAULT
            file_name = ''
            if isinstance(target_file, tuple):
                file_name = target_file[0]
                compile_mode = target_file[1]
            else:
                file_name = target_file
                compile_mode = COMPILATION_MODE_DEFAULT
            file_name = file_name.split('.')[0]

            is_succeed = compilation(file_name, compile_mode)
            if is_succeed is False:
                any_failed = True
                if go_through is False:
                    raise RuntimeError('Compile FAILED!')
        print('End File Compilation')
    
    if any_failed:
        return

    output_file_name = args.o
    if args.l or args.lf:
        go_through = args.lf
        print('Begin Linking')
        linke_files = []
        for target_file in COMPILE_FILES:
            file_name = ''
            if isinstance(target_file, tuple):
                file_name = target_file[0]
            else:
                file_name = target_file
            file_name = file_name.split('.')[0]
            file_name = '{0}.o'.format(file_name)
            linke_files.append(file_name)
        link_output_file_name = '{0}.elf'.format(output_file_name)
        is_succeed = link(linke_files, link_output_file_name)
        if is_succeed is False:
            any_failed = True
            if go_through is False:
                raise RuntimeError('Link FAILED!')
        print('End Linking')

    if any_failed:
        return
    
    if args.b:
        print('Begin Building')
        is_succeed = build(output_file_name)
        if is_succeed is False:
            raise RuntimeError('Build FAILED!')
        print('End Building')

    print('Finish Executations')

if __name__ == '__main__':
    main()