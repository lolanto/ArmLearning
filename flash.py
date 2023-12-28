# -*- coding: UTF-8 -*- 

import argparse
import re
import os
import subprocess

from config import *

# 负责扫描并返回可用的uart port接口名字，以及波特率
def uart_port_scan():
    cmd = 'mode'
    result = subprocess.run(cmd, stdout=subprocess.PIPE, shell=True, text=True)
    return_str = result.stdout
    com_pattern = r'COM\d+'
    matches = re.findall(com_pattern, return_str)
    br = [0]
    data_bit = 0
    target_com_name = ['']
    for com_name in matches:
        com_info = return_str.split(com_name)[1]
        data_bit_pattern = r'数据位:\s+(\d+)'
        data_bit = re.findall(data_bit_pattern, com_info)[0]
        if data_bit != '8':
            continue
        br_pattern = r'波特率:\s+(\d+)'
        br[0] = re.findall(br_pattern, com_info)[0]
        target_com_name[0] = com_name
        break

    if target_com_name[0] != '':
        return (target_com_name[0], br[0])
    else:
        return None

def flash(file_name, port, br):
    file_and_its_path = os.path.join(OUT_PATH, file_name)
    if not os.path.exists(file_and_its_path):
        raise RuntimeError('File {0} is not exists!'.format(file_and_its_path))
    print('Try to download {0}'.format(file_and_its_path))
    cmd = '{0} --connect port={1} br={2} --download "{3}" 0x8000000'.format(STM32_PROGRAMMER_BIN, port, br, file_and_its_path)
    os.system(cmd)
    # result = subprocess.run(cmd, stdout=subprocess.PIPE, shell=True, text=True, encoding='latin-1')
    # return_str = result.stdout.encode('utf-8').decode('utf-8')
    # if len(re.findall(r'Activating device: OK', return_str)) == 0:
    #     print(return_str)
    #     raise RuntimeError('Connect to {0} with br {1} FAILED!'.format(port, br))

    # if len(re.findall(r'File download complete', return_str)) == 0:
    #     print(return_str)
    #     raise RuntimeError('Download file {0} FAILED!'.format(file_and_its_path))
    # print('Download {0} SUCCEED!'.format(file_and_its_path))

def main():
    parser = argparse.ArgumentParser(description='负责烧写编译内容')
    parser.add_argument('-file', type=str, default='application.bin', help='文件名称，记得带上后缀')
    args = parser.parse_args()
    result = uart_port_scan()
    if result is None:
        raise RuntimeError("Can not find out valid port")
    print('try to use port {0}, br {1}'.format(result[0], result[1]))
    flash(args.file, result[0], result[1])

if __name__ == '__main__':
    main()