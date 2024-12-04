#!/usr/bin/python3
#coding=utf-8

'''
用于创建虚拟终端，可以当虚拟串口用
注意一定要用python3打开, python2不支持
'''

import os
import threading
import sys
import argparse


def quit(signum, frame):
    sys.exit()


def mkpty():
    master, slave = os.openpty()
    slaveName = os.ttyname(slave)

    print('virtual port names:', slaveName)
    return master


def read(master, send_back):
    while True:
        str = os.read(master, 100)
        if str != "":
            chars = []
            for c in str:
                hexstr = hex(c)
                chars.append(hexstr)
            print(chars)
            if send_back:
                os.write(master, str)  # 将读取的数据写回串口


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Virtual Serial Port')
    parser.add_argument('-s', '--send-back', action='store_true', help='Send back received data')
    args = parser.parse_args()

    master = mkpty()
    t1 = threading.Thread(target=read, args=(master, args.send_back))
    t1.start()
    t1.join()
