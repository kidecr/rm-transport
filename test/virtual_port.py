#!/usr/bin/python3
#coding=utf-8

'''
用于创建虚拟终端，可以当虚拟串口用
注意一定要用python3打开, python2不支持
'''

import os
import threading
import sys


def quit(signum, frame):
    sys.exit()


def mkpty():
    master, slave = os.openpty()
    slaveName = os.ttyname(slave)

    print('virtual port names:', slaveName)
    return master


def read():
    while True:
        str = os.read(master, 100)
        if str != "":
            chars = []
            for c in str:
                hexstr = hex(c)
                chars.append(hexstr)
            print(chars)


if __name__ == "__main__":
    master = mkpty()
    t1 = threading.Thread(target=read, args=())
    t1.start()
    t1.join()