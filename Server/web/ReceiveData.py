#!/usr/bin/python3
# -*- coding: UTF-8 -*-
# 引入必要的module
import serial
import socket
import pickle
import time

def main():
    # read serial
    x = serial.Serial('COM8', 115200) #设置串口号，波特率
    r = []
    num = 0
    num1 = 0
    length = 0
    j = 0
    s = socket.socket()
    host='192.168.43.178' # 服务器的ip
    port = 10040 # 所对应的端口
    s.connect((host, port))
    while True:
        str_read = x.read()
        i = ['%02x' % b for b in str_read]
        print(i)
        if i[0] == 'aa' and num == 0:
            num = num + 1 
            continue 
        elif i[0] == 'aa' and num == 1: # 数据帧头为 AA AA
            num = num + 1
            continue
        if num >= 2 and num < 15: # 数据帧的长度为14
            r.extend(i)
            num = num + 1
            if len(r) == 12:
                print('ii', r)
                r.extend(["s1"])
                a = pickle.dumps(r)
                s.send(a) # 发送到服务端
        if num == 14:
           num = 0
           num1 = 0
           r = []
    s.close()


if __name__ == "__main__":
main()
