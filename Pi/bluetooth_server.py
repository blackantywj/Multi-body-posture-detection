import serial
import socket
import pickle
import time

bt_port = serial.Serial('/dev/rfcomm0',115200)

flag = 0
cnt = 0
sensors_data = {"AX":0,"AY":0,"AZ":0,"GX":0,"GY":0,"GZ":0,"Row":0,"Pitch":0,"Yaw":0,"T":0}
arrsum = 0
passage = 0
done = False
# add a label to positioning
core_node_position = '103'

client = socket.socket()
# host shoule be the web server IP
host = 'server.blackant.org'
port = 10010
print("set down!")
while True:
    try:
        client.connect((host, port))
        print("connected!")
        break
    except:
        print("connect failed at " + time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(time.time())) )
        time.sleep(1)

while True:
    try:
        bt_data = bt_port.read()
        if done == True:
            done = False
            ret = client.send(web_data)
            #print(ret)
        raw = ['%02x' % b for b in bt_data]
        #print(raw)
        if raw[0] == '55' and cnt == 0:
            cnt = cnt +1
            arrsum = int(raw[0],16)
            continue
        elif raw[0] == '51' and cnt == 1:
            cnt = cnt +1
            arrsum = arrsum + int(raw[0],16)
            passage = 1
            continue
        elif raw[0] == '52' and cnt == 1:
            cnt = cnt +1
            arrsum = arrsum + int(raw[0],16)
            passage = 2
            continue
        elif raw[0] == '53' and cnt == 1:
            cnt = cnt +1
            arrsum = arrsum + int(raw[0],16)
            passage = 3
            continue
        # data scratch
        if cnt == 2 and passage == 1:
            LByte = int(raw[0], 16)
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 3 and passage == 1:
            HByte = int(raw[0], 16)
            sensors_data['AX'] = float( HByte*256 + LByte ) / 32768 * 16
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 4 and passage == 1:
            LByte = int(raw[0], 16)
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 5 and passage == 1:
            HByte = int(raw[0], 16)
            sensors_data['AY'] = float( HByte*256 + LByte ) / 32768 * 16
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 6 and passage == 1:
            LByte = int(raw[0], 16)
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 7 and passage == 1:
            HByte = int(raw[0], 16)
            sensors_data['AZ'] = float( HByte*256 + LByte ) / 32768 * 16
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 8 and passage == 1:
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 9 and passage == 1:
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 2 and passage == 2:
            LByte = int(raw[0], 16)
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 3 and passage == 2:
            HByte = int(raw[0], 16)
            sensors_data['GX'] = float( HByte*256 + LByte ) / 32768 * 2000
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 4 and passage == 2:
            LByte = int(raw[0], 16)
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 5 and passage == 2:
            HByte = int(raw[0], 16)
            sensors_data['GY'] = float( HByte*256 + LByte ) / 32768 * 2000
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 6 and passage == 2:
            LByte = int(raw[0], 16)
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 7 and passage == 2:
            HByte = int(raw[0], 16)
            sensors_data['GZ'] = float( HByte*256 + LByte ) / 32768 * 2000
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 8 and passage == 2:
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 9 and passage == 2:
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 2 and passage == 3:
            LByte = int(raw[0], 16)
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 3 and passage == 3:
            HByte = int(raw[0], 16)
            sensors_data['Roll'] = float( HByte*256 + LByte ) / 32768 * 180.0
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 4 and passage == 3:
            LByte = int(raw[0], 16)
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 5 and passage == 3:
            HByte = int(raw[0], 16)
            sensors_data['Pitch'] = float( HByte*256 + LByte ) / 32768 * 180.0
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 6 and passage == 3:
            LByte = int(raw[0], 16)
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 7 and passage == 3:
            HByte = int(raw[0], 16)
            sensors_data['Yaw'] = float( HByte*256 + LByte ) / 32768 * 180.0
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        if cnt == 8 and passage == 3:
            LByte = int(raw[0], 16)
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        elif cnt == 9 and passage == 3:
            HByte = int(raw[0], 16)
            sensors_data['T'] = float( HByte*256 + LByte ) / 340 + 36.53
            arrsum = arrsum + int(raw[0],16)
            cnt = cnt +1
            continue
        # turn back
        if cnt == 10 and passage == 1:
            temp=str(hex(arrsum))[-2:]
            if (temp == raw[0]):
                cnt = 0
        elif cnt == 10 and passage == 2:
            temp=str(hex(arrsum))[-2:]
            if (temp == raw[0]):
                cnt = 0
        elif cnt == 10 and passage == 3:
            temp=str(hex(arrsum))[-2:]
            flag = flag +1
            if(flag == 1000):
                try:
                    print(sensors_data)
                    sensors_data_cp = sensors_data.copy()
                    # add label belong to the specific core node
                    sensors_data_cp['core_node_name'] = core_node_position
                    #time.strftime('%Y-%m-%d',time.localtime(time.time()))
                    web_data = pickle.dumps(sensors_data_cp)
                    #print(web_data)
                except:
                    print( "dump failed at " + time.strftime('%Y-%m-%d',time.localtime(time.time())) )
                done = True
                flag = 0
                cnt = 0
    except:
        break
bt_port.close()
