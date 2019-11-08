# -*- coding:utf-8 -*-
import json
import re
import numpy
from flask import Flask, request, render_template, jsonify
import sqlite3
from getdata import get_data
app = Flask(__name__)


@app.route('/')
def home():
    return render_template('merge.html')

@app.route('/api/', methods=['GET', 'POST'])
def api():	#读取数据库并且将病人信息数据传递到前端
    if request.method == 'POST':
        data = request.form['data']
        nam=data[:-1]	#最后一位是id
        idd=int(data[-1])
        print(nam,idd)
        conn = sqlite3.connect('test.db')
        print("Opened database successfully")

        conn.execute("INSERT INTO PATIENT (ID,NAME) \
              VALUES ({},'{}')".format(int(idd),nam));
        conn.commit()
        return jsonify({'result': 1})
    else:
        try:
            conn = sqlite3.connect('test.db')	#数据库第一列是id，第二列为姓名
            print("Opened database successfully")
            cursor = conn.execute("SELECT id, name from PATIENT")
            data_pack = []	#用于将id与姓名打包成键值，并按顺序赋予数字键名
            count = 0	#数字键名
            for row in cursor:
                item = [str(count), str(row[0]) + row[1]]
                count = count + 1
                data_pack.append(item)
            data_pack.append(['number', count])
            data_pack = dict(data_pack)
            data= str(data_pack)
        except IndexError:
            pass
        return jsonify({"data":json.loads(data.replace("'", '"'))})



#获取前端传递过来的病人id，并按id删除病人数据
@app.route('/api2/', methods=['GET', 'POST'])
def api2():
    #从删库到跑路
    if request.method == 'POST':
        data = request.form['data']
        iddd=int(data[-1])
        conn = sqlite3.connect('test.db')
        conn.execute("DELETE from PATIENT where ID={};".format(iddd))
        conn.commit()
        if re.match('[0-9a-fA-F]*$', data):
            socket_server.response_data.appendleft(data.encode())
            return jsonify({'result': 1})
        else:
            return jsonify({'result': 0})
@app.route('/api3/', methods=['GET', 'POST'])
def api3():
    data0 = {"device": "s1", "id":"1", "roll": 0.0, "pitch":0.0, "status":"Normal"}
    if request.method == "GET":
        id = request.form['data']
        print("id is ", id)
        if id == "0":
            data0 = {"id":"0", "status":"Normal"}
        elif id == "1":
            data0 = {"id":"1", "status":"Normal"}
        print(data0)
        data0 = str(data0)
        return jsonify({"data":json.loads(data0.replace("'", '"'))})
    elif request.method == "POST":
        data1 = get_data.web_sensors_data_1_stack.pop()
        data2 = get_data.web_sensors_data_2_stack.pop()
        print(data1)
        print(data2)
        if len(data1)!= 0:
            data0["id"] = data1[0]
            data0["roll"] = data1[1]
            data0["pitch"] = data1[2]
            data0["status"] = data1[3]
        if len(data2) != 0:
            data0["id"] = data2[0]
            data0["roll"] = data2[1]
            data0["pitch"] = data2[2]
            data0["status"] = data2[3]
        print(data0)
        data0 = str(data0)
        return jsonify({"data":json.loads(data0.replace("'", '"'))})
if __name__ == '__main__':
    get_data.run()
    app.run(host='0.0.0.0', port=8090, debug=False, use_reloader=False)
