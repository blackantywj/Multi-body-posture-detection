import socket
import threading
import pickle
from collections import deque
class Getdata():
	def __init__(self):
		self.receivedata = 0
		self.sensor_data_1 = 0
		self.sensor_data_2 = 0
		self.web_sensors_data_1_stack = deque(maxlen = 3)
		self.web_sensors_data_2_stack = deque(maxlen = 3)
	def tcp_server(self):
		s = socket.socket()
		host = '0.0.0.0'
		port = 10040
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		s.bind((host, port))
		s.listen(5)

		while True:
			sock, addr = s.accept()
			print('tcp client addr: ', addr)
			t = threading.Thread(target=self.tcplink, args=(sock, addr))
			t.start()
	def tcplink(self, sock, addr):
		print('Accept new connection from %s:%s...' % addr)
		global predictions_data
		# set predictions_data = 4, represent connect
		predictions_data = 4
		# in order to initial system every time, when a new connection is constructed, so set flag = 0
		global flag
		flag = 0

		while True:
			client_data = sock.recv(1024)
			#print(client_data)
			if not client_data:
				print('disconnect')

				# set predictions_data = 5, represent disconnect
				predictions_data = 5
				break
			try:
				self.receivedata = pickle.loads(client_data)
				#print(self.receivedata)
			except:
				print('pickle.loads error')
				continue
			self.sensor_data_1, self.sensor_data_2 = self.slice_data(self.receivedata)
			#print(self.sensor_data_1)
			#print(self.sensor_data_2)
			#web_data = predictions_decision_tree(sensor_data, velocity)
			#if len(self.sensor_data_1)!=0 or len(self.sensor_data_2)!=0:
			#if len(self.sensor_data_1)!=0:
			self.web_sensors_data_1_stack.appendleft(self.sensor_data_1)
			#if len(self.sensor_data_2)!=0:
			self.web_sensors_data_2_stack.appendleft(self.sensor_data_2)
		sock.close()
		print('Connection from %s:%s closed.' % addr)
	def slice_data(self, data):
		sensor_data_1=[]
		sensor_data_2=[]
		if data[1] == "00":
			roll = int(data[6] + data[7], 16)/100
			pitch = 0 - int(data[8] + data[9], 16)/100
			sensor_data_1.append("0")
			sensor_data_1.append(roll)
			sensor_data_1.append(pitch)
			if pitch >= -100:
				sensor_data_1.append("Innormal")
			else:
				sensor_data_1.append("Normal")
		elif data[1] == "01":
			roll = int(data[6] + data[7], 16)/100
			pitch = 0 - int(data[8] + data[9], 16)/100
			sensor_data_2.append("1")
			sensor_data_2.append(roll)
			sensor_data_2.append(pitch)
			if pitch >= -180:
				sensor_data_2.append("Innormal")
			else:
				sensor_data_2.append("Normal")
		return sensor_data_1, sensor_data_2
	def run(self):
		thread1 = threading.Thread(target=self.tcp_server, args=())
		thread1.start()
get_data = Getdata()
