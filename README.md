# person_pose_estimator

This project is used as a helper for hospital to know the status of each patient. It can detect patient's pose and positioning in real-time, because we place many known zigbee labels in each rooms and equip with a sensor for every patient. As the abnormal circumstance occurs, the nurse can find those changes from the monitor screen system. For web, we use flask as our development framework, and zigbee network is used in pi instead.

Also, in order to test conveniently, we have studied two versions of hardware, [bluetooth](README_Bluetooth.md) and [Zigbee](README_Zigbee.md). One is to demonstrate portably with a single IMU sensor , the other is to construct a cluster with many IMU sensors, as you know, zigbee is a special network to networking easily.

Now, Let's start to introduce the file structure of our project. Generally, our project is divided into three parts, with micro-controller(sensors, receivers and et. al.), Gateway(pi), and Server(web server).



## Get Started

### Pi

```
git clone https://github.com/ZhuChaozheng/person_pose_estimator
```

scan your sensor device to find out its bluetooth address

```
sudo bluetoothctl
scan on 
```

the screen will be printed as the following

```
[NEW] Device 20:19:03:27:28:21 HC-06
```

```
exit
```

change the bluetooth address in *rc.local*

```
vi Pi/rc.local
```

also modify the server IP in *bt_S.py*

Now, we have finished the deloyment of hardware layer. Next, Let us consider the web. As you can see, we have a folder of web, you should move it to your web server.

```
scp -r Server/web/ blackant@192.168.1.102:/var/www/html
```

### Ubuntu server

the following operations please switch to your web server

```
cd /var/www/html/web/
python3 manange.py
```