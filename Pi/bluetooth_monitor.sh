#!/bin/sh

:<<EOF
for bluetooth in /home/pi/person_pose_estimator/bluetooth
EOF

while true
do
	ps -ef | grep bt_S.py | grep -v grep
	if [ "$?" -eq 1 ]
	then
		python3 /home/pi/person_pose_estimator/bt_S.py > /home/pi/log.txt
		echo "process has been restarted!"
	else
		echo "process already started!"
	fi
	sleep 15
done
