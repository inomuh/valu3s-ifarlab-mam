# VALUE3S IFARLAB Marver Advance Monitoring

The repository is about MARVERS ADVANCE MONITORING.

## Features

It visualize environment, gazebo world and OMA, ODT and OHT data.

## Requirements
Ubuntu 20.04
ROS Noetic 
python3
python3-pip

requirements.txt:
PyQt5
pyqt5-tools
reelay
PyYAML
websocket-client
rospy-message-converter
rospkg
netifaces
nodeeditor
psycopg2
pyqtgraph

# Installation

'''
$ git clone https://github.com/inomuh/valu3s-ifarlab-mam.git

$ sudo apt install python3-pip

$ pip3 install -r requirements.txt
'''

# How to Start 

You have to 
'''
$ sudo nano /etc/hosts
'''
add ROS_MASTER_IP and ROS MASTER NAME to the hosts file.
example: 
192.168.3.4 otapc

'''
$ export ROS_MASTER_URI=http://<ROS_MASTER_IP>:11311

$ python3 "dir_to_marver_advance_monitoring"/marver_advance_monitoring.py
'''

# The Contributors
- [Didem Özüpek Taş](https://github.com/DidemOzupekTas) 

## License
See [LICENSE-APACHE](LICENSE-APACHE) and [LICENSE-MIT](LICENSE-MIT) for details.