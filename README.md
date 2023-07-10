# VALUE3S IFARLAB Marver Advance Monitoring

The repository is about MARVERS ADVANCE MONITORING.

## Features

<p>It visualize environment, gazebo world and OMA, ODT and OHT data.</p>

<img
  src="img/Screenshot from 2023-05-24 14-13-07.png"
  alt="Alt text"
  title="Optional title"
  style="display: inline-block; margin: 0 auto; max-width: 300px">

## Requirements
- Ubuntu 20.04
- ROS Noetic 
- python3
- python3-pip

requirements.txt:
- PyQt5
- pyqt5-tools
- reelay
- PyYAML
- websocket-client
- rospy-message-converter
- rospkg
- netifaces
- nodeeditor
- psycopg2
- pyqtgraph

# Installation

```
$ git clone https://github.com/inomuh/valu3s-ifarlab-mam.git

$ sudo apt install python3-pip

$ pip3 install -r requirements.txt
```

# How to Start 

<p>You have to add your master ip to host list.</p>

```
$ sudo nano /etc/hosts
```
<p>Add ROS_MASTER_IP and ROS MASTER NAME to the hosts file.</p>

<p>example:</p>

192.168.3.4 otapc

```
$ export ROS_MASTER_URI=http://<ROS_MASTER_IP>:11311
```
<p>You should build adservice ros package in your workspace. If you don't have any workspace create catkin_ws and src folders. </p>

```
$ mkdir -p catkin_ws/src
```
<p>Cut and paste adservice folder in to src folder than build catkin_ws workspace.</p>

```
$ cd ~/catkin_ws && catkin_make
```
<p>You should source ros and your workspace from your bashrc file.</p>

```
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

<p>After these steps you can run MAM at your new terminal.</p>

```
$ python3 <dir_to_marver_advance_monitoring>/marver_advance_monitoring.py
```

## Published Topics:

- mam_chkbx (std_msgs/String)
<p>Checkbox status on user interface</p>

## Subscribed Topics:
- env_cam (sensor_msgs/Image)
<p>Environment camera topic</p>

- outside_camera1/image_raw (sensor_msgs/Image)
<p>Gazebo environment camera topic</p>

- outside_camera2/image_raw (sensor_msgs/Image)
<p>Gazebo environment camera topic</p>

- outside_camera3/image_raw (sensor_msgs/Image)
<p>Gazebo environment camera topic</p>

- odt_json (std_msgs/String)
<p>Data of ODT</p>

- mon_odt/monitor_verdict (std_msgs/String)
<p>ODT Runtime verification</p>

- mam_oma (std_msgs/String)
<p>Data of OMA</p>

- mon_oma/monitor_verdict (std_msgs/String)
<p>OMA Runtime verification</p>

- task_status (std_msgs/String)
<p>Data of system task status</p>

- adrv (adservice/adrv)
<p>Data of Adrv</p>

- mon_adrv/monitor_verdict (std_msgs/String)
<p>Adrv Runtime verification</p>

- oht (std_msgs/Float32)
<p>Closest distance of human joint to robot joint</p>

- mon_oht/monitor_verdict
<p>OHT Runtime verification</p>

# The Contributors
- [Didem Özüpek Taş](https://github.com/DidemOzupekTas) 

## License
See [LICENSE-APACHE](LICENSE-APACHE) and [LICENSE-MIT](LICENSE-MIT) for details.