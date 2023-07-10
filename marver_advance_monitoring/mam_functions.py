import numpy as np
import cv2
from cv_bridge import CvBridge
from PyQt5 import QtCore, QtGui, QtWidgets

import rospy

bridge = CvBridge() # bridge used to convert camera data

def calculate_dist(data):
    """
    Calculation distance model position
    """
    fin_pose = np.array([float(data["fin_pose"]["x"]), \
                    float(data["fin_pose"]["y"]) -0.26, \
                    float(data["fin_pose"]["z"])])
    strt_pose = np.array([float(data["strt_pose"]["x"]), \
                    float(data["strt_pose"]["y"]) -0.26, \
                    float(data["strt_pose"]["z"])])
    dist = str(round(np.linalg.norm(fin_pose-strt_pose),2))
    return dist

def draw_frames(msg, object, label_object):
    """
    Show frame of camera on ui
    """
    try:
        width = object.width()
        height = object.height()
        label_object.resize(width, height)
        camera_map = cv2.resize(bridge.imgmsg_to_cv2(msg, \
                            desired_encoding="passthrough"), (width,height))
        bridge.cv2_to_imgmsg
        bytesPerLine = int(camera_map.nbytes / height)
        qimg = QtGui.QImage(camera_map.data, camera_map.shape[1], camera_map.shape[0], \
                        bytesPerLine, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        label_object.setPixmap(pixmap)
        label_object.show()
    except Exception as exp:
        print(exp)

def camera_combo_box_clicked(digital_twin_cam_topics_list):
    """
    Gazebo camera combobox click function
    """
    try:
        # update rostopic list
        digital_twin_cam_topics_list.clear()
        topic_list = []
        topic_list_dict = dict(rospy.get_published_topics())
        for key in topic_list_dict.keys():
            if topic_list_dict[key] == 'sensor_msgs/Image' and "outside" in key:
                topic_list.append(key)
        topic_list.sort()
        digital_twin_cam_topics_list.addItems(topic_list)
    except:
        pass

def net_graph_funcion(net_graphs, layout):
    """
    Function is used to hide or show selected graph
    """
    for index in range(len(net_graphs)):
        net_graphs[index].show()
        layout.addWidget(net_graphs[index], 0, index)

def group_show_func(item_arr, group_states):
    if group_states == "Normal":
        for hide_item in item_arr:
            hide_item.hide()
        group_states = "Maximized"
    else:
        for show_item in item_arr:
            show_item.show()
        group_states = "Normal"
    return group_states

def graph_funcion(layout, graph_list, grid_list, graph_element, name, func):
    """
    Function is used to hide or show selected graph
    """
    if func == "hide":
        try:
            graph_element.hide()
            graph_list.remove(name)
        except Exception as expt1:
            pass
    elif func == "show":
        try:
            index = graph_list.index(name)
            layout.addWidget(graph_element, grid_list[index][0]+1, \
                                        grid_list[index][1])
            graph_element.show()
        except Exception as expt2:
            pass
    return graph_list