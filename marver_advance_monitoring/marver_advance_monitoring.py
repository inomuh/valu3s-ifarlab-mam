# -*- coding: utf-8 -*-
"""
 Created by: PyQt5 UI code generator 5.14.1
"""
import subprocess
import json
import datetime
import os
import pyqtgraph as pg

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QEvent, QObject

import rospy
import rosnode
from sensor_msgs.msg import Image
from std_msgs.msg import String
from thread_class import *
from mam_functions import *

class Ui_MainWindow(QtWidgets.QMainWindow):
    """
    MARVER ADVANCE MONITORING USER INTERFACE
    """
    def __init__(self):
        super(Ui_MainWindow, self).__init__()
        rospy.init_node('valu3s_mam', anonymous=True)
        self.chkbx_publisher = rospy.Publisher("mam_chkbx", String, queue_size=10)
        rospy.Subscriber("odt_json", String, self.odt_callback)
        rospy.Subscriber("task_status", String, self.task_status_callback)
        rospy.Subscriber("env_cam", Image, self.env_camera_callback)

        # odt distance variable for distX_val
        self.dist0_val, self.dist1_val, self.dist2_val, self.dist3_val = None, None, None, None
        # odt distance array for graph distX_arr
        self.dist0_arr, self.dist1_arr, self.dist2_arr, self.dist3_arr = [], [], [], []

        # Graph flags use for open and close graphs according to checkbox choice
        self.odt_graph_flag, self.oma_graph_flag, self.oht_graph_flag, self.network_graph_flag = \
        True, True, True, True

        # OMA joint specs: position -> pose, velocity -> vel, acceleration -> acc, jerk -> jerk
        self.j1_pose, self.j2_pose, self.j3_pose, self.j4_pose, self.j5_pose, self.j6_pose =  [],\
            [], [], [], [], []
        self.j1_vel, self.j2_vel, self.j3_vel, self.j4_vel, self.j5_vel, self.j6_vel =  [],\
            [], [], [], [], []
        self.j1_acc, self.j2_acc, self.j3_acc, self.j4_acc, self.j5_acc, self.j6_acc =  [],\
            [], [], [], [], []
        self.j1_jerk, self.j2_jerk, self.j3_jerk, self.j4_jerk, self.j5_jerk, self.j6_jerk =  [],\
            [], [], [], [], []
        self.oma_pose_obj = [self.j1_pose,self.j2_pose,self.j3_pose,self.j4_pose,self.j5_pose,self.j6_pose]
        self.oma_vel_obj = [self.j1_vel,self.j2_vel,self.j3_vel,self.j4_vel,self.j5_vel,self.j6_vel]
        self.oma_acc_obj = [self.j1_acc,self.j2_acc,self.j3_acc,self.j4_acc,self.j5_acc,self.j6_acc]
        self.oma_jerk_obj = [self.j1_jerk,self.j2_jerk,self.j3_jerk,self.j4_jerk,self.j5_jerk,self.j6_jerk]

        self.selected_element = "joint1" # selected element for oma
        self.sub_camera = None # gazebo camera subscriber
        self.group_states = {} # dict used to maximize or make normal size for ui sections
        self.graph_list = [] # fills with selected graph name
        self.grid_list = [[0,0],[0,1],[1,0],[1,1]] # used for graph show and hide in grid
        self.net_graph_list = [] # fills with selected graph name
        self.net_graphs = []
        self.net_grid_list = [[0],[1],[2]] # used for graph show and hide in grid
        self.log_selected_topics = [] # selected topic lists used for rosbag

        self.odt_x = [] # x axis data list for odt graph
        self.odt_rv_data = [[],[]] # [x,y] list for odt runtime verification data

        self.oma_x = [] # x axis data list for oma graph
        self.oma_rv_data = [[],[]] # [x,y] list for oma runtime verification data

        self.oht_data = [[], []]  # [x,y] list for oht data
        # [x,y] list for oht runtime verification data
        self.oht_rv_data = [[], []]

        self.ad_result_data = [[], []] # [x,y] list for adrv data
        self.ad_state_data = [[],[]]
        self.adrv_rv_data = [[], []] # [x,y] list for adrv runtime verification data

        # logo files
        marvers_logo = 'mam_marvers_logo.png'
        ifarlab_logo = 'mam_ifarlab_logo.png'
        valu3s_logo = 'mam_value3s_logo.png'

        # logo directory variables
        file_dir = os.path.abspath(__file__)
        file_dir_list = file_dir.split("/")
        file_dir = ""
        for dir_index in range(1,len(file_dir_list)-1):
            file_dir += "/" + file_dir_list[dir_index]
        self.marvers_logo_file = file_dir + "/logo/" + marvers_logo
        self.ifarlab_logo_file = file_dir + "/logo/" + ifarlab_logo
        self.valu3s_logo_file = file_dir + "/logo/" + valu3s_logo

        main_window.setObjectName("MainWindow")
        main_window.resize(1415, 1079)
        self.centralwidget = QtWidgets.QWidget(main_window)
        self.centralwidget.setObjectName("centralwidget")
        self.custom_font=QtGui.QFont()
        self.grid_layout_7 = QtWidgets.QGridLayout(self.centralwidget)
        self.grid_layout_7.setObjectName("grid_layout_7")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.horizontal_layout_23 = QtWidgets.QHBoxLayout()
        self.horizontal_layout_23.setObjectName("horizontal_layout_23")
        self.grid_layout_8 = QtWidgets.QGridLayout()
        self.grid_layout_8.setObjectName("grid_layout_8")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setMaximumSize(QtCore.QSize(100, 65))
        self.label_6.setText("")
        self.label_6.setPixmap(QtGui.QPixmap(self.valu3s_logo_file))
        self.label_6.setScaledContents(True)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.grid_layout_8.addWidget(self.label_6, 1, 0, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setMaximumSize(QtCore.QSize(65, 65))
        self.label_5.setText("")
        self.label_5.setPixmap(QtGui.QPixmap(self.ifarlab_logo_file))
        self.label_5.setScaledContents(True)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.grid_layout_8.addWidget(self.label_5, 1, 2, 1, 1)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap(self.marvers_logo_file))
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.grid_layout_8.addWidget(self.label, 1, 1, 1, 1)
        self.graph_group = QtWidgets.QGroupBox(self.centralwidget)
        size_policy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, \
                                           QtWidgets.QSizePolicy.Expanding)
        size_policy.setHorizontalStretch(0)
        size_policy.setVerticalStretch(0)
        size_policy.setHeightForWidth(self.graph_group.sizePolicy().hasHeightForWidth())
        self.graph_group.setSizePolicy(size_policy)
        self.graph_group.setObjectName("graph_group")
        self.graph_group.setAlignment(QtCore.Qt.AlignCenter)
        self.graph_group.setMinimumSize(QtCore.QSize(0, 435))

        self.grid_layout = QtWidgets.QGridLayout(self.graph_group)
        self.grid_layout.setObjectName("grid_layout")
        self.chkbx_grid = QtWidgets.QGridLayout()
        self.chkbx_grid.setObjectName("chkbx_grid")
        self.network_traffic_graph_chkbx = QtWidgets.QCheckBox(self.graph_group)
        self.network_traffic_graph_chkbx.setObjectName("network_traffic_graph_chkbx")
        self.chkbx_grid.addWidget(self.network_traffic_graph_chkbx, 0, 1, 1, 1)
        self.odt_graph_chkbx = QtWidgets.QCheckBox(self.graph_group)
        self.odt_graph_chkbx.setObjectName("odt_graph_chkbx")
        self.chkbx_grid.addWidget(self.odt_graph_chkbx, 0, 2, 1, 1)
        self.oht_graph_chkbx = QtWidgets.QCheckBox(self.graph_group)
        self.oht_graph_chkbx.setObjectName("oht_graph_chkbx")
        self.chkbx_grid.addWidget(self.oht_graph_chkbx, 0, 6, 1, 1)
        self.oma_graph_chkbx = QtWidgets.QCheckBox(self.graph_group)
        self.oma_graph_chkbx.setObjectName("oma_graph_chkbx")
        self.chkbx_grid.addWidget(self.oma_graph_chkbx, 0, 3, 1, 1)
        self.oma_joint_list = QtWidgets.QComboBox(self.graph_group)
        self.oma_joint_list.setObjectName("oma_joint_list")
        self.chkbx_grid.addWidget(self.oma_joint_list, 0, 4, 1, 1)
        self.oma_joint_list.addItem("position")
        self.oma_joint_list.addItem("velocity")
        self.oma_joint_list.addItem("acceleration")
        self.oma_joint_list.addItem("jerk")
        self.oma_joint_list.hide()
        self.odt_rv_chkbx = QtWidgets.QCheckBox(self.graph_group)
        self.odt_rv_chkbx.setObjectName("odt_rv_chkbx")
        self.chkbx_grid.addWidget(self.odt_rv_chkbx, 1, 2, 1, 1)
        self.odt_rv_chkbx.hide()
        self.oma_rv_chkbx = QtWidgets.QCheckBox(self.graph_group)
        self.oma_rv_chkbx.setObjectName("oma_rv_chkbx")
        self.chkbx_grid.addWidget(self.oma_rv_chkbx, 1, 3, 1, 1)
        self.oma_rv_chkbx.hide()
        self.oht_rv_chkbx = QtWidgets.QCheckBox(self.graph_group)
        self.oht_rv_chkbx.setObjectName("oht_rv_chkbx")
        self.chkbx_grid.addWidget(self.oht_rv_chkbx, 1, 6, 1, 1)
        self.oht_rv_chkbx.hide()
        self.adrv_chkbx = QtWidgets.QCheckBox(self.graph_group)
        self.adrv_chkbx.setObjectName("adrv")
        self.chkbx_grid.addWidget(self.adrv_chkbx, 1, 1, 1, 1)
        self.adrv_rv_chkbx = QtWidgets.QCheckBox(self.graph_group)
        self.adrv_rv_chkbx.setObjectName("adrv_rv")
        self.chkbx_grid.addWidget(self.adrv_rv_chkbx, 2, 1, 1, 1)

        # Set the text style using CSS
        self.odt_graph_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")
        self.network_traffic_graph_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")
        self.oma_graph_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")
        self.oht_graph_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")
        self.oma_rv_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")
        self.odt_rv_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")
        self.oht_rv_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")
        self.oma_joint_list.setStyleSheet("QComboBox { font-weight: normal;}")
        self.adrv_chkbx.setStyleSheet("QComboBox { font-weight: normal;}")
        self.adrv_rv_chkbx.setStyleSheet("QComboBox { font-weight: normal;}")

        # connect functions with objects
        self.odt_graph_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.network_traffic_graph_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.oma_graph_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.oht_graph_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.oma_joint_list.currentIndexChanged.connect(self.joint_select)
        self.odt_rv_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.oma_rv_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.oht_rv_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.adrv_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.adrv_rv_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.grid_layout.addLayout(self.chkbx_grid, 0, 0, 1, 1)

        self.net_grid = QtWidgets.QGridLayout()
        self.net_grid.setObjectName("net_grid")
        self.net_plot = pg.PlotWidget(self.graph_group)
        net_size_policy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        net_size_policy.setHorizontalStretch(1)
        net_size_policy.setVerticalStretch(1)
        net_size_policy.setHeightForWidth(self.net_plot.sizePolicy().hasHeightForWidth())
        self.net_plot.setSizePolicy(net_size_policy)
        self.net_plot.setObjectName("net_plot")
        # self.net_grid.addWidget(self.net_plot, 0, 0, 1, 1)
        # set up the data
        self.net_data = [[],[]]
        self.graph_net = self.net_plot.plot(pen='r')
        # customize the plot
        self.net_plot.setTitle("Network Traffic")
        self.net_plot.setLabel("left", "Frequency")

        self.adrv_plot = pg.PlotWidget(self.graph_group)
        self.adrv_plot.setSizePolicy(net_size_policy)
        self.adrv_plot.setObjectName("adrv_plot")
        self.adrv_plot.addLegend()
        self.graph_result = self.adrv_plot.plot(pen='b', name="Result")
        self.graph_state = self.adrv_plot.plot(pen='g', name="Attack State")  
        # self.net_grid.addWidget(self.adrv_plot, 0, 1, 1, 1)
        self.adrv_rv_plot = pg.PlotWidget(self.graph_group)
        self.adrv_rv_plot.setSizePolicy(net_size_policy)
        self.adrv_rv_plot.setObjectName("adrv_rv_plot")
        self.graph_adrv_rv = self.adrv_rv_plot.plot(pen='r')
        # self.net_grid.addWidget(self.adrv_rv_plot, 0, 2, 1, 1)
        self.adrv_rv_plot.setTitle("AD RV Status")
        self.adrv_plot.setTitle("Anomaly Detection")

        self.odt_grid = QtWidgets.QGridLayout()
        self.odt_grid.setObjectName("odt_grid")
        self.odt_plot = pg.PlotWidget(self.graph_group)
        self.odt_plot.setObjectName("odt_plot")
        self.odt_plot.addLegend()
        self.odt_rv_plot = pg.PlotWidget(self.graph_group)
        self.odt_rv_plot.setObjectName("odt_rv_plot")
        self.odt_rv_plot.setTitle("ODT RV Status")
        self.odt_rv_plot.setLabel("left", "Frequency")
        self.graph_odt_rv = self.odt_rv_plot.plot(pen='r')
        self.odt_grid.addWidget(self.odt_rv_plot, 0, 1, 1, 1)

        self.odt_plot.setTitle("Joint-Chassis Distance (m)")
        self.graph_odt_d0 = self.odt_plot.plot(pen='r', name="dist1")
        self.graph_odt_d1 = self.odt_plot.plot(pen='w', name="dist2")
        self.graph_odt_d2 = self.odt_plot.plot(pen='b', name="dist3")
        self.graph_odt_d3 = self.odt_plot.plot(pen='g', name="dist4")
        self.odt_grid.addWidget(self.odt_plot, 0, 0, 1, 1)

        self.oma_grid = QtWidgets.QGridLayout()
        self.oma_grid.setObjectName("oma_grid")
        self.oma_plot = pg.PlotWidget(self.graph_group)
        self.oma_plot.setObjectName("oma_plot")
        self.oma_plot.addLegend()
        self.oma_grid.addWidget(self.oma_plot, 0, 0, 1, 1)
        self.oma_rv_plot = pg.PlotWidget(self.graph_group)
        self.oma_rv_plot.setObjectName("oma_rv_plot")
        self.oma_rv_plot.setTitle("OMA RV Status")
        self.graph_oma_rv = self.oma_rv_plot.plot(pen='r')

        self.oma_grid.addWidget(self.oma_rv_plot, 0, 1, 1, 1)
        self.arr_j1 = self.oma_plot.plot(pen='r', name="joint1")
        self.arr_j2 = self.oma_plot.plot(pen='m', name="joint2")
        self.arr_j3 = self.oma_plot.plot(pen='b', name="joint3")
        self.arr_j4 = self.oma_plot.plot(pen='g', name="joint4")
        self.arr_j5 = self.oma_plot.plot(pen='y', name="joint5")
        self.arr_j6 = self.oma_plot.plot(pen='c', name="joint6")
        self.oma_array = [self.arr_j1,self.arr_j2,self.arr_j3,self.arr_j4,self.arr_j5,self.arr_j6]

        self.oht_grid = QtWidgets.QGridLayout()
        self.oht_grid.setObjectName("oht_grid")
        self.oht_plot = pg.PlotWidget(self.graph_group)
        self.oht_plot.setObjectName("oht_plot")
        self.oht_plot.setTitle("OHT Min Distance")
        self.graph_oht = self.oht_plot.plot(pen='b')
        self.oht_grid.addWidget(self.oht_plot, 0, 0, 1, 1)
        self.oht_rv_plot = pg.PlotWidget(self.graph_group)
        self.oht_rv_plot.setObjectName("oht_rv_plot")
        self.oht_rv_plot.setTitle("OHT RV Status")
        self.graph_oht_rv = self.oht_rv_plot.plot(pen='r')
        self.oht_plot.addLegend()
        self.oht_grid.addWidget(self.oht_rv_plot, 0, 1, 1, 1)

        self.network_area = QtWidgets.QScrollArea()
        self.network_area.setBackgroundRole(QtGui.QPalette.Dark)
        self.network_area.setWidgetResizable(True)
        self.network_area.setLayout(self.net_grid)
        self.odt_area = QtWidgets.QScrollArea()
        self.odt_area.setBackgroundRole(QtGui.QPalette.Dark)
        self.odt_area.setWidgetResizable(True)
        self.odt_area.setLayout(self.odt_grid)
        self.oma_area = QtWidgets.QScrollArea()
        self.oma_area.setBackgroundRole(QtGui.QPalette.Dark)
        self.oma_area.setWidgetResizable(True)
        self.oma_area.setLayout(self.oma_grid)
        self.oht_area = QtWidgets.QScrollArea()
        self.oht_area.setBackgroundRole(QtGui.QPalette.Dark)
        self.oht_area.setWidgetResizable(True)
        self.oht_area.setLayout(self.oht_grid)

        self.horizontal_layout_23.addWidget(self.graph_group)
        self.log_group = QtWidgets.QGroupBox(self.centralwidget)
        self.log_group.setMaximumSize(QtCore.QSize(220, 220))
        self.log_group.setAlignment(QtCore.Qt.AlignCenter)
        self.log_group.setObjectName("log_group")

        self.grid_layout_6 = QtWidgets.QGridLayout(self.log_group)
        self.grid_layout_6.setObjectName("grid_layout_6")
        self.label_4 = QtWidgets.QLabel(self.log_group)
        self.label_4.setObjectName("label_4")
        self.grid_layout_6.addWidget(self.label_4, 2, 0, 1, 1)
        # Set the text style using CSS
        self.label_4.setStyleSheet("QCheckBox { font-weight: normal;}")

        self.log_button = QtWidgets.QPushButton(self.log_group)
        self.log_button.setObjectName("log_button")
        self.grid_layout_6.addWidget(self.log_button, 5, 0, 1, 3)
        self.selected_topics = QtWidgets.QRadioButton(self.log_group)
        self.selected_topics.setObjectName("selected_topics")
        # Set the text style using CSS
        self.selected_topics.setStyleSheet("QCheckBox { font-weight: normal;}")
        self.selected_topics.clicked.connect(self.show_popup)

        self.grid_layout_6.addWidget(self.selected_topics, 4, 2, 1, 1)
        self.all_topic = QtWidgets.QRadioButton(self.log_group)
        self.all_topic.setObjectName("all_topic")
        # Set the text style using CSS
        self.all_topic.setStyleSheet("QCheckBox { font-weight: normal;}")
        self.all_topic.clicked.connect(self.slct_all_topics)

        self.grid_layout_6.addWidget(self.all_topic, 4, 0, 1, 1)
        self.log_name = QtWidgets.QLineEdit(self.log_group)
        self.log_name.setObjectName("log_name")
        self.grid_layout_6.addWidget(self.log_name, 2, 1, 1, 2)
        self.horizontal_layout_23.addWidget(self.log_group)
        self.log_button.clicked.connect(self.on_button_clicked)
        self.log_button_data = False
        self.graph_group.installEventFilter(self)
        self.group_states["graph_group"] = "Normal"

        self.gridLayout.addLayout(self.horizontal_layout_23, 2, 0, 1, 2)
        self.gridLayout.addLayout(self.grid_layout_8, 0, 0, 1, 2)
        self.digital_twin_group = QtWidgets.QGroupBox(self.centralwidget)
        self.digital_twin_group.setAlignment(QtCore.Qt.AlignCenter)
        self.digital_twin_group.setObjectName("digital_twin_group")
        self.digital_twin_group.installEventFilter(self)
        self.group_states["digital_twin_group"] = "Normal"

        self.grid_layout_4 = QtWidgets.QGridLayout(self.digital_twin_group)
        self.grid_layout_4.setObjectName("grid_layout_4")

        self.odt_joint_dist = QtWidgets.QCheckBox(self.digital_twin_group)
        self.odt_joint_dist.setObjectName("odt_joint_dist")
        # Set the text style using CSS
        self.odt_joint_dist.setStyleSheet("QCheckBox { font-weight: normal;}")
        self.grid_layout_4.addWidget(self.odt_joint_dist, 3, 2, 1, 1)

        self.odt_cylinder = QtWidgets.QCheckBox(self.digital_twin_group)
        self.odt_cylinder.setObjectName("odt_cylinder")
        self.grid_layout_4.addWidget(self.odt_cylinder, 2, 2, 1, 1)
        # Set the text style using CSS
        self.odt_cylinder.setStyleSheet("QCheckBox { font-weight: normal;}")

        self.odt_cylinder.stateChanged.connect(self.get_chkbx_value)
        self.odt_joint_dist.stateChanged.connect(self.get_chkbx_value)

        self.digital_twin_cam_topics_list = QtWidgets.QComboBox(self.digital_twin_group)
        self.digital_twin_cam_topics_list.setMaximumSize(QtCore.QSize(150, 16777215))
        self.digital_twin_cam_topics_list.setObjectName("digital_twin_cam_topics_list")
        # Set the text style using CSS
        self.digital_twin_cam_topics_list.setStyleSheet("QComboBox { font-weight: normal;}")
        self.grid_layout_4.addWidget(self.digital_twin_cam_topics_list, 1, 2, 1, 1)
        camera_combo_box_clicked(self.digital_twin_cam_topics_list)
        self.digital_twin_cam_topics_list.currentIndexChanged.connect(self.camera_topic_selected)

        self.digital_twin_cam = QtWidgets.QFrame(self.digital_twin_group)
        size_policy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, \
                                           QtWidgets.QSizePolicy.Expanding)
        size_policy.setHorizontalStretch(0)
        size_policy.setVerticalStretch(0)
        size_policy.setHeightForWidth(self.digital_twin_cam.sizePolicy().hasHeightForWidth())
        self.digital_twin_cam.setSizePolicy(size_policy)
        self.digital_twin_cam.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.digital_twin_cam.setFrameShadow(QtWidgets.QFrame.Raised)
        self.digital_twin_cam.setObjectName("digital_twin_cam")
        self.digital_twin_cam.installEventFilter(self)
        self.digital_twin_cam_label = QtWidgets.QLabel(self.digital_twin_cam)
        self.digital_twin_cam_label.setScaledContents(True)
        self.grid_layout_4.addWidget(self.digital_twin_cam, 1, 1, 4, 1)

        self.grid_layout_2 = QtWidgets.QGridLayout()
        self.grid_layout_2.setObjectName("grid_layout_2")

        self.odt_text = QtWidgets.QPlainTextEdit(self.digital_twin_group)
        self.odt_text_label = QtWidgets.QLabel(self.odt_text)
        self.odt_text_label.setMinimumSize(QtCore.QSize(160, 100))
        # Set the text style using CSS
        self.odt_text_label.setStyleSheet("QComboBox { font-weight: normal;}")

        self.odt_text.setMaximumSize(QtCore.QSize(170, 16777215))
        self.odt_text.setObjectName("odt_text")
        self.grid_layout_2.addWidget(self.odt_text, 1, 0, 1, 1)

        self.task_status = QtWidgets.QPlainTextEdit(self.digital_twin_group)
        self.task_text_label = QtWidgets.QLabel(self.task_status)
        self.task_text_label.setMinimumSize(QtCore.QSize(160, 100))
        # Set the text style using CSS
        self.task_text_label.setStyleSheet("QComboBox { font-weight: normal;}")
        self.task_status.setMaximumSize(QtCore.QSize(170, 16777215))
        self.task_status.setObjectName("task_status")
        self.grid_layout_2.addWidget(self.task_status, 3, 0, 1, 1)

        self.label_2 = QtWidgets.QLabel(self.digital_twin_group)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.label_2.setFont(self.custom_font)
        self.grid_layout_2.addWidget(self.label_2, 2, 0, 1, 1)

        self.label_3 = QtWidgets.QLabel(self.digital_twin_group)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.grid_layout_2.addWidget(self.label_3, 0, 0, 1, 1)
        self.label_3.setFont(self.custom_font)
        self.grid_layout_4.addLayout(self.grid_layout_2, 4, 2, 1, 1)

        self.gridLayout.addWidget(self.digital_twin_group, 1, 1, 1, 1)
        self.environment_group = QtWidgets.QGroupBox(self.centralwidget)
        size_policy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, \
                                           QtWidgets.QSizePolicy.Expanding)
        size_policy.setHorizontalStretch(0)
        size_policy.setVerticalStretch(0)
        size_policy.setHeightForWidth(self.environment_group.sizePolicy().hasHeightForWidth())
        self.environment_group.setSizePolicy(size_policy)
        self.environment_group.setMinimumSize(QtCore.QSize(0, 0))
        self.environment_group.setAlignment(QtCore.Qt.AlignCenter)
        self.environment_group.setObjectName("environment_group")
        self.environment_group.installEventFilter(self)

        self.grid_layout_3 = QtWidgets.QGridLayout(self.environment_group)
        self.grid_layout_3.setObjectName("grid_layout_3")
        self.env_cam = QtWidgets.QFrame(self.environment_group)
        self.group_states["environment_group"] = "Normal"

        self.env_cam.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.env_cam.setFrameShadow(QtWidgets.QFrame.Raised)
        self.env_cam.setObjectName("env_cam")

        self.env_cam_label = QtWidgets.QLabel(self.env_cam)
        self.env_cam_label.setScaledContents(True)
        self.env_cam_label.installEventFilter(self)

        ## Plot graphs thread
        self.class_net = GraphPlotNet()
        self.class_net.net_signal.connect(self.update_plot_net)
        self.class_net.start()
        self.net_graph_count = 0

        self.class_odt = GraphPlotOdt()
        self.class_odt.odt_signal.connect(self.update_plot_odt)
        self.class_odt.start()
        self.odt_graph_count = 0

        self.class_odt_rv = GraphPlotOdtRV()
        self.class_odt_rv.odt_rv_signal.connect(self.update_plot_odt_rv)
        self.class_odt_rv.start()
        self.odt_rv_graph_count = 0

        self.class_oma = GraphPlotOma()
        self.class_oma.oma_signal.connect(self.update_plot_oma)
        self.class_oma.start()
        self.oma_graph_count = 0

        self.class_oma_rv = GraphPlotOmaRV()
        self.class_oma_rv.oma_rv_signal.connect(self.update_plot_oma_rv)
        self.class_oma_rv.start()
        self.oma_rv_graph_count = 0
        
        self.class_oht = GraphPlotOht()
        self.class_oht.oht_signal.connect(self.update_plot_oht)
        self.class_oht.start()
        self.oht_graph_count = 0
        
        self.class_oht_rv = GraphPlotOhtRV()
        self.class_oht_rv.oht_rv_signal.connect(self.update_plot_oht_rv)
        self.class_oht_rv.start()
        self.oht_rv_graph_count = 0

        self.class_adrv = GraphPlotAdrv()
        self.class_adrv.adrv_signal.connect(self.update_plot_adrv)
        self.class_adrv.start()
        self.adrv_graph_count = 0

        self.class_adrv_rv = GraphPlotAdrvRV()
        self.class_adrv_rv.adrv_rv_signal.connect(self.update_plot_adrv_rv)
        self.class_adrv_rv.start()
        self.adrv_rv_graph_count = 0
        #########

        # Checkbox status publish thread
        self.chkbx_pub = ChkbxPub()
        self.chkbx_pub.signal_chkbx.connect(self.chkbx_pub_func)
        self.odt_msg = String()
        self.chkbx_pub.start()

        self.grid_layout_3.addWidget(self.env_cam, 1, 0, 1, 1)
        self.gridLayout.addWidget(self.environment_group, 1, 0, 1, 1)
        self.grid_layout_7.addLayout(self.gridLayout, 0, 0, 1, 1)
        main_window.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(main_window)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1415, 22))
        self.menubar.setObjectName("menubar")
        main_window.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(main_window)
        self.statusbar.setObjectName("statusbar")
        main_window.setStatusBar(self.statusbar)

        # set selected on combobox
        # self.odt_graph_chkbx.setChecked(True)
        self.network_traffic_graph_chkbx.setChecked(True)
        self.adrv_chkbx.setChecked(True)
        self.adrv_rv_chkbx.setChecked(True)
        self.digital_twin_cam_topics_list.setCurrentIndex(0)
        self.camera_topic_selected("outside1_camera/image_raw")
        self.custom_font.setBold(True)
        self.retranslate_ui(main_window)
        QtCore.QMetaObject.connectSlotsByName(main_window)

    def retranslate_ui(self, main_window):
        _translate = QtCore.QCoreApplication.translate
        main_window.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.graph_group.setTitle(_translate("MainWindow", "Graphs"))
        self.graph_group.setFont(self.custom_font)
        self.network_traffic_graph_chkbx.setText(_translate("MainWindow", "Network Traffic"))
        self.odt_graph_chkbx.setText(_translate("MainWindow", "ODT"))
        self.oma_graph_chkbx.setText(_translate("MainWindow", "OMA"))
        self.oht_graph_chkbx.setText(_translate("MainWindow", "OHT"))
        self.odt_rv_chkbx.setText(_translate("MainWindow", "ODT_RV_Stat"))
        self.oma_rv_chkbx.setText(_translate("MainWindow", "OMA_RV_Stat"))
        self.oht_rv_chkbx.setText(_translate("MainWindow", "OHT_RV_Stat"))
        self.adrv_chkbx.setText(_translate("MainWindow", "Adrv"))
        self.adrv_rv_chkbx.setText(_translate("MainWindow", "Ad_RV_Stat"))
        self.log_group.setTitle(_translate("MainWindow", "Record"))
        self.log_group.setFont(self.custom_font)
        self.label_4.setText(_translate("MainWindow", "Bag name:"))
        self.label_2.setText(_translate("MainWindow", "Task Status"))
        self.label_3.setText(_translate("MainWindow", "Joint Distances"))
        self.all_topic.setText(_translate("MainWindow", "All Topics"))
        self.selected_topics.setText(_translate("MainWindow", "Selected\n""  Topics"))
        self.log_button.setText(_translate("MainWindow", "Start"))
        self.digital_twin_group.setTitle(_translate("MainWindow", "Digital Twin"))
        self.digital_twin_group.setFont(self.custom_font)
        self.odt_joint_dist.setText(_translate("MainWindow", "Joint Distance"))
        self.odt_cylinder.setText(_translate("MainWindow", "Cylinder"))
        self.environment_group.setTitle(_translate("MainWindow", "Environment"))
        self.environment_group.setFont(self.custom_font)

    def odt_callback(self, msg):
        """
        odt_json ros callback function
        """
        json_data = json.loads(msg.data)
        self.dist0_val = calculate_dist(json_data["dist0"])
        self.dist1_val = calculate_dist(json_data["dist1"])
        self.dist2_val = calculate_dist(json_data["dist2"])
        self.dist3_val = calculate_dist(json_data["dist3"])
        data = "  dist-0: " + str(self.dist0_val) + "\n" + "  dist-1: " + str(self.dist1_val) + \
            "\n" + "  dist-2: " + str(self.dist2_val) + "\n" + "  dist-3: " + str(self.dist3_val) \
            + "\n"
        self.odt_text_label.setText(data)
        self.odt_text_label.setAlignment(Qt.AlignCenter)

    def eventFilter(self, source: QObject, event: QEvent) -> bool:
        """
        Method to capture the events for objects with an event filter installed.
        """
        if event.type() == QtCore.QEvent.MouseButtonDblClick:
            if source.objectName() == 'digital_twin_group':
                group_array = [self.graph_group, self.environment_group,self.log_group]
                self.group_states["digital_twin_group"] = group_show_func(group_array, \
                                                        self.group_states["digital_twin_group"])
            elif source.objectName() == 'environment_group':
                group_array = [self.graph_group, self.digital_twin_group,self.log_group]
                self.group_states["environment_group"] = group_show_func(group_array, \
                                                        self.group_states["environment_group"])
            elif source.objectName() == 'graph_group':
                group_array = [self.environment_group, self.digital_twin_group,self.log_group]
                self.group_states["graph_group"] = group_show_func(group_array, \
                                                    self.group_states["graph_group"])
            else:
                return super(Ui_MainWindow, self).eventFilter(source, event)
            return True
        else:
            return super(Ui_MainWindow, self).eventFilter(source, event)

    def dt_camera_callback(self, msg):
        """
        Gazebo camera callback
        """
        try:
            draw_frames(msg, self.digital_twin_cam, self.digital_twin_cam_label)
        except:
            pass

    def env_camera_callback(self, msg):
        """
        Environment camera callback
        """
        try:
            draw_frames(msg, self.env_cam, self.env_cam_label)
        except:
            pass

    def camera_topic_selected(self, selected_topic):
        """
        Seletion of camera topic from combobox
        """
        selected_topic = self.digital_twin_cam_topics_list.currentText()

        if selected_topic != '':
            if self.sub_camera is not None:
                self.sub_camera.unregister()
            self.sub_camera = rospy.Subscriber(selected_topic, Image, self.dt_camera_callback)

    def task_status_callback(self, msg):
        """
        task_status ros callback
        """
        json_data = json.loads(msg.data.replace("'","\""))
        task_status = json_data["task_status"]
        task_id = json_data["active_id"]
        data = "Task Status: \n" + task_status +"\n\n" +\
            "Task ID: " + task_id
        self.task_text_label.setText(data)
        self.task_text_label.setAlignment(Qt.AlignCenter)

    def update_plot_net(self, data):
        """
        Plotting topic frequency data
        """
        self.net_graph_count += 1
        # add the new data to the plot
        self.net_data[0].append(self.net_graph_count)
        self.net_data[1].append(data)
        if self.net_graph_count > 100:
            self.net_plot.setXRange(self.net_graph_count-100,self.net_graph_count)
        self.graph_net.setData(self.net_data[0], self.net_data[1])

    def update_plot_odt(self, dist_arr):
        """
        Plotting ODT data
        """
        self.odt_graph_count += 1
        # add the new data to the plot
        self.odt_x.append(self.odt_graph_count)
        self.dist0_arr.append(dist_arr[0]) if dist_arr[0] else self.dist0_arr.append(0)
        self.dist1_arr.append(dist_arr[1]) if dist_arr[1] else self.dist1_arr.append(0)
        self.dist2_arr.append(dist_arr[2]) if dist_arr[2] else self.dist2_arr.append(0)
        self.dist3_arr.append(dist_arr[3]) if dist_arr[3] else self.dist3_arr.append(0)
        if self.odt_graph_count > 100:
            self.odt_plot.setXRange(self.odt_graph_count-100,self.odt_graph_count)
        self.graph_odt_d0.setData(self.odt_x, self.dist0_arr)
        self.graph_odt_d1.setData(self.odt_x, self.dist1_arr)
        self.graph_odt_d2.setData(self.odt_x, self.dist2_arr)
        self.graph_odt_d3.setData(self.odt_x, self.dist3_arr)

    def update_plot_odt_rv(self, data):
        """
        Plotting ODT runtime verification data
        """
        self.odt_rv_graph_count += 1
        # add the new data to the plot
        self.odt_rv_data[0].append(self.odt_rv_graph_count)
        self.odt_rv_data[1].append(data)
        if self.odt_rv_graph_count > 100:
            self.odt_rv_plot.setXRange(self.odt_rv_graph_count-100,self.odt_rv_graph_count)
        self.graph_odt_rv.setData(self.odt_rv_data[0], self.odt_rv_data[1])

    def joint_select(self):
        """
        Get OMA selected element
        """
        self.selected_element = self.oma_joint_list.currentText()

    def update_plot_oma(self, msg):
        """
        Plotting OMA data
        """
        if len(msg.data) > 0:
            self.oma_graph_count += 1
            self.oma_x.append(self.odt_graph_count)
            if self.oma_graph_count > 100:
                self.oma_plot.setXRange(self.oma_graph_count-100,self.oma_graph_count)
            self.j1_pose.append(msg.data[0])
            self.j1_vel.append(msg.data[6])
            self.j1_acc.append(msg.data[12])
            self.j1_jerk.append(msg.data[18])
            self.j2_pose.append(msg.data[1])
            self.j2_vel.append(msg.data[7])
            self.j2_acc.append(msg.data[13])
            self.j2_jerk.append(msg.data[19])
            self.j3_pose.append(msg.data[2])
            self.j3_vel.append(msg.data[8])
            self.j3_acc.append(msg.data[14])
            self.j3_jerk.append(msg.data[20])
            self.j4_pose.append(msg.data[3])
            self.j4_vel.append(msg.data[9])
            self.j4_acc.append(msg.data[15])
            self.j4_jerk.append(msg.data[21])
            self.j5_pose.append(msg.data[4])
            self.j5_vel.append(msg.data[10])
            self.j5_acc.append(msg.data[16])
            self.j5_jerk.append(msg.data[22])
            self.j6_pose.append(msg.data[5])
            self.j6_vel.append(msg.data[11])
            self.j6_acc.append(msg.data[17])
            self.j6_jerk.append(msg.data[23])
            
            if self.selected_element == "position":
                for i in range(len(self.oma_array)):
                    self.oma_array[i].setData(self.oma_x, self.oma_pose_obj[i])
                self.oma_plot.setTitle("position")
            if self.selected_element == "velocity":
                for i in range(len(self.oma_array)):
                    self.oma_array[i].setData(self.oma_x, self.oma_vel_obj[i])
                self.oma_plot.setTitle("velocity")
            if self.selected_element == "acceleration":
                for i in range(len(self.oma_array)):
                    self.oma_array[i].setData(self.oma_x, self.oma_acc_obj[i])
                self.oma_plot.setTitle("acceleration")
            if self.selected_element == "jerk":
                for i in range(len(self.oma_array)):
                    self.oma_array[i].setData(self.oma_x, self.oma_jerk_obj[i])
                self.oma_plot.setTitle("jerk")

    def update_plot_oma_rv(self, data):
        """
        Plotting OMA runtim verification data
        """
        self.oma_rv_graph_count += 1
        # add the new data to the plot
        self.oma_rv_data[0].append(self.oma_rv_graph_count)
        self.oma_rv_data[1].append(data)
        if self.oma_rv_graph_count > 100:
            self.oma_rv_plot.setXRange(self.oma_rv_graph_count-100,self.oma_rv_graph_count)
        self.graph_oma_rv.setData(self.oma_rv_data[0], self.oma_rv_data[1])

    def update_plot_oht(self, data):
        """
        Plotting OHT data
        """
        self.oht_graph_count += 1
        # add the new data to the plot
        self.oht_data[0].append(self.oht_graph_count)
        self.oht_data[1].append(data)
        if self.oht_graph_count > 100:
            self.oht_plot.setXRange(self.oht_graph_count-100,self.oht_graph_count)
        self.graph_oht.setData(self.oht_data[0], self.oht_data[1])

    def update_plot_oht_rv(self, data):
        """
        Plotting OHT runtim verification data
        """
        self.oht_rv_graph_count += 1
        # add the new data to the plot
        self.oht_rv_data[0].append(self.oht_rv_graph_count)
        self.oht_rv_data[1].append(data)
        if self.oht_rv_graph_count > 100:
            self.oht_rv_plot.setXRange(self.oht_rv_graph_count-100,self.oht_rv_graph_count)
        self.graph_oht_rv.setData(self.oht_rv_data[0], self.oht_rv_data[1])

    def update_plot_adrv(self, data):
        """
        Plotting Adrv data
        """       
        self.adrv_graph_count += 1
        # add the new data to the plot
        self.ad_result_data[0].append(self.adrv_graph_count)
        self.ad_result_data[1].append(data[0])
        self.ad_state_data[0].append(self.adrv_graph_count)
        self.ad_state_data[1].append(data[1])
        if self.adrv_graph_count > 100:
            self.adrv_plot.setXRange(self.adrv_graph_count-100,self.adrv_graph_count)
        self.graph_result.setData(self.ad_result_data[0], self.ad_result_data[1])
        self.graph_state.setData(self.ad_state_data[0], self.ad_state_data[1])

    def update_plot_adrv_rv(self, data):
        """
        Plotting adrv runtim verification data
        """
        self.adrv_rv_graph_count += 1
        # add the new data to the plot
        self.adrv_rv_data[0].append(self.adrv_rv_graph_count)
        self.adrv_rv_data[1].append(data)
        if self.adrv_rv_graph_count > 100:
            self.adrv_rv_plot.setXRange(self.adrv_rv_graph_count-100,self.adrv_rv_graph_count)
        self.graph_adrv_rv.setData(self.adrv_rv_data[0], self.adrv_rv_data[1])

    def on_button_clicked(self):
        """
        Start and stop rosbag via button
        """
        if not self.log_button_data:
            self.log_button_data = True
            name = str(datetime.datetime.now().strftime("%d%m%Y_%H:%M:%S"))
            command = ['rosbag', 'record']
            if self.log_name.text() != "":
                name = self.log_name.text() + "_" + name
            if self.log_selected_topics:
                if self.log_selected_topics[0] == "all":
                    command.append("-a")
                    command.append("-O")
                    command.append(name)
                    subprocess.Popen(command,\
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                else:
                    for topic in self.log_selected_topics:
                        command.append(topic)
                    command.append("-O")
                    command.append(name)
                    subprocess.Popen(command,\
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.log_button.setText('Stop')
            else:
                msg = QtWidgets.QMessageBox()
                msg.setIcon(QtWidgets.QMessageBox.Warning)
                msg.setText("WARNING")
                msg.setInformativeText("No topic selected!")
                msg.setWindowTitle("Warning")
                msg.exec_()
        else:
            result = [x for x in rosnode.get_node_names() if "record" in x]
            rosnode.kill_nodes(result)
            self.log_button.setText('Start')
            self.log_button_data = False

    def get_chkbx_value(self):
        """
        Get checkbox status if graph selected show else hide
        """
        odt_graph = self.odt_graph_chkbx.isChecked()
        oma_graph = self.oma_graph_chkbx.isChecked()
        oht_graph = self.oht_graph_chkbx.isChecked()
        network_traffic = self.network_traffic_graph_chkbx.isChecked()
        odt_cylinder = self.odt_cylinder.isChecked()
        odt_joint = self.odt_joint_dist.isChecked()
        adrv_graph = self.adrv_chkbx.isChecked()
        adrv_rv_graph = self.adrv_rv_chkbx.isChecked()

        if odt_graph:
            self.odt_rv_chkbx.show()
            if "odt" not in self.graph_list:
                self.graph_list.append("odt")
                self.odt_rv_chkbx.setChecked(True)
        else:
            self.graph_list = graph_funcion(self.grid_layout, self.graph_list, \
                                            self.grid_list, self.odt_area, "odt", "hide")
            self.odt_rv_chkbx.hide()

        if oma_graph:
            self.oma_joint_list.show()
            self.oma_rv_chkbx.show()
            self.joint_select()
            if "oma" not in self.graph_list:
                self.graph_list.append("oma")
                self.oma_rv_chkbx.setChecked(True)
        else:
            self.graph_list = graph_funcion(self.grid_layout, self.graph_list, \
                                            self.grid_list, self.oma_area, "oma", "hide")
            self.oma_joint_list.hide()
            self.oma_rv_chkbx.hide()

        if oht_graph:
            self.oht_rv_chkbx.show()
            if "oht" not in self.graph_list:
                self.graph_list.append("oht")
                self.oht_rv_chkbx.setChecked(True)
        else:
            self.oht_rv_chkbx.hide()
            self.graph_list = graph_funcion(self.grid_layout, self.graph_list, \
                                            self.grid_list, self.oht_area, "oht", "hide")
        
        self.net_graphs = []

        if network_traffic or adrv_graph or adrv_rv_graph:
            if "net" not in self.graph_list:
                self.graph_list.append("net")
            # print(self.net_grid.geometry.width())

            if network_traffic:
                if "freq" not in self.net_graph_list:
                    self.net_graphs.append(self.net_plot)
            else:
                try:
                    self.net_plot.hide()
                    self.net_grid.removeWidget(self.net_plot)
                    self.net_graph_list.remove("freq")
                except:
                    pass

            if adrv_graph:
                if "adrv" not in self.net_graph_list:
                    self.net_graphs.append(self.adrv_plot)
            else:
                try:
                    self.adrv_plot.hide()
                    self.net_grid.removeWidget(self.adrv_plot)
                    self.net_graph_list.remove("adrv")
                except:
                    pass
            if adrv_rv_graph:
                if "adrv_rv" not in self.net_graph_list:
                    self.net_graphs.append(self.adrv_rv_plot)
            else:
                try:
                    self.adrv_rv_plot.hide()
                    self.net_grid.removeWidget(self.adrv_rv_plot)
                    self.net_graph_list.remove("adrv_rv")
                except:
                    pass
            net_graph_funcion(self.net_graphs, self.net_grid)
        else:
            self.graph_list = graph_funcion(self.grid_layout, self.graph_list, self.grid_list, \
                                            self.network_area, "net", "hide")

        self.graph_list = graph_funcion(self.grid_layout, self.graph_list, self.grid_list, \
            self.odt_area, "odt", "show")
        self.graph_list = graph_funcion(self.grid_layout, self.graph_list, self.grid_list, \
            self.oma_area, "oma", "show")
        self.graph_list = graph_funcion(self.grid_layout, self.graph_list, self.grid_list, \
            self.oht_area, "oht", "show")
        self.graph_list = graph_funcion(self.grid_layout, self.graph_list, self.grid_list, \
            self.network_area, "net", "show")
        odt_rv_graph = self.odt_rv_chkbx.isChecked()
        oma_rv_graph = self.oma_rv_chkbx.isChecked()
        oht_rv_graph = self.oht_rv_chkbx.isChecked()

        if odt_rv_graph: self.odt_rv_plot.show()
        else: self.odt_rv_plot.hide()

        if oma_rv_graph: self.oma_rv_plot.show()
        else: self.oma_rv_plot.hide()

        if oht_rv_graph: self.oht_rv_plot.show()
        else: self.oht_rv_plot.hide()

        self.chkbx_json = '{"graph":{"odt":' + str(odt_graph).lower() + ', "oht":' + \
            str(oht_graph).lower() + ', "network":' + str(network_traffic).lower() + \
            ', "oma":' + str(oma_graph).lower() + '}, "gui":{"cylinder":' + \
            str(odt_cylinder).lower() + ',"joint_dist":' + str(odt_joint).lower() + '}}'

    def filter_checkboxes(self, search_text):
        """
        Loop over all checkboxes in the popup window and show/hide them based on whether
        they contain the search text
        """
        for checkbox in self.popup.findChildren(QtWidgets.QCheckBox):
            if search_text.lower() in checkbox.text().lower(): checkbox.show()
            else: checkbox.hide()

    def slct_all_topics(self):
        self.log_selected_topics = ["all"]

    def show_popup(self):
        """
        Popup for selecting topics
        """
        checkbox_list = []
        # Create popup window
        self.popup = QtWidgets.QDialog(self)
        self.popup.setWindowTitle("Topics")
        popup_layout = QtWidgets.QVBoxLayout()
        self.popup.setLayout(popup_layout)
        self.log_selected_topics = []
        # Add a search bar to the popup window
        search_layout = QtWidgets.QHBoxLayout()
        search_label = QtWidgets.QLabel("Search:")
        self.search_edit = QtWidgets.QLineEdit()
        self.search_edit.textChanged.connect(self.filter_checkboxes)
        search_layout.addWidget(search_label)
        search_layout.addWidget(self.search_edit)
        popup_layout.addLayout(search_layout)
        popup_scroll_area = QtWidgets.QScrollArea()
        popup_scroll_widget = QtWidgets.QWidget()
        popup_scroll_layout = QtWidgets.QVBoxLayout()
        popup_scroll_widget.setLayout(popup_scroll_layout)
        popup_scroll_area.setWidgetResizable(True)
        popup_scroll_area.setWidget(popup_scroll_widget)
        popup_layout.addWidget(popup_scroll_area)
        topic_list_dict = dict(rospy.get_published_topics())
        for key in topic_list_dict.keys():
            checkbox = QtWidgets.QCheckBox(key)
            checkbox_list.append(checkbox)
            popup_scroll_layout.addWidget(checkbox)
        # Add button box with OK and Cancel buttons
        button_box = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | \
                                                QtWidgets.QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.popup.accept)
        button_box.rejected.connect(self.popup.reject)

        # Create layout for the popup window
        popup_layout.addWidget(button_box)

        # Show the popup window and wait for it to close
        if self.popup.exec_() == QtWidgets.QDialog.Accepted:
            for checkbox in checkbox_list:
                if checkbox.isChecked():
                    self.log_selected_topics.append(checkbox.text())
                    popup_scroll_layout.removeWidget(checkbox)
        else:
            self.log_selected_topics = []

    def chkbx_pub_func(self, data):
        """
        Checkbox status publisher function
        """
        self.odt_msg.data = data
        self.chkbx_publisher.publish(self.chkbx_json)

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main_window = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    main_window.show()
    sys.exit(app.exec_())
