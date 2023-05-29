# -*- coding: utf-8 -*-
# Created by: PyQt5 UI code generator 5.14.1
# WARNING! All changes made in this file will be lost!
import subprocess
import numpy as np
import rosnode
import cv2
import json
import datetime
import pyqtgraph as pg
import os

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64MultiArray, Float32
from visualization_msgs.msg import Marker

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, \
    QLabel, QGridLayout, QScrollArea, QSizePolicy, QVBoxLayout, QFrame, QDialogButtonBox
from PyQt5.QtGui import QPixmap, QIcon, QImage, QPalette
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QEvent, QObject
from sensor_msgs.msg import Image

class Ui_MainWindow(QMainWindow):
    def setupUi(self, MainWindow):
        rospy.init_node('end_iz', anonymous=True)
        self.chkbx_publisher = rospy.Publisher("mam_chkbx", String, queue_size=10)
        rospy.Subscriber("odt_json", String, self.odt_callback)
        rospy.Subscriber("task_status", String, self.task_status_callback)
        rospy.Subscriber("env_cam", Image, self.env_camera_callback)
        # rospy.Subscriber("mon_odt/monitor_verdict", String, self.odt_rv_callback)

        self.d0, self.d1, self.d2, self.d3 = None, None, None, None
        self.odt_graph_flag, self.oma_graph_flag, self.oht_graph_flag, self.network_graph_flag = \
        True, True, True, True
        self.j1_pose, self.j2_pose, self.j3_pose, self.j4_pose, self.j5_pose, self.j6_pose =  [],\
            [], [], [], [], []
        self.j1_vel, self.j2_vel, self.j3_vel, self.j4_vel, self.j5_vel, self.j6_vel =  [],\
            [], [], [], [], []
        self.j1_acc, self.j2_acc, self.j3_acc, self.j4_acc, self.j5_acc, self.j6_acc =  [],\
            [], [], [], [], []
        self.j1_jerk, self.j2_jerk, self.j3_jerk, self.j4_jerk, self.j5_jerk, self.j6_jerk =  [],\
            [], [], [], [], []
        self.selected_joint = "joint1"
        self.sub_camera = None
        self.camera_msg = None
        self.group_states = {}
        self.bridge = CvBridge()
        self.graph_list = []
        self.grid_list = [[0,0],[0,1],[1,0],[1,1]]
        self.log_selected_topics = []

        # search files for logo
        marvers_logo = 'mam_marvers_logo.png'
        ifarlab_logo = 'mam_ifarlab_logo.png'
        valu3s_logo = 'mam_value3s_logo.png'

        marvers_logo_file = None
        ifarlab_logo_file = None
        valu3s_logo_file = None

        for root, dirs, files in os.walk(os.path.expanduser('~')):
            if marvers_logo in files:
                marvers_logo_file = root + "/" + marvers_logo
            if ifarlab_logo in files:
                ifarlab_logo_file = root + "/" + ifarlab_logo
            if valu3s_logo in files:
                valu3s_logo_file = root + "/" + valu3s_logo
            if marvers_logo_file and ifarlab_logo_file and valu3s_logo_file:
                break
        
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1415, 1079)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.horizontalLayout_23 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_23.setObjectName("horizontalLayout_23")

        self.gridLayout_8 = QtWidgets.QGridLayout()
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setMaximumSize(QtCore.QSize(100, 65))
        self.label_6.setText("")
        self.label_6.setPixmap(QtGui.QPixmap(valu3s_logo_file))
        self.label_6.setScaledContents(True)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.gridLayout_8.addWidget(self.label_6, 1, 0, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setMaximumSize(QtCore.QSize(65, 65))
        self.label_5.setText("")
        self.label_5.setPixmap(QtGui.QPixmap(ifarlab_logo_file))
        self.label_5.setScaledContents(True)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.gridLayout_8.addWidget(self.label_5, 1, 2, 1, 1)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap(marvers_logo_file))
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.gridLayout_8.addWidget(self.label, 1, 1, 1, 1)

        self.graph_group = QtWidgets.QGroupBox(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.graph_group.sizePolicy().hasHeightForWidth())

        self.myFont=QtGui.QFont()

        self.graph_group.setSizePolicy(sizePolicy)
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

        self.odt_graph_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS
        self.network_traffic_graph_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS
        self.oma_graph_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS
        self.oht_graph_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS
        self.oma_rv_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS
        self.odt_rv_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS
        self.oht_rv_chkbx.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS
        self.oma_joint_list.setStyleSheet("QComboBox { font-weight: normal;}")  # Set the text style using CSS

        self.odt_graph_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.network_traffic_graph_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.oma_graph_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.oht_graph_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.oma_joint_list.currentIndexChanged.connect(self.joint_select)
        self.odt_rv_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.oma_rv_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.oht_rv_chkbx.stateChanged.connect(self.get_chkbx_value)
        self.grid_layout.addLayout(self.chkbx_grid, 0, 0, 1, 1)

        self.net_plot = pg.PlotWidget(self.graph_group)
        self.net_plot.setObjectName("net_plot")
        # set up the data
        self.net_data = [[],[]]
        self.curve = self.net_plot.plot(pen='r')
        # customize the plot
        self.net_plot.setTitle("Network Traffic")
        self.net_plot.setLabel("left", "Frequency")

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
        self.odt_x = []
        self.odt_rv_data = [[],[]]
        self.d0data, self.d1data, self.d2data, self.d3data = [], [], [], []
        self.odt_plot.setTitle("Joint-Chassis Distance (m)")
        self.curve21 = self.odt_plot.plot(pen='r', name="dist1")
        self.curve22 = self.odt_plot.plot(pen='w', name="dist2")
        self.curve23 = self.odt_plot.plot(pen='b', name="dist3")
        self.curve24 = self.odt_plot.plot(pen='g', name="dist4")
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
        self.oma_rv_data = [[],[]]
        self.oma_grid.addWidget(self.oma_rv_plot, 0, 1, 1, 1)
        self.oma_x = [] 
        self.arr_j1 = self.oma_plot.plot(pen='r', name="joint1")
        self.arr_j2 = self.oma_plot.plot(pen='m', name="joint2")
        self.arr_j3 = self.oma_plot.plot(pen='b', name="joint3")
        self.arr_j4 = self.oma_plot.plot(pen='g', name="joint4")
        self.arr_j5 = self.oma_plot.plot(pen='y', name="joint5")
        self.arr_j6 = self.oma_plot.plot(pen='c', name="joint6")

        self.oht_grid = QtWidgets.QGridLayout()
        self.oht_grid.setObjectName("oht_grid")
        self.oht_plot = pg.PlotWidget(self.graph_group)
        self.oht_plot.setObjectName("oht_plot")
        self.oht_grid.addWidget(self.oht_plot, 0, 0, 1, 1)
        self.oht_rv_plot = pg.PlotWidget(self.graph_group)
        self.oht_rv_plot.setObjectName("oht_rv_plot")
        self.oht_rv_plot.setTitle("OHT RV Status")
        self.oht_rv_data = [[],[]]
        self.graph_oht_rv = self.oht_rv_plot.plot(pen='r')
        self.oht_plot.addLegend()
        self.oht_grid.addWidget(self.oht_rv_plot, 0, 1, 1, 1)

        self.QScrollArea_1 = QScrollArea()
        self.QScrollArea_1.setBackgroundRole(QPalette.Dark)
        self.QScrollArea_1.setWidgetResizable(True)
        self.QScrollArea_1.setWidget(self.net_plot)
        self.QScrollArea_2 = QScrollArea()
        self.QScrollArea_2.setBackgroundRole(QPalette.Dark)
        self.QScrollArea_2.setWidgetResizable(True)
        self.QScrollArea_2.setLayout(self.odt_grid)
        self.QScrollArea_3 = QScrollArea()
        self.QScrollArea_3.setBackgroundRole(QPalette.Dark)
        self.QScrollArea_3.setWidgetResizable(True)
        self.QScrollArea_3.setLayout(self.oma_grid)
        self.QScrollArea_4 = QScrollArea()
        self.QScrollArea_4.setBackgroundRole(QPalette.Dark)
        self.QScrollArea_4.setWidgetResizable(True)
        self.QScrollArea_4.setLayout(self.oht_grid)

        self.horizontalLayout_23.addWidget(self.graph_group)
        self.log_group = QtWidgets.QGroupBox(self.centralwidget)
        self.log_group.setMaximumSize(QtCore.QSize(220, 220))
        self.log_group.setAlignment(QtCore.Qt.AlignCenter)
        self.log_group.setObjectName("log_group")

        self.gridLayout_6 = QtWidgets.QGridLayout(self.log_group)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.label_4 = QtWidgets.QLabel(self.log_group)
        self.label_4.setObjectName("label_4")
        self.gridLayout_6.addWidget(self.label_4, 2, 0, 1, 1)
        self.label_4.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS

        self.log_button = QtWidgets.QPushButton(self.log_group)
        self.log_button.setObjectName("log_button")
        self.gridLayout_6.addWidget(self.log_button, 5, 0, 1, 3)
        self.selected_topics = QtWidgets.QRadioButton(self.log_group)
        self.selected_topics.setObjectName("selected_topics")
        self.selected_topics.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS
        self.selected_topics.clicked.connect(self.show_popup)

        self.gridLayout_6.addWidget(self.selected_topics, 4, 2, 1, 1)
        self.all_topic = QtWidgets.QRadioButton(self.log_group)
        self.all_topic.setObjectName("all_topic")
        self.all_topic.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS
        self.all_topic.clicked.connect(self.slct_all_topics)

        self.gridLayout_6.addWidget(self.all_topic, 4, 0, 1, 1)
        self.log_name = QtWidgets.QLineEdit(self.log_group)
        self.log_name.setObjectName("log_name")
        self.gridLayout_6.addWidget(self.log_name, 2, 1, 1, 2)
        self.horizontalLayout_23.addWidget(self.log_group)
        self.log_button.clicked.connect(self.on_button_clicked)
        self.log_button_data = False
        self.graph_group.installEventFilter(self)
        self.group_states["graph_group"] = "Normal"

        self.gridLayout.addLayout(self.horizontalLayout_23, 2, 0, 1, 2)
        self.gridLayout.addLayout(self.gridLayout_8, 0, 0, 1, 2)
        self.digital_twin_group = QtWidgets.QGroupBox(self.centralwidget)
        self.digital_twin_group.setAlignment(QtCore.Qt.AlignCenter)
        self.digital_twin_group.setObjectName("digital_twin_group")
        self.digital_twin_group.installEventFilter(self)
        self.group_states["digital_twin_group"] = "Normal"

        self.gridLayout_4 = QtWidgets.QGridLayout(self.digital_twin_group)
        self.gridLayout_4.setObjectName("gridLayout_4")
        
        self.odt_joint_dist = QtWidgets.QCheckBox(self.digital_twin_group)
        self.odt_joint_dist.setObjectName("odt_joint_dist")
        self.odt_joint_dist.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS
        self.gridLayout_4.addWidget(self.odt_joint_dist, 3, 2, 1, 1)
        
        self.odt_cylinder = QtWidgets.QCheckBox(self.digital_twin_group)
        self.odt_cylinder.setObjectName("odt_cylinder")
        self.gridLayout_4.addWidget(self.odt_cylinder, 2, 2, 1, 1)
        self.odt_cylinder.setStyleSheet("QCheckBox { font-weight: normal;}")  # Set the text style using CSS

        self.odt_cylinder.stateChanged.connect(self.get_chkbx_value)
        self.odt_joint_dist.stateChanged.connect(self.get_chkbx_value)

        self.digital_twin_cam_topics_list = QtWidgets.QComboBox(self.digital_twin_group)
        self.digital_twin_cam_topics_list.setMaximumSize(QtCore.QSize(150, 16777215))
        self.digital_twin_cam_topics_list.setObjectName("digital_twin_cam_topics_list")
        self.digital_twin_cam_topics_list.setStyleSheet("QComboBox { font-weight: normal;}")  # Set the text style using CSS
        self.gridLayout_4.addWidget(self.digital_twin_cam_topics_list, 1, 2, 1, 1)
        self.camera_combo_box_clicked()
        self.digital_twin_cam_topics_list.currentIndexChanged.connect(self.camera_topic_selected)

        self.digital_twin_cam = QtWidgets.QFrame(self.digital_twin_group)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.digital_twin_cam.sizePolicy().hasHeightForWidth())
        self.digital_twin_cam.setSizePolicy(sizePolicy)
        self.digital_twin_cam.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.digital_twin_cam.setFrameShadow(QtWidgets.QFrame.Raised)
        self.digital_twin_cam.setObjectName("digital_twin_cam")
        self.digital_twin_cam.installEventFilter(self)
        self.digital_twin_cam_label = QtWidgets.QLabel(self.digital_twin_cam)
        self.digital_twin_cam_label.setScaledContents(True)
        self.gridLayout_4.addWidget(self.digital_twin_cam, 1, 1, 4, 1)

        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")

        self.odt_text = QtWidgets.QPlainTextEdit(self.digital_twin_group)
        self.odt_text_label = QtWidgets.QLabel(self.odt_text)
        self.odt_text_label.setMinimumSize(QtCore.QSize(160, 100))
        self.odt_text_label.setStyleSheet("QComboBox { font-weight: normal;}")  # Set the text style using CSS

        self.odt_text.setMaximumSize(QtCore.QSize(170, 16777215))
        self.odt_text.setObjectName("odt_text")
        self.gridLayout_2.addWidget(self.odt_text, 1, 0, 1, 1)

        self.task_status = QtWidgets.QPlainTextEdit(self.digital_twin_group)
        self.task_text_label = QtWidgets.QLabel(self.task_status)
        self.task_text_label.setMinimumSize(QtCore.QSize(160, 100))
        self.task_text_label.setStyleSheet("QComboBox { font-weight: normal;}")  # Set the text style using CSS
        self.task_status.setMaximumSize(QtCore.QSize(170, 16777215))
        self.task_status.setObjectName("task_status")
        self.gridLayout_2.addWidget(self.task_status, 3, 0, 1, 1)

        self.label_2 = QtWidgets.QLabel(self.digital_twin_group)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.label_2.setFont(self.myFont)
        self.gridLayout_2.addWidget(self.label_2, 2, 0, 1, 1)

        self.label_3 = QtWidgets.QLabel(self.digital_twin_group)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 0, 0, 1, 1)
        self.label_3.setFont(self.myFont)

        self.gridLayout_4.addLayout(self.gridLayout_2, 4, 2, 1, 1)

        self.gridLayout.addWidget(self.digital_twin_group, 1, 1, 1, 1)
        self.environment_group = QtWidgets.QGroupBox(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.environment_group.sizePolicy().hasHeightForWidth())
        self.environment_group.setSizePolicy(sizePolicy)
        self.environment_group.setMinimumSize(QtCore.QSize(0, 0))
        self.environment_group.setAlignment(QtCore.Qt.AlignCenter)
        self.environment_group.setObjectName("environment_group")
        self.environment_group.installEventFilter(self)

        self.gridLayout_3 = QtWidgets.QGridLayout(self.environment_group)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.env_cam = QtWidgets.QFrame(self.environment_group)
        self.group_states["environment_group"] = "Normal"

        self.env_cam.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.env_cam.setFrameShadow(QtWidgets.QFrame.Raised)
        self.env_cam.setObjectName("env_cam")

        self.env_cam_label = QtWidgets.QLabel(self.env_cam)
        self.env_cam_label.setScaledContents(True)
        self.env_cam_label.installEventFilter(self)

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

        self.class_oht_rv = GraphPlotOhtRV()
        self.class_oht_rv.oht_rv_signal.connect(self.update_plot_oht_rv)
        self.class_oht_rv.start()
        self.oht_rv_graph_count = 0

        self.chkbx_pub = ChkbxPub()
        self.chkbx_pub.signal_chkbx.connect(self.chkbx_pub_func)
        self.odt_msg = String()
        self.chkbx_pub.start()

        self.gridLayout_3.addWidget(self.env_cam, 1, 0, 1, 1)
        self.gridLayout.addWidget(self.environment_group, 1, 0, 1, 1)
        self.gridLayout_7.addLayout(self.gridLayout, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1415, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        # set selected
        self.odt_graph_chkbx.setChecked(True)
        self.network_traffic_graph_chkbx.setChecked(True)
        self.digital_twin_cam_topics_list.setCurrentIndex(0)
        self.camera_topic_selected("outside1_camera/image_raw")

        self.myFont.setBold(True)
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.graph_group.setTitle(_translate("MainWindow", "Graphs"))
        self.graph_group.setFont(self.myFont)

        self.network_traffic_graph_chkbx.setText(_translate("MainWindow", "Network Traffic"))
        self.odt_graph_chkbx.setText(_translate("MainWindow", "ODT"))
        self.oma_graph_chkbx.setText(_translate("MainWindow", "OMA"))
        self.oht_graph_chkbx.setText(_translate("MainWindow", "OHT"))
        self.odt_rv_chkbx.setText(_translate("MainWindow", "ODT_RV_Stat"))
        self.oma_rv_chkbx.setText(_translate("MainWindow", "OMA_RV_Stat"))
        self.oht_rv_chkbx.setText(_translate("MainWindow", "OHT_RV_Stat"))

        self.log_group.setTitle(_translate("MainWindow", "Record"))
        self.log_group.setFont(self.myFont)

        self.label_4.setText(_translate("MainWindow", "Bag name:"))
        self.label_2.setText(_translate("MainWindow", "Task Status"))
        self.label_3.setText(_translate("MainWindow", "Joint Distances"))
        self.all_topic.setText(_translate("MainWindow", "All Topics"))
        self.selected_topics.setText(_translate("MainWindow", "Selected\n"
"  Topics"))
        self.log_button.setText(_translate("MainWindow", "Start"))
        self.digital_twin_group.setTitle(_translate("MainWindow", "Digital Twin"))
        self.digital_twin_group.setFont(self.myFont)
        self.odt_joint_dist.setText(_translate("MainWindow", "Joint Distance"))
        self.odt_cylinder.setText(_translate("MainWindow", "Cylinder"))
        self.environment_group.setTitle(_translate("MainWindow", "Environment"))
        self.environment_group.setFont(self.myFont)

    def odt_callback(self, msg):
        json_data = json.loads(msg.data)
        self.d0 = self.calculate_dist(json_data["dist0"])
        self.d1 = self.calculate_dist(json_data["dist1"])
        self.d2 = self.calculate_dist(json_data["dist2"])
        self.d3 = self.calculate_dist(json_data["dist3"])
        
        data = "  dist-0: " + str(self.d0) + "\n" + "  dist-1: " + str(self.d1) + "\n" + "  dist-2: "\
              + str(self.d2) + "\n" + "  dist-3: " + str(self.d3) + "\n"
        self.odt_text_label.setText(data)
        self.odt_text_label.setAlignment(Qt.AlignCenter)       
    
    def calculate_dist(self, data):
        fin_pose = np.array([float(data["fin_pose"]["x"]), \
                      float(data["fin_pose"]["y"]) -0.26, \
                      float(data["fin_pose"]["z"])])
        strt_pose = np.array([float(data["strt_pose"]["x"]), \
                      float(data["strt_pose"]["y"]) -0.26, \
                      float(data["strt_pose"]["z"])])
        dist = str(round(np.linalg.norm(fin_pose-strt_pose),2))
        return dist

    def camera_combo_box_clicked(self):
        try:
            # update rostopic list
            self.digital_twin_cam_topics_list.clear()
            topic_list = []
            topic_list_dict = dict(rospy.get_published_topics())

            for key in topic_list_dict.keys():
                if topic_list_dict[key] == 'sensor_msgs/Image' and "outside" in key:
                    topic_list.append(key)
            topic_list.sort()
            self.digital_twin_cam_topics_list.addItems(topic_list)

        except:
            pass

    def eventFilter(self, source: QObject, event: QEvent) -> bool:
        """
        Method to capture the events for objects with an event filter installed.
        :param source: The object for whom an event took place.
        :param event: The event that took place.
        :return: True if event is handled.
        """
        if event.type() == QtCore.QEvent.MouseButtonDblClick:
            if source.objectName() == 'digital_twin_group':
                if self.group_states["digital_twin_group"] == "Normal":
                    self.graph_group.hide()
                    self.environment_group.hide()
                    self.log_group.hide()
                    self.group_states["digital_twin_group"] = "Maximized"
                else:
                    self.graph_group.show()
                    self.environment_group.show()
                    self.log_group.show()
                    self.group_states["digital_twin_group"] = "Normal"
            if source.objectName() == 'environment_group':
                if self.group_states["environment_group"] == "Normal":
                    self.graph_group.hide()
                    self.digital_twin_group.hide()
                    self.log_group.hide()
                    self.group_states["environment_group"] = "Maximized"
                else:
                    self.graph_group.show()
                    self.digital_twin_group.show()
                    self.log_group.show()
                    self.group_states["environment_group"] = "Normal"
            if source.objectName() == 'graph_group':
                if self.group_states["graph_group"] == "Normal":
                    self.digital_twin_group.hide()
                    self.environment_group.hide()
                    self.log_group.hide()
                    self.group_states["graph_group"] = "Maximized"
                else:
                    self.digital_twin_group.show()
                    self.environment_group.show()
                    self.log_group.show()
                    self.group_states["graph_group"] = "Normal"
            else:
                return super(Ui_MainWindow, self).eventFilter(source, event)
            return True
        else:
            return super(Ui_MainWindow, self).eventFilter(source, event)

    def camera_topic_selected(self, selected_topic):
        selected_topic = self.digital_twin_cam_topics_list.currentText()

        if selected_topic != '':
            if self.sub_camera is not None:
                self.sub_camera.unregister()
            self.sub_camera = rospy.Subscriber(selected_topic, Image, self.dt_camera_callback)

    def joint_select(self):
        self.selected_joint = self.oma_joint_list.currentText()

    def dt_camera_callback(self, msg):
        try:
            self.draw_frames(msg, self.digital_twin_cam, self.digital_twin_cam_label)
        except:
            pass

    def env_camera_callback(self, msg):
        try:
            self.draw_frames(msg, self.env_cam, self.env_cam_label)
        except:
            pass

    def task_status_callback(self, msg):
        json_data = json.loads(msg.data.replace("'","\""))
        task_status = json_data["task_status"]
        task_id = json_data["active_id"]
        data = "Task Status: \n" + task_status +"\n\n" +\
            "Task ID: " + task_id 
        self.task_text_label.setText(data)
        self.task_text_label.setAlignment(Qt.AlignCenter)

    def update_plot_net(self, x):
        self.net_graph_count += 1
        # add the new data to the plot
        self.net_data[0].append(self.net_graph_count)
        self.net_data[1].append(x)
        if self.net_graph_count > 100:
            self.net_plot.setXRange(self.net_graph_count-100,self.net_graph_count)
        self.curve.setData(self.net_data[0], self.net_data[1])

    def update_plot_odt_rv(self, x):
        self.odt_rv_graph_count += 1
        # add the new data to the plot
        self.odt_rv_data[0].append(self.odt_rv_graph_count)
        self.odt_rv_data[1].append(x)
        if self.odt_rv_graph_count > 100:
            self.odt_rv_plot.setXRange(self.odt_rv_graph_count-100,self.odt_rv_graph_count)
        self.graph_odt_rv.setData(self.odt_rv_data[0], self.odt_rv_data[1])

    def update_plot_oma_rv(self, x):
        self.oma_rv_graph_count += 1
        # add the new data to the plot
        self.oma_rv_data[0].append(self.oma_rv_graph_count)
        self.oma_rv_data[1].append(x)
        if self.oma_rv_graph_count > 100:
            self.oma_rv_plot.setXRange(self.oma_rv_graph_count-100,self.oma_rv_graph_count)
        self.graph_oma_rv.setData(self.oma_rv_data[0], self.oma_rv_data[1])

    def update_plot_oht_rv(self, x):
        self.oht_rv_graph_count += 1
        # add the new data to the plot
        self.oht_rv_data[0].append(self.oht_rv_graph_count)
        self.oht_rv_data[1].append(x)
        if self.oht_rv_graph_count > 100:
            self.oht_rv_plot.setXRange(self.oht_rv_graph_count-100,self.oht_rv_graph_count)
        self.graph_oht_rv.setData(self.oht_rv_data[0], self.oht_rv_data[1])

    def update_plot_odt(self, dist_arr):
        self.odt_graph_count += 1
        # add the new data to the plot
        self.odt_x.append(self.odt_graph_count)
        self.d0data.append(float(dist_arr[0])) if float(dist_arr[0]) else self.d0data.append(0)
        self.d1data.append(float(dist_arr[1])) if float(dist_arr[1]) else self.d1data.append(0)
        self.d2data.append(float(dist_arr[2])) if float(dist_arr[2]) else self.d2data.append(0)
        self.d3data.append(float(dist_arr[3])) if float(dist_arr[3]) else self.d3data.append(0)
        if self.odt_graph_count > 100:
            self.odt_plot.setXRange(self.odt_graph_count-100,self.odt_graph_count)
        self.curve21.setData(self.odt_x, self.d0data)
        self.curve22.setData(self.odt_x, self.d1data)
        self.curve23.setData(self.odt_x, self.d2data)
        self.curve24.setData(self.odt_x, self.d3data)

    def update_plot_oma(self, msg):
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

            if self.selected_joint == "position":
                self.arr_j1.setData(self.oma_x, self.j1_pose)
                self.arr_j2.setData(self.oma_x, self.j2_pose)
                self.arr_j3.setData(self.oma_x, self.j3_pose)
                self.arr_j4.setData(self.oma_x, self.j4_pose)
                self.arr_j5.setData(self.oma_x, self.j5_pose)
                self.arr_j6.setData(self.oma_x, self.j6_pose)
                self.oma_plot.setTitle("position")
            if self.selected_joint == "velocity":
                self.arr_j1.setData(self.oma_x, self.j1_vel)
                self.arr_j2.setData(self.oma_x, self.j2_vel)
                self.arr_j3.setData(self.oma_x, self.j3_vel)
                self.arr_j4.setData(self.oma_x, self.j4_vel)
                self.arr_j5.setData(self.oma_x, self.j5_vel)
                self.arr_j6.setData(self.oma_x, self.j6_vel)
                self.oma_plot.setTitle("velocity")
            if self.selected_joint == "acceleration":
                self.arr_j1.setData(self.oma_x, self.j1_acc)
                self.arr_j2.setData(self.oma_x, self.j2_acc)
                self.arr_j3.setData(self.oma_x, self.j3_acc)
                self.arr_j4.setData(self.oma_x, self.j4_acc)
                self.arr_j5.setData(self.oma_x, self.j5_acc)
                self.arr_j6.setData(self.oma_x, self.j6_acc)
                self.oma_plot.setTitle("acceleration")
            if self.selected_joint == "jerk":
                self.arr_j1.setData(self.oma_x, self.j1_jerk)
                self.arr_j2.setData(self.oma_x, self.j2_jerk)
                self.arr_j3.setData(self.oma_x, self.j3_jerk)
                self.arr_j4.setData(self.oma_x, self.j4_jerk)
                self.arr_j5.setData(self.oma_x, self.j5_jerk)
                self.arr_j6.setData(self.oma_x, self.j6_jerk)
                self.oma_plot.setTitle("jerk")

    def draw_frames(self, msg, object, label_object):
        try:
            w = object.width()
            h = object.height()
            label_object.resize(w, h)
            camera_map = cv2.resize(self.bridge.imgmsg_to_cv2(msg, \
                                desired_encoding="passthrough"), (w,h))
            self.bridge.cv2_to_imgmsg
            bytesPerLine = int(camera_map.nbytes / h)
            qimg = QImage(camera_map.data, camera_map.shape[1], camera_map.shape[0], bytesPerLine, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            label_object.setPixmap(pixmap)
            label_object.show()
        except Exception as e:
            print(e)

    def on_button_clicked(self):
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
                    process = subprocess.Popen(command,\
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                else:
                    for topic in self.log_selected_topics:
                        command.append(topic)
                    command.append("-O")
                    command.append(name)
                    process = subprocess.Popen(command,\
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
        odt_graph = self.odt_graph_chkbx.isChecked()
        oma_graph = self.oma_graph_chkbx.isChecked()
        oht_graph = self.oht_graph_chkbx.isChecked()
        network_traffic = self.network_traffic_graph_chkbx.isChecked()
        odt_cylinder = self.odt_cylinder.isChecked()
        odt_joint = self.odt_joint_dist.isChecked()

        if odt_graph:
            self.odt_rv_chkbx.show()
            if "odt" not in self.graph_list:
                self.graph_list.append("odt")
                self.odt_rv_chkbx.setChecked(True)
        else:
            self.graph_funcion(self.QScrollArea_2, "odt", "hide")
            self.odt_rv_chkbx.hide()

        if oma_graph:
            self.oma_joint_list.show()
            self.oma_rv_chkbx.show()
            self.joint_select()
            if "oma" not in self.graph_list:
                self.graph_list.append("oma")
                self.oma_rv_chkbx.setChecked(True)
        else:
            self.graph_funcion(self.QScrollArea_3, "oma", "hide")
            self.oma_joint_list.hide()
            self.oma_rv_chkbx.hide()

        if oht_graph:
            self.oht_rv_chkbx.show()
            if "oht" not in self.graph_list:
                self.graph_list.append("oht")
                self.oht_rv_chkbx.setChecked(True)
        else:
            self.oht_rv_chkbx.hide()
            self.graph_funcion(self.QScrollArea_4, "oht", "hide")

        if network_traffic:
            if "net" not in self.graph_list:
                self.graph_list.append("net")
        else:
            self.graph_funcion(self.QScrollArea_1, "net", "hide")

        self.graph_funcion(self.QScrollArea_2, "odt", "show")
        self.graph_funcion(self.QScrollArea_3, "oma", "show")
        self.graph_funcion(self.QScrollArea_4, "oht", "show")
        self.graph_funcion(self.QScrollArea_1, "net", "show")

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

    def graph_funcion(self, graph_element, name, func):
        if func == "hide":
            try:
                graph_element.hide()
                self.graph_list.remove(name)
            except Exception as e1:
                pass
        elif func == "show":
            try:
                index = self.graph_list.index(name)
                self.grid_layout.addWidget(graph_element, self.grid_list[index][0]+1, self.grid_list[index][1])
                graph_element.show()
            except Exception as e2:
                pass

    def filter_checkboxes(self, search_text):
        # Loop over all checkboxes in the popup window and show/hide them based on whether
        # they contain the search text
        for checkbox in self.popup.findChildren(QtWidgets.QCheckBox):
            if search_text.lower() in checkbox.text().lower(): checkbox.show()
            else: checkbox.hide()
    
    def slct_all_topics(self):
        self.log_selected_topics = ["all"]

    def show_popup(self):
        # Create popup window
        self.popup = QtWidgets.QDialog(self)
        self.popup.setWindowTitle("Topics")
        popup_layout = QVBoxLayout()
        self.popup.setLayout(popup_layout)
        self.log_selected_topics = []

        # Add a search bar to the popup window
        search_layout = QtWidgets.QHBoxLayout()
        search_label = QLabel("Search:")
        self.search_edit = QtWidgets.QLineEdit()
        self.search_edit.textChanged.connect(self.filter_checkboxes)
        search_layout.addWidget(search_label)
        search_layout.addWidget(self.search_edit)
        popup_layout.addLayout(search_layout)

        popup_scroll_area = QScrollArea()
        popup_scroll_widget = QWidget()
        popup_scroll_layout = QVBoxLayout()
        popup_scroll_widget.setLayout(popup_scroll_layout)
        popup_scroll_area.setWidgetResizable(True)
        popup_scroll_area.setWidget(popup_scroll_widget)
        popup_layout.addWidget(popup_scroll_area)

        checkbox_list = []

        topic_list_dict = dict(rospy.get_published_topics())
        for key in topic_list_dict.keys():
            checkbox = QtWidgets.QCheckBox(key)
            checkbox_list.append(checkbox)
            popup_scroll_layout.addWidget(checkbox)

        # Add button box with OK and Cancel buttons
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
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
        self.odt_msg.data = data
        self.chkbx_publisher.publish(self.chkbx_json)


class GraphPlotNet(QThread):
    net_signal = pyqtSignal(float)
    def __init__(self):
        super(GraphPlotNet, self).__init__()
        self.x = 100
        self.rate = rospy.Rate(10)
        self.sub_camera = rospy.Subscriber("ControllerFreq", Float32, self.net_callback)

    def net_callback(self, msg):
        print("hey ya")
        self.x = msg.data
    
    def run(self) -> None:
        while not rospy.is_shutdown():
            if self.x != None:
                self.net_signal.emit(self.x)
            self.rate.sleep()

class GraphPlotOdt(QThread):
    odt_signal = pyqtSignal(list)
    def __init__(self):
        super(GraphPlotOdt, self).__init__()
        self.d0, self.d1, self.d2, self.d3 = 0, 0, 0, 0
        rospy.Subscriber("odt_json", String, self.odt_callback)

        self.rate = rospy.Rate(10)
    
    def run(self) -> None:
        while not rospy.is_shutdown():
            self.odt_signal.emit([self.d0, self.d1, self.d2, self.d3])
            self.rate.sleep()

    def odt_callback(self, msg):
        json_data = json.loads(msg.data)
        self.d0 = self.calculate_dist(json_data["dist0"])
        self.d1 = self.calculate_dist(json_data["dist1"])
        self.d2 = self.calculate_dist(json_data["dist2"])
        self.d3 = self.calculate_dist(json_data["dist3"])

    def calculate_dist(self, data):
        fin_pose = np.array([float(data["fin_pose"]["x"]), \
                      float(data["fin_pose"]["y"]) -0.26, \
                      float(data["fin_pose"]["z"])])
        strt_pose = np.array([float(data["strt_pose"]["x"]), \
                      float(data["strt_pose"]["y"]) -0.26, \
                      float(data["strt_pose"]["z"])])
        dist = str(round(np.linalg.norm(fin_pose-strt_pose),2))
        return dist

class GraphPlotOma(QThread):
    oma_signal = pyqtSignal(Float64MultiArray)
    def __init__(self):
        super(GraphPlotOma, self).__init__()
        self.msg = Float64MultiArray()
        rospy.Subscriber("mam_oma", Float64MultiArray, self.mam_oma_callback)
        self.rate = rospy.Rate(10)

    def mam_oma_callback(self, msg):
        self.msg = msg

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.oma_signal.emit(self.msg)
            self.rate.sleep()

class GraphPlotOdtRV(QThread):
    odt_rv_signal = pyqtSignal(int)
    def __init__(self):
        super(GraphPlotOdtRV, self).__init__()
        self.msg = 0
        rospy.Subscriber("mon_odt/monitor_verdict", String, self.odt_rv_callback)
        self.rate = rospy.Rate(10)

    def odt_rv_callback(self, msg):
        self.msg = 0 if "true" in msg.data else 1

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.odt_rv_signal.emit(self.msg)
            self.rate.sleep()

class GraphPlotOmaRV(QThread):
    oma_rv_signal = pyqtSignal(int)
    def __init__(self):
        super(GraphPlotOmaRV, self).__init__()
        self.msg = 0
        rospy.Subscriber("mon_oma/monitor_verdict", String, self.oma_rv_callback)
        self.rate = rospy.Rate(10)

    def oma_rv_callback(self, msg):
        print(self.msg)
        self.msg = 0 if "true" in msg.data else 1

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.oma_rv_signal.emit(self.msg)
            self.rate.sleep()

class GraphPlotOhtRV(QThread):
    oht_rv_signal = pyqtSignal(int)
    def __init__(self):
        super(GraphPlotOhtRV, self).__init__()
        self.msg = 0
        rospy.Subscriber("mon_oht/monitor_verdict", String, self.oht_rv_callback)
        self.rate = rospy.Rate(10)

    def oht_rv_callback(self, msg):
        self.msg = 0 if "true" in msg.data else 1

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.oht_rv_signal.emit(self.msg)
            self.rate.sleep()

class ChkbxPub(QThread):
    signal_chkbx = pyqtSignal(str)
    def __init__(self):
        super(ChkbxPub, self).__init__()
        self.data = "None"
        self.rate = rospy.Rate(2)
    
    def run(self) -> None:
        while not rospy.is_shutdown():
            self.signal_chkbx.emit(self.data)
            self.rate.sleep()

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
