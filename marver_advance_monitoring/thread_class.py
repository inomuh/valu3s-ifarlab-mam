"""
Thread classes for marver_advance_monitoring.py
"""
import json
import numpy as np

import rospy
from std_msgs.msg import String, Float64MultiArray, Float32
from adservice.msg import adrv

from PyQt5 import QtCore

class GraphPlotNet(QtCore.QThread):
    """
    Network graph thread
    """
    net_signal = QtCore.pyqtSignal(float)
    def __init__(self):
        super(GraphPlotNet, self).__init__()
        self.x_data = 100
        self.rate = rospy.Rate(10)
        self.sub_camera = rospy.Subscriber("ControllerFreq", Float32, self.net_callback)

    def net_callback(self, msg):
        self.x_data = msg.data

    def run(self) -> None:
        """
        run thread
        """
        while not rospy.is_shutdown():
            if self.x_data != None:
                self.net_signal.emit(self.x_data)
            self.rate.sleep()

class GraphPlotOdt(QtCore.QThread):
    """
    ODT graph thread
    """
    odt_signal = QtCore.pyqtSignal(list)
    def __init__(self):
        super(GraphPlotOdt, self).__init__()
        self.odt_dist0, self.odt_dist1, self.odt_dist2, self.odt_dist3 = 0, 0, 0, 0
        rospy.Subscriber("odt_json", String, self.odt_callback)

        self.rate = rospy.Rate(10)

    def run(self) -> None:
        """
        run thread
        """
        while not rospy.is_shutdown():
            self.odt_signal.emit([self.odt_dist0, self.odt_dist1, self.odt_dist2, self.odt_dist3])
            self.rate.sleep()

    def odt_callback(self, msg):
        """
        odt_json ros callback
        """
        json_data = json.loads(msg.data)
        self.odt_dist0 = self.calculate_dist(json_data["dist0"])
        self.odt_dist1 = self.calculate_dist(json_data["dist1"])
        self.odt_dist2 = self.calculate_dist(json_data["dist2"])
        self.odt_dist3 = self.calculate_dist(json_data["dist3"])

    def calculate_dist(self, data):
        """
        Calculation distance model position
        """
        fin_pose = np.array([float(data["fin_pose"]["x"]), \
                      float(data["fin_pose"]["y"]) -0.26, \
                      float(data["fin_pose"]["z"])])
        strt_pose = np.array([float(data["strt_pose"]["x"]), \
                      float(data["strt_pose"]["y"]) -0.26, \
                      float(data["strt_pose"]["z"])])
        dist = round(np.linalg.norm(fin_pose-strt_pose),2)
        return dist

class GraphPlotOma(QtCore.QThread):
    """
    OMA graph thread
    """
    oma_signal = QtCore.pyqtSignal(Float64MultiArray)
    def __init__(self):
        super(GraphPlotOma, self).__init__()
        self.msg = Float64MultiArray()
        rospy.Subscriber("mam_oma", Float64MultiArray, self.mam_oma_callback)
        self.rate = rospy.Rate(10)

    def mam_oma_callback(self, msg):
        self.msg = msg

    def run(self) -> None:
        """
        run thread
        """
        while not rospy.is_shutdown():
            self.oma_signal.emit(self.msg)
            self.rate.sleep()

class GraphPlotOdtRV(QtCore.QThread):
    """
    ODT Runtime verification graph thread
    """
    odt_rv_signal = QtCore.pyqtSignal(int)
    def __init__(self):
        super(GraphPlotOdtRV, self).__init__()
        self.msg = 0
        rospy.Subscriber("mon_odt/monitor_verdict", String, self.odt_rv_callback)
        self.rate = rospy.Rate(10)

    def odt_rv_callback(self, msg):
        self.msg = 0 if "true" in msg.data else 1

    def run(self) -> None:
        """
        run thread
        """
        while not rospy.is_shutdown():
            self.odt_rv_signal.emit(self.msg)
            self.rate.sleep()

class GraphPlotOmaRV(QtCore.QThread):
    """
    OMA Runtime verification graph thread
    """
    oma_rv_signal = QtCore.pyqtSignal(int)
    def __init__(self):
        super(GraphPlotOmaRV, self).__init__()
        self.msg = 0
        rospy.Subscriber("mon_oma/monitor_verdict", String, self.oma_rv_callback)
        self.rate = rospy.Rate(10)

    def oma_rv_callback(self, msg):
        self.msg = 0 if "true" in msg.data else 1

    def run(self) -> None:
        """
        run thread
        """
        while not rospy.is_shutdown():
            self.oma_rv_signal.emit(self.msg)
            self.rate.sleep()

class GraphPlotOht(QtCore.QThread):
    """
    OHT graph thread
    """
    oht_signal = QtCore.pyqtSignal(float)
    def __init__(self):
        super(GraphPlotOht, self).__init__()
        self.oht_data = 0
        rospy.Subscriber("/oht", Float32, self.oht_callback)
        self.rate = rospy.Rate(10)

    def oht_callback(self, msg):
        self.oht_data = msg.data 

    def run(self) -> None:
        """
        run thread
        """
        while not rospy.is_shutdown():
           try:
                self.oht_signal.emit(self.oht_data)
                self.rate.sleep()
           except:
               pass

class GraphPlotOhtRV(QtCore.QThread):
    """
    OHT Runtime verification graph thread
    """
    oht_rv_signal = QtCore.pyqtSignal(int)
    def __init__(self):
        super(GraphPlotOhtRV, self).__init__()
        self.msg = 0
        rospy.Subscriber("mon_oht/monitor_verdict", String, self.oht_rv_callback)
        self.rate = rospy.Rate(10)

    def oht_rv_callback(self, msg):
        self.msg = 0 if "true" in msg.data else 1

    def run(self) -> None:
        """
        run thread
        """
        while not rospy.is_shutdown():
            self.oht_rv_signal.emit(self.msg)
            self.rate.sleep()

class GraphPlotAdrv(QtCore.QThread):
    """
    OHT Runtime verification graph thread
    """
    adrv_signal = QtCore.pyqtSignal(list)
    def __init__(self):
        super(GraphPlotAdrv, self).__init__()
        self.msg = [0, 0]
        rospy.Subscriber("adrv", adrv, self.adrv_callback)
        self.rate = rospy.Rate(10)

    def adrv_callback(self, msg):
        self.msg = [msg.adResult, msg.attackState]

    def run(self) -> None:
        """
        run thread
        """
        while not rospy.is_shutdown():
            self.adrv_signal.emit(self.msg)
            self.rate.sleep()

class GraphPlotAdrvRV(QtCore.QThread):
    """
    ADRV Runtime verification graph thread
    """
    adrv_rv_signal = QtCore.pyqtSignal(int)
    def __init__(self):
        super(GraphPlotAdrvRV, self).__init__()
        self.msg = 0
        rospy.Subscriber("mon_adrv/monitor_verdict", String, self.adrv_rv_callback)
        self.rate = rospy.Rate(10)

    def adrv_rv_callback(self, msg):
        self.msg = 0 if "currently_true" == msg.data else 1

    def run(self) -> None:
        """
        run thread
        """
        while not rospy.is_shutdown():
            self.adrv_rv_signal.emit(self.msg)
            self.rate.sleep()

class ChkbxPub(QtCore.QThread):
    """
    Checkbox publisher thread
    """
    signal_chkbx = QtCore.pyqtSignal(str)
    def __init__(self):
        super(ChkbxPub, self).__init__()
        self.data = "None"
        self.rate = rospy.Rate(2)

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.signal_chkbx.emit(self.data)
            self.rate.sleep()
