#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, QTimer
from krsbi_pkg.manual_ui import Ui_MainWindow
from krsbi_interfaces.msg import DribbleSetting, RobotStatus, NavigationSetting, KickerModule

class ManualUINode(Node):
    def __init__(self):
        super().__init__("manual_ui_node")

        self.app = QtWidgets.QApplication(sys.argv)
        self.MainWindow = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.MainWindow)

        self.dribbleSetting = DribbleSetting()
        self.robotStatus = RobotStatus()
        self.navigationSetting = NavigationSetting()
        self.kickerModule = KickerModule()

        # Publisher
        self.pubDribbleSetting = self.create_publisher(DribbleSetting, '/ui/DribbleSetting', 10)
        self.pubRobotStatus = self.create_publisher(RobotStatus, '/ui/RobotStatus', 10)
        self.pubNavigationSetting = self.create_publisher(NavigationSetting, '/ui/NavigationSetting', 10)
        self.pubKickerModule = self.create_publisher(KickerModule, '/ui/KickerModule', 10)

        # State tracking
        self.charge_counter = 0
        self.reset_counter = 0
        self.charge_timer = QTimer()
        self.reset_timer = QTimer()
        self.kick_disabled = False

        self.customSetupCode()
        self.MainWindow.show()
        self.app.exec_()

    def customSetupCode(self):
        self.ui.speedSlider.valueChanged.connect(lambda:self.speedInputAction(self.ui.speedSlider.value()))
        self.ui.speedInput.valueChanged.connect(lambda:self.speedInputAction(self.ui.speedInput.value()))
        self.ui.directionDial.valueChanged.connect(lambda:self.directionDialAction(self.ui.directionDial.value()))
        self.ui.directionInput.valueChanged.connect(lambda:self.directionInputAction(self.ui.directionInput.value()))
        self.ui.rotationDial.valueChanged.connect(lambda:self.rotationDialAction(self.ui.rotationDial.value()))
        self.ui.rotationInput.valueChanged.connect(lambda:self.rotationInputAction(self.ui.rotationInput.value()))
        self.ui.runActionButton.clicked.connect(self.runAction)
        self.ui.stopActionButton.clicked.connect(self.stopAction)
        self.ui.kickButton.clicked.connect(self.kickButtonAction)
        self.ui.runDribbleActionButton.clicked.connect(self.dribbleAction)
        self.ui.stopDribbleActionButton.clicked.connect(self.dribbleAction)

    def speedInputAction(self, value):
        self.ui.speedInput.setValue(value)
        self.ui.speedSlider.setValue(value)
        self.publishNavigationSetting()

    def directionDialAction(self, value):
        self.ui.directionInput.setValue(value + 270)
    def directionInputAction(self, value):
        self.ui.directionDial.setValue(value - 270)
        self.publishNavigationSetting()

    def rotationDialAction(self, value):
        self.ui.rotationInput.setValue(value)
    def rotationInputAction(self, value):
        self.ui.rotationDial.setValue(value)
        self.publishNavigationSetting()

    def publishNavigationSetting(self):
        self.navigationSetting.speed = float(self.ui.speedInput.value())
        self.navigationSetting.direction = float(self.ui.directionInput.value())
        self.navigationSetting.rotation = float(self.ui.rotationInput.value())
        self.pubNavigationSetting.publish(self.navigationSetting)

    def runAction(self):
        if self.ui.runActionButton.isChecked():
            self.robotStatus.status = "running"
            self.pubRobotStatus.publish(self.robotStatus)

    def stopAction(self):
        if self.ui.stopActionButton.isChecked():
            self.robotStatus.status = "stop"
            self.pubRobotStatus.publish(self.robotStatus)

    def kickButtonAction(self):
        if self.kick_disabled:
            self.ui.kickerStatusLabel.setText("Kicker sedang cooldown...")
            return

        self.kick_disabled = True
        self.charge_counter = 0
        self.charge_duration = self.ui.kickChargeTimeInput.value()

        self.ui.kickButton.setEnabled(False)
        self.ui.kickerStatusLabel.setText("Charging: 0")

        # Publish "charging"
        self.kickerModule.status = "charging"
        self.pubKickerModule.publish(self.kickerModule)

        # Mulai timer charging
        self.charge_timer.timeout.connect(self.updateCharging)
        self.charge_timer.start(1000)  # 1 detik

    def updateCharging(self):
        self.charge_counter += 1
        if self.charge_counter <= self.charge_duration:
            self.ui.kickerStatusLabel.setText(f"Charging: {self.charge_counter}")
        else:
            self.charge_timer.stop()
            self.charge_timer.timeout.disconnect(self.updateCharging)
            self.ui.kickerStatusLabel.setText("KICK!")
            
            # Publish "kick"
            self.kickerModule.status = "kick"
            self.pubKickerModule.publish(self.kickerModule)

            # Delay 1 detik untuk "KICK!" lalu lanjut ke reset
            QTimer.singleShot(1000, self.startResetTimer)

    def startResetTimer(self):
        self.reset_counter = 5
        self.ui.kickerStatusLabel.setText(f"Reset: {self.reset_counter}")
        self.reset_timer.timeout.connect(self.updateReset)
        self.reset_timer.start(1000)

    def updateReset(self):
        self.reset_counter -= 1
        if self.reset_counter > 0:
            self.ui.kickerStatusLabel.setText(f"Reset: {self.reset_counter}")
        else:
            self.reset_timer.stop()
            self.reset_timer.timeout.disconnect(self.updateReset)
            self.ui.kickerStatusLabel.setText("Kicker Ready")
            self.kickerModule.status = "finish"
            self.pubKickerModule.publish(self.kickerModule)
            self.ui.kickButton.setEnabled(True)
            self.kick_disabled = False


    def dribbleAction(self):
        if self.ui.stopDribbleActionButton.isChecked():
            self.dribbleSetting.speed = 0.0
            self.dribbleSetting.status = "stop"
            self.pubDribbleSetting.publish(self.dribbleSetting)
        elif self.ui.runDribbleActionButton.isChecked():
            self.dribbleSetting.speed = float(self.ui.dribbleSpeedInput.value())
            self.dribbleSetting.status = "catch"
            self.pubDribbleSetting.publish(self.dribbleSetting)

def main(args=None):
    rclpy.init(args=args)
    node = ManualUINode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
