#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, QTimer
from krsbi_pkg.auto_ui import Ui_MainWindow
from krsbi_interfaces.msg import Coordinate, RobotMode

class AutoUINode(Node):
    def __init__(self):
        super().__init__("auto_ui_node")

        self.app = QtWidgets.QApplication(sys.argv)
        self.MainWindow = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.MainWindow)

        self.coordinate = Coordinate()
        self.robotMode = RobotMode()
        
        # Publisher
        self.pubCoordinate = self.create_publisher(Coordinate, '/ui/Coordinate', 10)
        self.pubRobotMode = self.create_publisher(RobotMode, '/ui/RobotMode', 10)

        self.setupTimers()
        self.customSetupCode()
        self.MainWindow.show()
        self.app.exec_()

    def customSetupCode(self):
        self.ui.runGoToCoordinateActionButton.stateChanged.connect(self.updateGoToCoordinateButton)
        self.ui.goToCoordinateButtonPush.setEnabled(False) # Matikan diawal
        self.ui.goToCoordinateButtonPush.clicked.connect(self.goToCoordinate)
        self.ui.catchBallButton.clicked.connect(self.catchBall)
        self.ui.catchBallButton_2.clicked.connect(self.catchBall2)
        self.ui.catchBallButton_3.clicked.connect(self.ballControl)
        self.ui.passingButton.clicked.connect(self.passing)
        self.ui.shootButton.clicked.connect(self.shooting)
        self.ui.goHomeButton.clicked.connect(self.goHome)
        self.ui.SendSetpoint.clicked.connect(self.sendingHeadingSetPoint)
        self.ui.stopButton.clicked.connect(self.stopScenario)
        self.ui.Scenario1.clicked.connect(self.robotScenario1)
        self.ui.Scenario2.clicked.connect(self.robotScenario2)
        self.ui.Scenario3.clicked.connect(self.robotScenario3)
        self.ui.Scenario4.clicked.connect(self.robotScenario4)
        self.ui.Scenario5.clicked.connect(self.robotScenario5)
        self.ui.Scenario6.clicked.connect(self.robotScenario6)

    def setupTimers(self):
        # Passing
        self.passing_counter = 0
        self.passing_timer = QTimer()
        self.passing_timer.timeout.connect(self.updatePassing)
        self.passing_cooldown = 0
        self.passing_cooldown_timer = QTimer()
        self.passing_cooldown_timer.timeout.connect(self.updatePassingCooldown)

        # Shooting
        self.shooting_counter = 0
        self.shooting_timer = QTimer()
        self.shooting_timer.timeout.connect(self.updateShooting)
        self.shooting_cooldown = 0
        self.shooting_cooldown_timer = QTimer()
        self.shooting_cooldown_timer.timeout.connect(self.updateShootingCooldown)

    def updateGoToCoordinateButton(self, state):
        self.ui.goToCoordinateButtonPush.setEnabled(state == Qt.Checked)   

    def goToCoordinate(self):
        self.coordinate.x = float(self.ui.spinBox.value())
        self.coordinate.y = float(self.ui.spinBox_2.value())
        self.pubCoordinate.publish(self.coordinate)
        self.robotMode.mode = "goToCoordinate"
        self.pubRobotMode.publish(self.robotMode)

    def catchBall(self):
        self.robotMode.mode = "catchBall"
        self.pubRobotMode.publish(self.robotMode)

    def catchBall2(self):
        self.robotMode.mode = "catchBall2"
        self.pubRobotMode.publish(self.robotMode)

    def ballControl(self):
        self.robotMode.mode = "ballControl"
        self.pubRobotMode.publish(self.robotMode)

    def passing(self):
        self.ui.passingButton.setEnabled(False)
        self.ui.shootButton.setEnabled(False)

        self.ui.goToCoordinateButtonPush.setEnabled(False)
        self.ui.catchBallButton.setEnabled(False)
        self.ui.catchBallButton_2.setEnabled(False)
        self.ui.catchBallButton_3.setEnabled(False)
        self.ui.goHomeButton.setEnabled(False)
        self.ui.stopButton.setEnabled(False)
        self.ui.Scenario1.setEnabled(False)
        self.ui.Scenario2.setEnabled(False)
        self.ui.Scenario3.setEnabled(False)
        self.ui.Scenario4.setEnabled(False)
        self.ui.Scenario5.setEnabled(False)
        self.ui.Scenario6.setEnabled(False)

        self.passing_counter = 0
        self.robotMode.mode = "passing"
        self.pubRobotMode.publish(self.robotMode)
        self.ui.passingButton.setText("Charging: 0")
        self.passing_timer.start(1000)

    def updatePassing(self):
        self.passing_counter += 1
        if self.passing_counter <= 3:
            self.ui.passingButton.setText(f"Charging: {self.passing_counter}")
        else:
            self.passing_timer.stop()
            self.ui.passingButton.setText("PASS!")
            QTimer.singleShot(100, self.startPassingCooldown)

    def startPassingCooldown(self):
        self.passing_cooldown = 5
        self.ui.passingButton.setText("Cooldown: 5")
        self.passing_cooldown_timer.start(1000)
        self.ui.passingButton.setStyleSheet("background-color: red; color: white;")

    def updatePassingCooldown(self):
        self.passing_cooldown -= 1
        if self.passing_cooldown > 0:
            self.ui.passingButton.setText(f"Cooldown: {self.passing_cooldown}")
        else:
            self.ui.passingButton.setStyleSheet("")
            self.passing_cooldown_timer.stop()
            self.ui.passingButton.setText("Passing")
            self.ui.passingButton.setEnabled(True)
            self.ui.shootButton.setEnabled(True)

            self.ui.goToCoordinateButtonPush.setEnabled(True)
            self.ui.catchBallButton.setEnabled(True)
            self.ui.catchBallButton_2.setEnabled(True)
            self.ui.catchBallButton_3.setEnabled(True)
            self.ui.goHomeButton.setEnabled(True)
            self.ui.stopButton.setEnabled(True)
            self.ui.Scenario1.setEnabled(True)
            self.ui.Scenario2.setEnabled(True)
            self.ui.Scenario3.setEnabled(True)
            self.ui.Scenario4.setEnabled(True)
            self.ui.Scenario5.setEnabled(True)
            self.ui.Scenario6.setEnabled(True)

    def shooting(self):
        self.ui.shootButton.setEnabled(False)
        self.ui.passingButton.setEnabled(False)

        self.ui.goToCoordinateButtonPush.setEnabled(False)
        self.ui.catchBallButton.setEnabled(False)
        self.ui.catchBallButton_2.setEnabled(False)
        self.ui.catchBallButton_3.setEnabled(False)
        self.ui.goHomeButton.setEnabled(False)
        self.ui.stopButton.setEnabled(False)
        self.ui.Scenario1.setEnabled(False)
        self.ui.Scenario2.setEnabled(False)
        self.ui.Scenario3.setEnabled(False)
        self.ui.Scenario4.setEnabled(False)
        self.ui.Scenario5.setEnabled(False)
        self.ui.Scenario6.setEnabled(False)

        self.shooting_counter = 0
        self.robotMode.mode = "shooting"
        self.pubRobotMode.publish(self.robotMode)
        self.ui.shootButton.setText("Charging: 0")
        self.shooting_timer.start(1000)

    def updateShooting(self):
        self.shooting_counter += 1
        if self.shooting_counter <= 7:
            self.ui.shootButton.setText(f"Charging: {self.shooting_counter}")
        else:
            self.shooting_timer.stop()
            self.ui.shootButton.setText("SHOOT!")
            QTimer.singleShot(100, self.startShootingCooldown)

    def startShootingCooldown(self):
        self.shooting_cooldown = 5
        self.ui.shootButton.setText("Cooldown: 5")
        self.shooting_cooldown_timer.start(1000)
        self.ui.shootButton.setStyleSheet("background-color: red; color: white;")

    def updateShootingCooldown(self):
        self.shooting_cooldown -= 1
        if self.shooting_cooldown > 0:
            self.ui.shootButton.setText(f"Cooldown: {self.shooting_cooldown}")
        else:
            self.ui.shootButton.setStyleSheet("")
            self.shooting_cooldown_timer.stop()
            self.ui.shootButton.setText("Shooting")
            self.ui.shootButton.setEnabled(True)
            self.ui.passingButton.setEnabled(True)

            self.ui.goToCoordinateButtonPush.setEnabled(True)
            self.ui.catchBallButton.setEnabled(True)
            self.ui.catchBallButton_2.setEnabled(True)
            self.ui.catchBallButton_3.setEnabled(True)
            self.ui.goHomeButton.setEnabled(True)
            self.ui.stopButton.setEnabled(True)
            self.ui.Scenario1.setEnabled(True)
            self.ui.Scenario2.setEnabled(True)
            self.ui.Scenario3.setEnabled(True)
            self.ui.Scenario4.setEnabled(True)
            self.ui.Scenario5.setEnabled(True)
            self.ui.Scenario6.setEnabled(True)

    def goHome(self):
        self.robotMode.mode = "goHome"
        self.pubRobotMode.publish(self.robotMode)

    def sendingHeadingSetPoint(self):
        self.robotMode.mode = "sendSetPoint"
        self.pubRobotMode.publish(self.robotMode)

    def stopScenario(self):
        self.robotMode.mode = "nothing"
        self.pubRobotMode.publish(self.robotMode)

    def robotScenario1(self):
        self.robotMode.mode = "scenario1"
        self.pubRobotMode.publish(self.robotMode)

    def robotScenario2(self):
        self.robotMode.mode = "scenario2"
        self.pubRobotMode.publish(self.robotMode)

    def robotScenario3(self):
        self.robotMode.mode = "scenario3"
        self.pubRobotMode.publish(self.robotMode)

    def robotScenario4(self):
        self.robotMode.mode = "scenario4"
        self.pubRobotMode.publish(self.robotMode)

    def robotScenario5(self):
        self.robotMode.mode = "scenario5"
        self.pubRobotMode.publish(self.robotMode)

    def robotScenario6(self):
        self.robotMode.mode = "scenario6"
        self.pubRobotMode.publish(self.robotMode)
        
def main(args=None):
    rclpy.init(args=args)
    node = AutoUINode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
