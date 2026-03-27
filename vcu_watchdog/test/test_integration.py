import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import rclpy
import time

from vcu_msgs.msg import AccelBrakeSteer, Watchdog
from ix_msgs.msg import Energymeter

from ament_index_python.packages import get_package_share_directory


def get_config_file_path(name, dir='launch/vcu/params'):
    ret = os.path.join(
        get_package_share_directory('vcu_launch'),
        dir,
        name
    )
    return ret


def generate_test_description():
    config_watchdog = get_config_file_path("watchdogParams.yaml")
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                launch_ros.actions.Node(
                    package='vcu_watchdog',
                    executable='vcu_watchdog',
                    parameters=[config_watchdog],
                    output="screen",
                    emulate_tty=True
                ),
                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ), {},
    )


# Active tests
class TestWatchdog(unittest.TestCase):

    in_test = False
    received = False

    brakePressureFront = False
    brakePressureRear = False

    pedalsImplausible = False
    appsImplausible = False
    ebsError = False

    def callbackWatchdog(self, msg):
        self.received = True
        if (time.time() - self.publishTime) > 0.5:
            assert msg.apps_implausible == self.appsImplausible
            assert msg.brake_pressure_front_error == self.brakePressureFront
            assert msg.brake_pressure_rear_error == self.brakePressureRear
            assert msg.pedals_implausible == self.pedalsImplausible
            assert msg.ebs_front_error == self.ebsError
            assert msg.ebs_rear_error == self.ebsError

    def assembleAccelBrakeSteer(self, apps1, apps2, brakeFront, brakeRear, ebsFront, ebsRear):
        out = AccelBrakeSteer()
        out.apps1_pressed = 11.7
        out.apps1_released = 19.2
        out.apps2_pressed = 21.7
        out.apps2_released = 29.2

        out.apps1 = apps1
        out.apps2 = apps2
        out.brake_pressure_front = brakeFront
        out.brake_pressure_rear = brakeRear

        out.ebs_pressure_front = ebsFront
        out.ebs_pressure_rear = ebsRear

        return out

    def assembleEnergymeter(self, ts_current, ts_voltage):
        out = Energymeter()
        out.ts_current = ts_current
        out.ts_voltage = ts_voltage

        return out

    def sendToWatchdog(self, msg, publisher, energymeter_msg=None, energymeter_pub=None):
        end_time = time.time() + 3
        self.publishTime = time.time()
        while time.time() < end_time:
            publisher.publish(msg)
            if energymeter_msg is None:
                pass
            elif energymeter_pub is None:
                pass
            else:
                energymeter_pub.publish(energymeter_msg)
            # spin to get subscriber callback executed
            rclpy.spin_once(self.node, timeout_sec=0.005)

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_watchdog')

    def tearDown(self):
        self.node.destroy_node()

    def test_apps_implausibility(self, proc_output):
        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)

        self.in_test = True
        self.received = False
        self.appsImplausible = False
        self.pedalsImplausible = False

        self.node.create_subscription(
            Watchdog, '/vcu/watchdog', self.callbackWatchdog, 1)
        accel_pub = self.node.create_publisher(
            AccelBrakeSteer, '/vcu/data/accelBrakeSteer', 1)

        out_accel = self.assembleAccelBrakeSteer(
            19.3, 29.3, 0.0, 0.0, 0.0, 0.0)
        accel_pub.publish(out_accel)

        self.sendToWatchdog(out_accel, accel_pub)

        assert self.received

        self.in_test = False

    def test_apps_implausibility_2(self, proc_output):
        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)

        self.in_test = True
        self.received = False
        self.appsImplausible = True

        self.node.create_subscription(
            Watchdog, '/vcu/watchdog', self.callbackWatchdog, 1)
        accel_pub = self.node.create_publisher(
            AccelBrakeSteer, '/vcu/data/accelBrakeSteer', 1)

        out_accel = self.assembleAccelBrakeSteer(
            17.3, 29.3, 0.0, 0.0, 0.0, 0.0)
        self.sendToWatchdog(out_accel, accel_pub)

        assert self.received

        self.in_test = False

    def test_brake_implausibility(self, proc_output):
        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)

        self.in_test = True
        self.received = False
        self.appsImplausible = False
        self.brakePressureRear = False
        self.brakePressureFront = False
        self.pedalsImplausible = False

        self.node.create_subscription(
            Watchdog, '/vcu/watchdog', self.callbackWatchdog, 1)
        accel_pub = self.node.create_publisher(
            AccelBrakeSteer, '/vcu/data/accelBrakeSteer', 1)

        out_accel = self.assembleAccelBrakeSteer(
            19.3, 29.3, 0.0, 0.0, 0.0, 0.0)
        self.sendToWatchdog(out_accel, accel_pub)

        assert self.received

        self.in_test = False

    def test_brake_implausibility_2(self, proc_output):
        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)

        self.in_test = True
        self.received = False
        self.appsImplausible = False
        self.brakePressureRear = True
        self.brakePressureFront = True
        self.pedalsImplausible = False

        self.node.create_subscription(
            Watchdog, '/vcu/watchdog', self.callbackWatchdog, 1)
        accel_pub = self.node.create_publisher(
            AccelBrakeSteer, '/vcu/data/accelBrakeSteer', 1)

        out_accel = self.assembleAccelBrakeSteer(
            19.3, 29.3, -3.0, 130.0, 0.0, 0.0)
        self.sendToWatchdog(out_accel, accel_pub)

        assert self.received

        self.in_test = False

    def test_energymeter(self, proc_output):
        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)

        self.in_test = True
        self.received = False
        self.appsImplausible = False
        self.brakePressureRear = False
        self.brakePressureFront = False
        self.pedalsImplausible = False

        self.node.create_subscription(
            Watchdog, '/vcu/watchdog', self.callbackWatchdog, 1)
        accel_pub = self.node.create_publisher(
            AccelBrakeSteer, '/vcu/data/accelBrakeSteer', 1)
        energymeter_pub = self.node.create_publisher(
            Energymeter, '/vcu/data/energymeter', 1)

        out_accel = self.assembleAccelBrakeSteer(
            19.3, 29.3, 0.0, 0.0, 0.0, 0.0)
        out_energymeter = self.assembleEnergymeter(10.0, 600.0)
        self.sendToWatchdog(out_accel, accel_pub,
                            out_energymeter, energymeter_pub)

        assert self.received

        self.in_test = False

    def test_energymeter_2(self, proc_output):
        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)

        self.in_test = True
        self.received = False
        self.appsImplausible = False
        self.brakePressureRear = False
        self.brakePressureFront = False
        self.pedalsImplausible = False

        self.node.create_subscription(
            Watchdog, '/vcu/watchdog', self.callbackWatchdog, 1)
        accel_pub = self.node.create_publisher(
            AccelBrakeSteer, '/vcu/data/accelBrakeSteer', 1)
        energymeter_pub = self.node.create_publisher(
            Energymeter, '/vcu/data/energymeter', 1)

        out_accel = self.assembleAccelBrakeSteer(
            19.3, 29.3, 11.0, 0.0, 0.0, 0.0)
        out_energymeter = self.assembleEnergymeter(10.0, 600.0)
        self.sendToWatchdog(out_accel, accel_pub,
                            out_energymeter, energymeter_pub)

        assert self.received

        self.in_test = False

    def test_ebs(self, proc_output):
        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)

        self.in_test = True
        self.received = False
        self.appsImplausible = False
        self.brakePressureRear = False
        self.brakePressureFront = False
        self.pedalsImplausible = False
        self.ebsError = False

        self.node.create_subscription(
            Watchdog, '/vcu/watchdog', self.callbackWatchdog, 1)
        accel_pub = self.node.create_publisher(
            AccelBrakeSteer, '/vcu/data/accelBrakeSteer', 1)
        energymeter_pub = self.node.create_publisher(
            Energymeter, '/vcu/data/energymeter', 1)

        out_accel = self.assembleAccelBrakeSteer(
            19.3, 29.3, 0.0, 0.0, 0.0, 0.0)
        out_energymeter = self.assembleEnergymeter(10.0, 600.0)
        self.sendToWatchdog(out_accel, accel_pub,
                            out_energymeter, energymeter_pub)

        assert self.received

        self.in_test = False

    def test_ebs_2(self, proc_output):
        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)

        self.in_test = True
        self.received = False
        self.appsImplausible = False
        self.brakePressureRear = False
        self.brakePressureFront = False
        self.pedalsImplausible = False
        self.ebsError = True

        self.node.create_subscription(
            Watchdog, '/vcu/watchdog', self.callbackWatchdog, 1)
        accel_pub = self.node.create_publisher(
            AccelBrakeSteer, '/vcu/data/accelBrakeSteer', 1)
        energymeter_pub = self.node.create_publisher(
            Energymeter, '/vcu/data/energymeter', 1)

        out_accel = self.assembleAccelBrakeSteer(
            19.3, 29.3, 0.0, 0.0, -3.0, 120.0)
        out_energymeter = self.assembleEnergymeter(10.0, 600.0)
        self.sendToWatchdog(out_accel, accel_pub,
                            out_energymeter, energymeter_pub)

        assert self.received

        self.in_test = False
