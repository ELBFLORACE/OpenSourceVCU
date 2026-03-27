import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node
import rclpy
from vcu_msgs.msg import WatchdogDebug, AccelBrakeSteer, Watchdog


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
        LaunchDescription(
            [
                # Nodes under test
                Node(
                    package='vcu_watchdog',
                    executable='vcu_watchdog',
                    parameters=[config_watchdog],
                    output="screen",
                    emulate_tty=True
                ),
                # Launch tests 0.5 s later
                TimerAction(
                    period=0.5, actions=[ReadyToTest()]),
            ]
        ), {},
    )

# Active tests
class TestWatchdog(unittest.TestCase):

    in_test = False
    received = False

    should_apps1_error = False
    should_apps2_error = False
    should_apps_implausible = False

    should_apps1_error_later = False
    should_apps2_error_later = False


    def sendMessage(self, pubDict):
        end_time = time.time() + 1
        self.publishTime = time.time()
        while time.time() < end_time:
            for pub in pubDict:
                pub.publish(pubDict[pub])
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def sendMessageWithDelay(self, pubDict, pubDictLater):
        end_time = time.time() + 2
        middle_time = time.time() + 1
        self.publishTime = time.time()
        while time.time() < middle_time:
            for pub in pubDict:
                pub.publish(pubDict[pub])
            rclpy.spin_once(self.node, timeout_sec=0.005)
        while time.time() < end_time:
            for pub in pubDictLater:
                pub.publish(pubDictLater[pub])
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def callbackWatchdog(self, msg):
        self.received = True
        if (time.time() - self.publishTime) > 0.2 and (time.time() - self.publishTime) < 0.8:
            assert msg.apps1_error == self.should_apps1_error
            assert msg.apps2_error == self.should_apps2_error
            assert msg.apps_implausible == self.should_apps_implausible
        if (time.time() - self.publishTime) > 1.2 and (time.time() - self.publishTime) < 1.8:
            assert msg.apps1_error == self.should_apps1_error_later
            assert msg.apps2_error == self.should_apps2_error_later
            assert msg.apps_implausible == self.should_apps_implausible

    def assembleAccelBrakeSteer(self, a1, a2, bf, bpf, bpr, epf, epr, s, ap1, ar1, ap2, ar2):
        
        out = AccelBrakeSteer()

        out.apps1 = a1
        out.apps2 = a2
        out.brakeforce = bf
        out.brake_pressure_front = bpf
        out.brake_pressure_rear = bpr
        out.ebs_pressure_front = epf
        out.ebs_pressure_rear = epr
        out.steering_angle = s
        out.apps1_pressed = ap1
        out.apps1_released = ar1
        out.apps2_pressed = ap2
        out.apps2_released = ar2

        return out


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

    def test_apps_implausiblity(self, proc_output):
        
        msgs_rx = []
        sub = self.node.create_subscription(
            WatchdogDebug, '/vcu/watchdog/debug',
            lambda msg: msgs_rx.append(msg), 100)
        try:
            # Listen to the pose topic for 10 s
            end_time = time.time() + 10
            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)
            # There should have been 100 messages received
            assert len(msgs_rx) > 100
        finally:
            self.node.destroy_subscription(sub)

    def test_NoAppsErrors(self, proc_output):

        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=3)

        self.in_test = True
        self.received = False

        self.should_apps1_error = False
        self.should_apps2_error = False
        self.should_apps_implausible = False


        self.node.create_subscription(
            Watchdog, "/vcu/watchdog", self.callbackWatchdog, 1)
        
        accelBrakeSteerPub = self.node.create_publisher(
            AccelBrakeSteer, "/vcu/data/accelBrakeSteer", 1)

        out_accelBrakeSteer = self.assembleAccelBrakeSteer(11.7, 21.7, 100.0, 0.0, 0.0, 0.0, 0.0, 10.0, 11.7, 19.2, 21.7, 29.2)

        pubDict = {accelBrakeSteerPub: out_accelBrakeSteer}

        accelBrakeSteerPub.publish(out_accelBrakeSteer)

        self.sendMessage(pubDict)        
        
        assert self.received

        self.in_test = False

    def test_apps1Error(self, proc_output):

        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=3)

        self.in_test = True
        self.received = False

        self.should_apps1_error = True
        self.should_apps2_error = False
        self.should_apps_implausible = False


        self.node.create_subscription(
            Watchdog, "/vcu/watchdog", self.callbackWatchdog, 1)
        
        accelBrakeSteerPub = self.node.create_publisher(
            AccelBrakeSteer, "/vcu/data/accelBrakeSteer", 1)

        out_accelBrakeSteer = self.assembleAccelBrakeSteer(11.0, 21.7, 100.0, 0.0, 0.0, 0.0, 0.0, 10.0, 11.7, 19.2, 21.7, 29.2)

        pubDict = {accelBrakeSteerPub: out_accelBrakeSteer}

        accelBrakeSteerPub.publish(out_accelBrakeSteer)

        self.sendMessage(pubDict)        
        
        assert self.received

        self.in_test = False

    def test_apps1ErrorLaterApps2Error(self, proc_output):

        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=3)

        self.in_test = True
        self.received = False

        self.should_apps1_error = True
        self.should_apps2_error = False
        self.should_apps_implausible = False

        self.should_apps1_error_later = True
        self.should_apps2_error_later = True

        self.node.create_subscription(
            Watchdog, "/vcu/watchdog", self.callbackWatchdog, 1)
        
        accelBrakeSteerPub = self.node.create_publisher(
            AccelBrakeSteer, "/vcu/data/accelBrakeSteer", 1)

        out_accelBrakeSteer = self.assembleAccelBrakeSteer(11.0, 21.7, 100.0, 0.0, 0.0, 0.0, 0.0, 10.0, 11.7, 19.2, 21.7, 29.2)
        out_accelBrakeSteerLater = self.assembleAccelBrakeSteer(11.0, 21.0, 100.0, 0.0, 0.0, 0.0, 0.0, 10.0, 11.7, 19.2, 21.7, 29.2)

        pubDict = {accelBrakeSteerPub: out_accelBrakeSteer}
        pubDictLater = {accelBrakeSteerPub: out_accelBrakeSteerLater}

        accelBrakeSteerPub.publish(out_accelBrakeSteer)

        self.sendMessageWithDelay(pubDict, pubDictLater)        
        
        assert self.received

        self.in_test = False




