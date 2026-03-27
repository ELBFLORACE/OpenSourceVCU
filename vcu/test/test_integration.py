import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import rclpy

from ix_msgs.msg._inverter_setpoints import InverterSetpoints
from ix_msgs.msg._wheels import Wheels

from vcu_msgs.msg._vehicle_state import VehicleState
from ix_msgs.msg._inverter import Inverter

from ament_index_python.packages import get_package_share_directory


def get_config_file_path(name, dir='launch/vcu/params'):
    ret = os.path.join(
        get_package_share_directory('vcu_launch'),
        dir,
        name
    )
    return ret


def generate_test_description():
    vehicle_mission = "acceleration"
    vehicle_mode = "ev"
    config_vcu = get_config_file_path('vcuParams.yaml')
    config_inverter = get_config_file_path("inverterParams.yaml")
    config_discipline = get_config_file_path(
        f"{vehicle_mode}_{vehicle_mission}.yaml")
    return (
        launch.LaunchDescription(
            [
                # Nodes under test


                launch_ros.actions.Node(
                    package='vcu',
                    executable='vcu',
                    parameters=[config_inverter,
                                config_discipline, config_vcu],
                    output="screen",
                    emulate_tty=True,
                    name='vcu',
                ),
                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ), {},
    )


# Active tests
class TestVCU(unittest.TestCase):

    wspd = 500
    torque_upper = 0
    torque_lower = 0
    received = False
    in_test = False

    def send_to_inverter(self, wspd, torque_upper, torque_lower, publisher):
        out_controls = InverterSetpoints()
        out_controls.wheelspeed_setpoints = wspd
        out_controls.upper_torque_bounds = torque_upper
        out_controls.lower_torque_bounds = torque_lower

        end_time = time.time() + 1
        while time.time() < end_time:
            publisher.publish(out_controls)
            # spin to get subscriber callback executed
            rclpy.spin_once(self.node, timeout_sec=1)

    def assembleWheelsMessage(self, value):
        out = Wheels()
        out.fl = value
        out.fr = value
        out.rl = value
        out.rr = value

        return out

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def callbackInverterOutput(self, msg):
        self.received = True
        if msg.wheelspeed_setpoints.fl != self.wspd:
            print(msg.wheelspeed_setpoints.fl)
            print(self.wspd)
        assert msg.wheelspeed_setpoints.fl == self.wspd
        assert msg.wheelspeed_setpoints.fr == self.wspd
        assert msg.wheelspeed_setpoints.rl == self.wspd
        assert msg.wheelspeed_setpoints.rr == self.wspd

        assert msg.lower_torque_bounds.fl == self.torque_lower
        assert msg.lower_torque_bounds.fr == self.torque_lower
        assert msg.lower_torque_bounds.rl == self.torque_lower
        assert msg.lower_torque_bounds.rr == self.torque_lower

        assert msg.upper_torque_bounds.fl == self.torque_upper
        assert msg.upper_torque_bounds.fr == self.torque_upper
        assert msg.upper_torque_bounds.rl == self.torque_upper
        assert msg.upper_torque_bounds.rr == self.torque_upper

    def setUp(self):
        self.node = rclpy.create_node('test_vcu')

    def tearDown(self):
        self.node.destroy_node()

    def test_inverter_output_not_allowed_to_actuate(self, proc_output):
        """Check whether inverter output is correct when not allowed to actuate"""
        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)

        self.in_test = True
        self.node.create_subscription(
            Inverter, '/vcu/inverter/output', self.callbackInverterOutput, 1)
        controls_pub = self.node.create_publisher(
            InverterSetpoints, '/controls/inverter/request', 1)
        state_machine_pub = self.node.create_publisher(
            VehicleState, "/vcu/vehicleState", 1)

        out_vehicle_state = VehicleState()
        out_vehicle_state.actuators_allowed = False
        out_vehicle_state.state = 7

        state_machine_pub.publish(out_vehicle_state)

        rclpy.spin_once(self.node, timeout_sec=1)

        self.wspd = 500.0
        self.torque_lower = 0.0
        self.torque_upper = 0.0
        self.received = False

        out_wspd = self.assembleWheelsMessage(800.0)
        out_torque_upper = self.assembleWheelsMessage(10.0)
        out_torque_lower = self.assembleWheelsMessage(0.0)

        self.send_to_inverter(out_wspd, out_torque_upper,
                              out_torque_lower, controls_pub)

        assert self.received
        self.in_test = False

    def test_inverter_output_crop_allowed_to_actuate(self, proc_output):
        """Check whether inverter output is correct when allowed to actuate"""

        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)
        controls_pub = self.node.create_publisher(
            InverterSetpoints, '/controls/inverter/request', 1)
        state_machine_pub = self.node.create_publisher(
            VehicleState, "/vcu/vehicleState", 1)
        self.node.create_subscription(
            Inverter, '/vcu/inverter/output', self.callbackInverterOutput, 1)

        out_vehicle_state = VehicleState()
        out_vehicle_state.actuators_allowed = True
        out_vehicle_state.state = 7

        state_machine_pub.publish(out_vehicle_state)

        rclpy.spin_once(self.node, timeout_sec=1)

        self.wspd = 800.0
        self.torque_lower = 0.0
        self.torque_upper = 10.0
        self.received = False

        out_wspd = self.assembleWheelsMessage(800.0)
        out_torque_upper = self.assembleWheelsMessage(10.0)
        out_torque_lower = self.assembleWheelsMessage(0.0)

        self.send_to_inverter(out_wspd, out_torque_upper,
                              out_torque_lower, controls_pub)

        assert self.received
        self.in_test = False

    def test_inverter_output_negative_wheel_spin(self, proc_output):
        """Check whether inverter output can become negative"""

        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=1)

        controls_pub = self.node.create_publisher(
            InverterSetpoints, '/controls/inverter/request', 1)
        state_machine_pub = self.node.create_publisher(
            VehicleState, "/vcu/vehicleState", 1)
        self.node.create_subscription(
            Inverter, '/vcu/inverter/output', self.callbackInverterOutput, 1)

        out_vehicle_state = VehicleState()
        out_vehicle_state.actuators_allowed = True
        out_vehicle_state.state = 7

        state_machine_pub.publish(out_vehicle_state)

        rclpy.spin_once(self.node, timeout_sec=1)

        self.wspd = 0.0
        self.torque_lower = 0.0
        self.torque_upper = 10.0
        self.received = False

        out_wspd = self.assembleWheelsMessage(-800.0)
        out_torque_upper = self.assembleWheelsMessage(10.0)
        out_torque_lower = self.assembleWheelsMessage(0.0)

        self.send_to_inverter(out_wspd, out_torque_upper,
                              out_torque_lower, controls_pub)

        assert self.received

        self.received = False
        self.wspd = 0.0
        self.torque_lower = 0.0
        self.torque_upper = 0.0

        out_torque_upper = self.assembleWheelsMessage(-10.0)

        self.send_to_inverter(out_wspd, out_torque_upper,
                              out_torque_lower, controls_pub)

        assert self.received

        self.received = False
        self.wspd = 800.0
        self.torque_lower = 0.0
        self.torque_upper = 0.0

        out_wspd = self.assembleWheelsMessage(800.0)

        self.send_to_inverter(out_wspd, out_torque_upper,
                              out_torque_lower, controls_pub)

        assert self.received
        self.in_test = False
