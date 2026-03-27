import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node
import rclpy
from vcu_msgs.msg import PDUOutput, VehicleState, Temps


def get_config_file_path(name, dir='launch/vcu/params'):
    ret = os.path.join(
        get_package_share_directory('vcu_launch'),
        dir,
        name
    )
    return ret

def generate_test_description():
    config_cooling = get_config_file_path("coolingParams.yaml")
    config_base = get_config_file_path("baseParams.yaml")
    config_inverter = get_config_file_path("inverterParams.yaml")
    config_vcu = get_config_file_path('vcuParams.yaml')
    config_stateMachine = get_config_file_path('stateMachineParams.yaml')
    return (
        LaunchDescription(
            [
                # Nodes under test
                Node(
                    package='state_machine',
                    executable='state_machine_node',
                    parameters=[config_stateMachine],
                    output="screen",
                    emulate_tty=True
                ),
                Node(
                    package='vcu',
                    executable='vcu',
                    parameters=[config_base, config_inverter, config_vcu],
                    output="screen",
                    emulate_tty=True
                ),
                Node(
                    package='cooling',
                    executable='cooling_vcu',
                    parameters=[config_cooling],
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
class TestCooling(unittest.TestCase):

    in_test = False
    received = False
    receivedSM = False

    should_cooling_pump = 0.0
    should_pwm_swr = 0.0
    should_pwm_swl = 0.0

    def sendToCooling(self, pubDict):
        end_time = time.time() + 3
        self.publishTime = time.time()
        while time.time() < end_time:
            for pub in pubDict:
                pub.publish(pubDict[pub])
            rclpy.spin_once(self.node, timeout_sec=0.005)


    def callbackSM(self, msg):
        self.receivedSM = True

    def callbackPDUOut(self, msg):
        self.received = True
        if (time.time() - self.publishTime) > 0.2:
            print("-------------pwmCooling=" + str(msg.pwm_cooling_pump) + "should cooling" + str(self.should_cooling_pump))
            assert msg.pwm_cooling_pump >= self.should_cooling_pump
            assert msg.pwm_swl_fan >= self.should_pwm_swl
            assert msg.pwm_swr_fan >= self.should_pwm_swr


    # def assembleInverterTemps(self, t_m1, t_m2, t_m3, t_m4, t_p1, t_p2, t_p3, t_p4, t_s0, t_s5, t_c):
    #     out = InverterData()

    #     out.actual_temp_motor1 = t_m1
    #     out.actual_temp_motor2 = t_m2
    #     out.actual_temp_motor3 = t_m3
    #     out.actual_temp_motor4 = t_m4

    #     out.actual_temp_pwr_module1 = t_p1
    #     out.actual_temp_pwr_module2 = t_p2
    #     out.actual_temp_pwr_module3 = t_p3
    #     out.actual_temp_pwr_module4 = t_p4

    #     out.actual_temp_add_sensor0 = t_s0
    #     out.actual_temp_add_sensor5 = t_s5

    #     out.actual_temp_carrier = t_c

    #     return out
    

    # def assembleRadiatorTemps(self, swl_in, swl_out, swr_in, swr_out):

    #     out = Can1TempRadiator()

    #     out.temp_rad_swl_in = swl_in
    #     out.temp_rad_swl_out = swl_out
    #     out.temp_rad_swr_in = swr_in
    #     out.temp_rad_swr_out = swr_out

    #     return out
    
    def assembleTemps(self, t_m1, t_m2, t_m3, t_m4, t_p1, t_p2, t_p3, t_p4, t_s0, t_s5, t_c, swl_in, swl_out, swr_in, swr_out):
        out = Temps()

        out.actual_temp_motor1 = t_m1
        out.actual_temp_motor2 = t_m2
        out.actual_temp_motor3 = t_m3
        out.actual_temp_motor4 = t_m4

        out.actual_temp_pwr_module1 = t_p1
        out.actual_temp_pwr_module2 = t_p2
        out.actual_temp_pwr_module3 = t_p3
        out.actual_temp_pwr_module4 = t_p4

        out.actual_temp_add_sensor0 = t_s0
        out.actual_temp_add_sensor5 = t_s5

        out.actual_temp_carrier = t_c

        out.temp_rad_swl_in = swl_in
        out.temp_rad_swl_out = swl_out
        out.temp_rad_swr_in = swr_in
        out.temp_rad_swr_out = swr_out

        return out



    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_cooling')

    def tearDown(self):
        self.node.destroy_node()

    def test_cooling_pump(self, proc_output):

        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=5)

        self.in_test = True
        self.received = False
        self.receivedSM = False
        
        self.should_cooling_pump = 240.0

        self.node.create_subscription(
            PDUOutput, "/vcu/pdu/output", self.callbackPDUOut, 1)
        self.node.create_subscription(
            VehicleState, "/vcu/vehicleState", self.callbackSM, 1)
        
        tempPub = self.node.create_publisher(
            Temps, "/vcu/temps", 1)

        out_temps = self.assembleTemps(
            140.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0)

        pubDict = {tempPub: out_temps}

        tempPub.publish(out_temps)

        self.sendToCooling(pubDict)        
        

        assert self.receivedSM
        assert self.received

        self.in_test = False

    def test_both_sw_cooling(self, proc_output):

        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=5)

        self.in_test = True
        self.received = False
        self.receivedSM = False
        
        self.should_pwm_swr = 240.0
        self.should_pwm_swl = 240.0
        self.should_cooling_pump = 0

        self.node.create_subscription(
            PDUOutput, "/vcu/pdu/output", self.callbackPDUOut, 1)
        self.node.create_subscription(
            VehicleState, "/vcu/vehicleState", self.callbackSM, 1)
        
        tempPub = self.node.create_publisher(
            Temps, "/vcu/temps", 1)

        out_temps = self.assembleTemps(
             23.0, 43.0, 10.0, 12.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 120.0, 121.0, 112.0, 123.0)

        pubDict = {tempPub: out_temps}

        tempPub.publish(out_temps)

        self.sendToCooling(pubDict)        
        
        assert self.receivedSM
        assert self.received

        self.in_test = False

    def test_cooling_without_ssbr(self, proc_output):

        while self.in_test:
            rclpy.spin_once(self.node, timeout_sec=0)

        self.in_test = True
        self.received = False
        self.receivedSM = False
        
        self.should_pwm_swr = 0.0
        self.should_pwm_swl = 0.0
        self.should_cooling_pump = 127.0

        self.node.create_subscription(
            PDUOutput, "/vcu/pdu/output", self.callbackPDUOut, 1)
        self.node.create_subscription(
            VehicleState, "/vcu/vehicleState", self.callbackSM, 1)
        
        tempPub = self.node.create_publisher(
            Temps, "/vcu/temps", 1)

        out_temps = self.assembleTemps(
             23.0, 43.0, 10.0, 12.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 0.0, 0.0, 0.0, 0.0)

        pubDict = {tempPub: out_temps}

        tempPub.publish(out_temps)

        self.sendToCooling(pubDict)        
        

        assert self.receivedSM
        assert self.received

        self.in_test = False



