## Contact Person(s)
Niklas Leukroth niklas.leukroth@elbflorace.de
## Intention
This packages takes raw UDP frames and transforms them into readable ROS messages. 
## Features
Translation of all messages which are defined in docs/CanÜbersichtEFR13.xlsx in the DV-UDP tab. Publishes to respective topics.
## Usage
### Examples
        roslaunch efr_launch mailman.launch
### Interfaces
* One or more pictures of the communication between the nodes when starting the launch files and rqt\_graph (so that the topics or services are visible)
#### Input
* Reference nodes/packages whose output you're using (topics/services).
* */vehicle_control/longitudinal : ix_msgs/LongitudinalController* - Thorttle.
* */controls/steering_front_setpoint : ix_msgs/LateralController* - Steering angle.
* */state_controller/observation : ix_msgs/    observer* - State of observer node.
#### Output
* */mailman/state_estimation : ix_msgs/StateEstimation* - State estimation data which comes from VCU.
* */sensors/motor_speeds : ix_msgs/WSPD* - Wheelspeed data from the inverters.
* */sensors/motor_torques : ix_msgs::WSPD* - Torque data from the inverters.
* */sensors/steering_wheel_angle : std_msgs::Float32* - Steering angle from SSB.

### Launch Files
* efr_launch/launch/start_up/mailman.launch
* Parameters:
    * /mailman/pc/ports/X: ports of the corresponsing incoming packages
    * /mailman/vcu/ip_address
    * /mailman/vcu/ports/X: ports of the corresponsing outgoing packages
    
## Possible Features in the Future
* Maybe you could create a generic system which can load the translations from a config file
## Resources
* docs/CanÜbersichtEFR13.xlsx containing message definitions.

## Notes for debugging
Create virtual can interface to see sent messages
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip link show vcan0
candump vcan0
``` 

# Setup cans on IVPC
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can3 type can bitrate 1000000

sudo ip link set down can0
sudo ip link set down can1
sudo ip link set down can2
sudo ip link set down can3

sudo ip link set up can0
sudo ip link set up can1
sudo ip link set up can2
sudo ip link set up can3
```
can0 -> can 1
can3 -> can 4