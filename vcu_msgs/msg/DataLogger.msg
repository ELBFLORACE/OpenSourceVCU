std_msgs/Header header

# Cycle time 100ms

# Frame id:   0x511
bool send_asf_message
float32 ebs_front_pressure # bar
float32 ebs_rear_pressure # bar
float32 brakepressure_front # bar
float32 brakepressure_rear # bar

# Frame id:   0x500
float32 speed_actual # km/h
float32 speed_target # km/h
float32 steering_angle_actual # °
float32 steering_angle_target # °
float32 brake_hydr_actual # %
float32 brake_hydr_target # %
float32 motor_moment_actual # %
float32 motor_moment_target # %

# Frame id:   0x501
float32 acceleration_longitudinal # m/s²
float32 acceleration_lateral # m/s²
float32 yaw_rate # °/s

# Frame id:   0x502

# 1 Off
# 2 Ready
# 3 Driving
# 4 Emergency
# 5 Finished
uint8 as_state

# 1 Deactivated
# 2 Initial checkup passed
# 3 Activated
uint8 ebs_state

# 1 Acceleration
# 2 Skidpad
# 3 Trackdrive
# 4 Braketest
# 5 Inspection
# 6 Autocross
uint8 ami_state

bool steering_state

# ASB_redundancy_state
# 1 Deactivated
# 2 Engaged
# 3 Initial checkup passed
uint8 ebs_redundancy_state

uint16 lapcounter
uint16 cones_count_actual
uint32 cones_count_all
