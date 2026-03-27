std_msgs/Header header

ix_msgs/Wheels wheelspeed_setpoints        # our setpoint rotations / minute
ix_msgs/Wheels actual_wheelspeeds          # actual wheel rotations / minute
ix_msgs/Wheels actual_torque               # Nm; actual torques on the wheel
ix_msgs/Wheels torque_setpoints             # Nm; our torque setpoints

ix_msgs/Wheels inverter_connected          # Bool array
ix_msgs/Wheels dclink_ready                # Bool array
ix_msgs/Wheels derating                    # Bool array
ix_msgs/Wheels inverter_ready              # Bool array
ix_msgs/Wheels state                       # uint8

float32 current_dc_link_voltage_12
float32 current_dc_link_voltage_34

