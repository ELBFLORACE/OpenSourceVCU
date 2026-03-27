std_msgs/Header header

ix_msgs/Wheels wheelspeed_setpoints # rotations / minute
ix_msgs/Wheels lower_torque_bounds # Nm; torques cannot be smaller than that
ix_msgs/Wheels upper_torque_bounds # Nm; torques cannot be larger than that

ix_msgs/Wheels current_control             # Bool array
ix_msgs/Wheels asc_allowed                 # Bool array
ix_msgs/Wheels reset                       # Bool array
ix_msgs/Wheels enabled                     # Bool array
