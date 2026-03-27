# InverterDataOutput
# Cycle time: 2 ms (configurable)

# Motor 1-4 mappings:
#     1 - Rear right
#     2 - Rear left
#     3 - Front left
#     4 - Front right

#table 6.3 VCU to Inverter msg

# Unit: RPM -- NEW
int32 setpoint_speed_inv1
int32 setpoint_speed_inv2
int32 setpoint_speed_inv3
int32 setpoint_speed_inv4

# Unit: Nm -- NEW
float32 torque_limit_pos_motor1
float32 torque_limit_pos_motor2
float32 torque_limit_pos_motor3
float32 torque_limit_pos_motor4

# Unit: Nm -- NEW
float32 torque_limit_neg_motor1
float32 torque_limit_neg_motor2
float32 torque_limit_neg_motor3
float32 torque_limit_neg_motor4

# Unit: - -- NEW
# Datasheet Table 6.4
uint8 control_byte_motor1
uint8 control_byte_motor2
uint8 control_byte_motor3
uint8 control_byte_motor4

# Unit: A_rms -- NEW
float32 setpoint_current_id
float32 setpoint_current_iq
