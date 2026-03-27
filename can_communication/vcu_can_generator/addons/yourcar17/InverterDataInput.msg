# InverterDataInput
# Cycle time: 2 ms (configurable)

# Motor 1-4 mappings:
#     1 - Rear right
#     2 - Rear left
#     3 - Front left
#     4 - Front right

#Datasheet 6.6 - 6.8 Inverter to VCU msg

# Unit: RPM
int32 actual_speed_motor1
int32 actual_speed_motor2
int32 actual_speed_motor3
int32 actual_speed_motor4

# Unit: RPM
int32 setpoint_speed_motor1
int32 setpoint_speed_motor2
int32 setpoint_speed_motor3
int32 setpoint_speed_motor4

# Unit: Nm
float32 actual_torque_motor1
float32 actual_torque_motor2
float32 actual_torque_motor3
float32 actual_torque_motor4

# Unit: Nm
float32 setpoint_torque_motor1
float32 setpoint_torque_motor2
float32 setpoint_torque_motor3
float32 setpoint_torque_motor4

# +-------+---------------------+
# |  Bit  |     Description     |
# +-------+---------------------+
# |  6-7  | Reserved
# |   5   | Inverter connected (Inverter is marked as connected in the config, 0 = Inverter is not marked as connected in the config (and therefore remains deactivated))
# |   4   | DC-Link ready (1 = DC-Link voltage is above the minimum value defined in the config, 0 = DC-Link voltage too low)
# |   3   | Derating (1 = Due to derating, the currently permitted inverter/motor output current is lower than the specified value ’Is-Max’ in the config (this does not necessarily mean that the inverter is being derated at this moment))
# |   2   | Inverter ready (1 = Inverter is ready to enter drive state, 0 = Not all conditions are met for the inverter to enter drive state (see section 4.1)
# |  0-1  | State (0 = Idle, 1 = Drive, 2 = Error, 3 = config values missing)
# +-------+---------------------+
uint8 state_inv1
uint8 state_inv2
uint8 state_inv3
uint8 state_inv4

# Unit: °C
float32 actual_temp_motor1
float32 actual_temp_motor2
float32 actual_temp_motor3
float32 actual_temp_motor4

# Actual temperature of power switches
# Unit: °C
float32 actual_temp_pwr_module1
float32 actual_temp_pwr_module2
float32 actual_temp_pwr_module3
float32 actual_temp_pwr_module4

# Unit: A_rms
float32 currently_permitted_output_current_m1
float32 currently_permitted_output_current_m2
float32 currently_permitted_output_current_m3
float32 currently_permitted_output_current_m4

# Unit: A_rms
float32 setpoint_current_id_m1
float32 setpoint_current_id_m2
float32 setpoint_current_id_m3
float32 setpoint_current_id_m4

# Unit: A_rms
float32 actual_current_id_m1
float32 actual_current_id_m2
float32 actual_current_id_m3
float32 actual_current_id_m4

# Unit: A_rms
float32 setpoint_current_iq_m1
float32 setpoint_current_iq_m2
float32 setpoint_current_iq_m3
float32 setpoint_current_iq_m4

# Unit: A_rms
float32 actual_current_iq_m1
float32 actual_current_iq_m2
float32 actual_current_iq_m3
float32 actual_current_iq_m4

# Unit: V_rms
float32 output_voltage_ud_m1
float32 output_voltage_ud_m2
float32 output_voltage_ud_m3
float32 output_voltage_ud_m4

# Unit: V_rms
float32 output_voltage_uq_m1
float32 output_voltage_uq_m2
float32 output_voltage_uq_m3
float32 output_voltage_uq_m4

# Current DC link voltage of inverter 1 and 2
# Unit: V
float32 current_dc_link_voltage_12
# Current DC link voltage of inverter 3 and 4
# Unit: V
float32 current_dc_link_voltage_34

#Input DC current of inverter 1 and 2
# Unit: A
float32 input_dc_current_ampere_12

#Input DC current of inverter 3 and 4
# Unit: A
float32 input_dc_current_ampere_34

# Current switching frequency of inverters
# Unit: kHz
float32 current_switching_frequency

# Indication of the "Control Enable" input pin
uint8 mirror_control_enable_disable_input

float32 actual_temp_add_sensor0
float32 actual_temp_add_sensor1
float32 actual_temp_add_sensor2
float32 actual_temp_add_sensor3
float32 actual_temp_add_sensor4

# Actual temperature at the edge of HV-Board 1-2
# Unit: °C
float32 actual_temp_edge_hv_12

# Actual temperature at the middle of HV-Board 1-2
# Unit: °C
float32 actual_temp_middle_hv_12

# Actual temperature at the edge of HV-Board 3-4
# Unit: °C
float32 actual_temp_edge_hv_34

# Actual temperature at the middle of HV-Board 1-4
# Unit: °C
float32 actual_temp_middle_hv_14

# Actual temperature of the carrier board
# Unit: °C
float32 actual_temp_carrier

# Actual temperature of CPU
# Unit: °C
float32 actual_temp_cpu

# Actual temperature of System on Module (top side of board)
# Unit: °C
float32 actual_temp_module_top

# Actual temperature of System on Module (bottom side of board)
# Unit: °C
float32 actual_temp_module_bottom

# Actual low voltage current consumption of inverter
# Unit: A
float32 actual_lv_current

# Actual power supply voltage of inverter
# Unit: V
float32 actual_power_supply_voltage

# Elapsed time since startup of inverter
# Unit: s
float32 time
