# Message from the VCU node to the mailman for the Driver Information System (DIS)

std_msgs/Header header

uint8 selected_mission
int8 torque_upper_bound
int8 torque_lower_bound
float32 power_limit
float32 rpm_limit

float32 slip_acceleration
float32 slip_braking
bool as_state
bool asr_enabled
bool bsr_enabled
bool tv_enabled


uint8 bms_state
bool bms_sdc_state
float32 accumulator_voltage
float32 bms_soc
float32 cvh
float32 cvl
uint8 cth
uint8 ctl

uint16 ebs_pressure_front
uint16 ebs_pressure_rear
uint16 brake_pressure_front
uint16 brake_pressure_rear

bool reku_enabled
bool push_to_talk_state

# Pedal values 0-1
float32 accel_pedal
float32 reku_pedal

# Display Navigation
bool display_lock_screen
bool display_navigate_left
bool display_navigate_right
uint8 display_select_screen

