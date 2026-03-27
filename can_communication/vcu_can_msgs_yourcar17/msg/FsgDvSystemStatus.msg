# FsgDvSystemStatus
# Frame id:   0x502
# Senders:    VCU_Driverless
# Cycle time: -

builtin_interfaces/Time stamp

# Range: -
# Unit:  -
# Choices:
# 5 = AS Finish
# 4 = AS Error
# 3 = AS Drive
# 2 = AS Ready
# 1 = AS OFF
uint8 as_state

# Range: -
# Unit:  -
# Choices:
# 3 = activated
# 2 = armed
# 1 = unavailable
uint8 ebs_state

# Range: -
# Unit:  -
# Choices:
# 6 = autocross
# 5 = inspection
# 4 = braketest
# 3 = trackdrive
# 2 = skidpad
# 1 = acceleration
uint8 ami_state

# Range: 0..1 bool
# Unit:  bool
# Choices:
# 1 = ON
# 0 = OFF
uint8 steering_state

# Range: -
# Unit:  -
# Choices:
# 3 = available
# 2 = engaged
# 1 = disengaged
uint8 asb_redundancy_state

# Range: -
# Unit:  -
float32 lapcounter

# Range: -
# Unit:  -
float32 cones_count_actual

# Range: 0..65535 
# Unit:  -
float32 cones_count_all
