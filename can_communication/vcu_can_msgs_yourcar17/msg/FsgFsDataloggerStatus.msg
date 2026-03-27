# FsgFsDataloggerStatus
# Frame id:   0x430
# Senders:    Datalogger
# Cycle time: -

builtin_interfaces/Time stamp

# Range: 0..255 
# Unit:  -
float32 msg_cnt

# Range: -
# Unit:  -
float32 status_ready

# Range: 0..1 
# Unit:  -
float32 status_logging

# Range: -
# Unit:  -
float32 status_triggered_voltage

# Range: -
# Unit:  -
float32 status_triggered_current

# Range: 0..1048560 mV
# Unit:  mV
float32 voltage

# Range: 0..4194240 mA
# Unit:  mA
float32 current
