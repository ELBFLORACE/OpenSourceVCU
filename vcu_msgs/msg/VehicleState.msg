std_msgs/Header header

uint8 state                 # State of the state machine
bool sdc_activated          # SDC can be switched on on the PDU
bool sdc_relay_closed       # SDC relay on the ASSM
bool ts_activated           # TS is activated from the VCU
bool actuators_allowed      # Motors are allowed to spin
bool ebs_valve_front        # Valve for separating ebs tank front from brake front    
bool ebs_valve_rear         # Valve for separating ebs tank rear from brake rear
uint8 ebs_state             # EBS State Indicator 
bool watchdog               # lifeliness check state machine

# -----------------------------------------------------------------------------
# For detailed information about the available state look into the 
# documentation of the vcu_state_machine or into the vcu_shared_lib for a 
# programmical insight into the states.
# -----------------------------------------------------------------------------
