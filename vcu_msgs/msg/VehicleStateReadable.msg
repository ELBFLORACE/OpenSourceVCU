std_msgs/Header header

string<=15 vehicle_state        # Vehicle State as string
string<=20 ebs_state            # Emergency Brake System State as string
string<=25 esus_state           # EBSStartupSequenceState as string
string<=25 stfr                 # StateTransitionFailureReason as string
bool sdc_closed                 # SDC is reaching the accumulator
bool asms_state                 # ASMS State
bool actuators_allowed          # StateMachine allows actuation
bool sdc_state                  # SDC is turned on
bool finished                   # DV mission state
string<=22[] errors             # List of all current errors