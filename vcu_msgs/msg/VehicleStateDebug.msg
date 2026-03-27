std_msgs/Header header

uint8 esus_state    # EBSStartupSequence state, helps to determine where the EBS Startup failed
bool mission_set    # Shows if a mission is selected
bool asms_on        # State of the ASMS, also in the vcu_msgs/ASSM message
bool sdc_ready      # Shows if the HW watchdog is in ok state on the ASSM
uint8 stfr          # StateTransitionFailureReason, helps to determine why a state transition failed
bool finished       # DV state finished
uint32 errors       # Current errors as bitmap