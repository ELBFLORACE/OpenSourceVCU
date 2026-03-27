# Message from the VCU node to the State Machine comming from the Autonomous System State Machine PCB (ASSM) 

std_msgs/Header header

bool wd_reset                       # Mirroring the alternating watchdog sended to the ASSM
bool wd_ok                          # Indicator of HW watchdog state of the ASSM
bool wd_status                      # Something from the HW watchdog of the ASSM
bool sdc_ready_state                # HW Watchdog is in ok state and SDC reaches the ASSM
bool sdc_relay_state                # State of the SDC relais on the ASSM 
bool sdc_out                        # SDC is leaving the ASSM
bool sdc_in                         # SDC reaches the ASSM
bool sdc_close_state                # Mirror of the SDC relais ASSM control message
bool external_ts_on                 # External Tractive System button pressed
bool ebs_powerstage_rear_state      # Shows if EBS valve front is turned on
bool ebs_powerstage_front_state     # Shows if EBS valve rear is turned on
bool assi_blue_on                   # ASSIs are illuminated in blue
bool assi_yellow_on                 # ASSIs are illuminated in yellow
bool asms_on                        # ASMS state
bool ebs_front_release_state        # State of the EBS release 1
bool ebs_rear_release_state         # State of the EBS release 2
