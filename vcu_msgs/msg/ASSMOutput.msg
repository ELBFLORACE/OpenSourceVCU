# Message sended from the VCU node to the mailman for the Autonomous System State Machine PCB (ASSM)
std_msgs/Header header

uint8 as_state                  # Controls the ASSI color and on/off cycle
bool watchdog_assm              # Alternating for the HW watchdog on the ASSM
bool ebs_power_stage_front      # Control of the EBS valve front
bool ebs_power_stage_rear       # Control of the EBS valve rear
bool assm_sdc_close_cmd         # Control of the SDC relais on the ASSM
