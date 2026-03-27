std_msgs/Header header

bool pdu_timeout                    # Timeout for PCB that powers all other PCBs
bool ssb_front_timeout              # Timeout for PCB that transfers analog sensor signals into digital values for sensors like APPS
bool ssb_rear_timeout               # Timeout for PCB that transfers analog sensor signals into digital values for senors like cooling temp
bool bse_timeout                    # Timeout for PCB that does the evaluation of the recu pedal    
bool ams_timeout                    # Accu timeout
bool energymeter_timeout            # Timeout for the Energymeter in the accu
bool dis_timeout                    # Timeout for the display & steering wheel
bool inverter_timeout               # Inverter timeout
bool assm_timeout                   # Timeout for the PCB that controls ASSIs and EBS Valves
bool dv_timeout                     # Timeout of the driverless system
bool apps_implausible               # APPs values divergating more than allowed
bool apps1_error                    # Invalid value of APPs Sensor 1
bool apps2_error                    # Invalid value of APPs Sensor 2
bool brake_pressure_front_error     # Invalid value of brake pressure sensor for front brake system
bool brake_pressure_rear_error      # Invalid value of brake pressure sensor for rear brake system
bool ebs_front_error                # Invalid value of EBS pressure sensor for front brake system
bool ebs_rear_error                 # Invalid value of EBS pressure sensor for rear brake system
bool brake_force_error              # Invalid value of the recu pedal sensor
bool pedals_implausible             # Accelerator pedal and brake pedal pressed on the same time    
bool reku_state                     # Recu enabled