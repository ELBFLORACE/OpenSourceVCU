builtin_interfaces/Time stamp

int8 lapcount
int16 cone_count_total      # Cones in map
int16 cone_count_current    # Currently detected cones

int32 direction             # Driving direction?

int8 ami_mode               # 0 = manual, 1 acceleration, 2 skidpad, 3 autox, 4 trackdrive, 5 ebs test, 6 scruti
bool ready_to_drive
bool finished
