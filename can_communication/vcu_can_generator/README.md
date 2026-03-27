# EFR CAN generator

This script parses `.dbc` files from the DBC repositories (for example [`efr16ed04/can-dbc-efr16`](https://git.elbflorace.de/control/efr16ed04/can-dbc-efr16)) which define the CAN interfaces. It then generates necessary files for reading those out and converting them to ROS messages in the mailman (sending is also possible). The ROS messages then get used by the `efr_monitoring_backend` to provide live data for the monitoring frontend. The individual outputs are further described in the [Output](#output) section.

### Contact Person(s)

Marvin Jacob, marvin.jacob@elbflorace.de

# Setup

If you are using the docker container (which is recommended), all you need to do is execute the `generate-can` command in the bash shell of the container. This will trigger a full regeneration of all CAN related files, before running the command make sure that your submodules are up to date (See [#Updating Submodules](#updating-submodules)) and that you are using the correct version of the docker container (See [#Rebuilding the Container](#rebuilding-the-container). The command is stored in the [`docker/bash_aliases/can_generator.sh`](../docker/bash_aliases/can_generator.sh) script and internally uses the [`vcu_can_generator/script.sh`](./script.sh) which should not be used manually. There are a lot of things that can go wrong, if they do take a look at the [Troubleshooting](#troubleshooting) section.

When running the script the output should look like this:

```bash
$ generate-can
Using /home/devuser/vcu_ws/src/./vcu_can_msgs_yourcar17/can-dbc-yourcar17 for DBC file
Generating for season yourcar17
Found venv at /home/devuser/vcu_ws/src/vcu_can_generator/venv
............................................
Generating for ['CAN1_Vehicle.dbc', 'CAN2_Accu.dbc', 'CAN3_DV.dbc', 'CAN4_GPS_IMU.dbc', 'FSG_DBC_DV.dbc']
............................................
Replacing vcu_can_msgs_yourcar17
Replacing mailman_yourcar17
Replacing proto files in efr_monitoring_backend
```

<details>

<summary>Setup without docker</summary>

### protoc

This project uses version **25.3** of the [protoc](https://protobuf.dev/getting-started/pythontutorial/#compiling-protocol-buffers) compiler to generate the python code that handles the protobuf encoding. The compiler can be installed by downloading the binary from the github releases at [github.com/protocolbuffers/protobuf/releases/tag/v25.3](https://github.com/protocolbuffers/protobuf/releases/tag/v25.3), just download the zip for your platform and copy the `protoc` file from the `bin` folder to `/usr/local/bin/`.

</details>

<details>

<summary>Running the script manually</summary>

If you want to generate the files manually(not recommended) you can do so by using the [`script.sh`](./script.sh) file. To run the script you need to clone a DBC repository like the [`efr16ed04/can-dbc-efr16`](https://git.elbflorace.de/control/efr16ed04/can-dbc-efr16) repository, there should be no uncommitted changes to the `.dbc` files when the script is executed. The current git commit is included in every file to make it possible to validate that all generated files were generated from the same `.dbc` files.

The next step is to open your terminal in the root folder of this repository and run the following command

```bash
./vcu_can_generator/script.sh /path/to/can_dbc_files
```

> All inputs can be disabled by setting the `CI` environment variable, this will automatically apply the newly generated files aswell

</details>

<br /><br /><br /><br /><br />

# Output

```tree
dist
├── vcu_can_msgs
│   ├── CMakeLists.txt
│   ├── msg
│   │   └── CanXMessage.msg
│   ├── package.xml
│   └── README.md
├── mailman
│   ├── include
│   │   └── generated
│   │       └── CanX.hpp
│   └── src
│       └── generated
│           ├── CanXDecoding.cpp
│           └── CanXUnpacking.cpp
└── efr_monitoring_backend
    └── src
        └── proto
            ├── CanX_pb2.py
            ├── CanX_pb2.pyi
            ├── CanX.proto
            ├── Full_pb2.py
            ├── Full_pb2.pyi
            └── Full.proto
```

## vcu_can_msgs

This folder contains the entire `vcu_can_msgs` package which contains ROS Message definitions for every single CAN message. If you don't use the script and want to copy over the files yourself make sure you delete the old folder first before copying the new one. Otherwise messages that don't exist anymore might still be there after using the "overwrite" option in your file manager.

All generated messages have a header comment containing meta information about the message and comments above each field. Below is an example message:

```ros
# Can1AppsBseSteering
# generated with commit 081ed59d4222dec1dcdba9df0ec63ea53747c7c8
# Frame id:   0x11
# Senders:    SSB_Front
# Cycle time: 5.0 ms

builtin_interfaces/Time stamp

# Range: 0..1897.5 N
# Unit:  N
float32 brakeforce

# Range: -
# Unit:  -
# Choices:
# 1 = SDC closed
# 0 = SDC open
uint8 bms_sdc_state
...
```

The script includes the last commit hash of your local copy of the dbc repository. This is done to make trouble shooting issues easier as mixing files from different commtis can lead to hard to debug issues.

The frame id is simply a hexadecimal representation of the frame id of the CAN messages, senders is a list of senders specified in the `.dbc` file but has no further meaning for the current use cases. The cycle time is the interval at which the CAN messages get sent and should match the interval at which their ROS message equivalents get sent out by the `mailman`. The mailman sends out all messages under the subtopics of `/sensor/`. For example the message `Can1AppsBseSteering` would get sent on `/sensor/can1/apps_bse_steering`.

Every field also has a comment specifying it's range and unit if they are included in the `.dbc` files. While most of the time those are correct, they are not guaranteed to be correct and don't get checked. If the field is a enum with documented choices then those are also included as a comment like shown above.

> **NOTE:** All messages of the `vcu_can_msgs` package might be subject to change and should most likely not be relied on if a stable interface is needed.

<br /><br />

## mailman

This folder contains the necessary code for unpacking the CAN messages into structs (> `CanXUnpacking.cpp`). And the code for offsetting, scaling and converting those structs to ROS message to then publish them on their respective topics (> `CanXDecoding.cpp`). The header files in the `include` folder contain struct definitions for all CAN messages (> `CanX.hpp`). The C++ code generator is a modified version of the [cantools C generator](https://github.com/cantools/cantools/blob/master/src/cantools/subparsers/generate_c_source.py).

In this context unpacking means converting the raw bytes in the CAN frames to the structs defined in the header files through bitshifts and masks. At this point all values are integers and practically useless because they still need to get offset and scaled to the correct values.

Decoding means offsetting and scaling the values correctly, after this everything that isn't an enum becomes a 32 bit floating point number. After this step the values get sent out as a ROS message, the decoding files include the code for creating the necessary ROS publishers.

### Receiving

All the code necessary for receiving and republishing CAN frames as ROS messages is automatically generated. The CAN messages automatically get published on topics following the pattern of `/sensor/can1/apps_bse_steering` after the mailman is started.

<details>

<summary>Receiving flow</summary>

A typical receiving flow in the mailman would look like this

```cpp
// Unpack CAN frame to C++ struct
this->can2_0x115_energymeter_temperature_unpack(frame->data, bytes_read);
// "can2_0x115_energymeter_temperature" is a property of the CAN reader and gets reused
auto src_p = &can2_0x115_energymeter_temperature;

auto msg = vcu_can_msgs_yourcar17::msg::Can2EnergymeterTemperature(); // Initialise ROS message
msg.stamp = this->node->now(); // Set timestampt to receiving time

msg.temp_energymeter = (float)(src_p->temp_energymeter) * 0.1; // Scaling & Offset adjustments

can2_0x115_energymeter_temperature_pub->publish(msg); // Publish the message
```

</details>

### Sending

Messages can also be sent out, for this it is necessary to construct a ROS message of the CAN message that should be sent. The `send_on_can` method than handles the entire conversion like shown below:

```cpp
auto msg = vcu_can_msgs_yourcar17::msg::Can2VoltageOverview1();

msg.cellvoltage_1 = 50.3f;
msg.cellvoltage_2 = 50.3f;
msg.cellvoltage_3 = 50.3f;
msg.cellvoltage_4 = 50.3f;

send_on_can(mmData->fd_can3, &msg);
```

If manual adjustments to the bit representation of the CAN message are needed, instead of using the `send_on_can` method the following code can be used to be able to change bits in the raw CAN frame data. (Note: This is simply the content of the `send_on_can` method and the types need to be changed depending on the message)

```cpp
// send_on_can(mmData->fd_can3, &msg);

struct can_frame frame2;
frame2.can_id = 0x501;
frame2.can_dlc = 24;

struct can2_0x501_voltage_overview_1_t tmp;
can2_0x501_voltage_overview_1_t::from_msg(&tmp, &msg);
tmp.pack(frame2.data, 8);

// <- Adjust frame2.data here

write(mmData->fd_can3, &frame, sizeof(struct can_frame));
```

<br /><br />

## efr_monitoring_backend

The `efr_monitoring_backend/src/proto` folder contain protobuf definitions for their ROS message equivalents and python files which are used to serialize those messages. The `.pyi` files are only stub files necessary to get autocompletion in IDEs. Each `CanX.proto` file contains the protobuf definition of every CAN message and one big message called `CanX` that has every message of that CAN as an optional field. The `Full.proto` file contains one message called `FullMessage` which has a timestamp and all `CanX` messages as optional fields.

Making all fields optional allows sending out partial messages which is used to only send out the currently viewed data in the monitoring frontend. The protobuf files do not contain any comments with ranges or units, if those are needed look at the ROS message definitions in the `vcu_can_msgs` package.

The monitoring also uses Inverter Data, because of that the can generator also exports protobuf and ROS message definitions for those.

> **NOTE:** The `.proto` files also get used in the [`autonomous_system/live-monitoring-frontend`](https://git.elbflorace.de/autonomous_system/live-monitoring-frontend) dynamically

## Additional Notes

Everyt field that is not declared as an enum is assumed to be a floating point number, this is for simplicity and counteracts inconsistent definitions in the `.dbc` files.

The message `Applied import fix` means the imports in the `Full_pb2.py` file were modifiied to use relative paths, this resolves some issues with python being unable to import the modules.

<br /><br /><br /><br /><br />

# Troubleshooting

## ".../can-dbc-efrXX is empty"

Similiar to "error: /home/devuser/vcu_ws/src/vcu_can_msgs_yourcar17/can-dbc-yourcar17" and "Warning: There are uncommitted changes in ..../can-dbc-yourcar17". All of these problems are due to git submodules, most of the time simply entering the following commands should solve the issue.

### Updating submodules

```bash
git submodule init
git submodule update
# If it doesnt work try again with:  --force --recursive --remote
```

If you want to prevent this from happening in the future, you can tell git to automatically update submodules on checkout in the current repository by running `git config --local submodule.recurse true`. Note that this might create some unwanted behaviour though.

## Issues with python or pip

Try running `rm -rf vcu_can_generator/venv` to delete the current venv and then rerun the `generate-can` command.

## "command not found: protoc" or "command not found: generate-can"

### Rebuilding the Container

Most likely this is caused by running an outdated version of the docker container. If you are using VSCode you can run "Dev Containers: Rebuild Container" or "Dev Containers: Rebuild and Reopen in Container" in the command palette to get an up to date version of the docker container. If not using VSCode check the [`docker/run_docker.sh`](../docker/run_docker.sh) for instructions on how to build the docker container.

# References

The `efr16ed04/can-dbc-efr16` repository containing the `.dbc` files

https://git.elbflorace.de/control/efr16ed04/can-dbc-efr16

### Parsing the .dbc files

- The [cantools/cantools](https://github.com/cantools/cantools) python package
  - https://github.com/cantools/cantools/blob/master/src/cantools/subparsers/generate_c_source.py
  - https://github.com/cantools/cantools/blob/master/src/cantools/database/can/c_source.py

### Python and protobuf

https://protobuf.dev/getting-started/pythontutorial/
