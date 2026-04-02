# VCU

## For all new code follow theses style guides:

- Follow  these for [C++](https://google.github.io/styleguide/cppguide.html)
- Follow  these for [Python](https://google.github.io/styleguide/pyguide.html)

## General Information
This is the VCU repository of the Open Source Vehicle Control Software Stack. It is designed for the cars of the Formula Student Teams from ELBFLORACE Formula Student Team TU Dresden but can also be used by other formula student teams and applications.

The software is designed to run on Ubuntu 24.04 and ROS2 Jazzy. Informations about the components of the software can be found in the according READMEs of the pacakges.

If you want to contribute to the repository please read the guidelines in the [Contributing Guidlines](CONTRIBUTING.md)

<br />

----

# Getting started

## Setting up SSH

Make sure you have git and ssh installed on Ubuntu this can be done through `sudo apt install git ssh`.

Before you can clone the repo, you have to add your ssh key to GitHub for identification. For that go to Preferences->SSH Keys->Add new key. Run `ssh-keygen -t rsa`, press enter enter (for empty passphrase) twice. Then run `cat ~/.ssh/id_rsa.pub` and paste the output into the "Key" field and submit with "Add key".


## Setting up the Repository

First you'll need to create the ROS2 workspace folder structure and clone the repository into it

```bash
mkdir vcu_ws
cd vcu_ws
git clone --recursive git@github.com:ELBFLORACE/OpenSourceVCU.git src
```

### Bare metal

We don't reconmend runing bare metal unless you have specific reasons to do that. For the setup we have a single-point-of-truth [Dockerfile](./docker/Dockerfile), which is based on a ROS2 Jazzy install on Ubuntu 24.04

We have to build some libraries ourselves since they aren't in the standart repo and don't have a PPA. It is reconmended to build them in some other (permanent) directory than /tmp, so that they can be uninstalled cleanly (e.g. for an update) later with the uninstall script which was generated during the build.

### WSL
Using WSL2 can work, but running a native Ubuntu install is still reconmended since it has the best compatibility and gives the best performance.

Apart from that, it is reconmended to also use Docker inside WSL, otherwise it's the same procedure as for bare metal.


<br /><br />

----


# Setting up Docker

Please follow the respective guides for setting up docker for your operating system at [doc/docker/docker_linux.md](doc/docker/docker_linux.md) and [doc/docker/docker_windows.md](doc/docker/docker_windows.md) depending on wether you use Linux or Windows.

## Using plain docker (not recommended)

There is the [`./docker/run_docker.sh`](./docker/run_docker.sh) script which opens a shell inside the docker container and has everything setup (e.g. gui, workspace folder mounted). For this work it is important to run the script out of of the "src" directory and not out of the "docker" directory. You will also have to build the docker image, instructions can found in the file itself. For most usecases it is reconmended to use the dev container setup.

## Using VS Code dev containers

### Adjust devcontainer.json for Windows

Since the devcontainer is designed for Linux based systems, some Linux-specific commands must be commented out.

Comment out these three lines in the devcontainer.json:

````
"source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached",
"source=/dev/dri,target=/dev/dri,type=bind,consistency=cached",
"source=/run/dbus/system_bus_socket,target=/run/dbus/system_bus_socket,type=bind",
````

### Building the dev container

Dev containers are a feature of VS Code which allows to develop inside containers as seamlessly it would be on a native install. This allows to make full use features such as code completion, error checking or running with a debugger. For this to work you have to open the `src` folder in the `vcu_ws` using VSCode

1. Install the Dev Container extension in VS Code
2. Open the command palette (Ctrl+Shift+P) and run:
*Dev Containers: Rebuild and Reopen in container*.
3. Now the window should reopen. When everything went right, it should mostly look the same.
4. Open a terminal (Terminal->New Terminal). You should be in a directory called vcu_ws.
5. Build the repo using `colbuild --parallel-workers 4`. Depending on your available ram, you may increase or decrease the parallel workers.
6. Source the newly built workspace: `source install/setup.bash`
7. Open foxglove studio on your host for the visualizations (for details see [here](./doc/foxglove_how_to.md))
8. Now you should be able to run our vcu locally using 
`ros2 launch vcu_launch vcu.launch.py`

### Post setup notes

* Normally it's enough to just run *Dev Containers: Reopen in container* to save time, but make sure you rebuild your container regularly (e.g. after you pull or when experiencing some issues).

## Important notes about building

For building this workspace at least 16GB of RAM is reconmended. Otherwise limiting the amount of build jobs or building packages seperately may help.

Building in release mode makes the compiler perform optimizations which can make the code run ~10x faster than in Debug mode.

> [!caution]
> Building with colcon is (somehow) extremely demanding regarding memory, it's quite likely that you will run into issues (system freeze or build error) because of that. You can kind of limit the amount of ram used by setting the `--parallel-workers` argument to something smaller (e.g. 4).

An usual build should be called with something like that:

```bash
colbuild --parallel-workers 4
```

<br /><br /><br />

----


# Recommendations
* [VS Code](https://code.visualstudio.com/) is a pretty good editor for this kind of stuff.
    * If you are using VS Code, it will prompt you to install recommended extions after cloning the repo.
    * Please configure your VS Code to include auto formatting on save, as shown below ![](./pics/screenshot_vscode.png)
* [TMUX](https://wiki.ubuntuusers.de/tmux/) or [Tilix](https://gnunn1.github.io/tilix-web/) (Prettier and easier to use) for terminal multiplexing.
    * **HIGHLY RECOMMENDED** - Example of a tmux-session running roscore, roslaunch, rosbag and rviz altogether:
    * A good condig is on config/.tmux.conf, just copy it into your home folder
![Example of tmux session](pics/screenshot_tmux.png)


<br /><br /><br />

----


# Running test

Usual test commands:
```bash
colcon test --ctest-args tests

colcon test-result --all --verbose
```


## Running on DVPC

### Setup ssh
`sudo apt install ssh openssh-server`
`sudo systemctl enable ssh --now`

### Network profile
ip: `192.168.1.59`

netmask: `255.255.0.0`

### .ros directory
The .ros directory is mounted into into the container such that the ros logs are for kept even after rebuilding the container for later diagnostics. In case the ivpc has no .ros directory, create one in the host's home directory. In case one was already automatically created make sure that it has the right permissions (no root as owner), e.g. `sudo chown -R <user>:<user> ~/.ros`.

### Build docker image for PC
Build docker image in src/docker directory `docker build --build-arg USERNAME=devuser -t vcu:dev -f Dockerfile --target dev .`

### Command shortcuts

There are shortcuts for running the launch files, these can be typed (and tabbed) a lot faster than typing `ros2 launch vcu_launch ...` every time.

```bash
foxgloveLaunch       # Launches the foxglove bridge if it is not already running
vcuLaunch            # Launches the VCU, by default season 17
vcuFullRecording     # Starts a full recording
```

```bash
vcuReadable # ros2 topic echo /vcu/vehicleState/readable
```

<details>

<summary>

### More shortcuts

</summary>

### Shortcuts in the docker container

Helpful aliases for normal development workflows:

```bash
psel <package-name>     # Build single package as release
pseld <package-name>    # Build single package as debug

colbuild                # Release build
colbuild-debug          # Debug build
colbuild-reldeb         # Release build with debug infor

col-clean [package-name] # Clean build directories for this package
```

Shortcuts for testdays:

```bash
vcuParams           # Open parameters folder for the vcu
vcuReadable         # Echo /vcu/vehicleState/readable
reloadParameters    # Reload node parameters (only for supported nodes)
```

Generate new can messages from the current can dbc submodules states:

```bash
generate-can
```

### Shortcuts on the DVPC Hostmachine

These shortcuts will be loaded on every bash session started on the VCU / DVPC host machine

```bash
# Recover bag in current folder or the provided filename
recoverBag [filename]

# Open parameter folders
vcuParams
dvParams

# Open the folder containing the newest bag files record
vcuNewestFull
vcuNewestSmall
dvNewestRaw
dvNewestDebug

# Download all inverter logs from current day
syncInverterLogs
# Start a wireshark recording of communication between VCU and inverter
startInverterRecording

# Create a folder containing all bags of todays testday containing only the recovered versions of the files
# Resulting folder at: ~/Desktop/testdayBagFolder
createTestdayBagFolder
```

</details>

## Running on MacOS

Note: The devcontainer is designed for Linux systems and therefore only works to a limited extent under MacOS. In principle, however, it should be possible to build the repo.

## Credits

The example motor driver for the CubeMars AK70-10 has been implemented by the DFKI RIC Underactuated Lab, to whom we are very grateful. The original driver can be found here: 
https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can
