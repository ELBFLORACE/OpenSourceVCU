#!/bin/bash

foxgloveLaunch() {
    ros2 launch vcu_launch efr-foxglove.launch.py
}

vcuLaunch() {
    local season=$(getSeasonParameter "$@")
    ros2 launch vcu_launch efr$season-vcu.launch.py
}

vcuFullRecording() {
    local season=$(getSeasonParameter "$@")
    ros2 launch vcu_launch efr$season-fullRecording.launch
}

vcuParams() {
    local season=$(getSeasonParameter "$@")
    cd ~/vcu_ws/src/vcu_launch/launch/efr$season/params
}

vcuReadable() {
    ros2 topic echo /vcu/vehicleState/readable
}

reloadParameters() {
    ros2 service call /debug/reload_all_parameters std_srvs/srv/Trigger '{}'
}

function getSeasonParameter() {
    local defaultSeason="17"
    if [ $# -eq 0 ]; then
        echo $defaultSeason
    else
        echo $1
    fi
}
