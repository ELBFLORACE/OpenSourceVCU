#!/bin/bash

# Seasons separated by spaces, for example:
# seasons="16 17 18"
seasons="17"

generate-can() {
    cd ~/vcu_ws/src

    for currentSeason in $seasons; do
        CI=1 SEASON=$currentSeason ./can_communication/vcu_can_generator/script.sh "./can_communication/vcu_can_msgs_yourcar$currentSeason/can-dbc-yourcar$currentSeason"
        printf "\n\n\n"
    done
}
