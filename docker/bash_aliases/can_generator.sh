#!/bin/bash

# Seasons separated by spaces
seasons="16 17"

generate-can() {
    cd ~/vcu_ws/src

    for currentSeason in $seasons; do
        CI=1 SEASON=$currentSeason ./can_communication/vcu_can_generator/script.sh "./can_communication/efr_can_msgs_efr$currentSeason/can-dbc-efr$currentSeason"
        printf "\n\n\n"
    done
}
