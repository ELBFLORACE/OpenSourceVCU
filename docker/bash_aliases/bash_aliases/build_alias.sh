#!/bin/bash

___VCU_WS="$(echo ~/vcu_ws/)"
___COLCON_BUILD_BASE_COMMAND="colcon build --parallel-workers 12 --symlink-install"

psel() {
    cd $___VCU_WS
    $___COLCON_BUILD_BASE_COMMAND --mixin release mold --packages-select $*
    cd -
}

pseld() {
    cd $___VCU_WS
    $___COLCON_BUILD_BASE_COMMAND --mixin debug mold --packages-select $*
    cd -
}

colbuild() {
    cd $___VCU_WS
    $___COLCON_BUILD_BASE_COMMAND --mixin release mold $*
    cd -
}

colbuild-debug() {
    cd $___VCU_WS
    $___COLCON_BUILD_BASE_COMMAND --mixin debug mold $*
    cd -
}

colbuild-reldeb() {
    cd $___VCU_WS
    $___COLCON_BUILD_BASE_COMMAND --mixin rel-with-deb-info mold $*
    cd -
}

col-clean() {
    if [ $# -eq 0 ]; then
        echo "No arguments provided. Deleting all folders"
        rm -rf ~/vcu_ws/{build,log,install}
    else
        for arg in "$@"; do
            if [[ -z $arg ]]; then
                echo "Skipping: Argument '$arg' is empty."
            else
                rm -rf ~/vcu_ws/{build,log,install}/$arg
            fi
        done
    fi
}
