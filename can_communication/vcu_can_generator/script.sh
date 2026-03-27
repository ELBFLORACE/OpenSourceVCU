#!/bin/bash

# Automatically exit on error in any command
# See: https://stackoverflow.com/a/2871034/18448953
set -euo pipefail

# Defining colors
RED='\033[0;31m'
NC='\033[0m'



rel_can_dir=$1
if [[ ! -n "$rel_can_dir" ]]; then
    echo "$rel_can_dir is not a directory (was empty)"
    exit 1
fi


# Ensure the path is absolute because the python script will mess up otherwise
can_dir="$rel_can_dir"
if [[ ! "$can_dir" = /* ]]; then
    can_dir="$(pwd)/$can_dir"
fi
if [[ -n $(git diff $can_dir) ]]; then
    echo -e "${RED}Warning: There are uncommitted changes in $can_dir, you might want to execute 'git submodule update' or commit the changes${NC}"
fi


# Check if valid CAN directory
if [ ! -d "$can_dir" ]; then
    echo -e "${RED}$can_dir is not a directory${NC}" && exit 1
fi
# Check for empty CAN directory
if [ -z "$( ls -A $can_dir )" ]; then
   echo  -e "${RED}$can_dir is empty, please use 'git submodule init' and 'git submodule update' to initialise your DBC submdoule${NC}"  && exit 1
fi

echo "Using DBC files from: $can_dir"




if [ ! -v SEASON ]; then # Check that season is set
    echo "${RED}Environment variable 'SEASON' needs to be set, was: '$SEASON'${NC}"
    exit 1
fi

if [ ! -d "can_communication/vcu_can_generator" ]; then
    echo "${RED}Unable to find vcu_can_generator directory, are you running this script from the root of the repository?${NC}"
    exit 1
fi

if [ ! -v CI ]; then # Disable checks during CI
    read -p "Please confirm that you are running the script from the root of the git repository (y/n)" choice
    case "$choice" in
    y|Y ) ;;
    n|N ) exit 1;;
    * ) echo "Invalid choice";exit 1;;
    esac
fi


cd can_communication/vcu_can_generator
echo "Generating for season: yourcar$SEASON"


# Ensure that the python depencies are installed
if [ -f "venv/bin/activate" ]; then
    # echo "Found venv at $(pwd)/venv"
    source venv/bin/activate
    pip install -q -r requirements.txt
else
    if [ ! -v CI ]; then # Disable checks during CI
        read -p "This script requires some python dependencies, would you like to install them globally (g) or set up a venv (v), if you already have all depencies installed you can skip (s) this step (g/v/s)" choice
        case "$choice" in
        g|G )
            pip install -q -r requirements.txt
            ;;
        v|V )
            echo "Creating venv at $(pwd)/venv"
            python3 -m venv venv
            source venv/bin/activate
            pip install -q -r requirements.txt
            ;;
        s|S )
            ;;
        * ) echo "Invalid choice";exit 1;;
        esac
    else
    # During CI Run just assume a venv is intended
        python3 -m venv venv
        source venv/bin/activate
        pip install -q -r requirements.txt
    fi
fi


# Run the actual generator
python3 src/main.py --no-strict $can_dir

# Temporary directory for generation
tmp_dist_dir=$(realpath dist)
cd ../..

# The directory of the real destination repository
dist_dir=$(realpath .)
# Move old folder in trash, otherwise deprecated messages wont be deleted
if [ -v DIST_DIR ]; then
    dist_dir=$(realpath $DIST_DIR)
fi

if [ ! -v CI ]; then # Disable checks during CI
read -p "Do you want to apply the newly generated files (y/n)" choice
case "$choice" in
  y|Y ) ;;
  n|N ) exit 0;;
  * ) echo "Invalid choice";exit 1;;
esac
fi

# Rename folders to match season based structure
mv "$tmp_dist_dir/vcu_can_msgs" "$tmp_dist_dir/vcu_can_msgs_yourcar$SEASON"
# mv "$tmp_dist_dir/mailman" "$tmp_dist_dir/mailman"

# Copy the newly generated files
echo -n "Replacing vcu_can_msgs_efr$SEASON,  "

# Move old msg folder in trash, otherwise deprecated messages wont be deleted
if [ ! -v CI ]; then
    gio trash can_communication/vcu_can_msgs_yourcar$SEASON/msg
else
    # CI doesn't support gio trash
    rm -rf ./can_communication/vcu_can_msgs_yourcar$SEASON/msg
fi

cp -r "$tmp_dist_dir/vcu_can_msgs_yourcar$SEASON" $dist_dir/can_communication



echo -n "replacing mailman,  "
# Check if new mailman structure exists (vcu)
if [ -d "$dist_dir/mailman" ]; then
    cp -r $tmp_dist_dir/mailman $dist_dir
else
    echo -e "${RED}$dist_dir/mailman does not exist${NC}"
fi
