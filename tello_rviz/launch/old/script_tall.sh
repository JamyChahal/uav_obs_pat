#!/bin/bash

source ~/.bashrc
source ~/miniconda3/etc/profile.d/conda.sh

eval "$(conda shell.bash hook)"

conda activate ros_tall
source ~/tello_obs_pat_ws_tall/install/setup.bash

cd ~/tello_obs_pat_ws_tall/src/tello_obs_pat/tello_rviz/launch

python3 exp_launch.py