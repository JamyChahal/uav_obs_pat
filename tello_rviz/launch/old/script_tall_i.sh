#!/bin/bash

source ~/.bashrc
source ~/miniconda3/etc/profile.d/conda.sh

eval "$(conda shell.bash hook)"

conda activate ros_tall
source ~/uav_obs_pat_ws/install/setup.bash

cd ~/uav_obs_pat_ws/src/tello_obs_pat/tello_rviz/launch

python3 exp_launch_sigma.py