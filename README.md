# Observation and patrolling problem framework for UAV's

![](https://github.com/JamyChahal/uav_obs_pat/Gazebo.jpg)

## Paper citation

The code is open source. Please, if the code has been useful for your research paper, or in your project, please cite the following paper:

[TODO: Add the bibtex of thesis if accepted]



## Installation

The project is based on Ubuntu 20.04 LTS, with [ROS Galactic ](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) and Gazebo 11.11

After installing ROS Galactic, we recommand to install Gazebo as following:

`sudo apt -y install ros-galactic-gazebo-ros-pkgs`

The project has several dependencies: 

`sudo apt update -y && sudo apt upgrade -y`

`sudo apt -y install libasio-dev`

`sudo apt -y install ros-galactic-cv-bridge ros-galactic-camera-calibration-parsers`

`sudo apt -y install python3-colcon-common-extensions`



Then create your ROS2 workspace, here called ~/uav_obs_pat_ws :

`mkdir -p ~/uav_obs_pat_ws/src`

`cd ~/uav_obs_pat_ws/src`

`git clone https://github.com/JamyChahal/uav_obs_pat`

`git clone https://github.com/ptrmu/ros2_shared.git` 

`cd ..`

`colcon build --symlink-install`

## Run experimentation on Gazebo

Before running an experience, source the project such as:

`source ~/uav_obs_pat_ws/install/setup.bash`

### Run a single mission

The default command to run a mission is: 

`ros2 launch tello_gazebo mission.launch.py`

The arguments taken are: 

* **nbr_target** (INT value, default 1): Number of ground robot as targets
* **nbr_drone** (INT value, default 1): Number of agent acting like UAV
* **obs_range** (FLOAT value, default 3.): Observation range of an agent, in m 
* **com_range** (FLOAT value, default 5.): Communication range of an agent, in m 
* **max_agent_speed** (FLOAT value, default 1.0): Maximum agent speed, in m/s 
* **max_target_speed** (FLOAT value, default 0.5): Maximum target speed, in m/s
* **mission_duration** (FLOAT value, default 30): Duration of a mission, in seconds, before stopping it (the launch is killed then)
* **strategy** (STRING value, default 'run_random'): Agent strategy to be executed, among the list:
  * **run_random**: random agent behavior
  * **run_a_cmommt**: A-CMOMMT behavior
  * **run_i_cmommt**: I-CMOMMT behavior
  * **run_ffrl** : FFRL behavior
  * **run_f2marl** : F2MARL behavior
  * **run_malos**: MALOS behavior
* **gui** (BOOL value, default true): If Gazebo has to be shown in a client / GUI window

Adding an arg to the mission is done as follows, with here 3 drones using the I-CMOMMT behavior:

`ros2 launch tello_gazebo mission.launch.py nbr_drone:=3 strategy:=run_i_cmommt`

The size of the map can be changed in a static way at line 45 of the tello_gazebo/launch/mission.launch.py file. 
The other parameter, not described here, are listed with the command : 

`ros2 launch tello_gazebo mission.launch.py --show-args`

### Run several missions

To run several missions in a sequential manner, you can use script available in tello_gazebo/run_launch_loop/run_launch.py 

You can change the number of agents (as a list), targets (as a list), strategy (as a list), mission_duration, obs_range (as a list) and com_range, as well as the number of experience for each configuration. By default, the value set are the same as in the first experience of the paper. 

`cd ~/uav_obs_pat_ws/src/uav_obs_pat/tello_gazebo/run_launch_loop`

`python3 run_launch.py`

At the end of the execution, all the result will be stored in a result.txt file, structured as a csv file with ; separation. 

## Run experimentation on Rviz

Gazebo can be quite slow, particularly when running missions multiple  times and adjusting input parameters such as the number of agents. To  expedite mission execution, an approximation of the global behavior is conducted using Rviz.

Before running an experience, source the project such as:

`source ~/uav_obs_pat_ws/install/setup.bash`

### Run a single mission

The default command to run a mission is: 

`ros2 launch tello_rviz mission.launch.py`

The parameters are very close to the Gazebo mission. To have the list, perform : 

`ros2 launch tello_rviz mission.launch.py --show-args`

### Run several missions

To run several missions in a sequential manner, you can use script available in tello_rviz/script/exp_several_launch.py 

`cd ~/uav_obs_pat_ws/src/uav_obs_pat/tello_rviz/script`

`python3 exp_several_launch.py`

At the end of the execution, all the result will be stored in a result.txt file, structured as a csv file with ; separation. 

## Run experimentation on true UAVs

[TODO]

## Project Structure

Here is the structure of the repository: 

* **evaluator**: Node used to get the evaluation metric of a mission, and kill it when reaching the "mission_duration" value
* **ground_robot_description**: Ground target description for Gazebo representation
* **target_strategy**: Node used for the random target strategy 
* **tello_description**: UAV, called Tello, description for Gazebo representation
* **tello_driver**: Drivers for true Tello UAV's experimentation
* **tello_gazebo**: Tello Plugin for Gazebo, including launch and world
* **tello_msgs**: ROS2 message used by the Tello Plugin
* **tello_strategy**: Node used for the agent strategy, such as the A-CMOMMT, I-CMOMMT...



### Aknowledge

The project is based on the Tello Plugin developed [by clydemcqueen](https://github.com/clydemcqueen/tello_ros). An honest thank you for this open source code. 

