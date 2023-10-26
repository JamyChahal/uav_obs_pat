# https://answers.ros.org/question/361087/launching-and-killing-nodes-within-a-python-node-in-ros-2-foxy/
import os
import random
import signal
import subprocess
import sys
import time

import rclpy

from builtin_interfaces.msg import Duration
import rclpy
from rclpy import Parameter
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def main(args=None):
    rclpy.init(args=args)
    launch_process = None

    ### ---- PARAMETERS FOR THE SIMULATIONS ---- ###
    debug = False  # Debug set to True to display subprocess
    log_dir = "result"

    nbr_exp = 30  # Each configuration is run x times
    duration = 60 * 60  # 1hour of exp.

    nbr_agent = [2, 4, 6]
    nbr_target = [3, 5, 7]
    obs_range = [5.]
    com_range = [10.]
    map_size = [50.]  # so 100m x 100m
    max_agent_speed = [1.]
    max_target_speed = [0.25]
    sigma_factor = [0.5]
    sigma = [x * duration for x in sigma_factor]
    # Specify the method to evaluate :
    strategy = ['run_random', 'run_i_cmommt', 'run_a_cmommt', 'run_ci', 'run_hi']
    ### ---- END PARAMETERS FOR THE SIMULATIONS ---- ###

    cmd_list = []

    for a in nbr_agent:
        for t in nbr_target:
            if t > a:
                for o in obs_range:
                    for c in com_range:
                        for m in map_size:
                            for ma in max_agent_speed:
                                for mt in max_target_speed:
                                    for s in strategy:
                                        cmd = ['ros2', 'launch', "tello_rviz", "mission.launch.py", 'gui:=False']
                                        cmd += ['mission_duration:=' + str(duration)]
                                        cmd += ['nbr_drone:=' + str(a)]
                                        cmd += ['nbr_target:=' + str(t)]
                                        cmd += ['obs_range:=' + str(o)]
                                        cmd += ['com_range:=' + str(c)]
                                        cmd += ['map_size_x:=' + str(m)]
                                        cmd += ['map_size_y:=' + str(m)]
                                        cmd += ['max_agent_speed:=' + str(ma)]
                                        cmd += ['max_target_speed:=' + str(mt)]
                                        cmd += ['strategy:=' + str(s)]
                                        cmd += ['log_dir:=' + str(log_dir)]
                                        cmd += ['sigma:=' + str(sigma[0])]

                                        for i in range(0, nbr_exp):
                                            cmd_list.append(cmd)

    try:

        total_exp = len(cmd_list)
        i = 0

        for cmd in cmd_list:
            i += 1
            print("EXP " + str(i) + " / " + str(total_exp), flush=True)
            if not debug:
                launch_process = subprocess.Popen(cmd, shell=False, stdout=subprocess.DEVNULL,
                                                  stderr=subprocess.DEVNULL,
                                                  preexec_fn=os.setsid)
            else:
                launch_process = subprocess.Popen(cmd, shell=False, preexec_fn=os.setsid)
                launch_process.communicate()
            launch_process.wait()
    except KeyboardInterrupt:
        print("Finishing all the process")
        if launch_process is not None:
            print("Killing launch process")
            os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)  # SIGTERM

    rclpy.shutdown()


if __name__ == '__main__':
    main()
