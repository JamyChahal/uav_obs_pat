# https://answers.ros.org/question/361087/launching-and-killing-nodes-within-a-python-node-in-ros-2-foxy/
import os
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


class timestep(Node):

    def __init__(self):
        super().__init__('time_manager')
        self.set_parameters([Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        self.duration = 0
        self.sub_ = self.create_subscription(Clock, "/clock", self.callback, 0)


    def callback(self, data):
        self.duration = data.clock.sec

    def get_time(self):
        return self.duration


def main(args=None):
    rclpy.init(args=args)
    launch_process = None

    nbr_exp = 10
    duration = 3600

    nbr_agent = [2, 3, 5]
    nbr_target = [3, 5, 7]
    obs_range = [5]
    com_range = [6]
    map_size = [40]
    max_agent_speed = [1.]
    max_target_speed = [0.5]
    strategy = ['run_random', 'run_i_cmommt', 'run_ci', 'run_hi', 'run_a_cmommt']

    cmd_list = []

    for a in nbr_agent:
        for t in nbr_target:
            for o in obs_range:
                for c in com_range:
                    for m in map_size:
                        for ma in max_agent_speed:
                            for mt in max_target_speed:
                                for s in strategy:
                                    cmd = ['ros2', 'launch', "tello_rviz", "mission.launch.py", 'gui:=False']
                                    cmd += ['mission_duration:=' + str(duration)]
                                    cmd += ['nbr_drone:='+str(a)]
                                    cmd += ['nbr_target:='+str(t)]
                                    cmd += ['obs_range:='+str(o)]
                                    cmd += ['com_range:='+str(c)]
                                    cmd += ['map_size_x:='+str(m)]
                                    cmd += ['map_size_y:='+str(m)]
                                    cmd += ['max_agent_speed:='+str(ma)]
                                    cmd += ['max_target_speed:='+str(mt)]

                                    for i in range(0, nbr_exp):
                                        cmd_list.append(cmd)

    try:
        ts = timestep()

        total_exp = len(cmd_list)
        i = 0

        for cmd in cmd_list:
            i += 1
            print("EXP " + str(i) + " / " + str(total_exp))
            launch_process = subprocess.Popen(cmd, shell=False, stdout=subprocess.DEVNULL, preexec_fn=os.setsid)
            while rclpy.ok():
                rclpy.spin_once(ts)
                print('\r' + str(ts.get_time()) + " (s) / " + str(duration), end='\r')


    except KeyboardInterrupt:
        print("Finishing all the process")
        if launch_process is not None:
            print("Killing launch process")
            os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)  # SIGTERM

    rclpy.shutdown()


if __name__ == '__main__':
    main()
