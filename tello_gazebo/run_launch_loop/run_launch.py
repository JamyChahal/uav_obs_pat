# https://answers.ros.org/question/361087/launching-and-killing-nodes-within-a-python-node-in-ros-2-foxy/
import os
import signal
import subprocess


def main(args=None):
    launch_process = None

    nbr_exp = 10

    nbr_agent = [2, 4, 6, 8, 10]
    nbr_target = [5]
    strategy = ['run_random', 'run_i_cmommt', 'run_a_cmommt', 'run_malos']
    mission_duration = 60 * 2  # 2 min to test
    obs_range = [5]
    com_range = 8

    cmd_list = []
    for a in nbr_agent:
        for t in nbr_target:
            for s in strategy:
                for o in obs_range:
                    cmd = ['ros2', 'launch', 'tello_gazebo']
                    if s == 'run_malos':
                        cmd.append('mission.launch.py')
                    else:
                        cmd.append('mission_fast.launch.py')
                    cmd += ["gui:=false", 'mission_duration:=' +
                       str(mission_duration), 'nbr_drone:=' + str(a), 'nbr_target:=' + str(t), 'strategy:=' + str(s),
                       'obs_range:=' + str(o), 'com_range:=' + str(com_range)]
                    cmd_list.append(cmd)

    print(cmd_list)
    print("ESTIMATED TIME (h) : ")
    print(len(cmd_list) * mission_duration / (60*60) * nbr_exp)

    total_nbr_mission = len(cmd_list) * nbr_exp
    mission_number = 0

    try:
        for cmd_line in cmd_list:
            for i in range(0, nbr_exp):
                mission_number += 1
                print(str(mission_number) + " / " + str(total_nbr_mission))
                launch_process = subprocess.Popen(
                    cmd_line, shell=False, stdout=subprocess.DEVNULL, preexec_fn=os.setsid)
                launch_process.wait()
                print("Cleaning after")
                killer = subprocess.Popen(['killall', 'gzserver', 'gzclient'])
                killer.wait()
                print("Next simulation")

    except KeyboardInterrupt:
        print("Finishing all the process")
        if launch_process is not None:
            print("Killing launch process")
            os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)  # SIGTERM
        killer = subprocess.Popen(['killall', 'gzserver', 'gzclient'])
        killer.wait()


if __name__ == '__main__':
    main()
