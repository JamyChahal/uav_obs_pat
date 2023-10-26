from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


# USED TO CONNECT TO UAVs


def generate_launch_description():
    tello0 = Node(package='tello_driver', executable='tello_driver_main', output='screen', namespace="drone1",
                  parameters=[{'drone_ip': "192.168.2.9"}, {'rasp_port': 3010}, {'cmd_port': 3011}, {'state_port': 3012}, {'video_port': 3013}])  # "192.168.10.1" 192.168.123.231
    
    tello1 = Node(package='tello_driver', executable='tello_driver_main', output='screen', namespace="drone0",
                  parameters=[{'drone_ip': "192.168.2.10"}, {'rasp_port': 3020}, {'cmd_port': 3021}, {'state_port': 3022}, {'video_port': 3023}],
                  arguments=['--ros-args', '--log-level', 'debug'])  # "192.168.10.1" 192.168.123.231
                    
                    # arguments=['--ros-args', '--log-level', 'debug'])
    # tello2 = Node(package='tello_driver', executable='tello_driver_main', output='screen', namespace="drone2",
    #               parameters=[{'drone_ip': "192.168.10.1"}])  # "192.168.10.1" 192.168.123.158

    return LaunchDescription([
        tello0,
        tello1,
    ])
