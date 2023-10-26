from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


# USED TO CONNECT TO UAVs


def generate_launch_description():
    
    hostname = '192.168.2.6'
    buffer_size = 200
    topic_namespace = 'vicon'

    vicon = Node(package='vicon_receiver', executable='vicon_client', output='screen',
            parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}])
    
    redirect = Node(package='remapping_exp', executable='remapping', output='screen')

    clock = Node(package='evaluator', executable='run_clock', output='screen')
                    
                    # arguments=['--ros-args', '--log-level', 'debug'])
    # tello2 = Node(package='tello_driver', executable='tello_driver_main', output='screen', namespace="drone2",
    #               parameters=[{'drone_ip': "192.168.10.1"}])  # "192.168.10.1" 192.168.123.158

    return LaunchDescription([
        vicon,
        redirect,
        clock
    ])
