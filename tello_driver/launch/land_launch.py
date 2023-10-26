from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    max_nbr_drone = 3

    tello1 = Node(package='tello_driver', executable='tello_driver_main', output='screen', namespace="drone1",
                  parameters=[{'drone_ip': "192.168.2.20"}, {'rasp_port': 3020}, {'cmd_port': 3021},
                              {'state_port': 3022}, {'video_port': 3023}])  # "192.168.10.1" 192.168.123.231

    tello2 = Node(package='tello_driver', executable='tello_driver_main', output='screen', namespace="drone2",
                  parameters=[{'drone_ip': "192.168.2.19"}, {'rasp_port': 3010}, {'cmd_port': 3011},
                              {'state_port': 3012}, {'video_port': 3013}],
                  arguments=['--ros-args', '--log-level', 'debug'])  # "192.168.10.1" 192.168.123.231

    # arguments=['--ros-args', '--log-level', 'debug'])
    # tello2 = Node(package='tello_driver', executable='tello_driver_main', output='screen', namespace="drone2",
    #               parameters=[{'drone_ip': "192.168.10.1"}])  # "192.168.10.1" 192.168.123.158

    drone_strategy = []
    for i in range(0, max_nbr_drone):
        drone_strategy.append(TimerAction(
            period=2.0,
            actions=[Node(
                package='tello_strategy',
                executable="run_land",  # run_random
                name="drone_strategy_" + str(i),
                output='screen',  # TO DEBUG
                parameters=[
                    {'namespace': 'drone' + str(i)}
                ],
                condition=IfCondition(
                    PythonExpression([LaunchConfiguration('nbr_drone'), " > %s" % i])
                )
            )]))

    return LaunchDescription([
        # Set the args
        DeclareLaunchArgument('nbr_drone', default_value="1", description='number of drone'),

        *drone_strategy,
        tello1,
        tello2
    ])
