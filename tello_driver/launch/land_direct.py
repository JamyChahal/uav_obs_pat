from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    max_nbr_drone = 3


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
    ])
