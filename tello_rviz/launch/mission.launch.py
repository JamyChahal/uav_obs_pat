"""Simulate an observation and patrolling problem mission"""

import os
import random

import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_random_pose(map_size_x, map_size_y):
    return random.randint(-map_size_x, map_size_x), random.randint(-map_size_y, map_size_y)


def generate_pose(max_nbr_target, max_nbr_drone, map_size_x, map_size_y):
    # TODO : Here, it's the max of target and drone. Maybe generate random pose with 0.5 steps
    targets_pose = []
    drones_pose = []

    random.seed()

    for nt in range(0, max_nbr_target):
        x, y = generate_random_pose(map_size_x, map_size_y)
        while (x, y) in targets_pose:
            x, y = generate_random_pose(map_size_x, map_size_y)
        targets_pose.append((str(x), str(y)))

    for nd in range(0, max_nbr_drone):
        x, y = generate_random_pose(map_size_x, map_size_y)
        while (x, y) in targets_pose or (x, y) in drones_pose:
            x, y = generate_random_pose(map_size_x, map_size_y)
        drones_pose.append((str(x), str(y)))

    print("Generation end..")

    return targets_pose, drones_pose


def generate_launch_description():
    max_nbr_drone = 10
    max_nbr_target = 10

    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')

    nbr_drone = LaunchConfiguration('nbr_drone')
    nbr_target = LaunchConfiguration('nbr_target')
    max_agent_speed = LaunchConfiguration('max_agent_speed')
    max_target_speed = LaunchConfiguration('max_target_speed')
    obs_range = LaunchConfiguration('obs_range')
    com_range = LaunchConfiguration('com_range')
    safety_range = LaunchConfiguration('safety_range')
    dangerous_range = LaunchConfiguration('dangerous_range')
    mission_duration = LaunchConfiguration('mission_duration')
    strategy = LaunchConfiguration('strategy')
    t_strategy = LaunchConfiguration('target_strategy')
    timestep = LaunchConfiguration('timestep')
    time_wait = LaunchConfiguration("time_wait")
    gui = LaunchConfiguration('gui')
    fast = LaunchConfiguration('fast')
    sigma = LaunchConfiguration('sigma')
    det_range = LaunchConfiguration('det_range')
    log_dir = LaunchConfiguration('log_dir')
    map_discrete = LaunchConfiguration('map_discrete')
    ffrl_model = LaunchConfiguration('ffrl_model')

    pkg_share = launch_ros.substitutions.FindPackageShare(package='tello_rviz').find('tello_rviz')
    model_agent = os.path.join(pkg_share, 'urdf/agent.urdf')  # ROS2 doesn't like xacro :(
    model_target = os.path.join(pkg_share, 'urdf/target.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/config3.rviz')
    global_parameter = os.path.join(pkg_share, 'config', 'global_params.yaml')

    # GENERATE THE TARGETS AND DRONE POSE
    # targets_pose, drones_pose = generate_pose(max_nbr_drone, max_nbr_target, map_size_x, map_size_y)

    # TARGET DESCRIPTION, PUBLISHER AND SPAWN
    target_publisher = []
    target_strategy = []

    for i in range(0, max_nbr_target):
        target_publisher.append(Node(
            package='robot_state_publisher',
            namespace='target' + str(i),
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model_target')])}],
            condition=IfCondition(
                PythonExpression([nbr_target, " > %s" % i])
            )
        ))

        target_strategy.append(TimerAction(
            period=0.0,
            actions=[Node(
                package='target_strategy',
                executable=t_strategy,
                name="target_strategy_" + str(i),
                output='screen',  # TO DEBUG
                parameters=[
                    {'namespace': 'target' + str(i),
                     'map_size_x': map_size_x,
                     'map_size_y': map_size_y,
                     'max_speed': max_target_speed,
                     'det_range': det_range,
                     'use_sim_time': True,
                     'fast_rviz': True}
                ],
                condition=IfCondition(
                    PythonExpression([nbr_target, " > %s" % i])
                )
            )]))

    # DRONE DESCRIPTION, PUBLISHER AND SPAWN
    drone_publisher = []
    drone_strategy = []
    for i in range(0, max_nbr_drone):
        drone_publisher.append(Node(
            package='robot_state_publisher',
            namespace='drone' + str(i),
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model_agent')])}],
            condition=IfCondition(
                PythonExpression([nbr_drone, " > %s" % i])
            )
        ))

        drone_strategy.append(TimerAction(
            period=0.0,
            actions=[Node(
                package='tello_strategy',
                executable=strategy,  # run_random
                name="drone_strategy_" + str(i),
                output='screen',  # TO DEBUG
                parameters=[
                    {'namespace': 'drone' + str(i),
                     'com_range': com_range,
                     'obs_range': obs_range,
                     'map_size_x': map_size_x,
                     'map_size_y': map_size_y,
                     'max_speed': max_agent_speed,
                     'map_discrete': map_discrete,
                     'safety_range': safety_range,
                     'dangerous_range': dangerous_range,
                     'sigma': sigma,
                     'ffrl_model' : ffrl_model,
                     'use_sim_time': True,
                     'fast_rviz': True}
                ],
                condition=IfCondition(
                    PythonExpression([nbr_drone, " > %s" % i])
                )
            )]))

    evaluator = Node(
        package='evaluator',
        executable='run_evaluator',
        name="evaluator",
        output='screen',  # TO DEBUG
        parameters=[
            {'com_range': com_range,
             'obs_range': obs_range,
             'map_size_x': map_size_x,
             'map_size_y': map_size_y,
             'nbr_drone': nbr_drone,
             'nbr_target': nbr_target,
             'mission_duration': mission_duration,
             'map_discrete': map_discrete,
             'agent_speed': max_agent_speed,
             'target_speed': max_target_speed,
             'strategy': strategy,
             'sigma': sigma,
             'log_dir': log_dir,
             'ffrl_model': ffrl_model,
             'fast_rviz': True,
             'use_sim_time': True}
        ]
    )

    timestep_node = TimerAction(
        period=0.0,
        actions=[launch_ros.actions.Node(
            package="tello_rviz",
            executable="timestep",
            name="timestep",
            output="screen",
            emulate_tty="True",
            #prefix=["xterm -hold -e"],
            parameters=[{global_parameter},
                        {"nbr_drone": nbr_drone,
                         "nbr_target": nbr_target,
                         "timestep": timestep,
                         "fast": True,
                         "time_wait" : time_wait,
                         'use_sim_time': True}]
        )])

    flash_node = TimerAction(
        period=0.0,
        actions=[launch_ros.actions.Node(
            package="tello_rviz",
            executable="flash",
            name="flash",
            output="screen",
            emulate_tty="True",
            #prefix=["xterm -hold -e"],
            parameters=[{global_parameter},
                        {"nbr_drone": nbr_drone,
                         "nbr_target": nbr_target,
                         "timestep": timestep,
                         'map_size_x': map_size_x,
                         'map_size_y': map_size_y,
                         'use_sim_time': True}
                        ]
        )])

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both': 'log'},
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui')),
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Listen to stop the launch :
    # Check here https://github.com/ros2/launch/blob/a89671962220c8691ea4f128717bca599c711cda/launch/examples/launch_counters.py#L96-L98
    def listen_to_stop_handler(event):
        target_str = 'STOP'
        if target_str in event.text.decode():
            return launch.actions.EmitEvent(event=launch.events.Shutdown(reason="END OF SIMULATION"))

    stop_action = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessIO(
            target_action=evaluator,
            on_stdout=listen_to_stop_handler,
            on_stderr=listen_to_stop_handler,
        )
    )

    return LaunchDescription([
        # Set the args
        launch.actions.DeclareLaunchArgument(name='model_agent', default_value=model_agent,
                                             description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='model_target', default_value=model_target,
                                             description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),

        DeclareLaunchArgument('nbr_target', default_value="1", description='number of target'),
        DeclareLaunchArgument('nbr_drone', default_value="1", description='number of drone'),
        DeclareLaunchArgument('obs_range', default_value="5.", description='observation range (m)'),
        DeclareLaunchArgument('com_range', default_value="10.", description='communication range (m)'),
        DeclareLaunchArgument('map_size_x', default_value="50.", description='map size in x (m)'),
        DeclareLaunchArgument('map_size_y', default_value="50.", description='map size in y (m)'),
        DeclareLaunchArgument('safety_range', default_value="1.", description='ideal distance range btw drones, upper '
                                                                              'than dangerous range (m)'),
        DeclareLaunchArgument('dangerous_range', default_value="0.5", description='min distance range btw drones (m)'),

        DeclareLaunchArgument('det_range', default_value="5.", description='Detection range by evasive targets'),
        DeclareLaunchArgument('max_agent_speed', default_value="1.", description='maximum agent speed (m/s)'),
        DeclareLaunchArgument('max_target_speed', default_value="0.5", description='maximum target speed (m/s)'),
        DeclareLaunchArgument('mission_duration', default_value="30",
                              description='maximum mission duration before ending (s)'),
        DeclareLaunchArgument('strategy', default_value="run_random", description='strategy to run: run_random, '
                                                                                  'run_a_cmommt, run_i_cmommt, run_malos'),
        DeclareLaunchArgument('target_strategy', default_value="run_random",
                              description='strategy for target: run_random or run_evasive'),
        DeclareLaunchArgument('gui', default_value="True", description='display gazebo ?'),
        DeclareLaunchArgument('fast', default_value="False", description='fast simulation ?'),
        DeclareLaunchArgument('timestep', default_value="1.", description='Set timestep period in seconds'),
        DeclareLaunchArgument('time_wait', default_value="0.", description='Set wait time between two timestep'),
        DeclareLaunchArgument('sigma', default_value="900.", description='Sigma parameter for I-CMOMMT'),
        DeclareLaunchArgument('ffrl_model', default_value="col", description='Sigma parameter for I-CMOMMT'),
        DeclareLaunchArgument('log_dir', default_value="result", description='Log directory name'),
        DeclareLaunchArgument('map_discrete', default_value="1", description='Map discretization ratio'),

        # Launch Gazebo, loading tello.world
        # WITH GUI
        rviz_node,
        flash_node,

        *drone_publisher,
        *target_publisher,
        *drone_strategy,
        *target_strategy,
        evaluator,
        stop_action,
        timestep_node,
    ])
