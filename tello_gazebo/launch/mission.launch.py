"""Simulate an observation and patrolling problem mission"""

import os
import random

import launch
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition


def generate_random_pose(map_size_x, map_size_y):
    return random.uniform(-map_size_x, map_size_x), random.uniform(-map_size_y, map_size_y)


def generate_pose(max_nbr_target, max_nbr_drone, map_size_x, map_size_y):
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
    map_size_x = 10.
    map_size_y = 10.
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
    gui = LaunchConfiguration('gui')
    fast = LaunchConfiguration('fast')
    sigma = LaunchConfiguration('sigma')
    det_range = LaunchConfiguration('det_range')
    log_dir = LaunchConfiguration('log_dir')
    map_discrete = LaunchConfiguration('map_discrete')
    record = LaunchConfiguration('record')

    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'uav.world')
    world_path_fast = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'uav_faster.world')
    drone_path = os.path.join(get_package_share_directory('tello_description'), 'xacro', 'tello.xacro')
    target_path = os.path.join(get_package_share_directory('ground_robot_description'), 'xacro', 'robot.xacro')

    # GENERATE THE TARGETS AND DRONE POSE
    targets_pose, drones_pose = generate_pose(max_nbr_drone, max_nbr_target, map_size_x, map_size_y)

    # TOPIC LIST TO RECORD FOR ROSBAG
    topic_to_record = []
    for i in range(0, max_nbr_drone):
        topic_to_record.append('drone'+str(i)+'/pose')
    for i in range(0, max_nbr_target):
        topic_to_record.append('target'+str(i)+'/pose')

    # TARGET DESCRIPTION, PUBLISHER AND SPAWN
    target_description = []
    target_publisher = []
    target_spawn = []
    target_strategy = []

    for i in range(0, max_nbr_target):
        target_description.append(xacro.process_file(target_path, mappings={
            "namespace": 'target' + str(i)
        }).toxml())

        target_publisher.append(Node(
            package='robot_state_publisher',
            namespace='target' + str(i),
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': target_description[i],
            }],
            condition=IfCondition(
                PythonExpression([nbr_target, " > %s" % i])
            )
        ))

        target_spawn.append(TimerAction(
            period=1.0,
            actions=[Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity", "target" + str(i),
                    "-topic", "/target" + str(i) + "/robot_description",
                    "-x", targets_pose[i][0],
                    "-y", targets_pose[i][1]
                ],
                condition=IfCondition(
                    PythonExpression([nbr_target, " > %s" % i])
                )
            )]))

        target_strategy.append(TimerAction(
            period=2.0,
            actions=[Node(
                package='target_strategy',
                executable=t_strategy,
                name="target"+str(i)+"_strategy",
                output='screen',  # TO DEBUG
                parameters=[
                    {'namespace': 'target' + str(i),
                     'map_size_x': map_size_x,
                     'map_size_y': map_size_y,
                     'det_range': det_range, # Used only by evasive target
                     'max_speed': max_target_speed}
                ],
                condition=IfCondition(
                    PythonExpression([nbr_target, " > %s" % i])
                )
            )]))

    # DRONE DESCRIPTION, PUBLISHER AND SPAWN
    drone_description = []
    drone_publisher = []
    drone_spawn = []
    drone_strategy = []
    for i in range(0, max_nbr_drone):
        drone_description.append(xacro.process_file(drone_path, mappings={
            "namespace": 'drone' + str(i)
        }).toxml())

        drone_publisher.append(Node(
            package='robot_state_publisher',
            namespace='drone' + str(i),
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': drone_description[i],
            }],
            condition=IfCondition(
                PythonExpression([nbr_drone, " > %s" % i])
            )
        ))

        drone_spawn.append(TimerAction(
            period=1.0,
            actions=[Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity", "drone" + str(i),
                    "-topic", "/drone" + str(i) + "/robot_description",
                    "-x", drones_pose[i][0],
                    "-y", drones_pose[i][1]
                ],
                condition=IfCondition(
                    PythonExpression([nbr_drone, " > %s" % i])
                )
            )]))

        drone_strategy.append(TimerAction(
            period=2.0,
            actions=[Node(
                package='tello_strategy',
                executable=strategy,  # run_random
                name="drone"+str(i)+"_strategy",
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
                     'sigma': sigma,}
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
             'log_dir': log_dir}
        ]
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
        DeclareLaunchArgument('nbr_target', default_value="1", description='number of target'),
        DeclareLaunchArgument('nbr_drone', default_value="1", description='number of drone'),
        DeclareLaunchArgument('obs_range', default_value="5.", description='observation range (m)'),
        DeclareLaunchArgument('com_range', default_value="8.", description='communication range (m)'),
        DeclareLaunchArgument('safety_range', default_value="1.", description='ideal distance range btw drones, upper '
                                                                              'than dangerous range (m)'),
        DeclareLaunchArgument('dangerous_range', default_value="0.5", description='min distance range btw drones (m)'),
        DeclareLaunchArgument('max_agent_speed', default_value="1.", description='maximum agent speed (m/s)'),
        DeclareLaunchArgument('max_target_speed', default_value="0.5", description='maximum target speed (m/s)'),
        DeclareLaunchArgument('mission_duration', default_value="30", description='maximum mission duration before ending (s)'),
        DeclareLaunchArgument('strategy', default_value="run_random", description='strategy to run: run_random, '
                                                                                  'run_a_cmommt, run_i_cmommt, run_malos'),
        DeclareLaunchArgument('target_strategy', default_value="run_random", description='strategy for target: run_random or run_evasive'),
        DeclareLaunchArgument('gui', default_value="True", description='display gazebo ?'),
        DeclareLaunchArgument('fast', default_value="False", description='fast simulation ?'),
        DeclareLaunchArgument('sigma', default_value="900.", description='Sigma parameter for I-CMOMMT'),
        DeclareLaunchArgument('det_range', default_value="5.", description='Detection range by evasive targets'),
        DeclareLaunchArgument('log_dir', default_value="result", description='Log directory name'),
        DeclareLaunchArgument('map_discrete', default_value="1", description='Map discretization ratio'),
        DeclareLaunchArgument('record', default_value="False", description="Record mission in a ros bag ?"),

        # Record the mission
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + topic_to_record,
            output='screen',
            condition=IfCondition(record)
        ),

        # Launch Gazebo, loading tello.world
        # WITH GUI
        ExecuteProcess(cmd=[
            'gazebo',  # 'gazebo'
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen',
            condition=IfCondition(PythonExpression([gui, ' and not ', fast]))
        ),

        # FAST
        ExecuteProcess(cmd=[
            'gazebo',  # 'gazebo'
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path_fast
        ], output='screen',
            condition=IfCondition(PythonExpression([gui, ' and ', fast]))
        ),

        # WITHOUT GUI
        ExecuteProcess(cmd=[
            'gzserver',  # 'gazebo'
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen',
            condition=IfCondition(PythonExpression(['not ', gui, ' and not ', fast]))
        ),

        # FAST
        ExecuteProcess(cmd=[
            'gzserver',  # 'gazebo'
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path_fast
        ], output='screen',
            condition=IfCondition(PythonExpression(['not ', gui, ' and ', fast]))
        ),

        *drone_publisher,
        *target_publisher,
        *drone_spawn,
        *target_spawn,
        *drone_strategy,
        *target_strategy,
        evaluator,
        stop_action
    ])
