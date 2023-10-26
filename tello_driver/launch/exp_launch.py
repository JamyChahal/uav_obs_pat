from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


# USED TO RUN STRATEGIES


def generate_launch_description():
    max_nbr_drone = 3
    max_nbr_target = 3
    map_size_x = 1. # 1.5
    map_size_y = 1. # 1.5
    nbr_drone = LaunchConfiguration('nbr_drone')
    nbr_target = LaunchConfiguration('nbr_target')
    max_agent_speed = LaunchConfiguration('max_agent_speed')
    obs_range = LaunchConfiguration('obs_range')
    com_range = LaunchConfiguration('com_range')
    strategy = LaunchConfiguration('strategy')
    safety_range = LaunchConfiguration('safety_range')
    dangerous_range = LaunchConfiguration('dangerous_range')
    log_dir = LaunchConfiguration('log_dir')
    sigma = LaunchConfiguration('sigma')
    map_discrete = LaunchConfiguration('map_discrete')
    record = LaunchConfiguration('record')

    # TOPIC LIST TO RECORD FOR ROSBAG
    topic_to_record = []
    for i in range(0, max_nbr_drone):
        topic_to_record.append('drone'+str(i)+'/pose')
        topic_to_record.append('drone'+str(i)+'/flight_data')
        topic_to_record.append('drone'+str(i)+'/cmd_vel')
    for i in range(0, max_nbr_target):
        topic_to_record.append('target'+str(i)+'/pose')

    drone_strategy = []
    for i in range(0, max_nbr_drone):
        drone_strategy.append(TimerAction(
            period=2.0,
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
                     'is_exp': True,
                     'sigma': sigma
                     }
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
             'mission_duration': -1,
             'map_discrete': map_discrete,
             'agent_speed': max_agent_speed,
             'target_speed': 0.5,
             'strategy': strategy,
             'sigma': sigma,
             'log_dir': log_dir}
        ]
    )

    return LaunchDescription([
        # Set the args
        DeclareLaunchArgument('nbr_drone', default_value="1", description='number of drones'),
        DeclareLaunchArgument('nbr_target', default_value="1", description='number of targets'),
        DeclareLaunchArgument('obs_range', default_value="0.5", description='observation range (m)'),
        DeclareLaunchArgument('com_range', default_value="1.5", description='communication range (m)'),
        DeclareLaunchArgument('safety_range', default_value="1.", description='ideal distance range btw drones, upper '
                                                                              'than dangerous range (m)'),
        DeclareLaunchArgument('dangerous_range', default_value="0.9", description='min distance range btw drones (m)'),
        DeclareLaunchArgument('max_agent_speed', default_value="0.2", description='maximum agent speed (m/s)'),
        DeclareLaunchArgument('strategy', default_value="run_void", description='strategy to run: run_random, '),
        DeclareLaunchArgument('sigma', default_value="30.", description='Sigma parameter for I-CMOMMT'),
        DeclareLaunchArgument('log_dir', default_value="result_evry_juillet", description='Log directory name'),
        DeclareLaunchArgument('map_discrete', default_value="10", description='Map discretization ratio'),
        DeclareLaunchArgument('record', default_value="True", description="Record mission in a ros bag ?"),

        # Record the mission
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'], #+ topic_to_record,
            output='screen',
            condition=IfCondition(record)
        ),

        *drone_strategy,
        evaluator
    ])
