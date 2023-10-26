import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    # TODO : Read the yaml, and set the default value from it. Can be override by setting params
    nbr_tracker_max = 10
    nbr_target_max = 10

    pkg_share = launch_ros.substitutions.FindPackageShare(package='mas_tracking_agents').find('mas_tracking_agents')
    global_parameter = os.path.join(pkg_share, 'config', 'global_params.yaml')

    print(str(LaunchConfiguration('log_name')))

    global_parameter = str(LaunchConfiguration('log_name')) +"/params.yaml" # TODO : Not working !

    evaluator_node = launch_ros.actions.Node(
        package="mas_tracking_agents",
        executable="evaluator",
        name="evaluator",
        output="screen",
        emulate_tty="True",
        prefix=["xterm -hold -e"],
        parameters=[{global_parameter}]
    )

    logger_node = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('log_name')],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name="log_name", default_value="rosbag_without_name",
                                             description="Name of the log folder"),
        evaluator_node,
        logger_node
    ])
