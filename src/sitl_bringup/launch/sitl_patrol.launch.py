import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Nav2 自带的仿真 launch。车体模型和参数换成仓库里的两份。
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    sitl_bringup_dir = get_package_share_directory('sitl_bringup')

    nav2_params_yaml = os.path.join(sitl_bringup_dir, 'config', 'nav2_params.yaml')
    waffle_model_sdf = os.path.join(sitl_bringup_dir, 'models', 'waffle.model')

    tb3_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            # 要看 Gazebo 窗口。无头跑测试不走这个 launch。
            'headless': 'False',
            'use_sim_time': 'True',
            'params_file': nav2_params_yaml,
            'robot_sdf': waffle_model_sdf,
        }.items()
    )

    # 默认不延迟。代价地图订 /scan_delayed，AMCL 仍订 /scan。
    scan_delay_node = Node(
        package='sitl_mission_node',
        executable='scan_delay_node',
        name='scan_delay_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'delay_ms': 0,
        }]
    )

    # Gazebo 自己的 odom TF 已关掉，由这个节点重发。stall 默认 false。
    odom_tf_gate_node = Node(
        package='sitl_mission_node',
        executable='odom_tf_gate',
        name='odom_tf_gate',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'stall': False,
        }]
    )

    # ParameterValue 把 launch 参数收成节点参数的真实类型，避免都变成字符串。
    mission_node = Node(
        package='sitl_mission_node',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
            'patrol_loops': ParameterValue(LaunchConfiguration('patrol_loops'), value_type=int),
            'mqtt_enable': ParameterValue(LaunchConfiguration('mqtt_enable'), value_type=bool),
            'mqtt_host': LaunchConfiguration('mqtt_host'),
            'mqtt_port': ParameterValue(LaunchConfiguration('mqtt_port'), value_type=int),
            'mqtt_topic': LaunchConfiguration('mqtt_topic'),
            'mqtt_status_topic': LaunchConfiguration('mqtt_status_topic'),
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('patrol_loops', default_value='1'),
        DeclareLaunchArgument('mqtt_enable', default_value='false'),
        DeclareLaunchArgument('mqtt_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('mqtt_port', default_value='1883'),
        DeclareLaunchArgument('mqtt_topic', default_value='sitl/patrol/task'),
        DeclareLaunchArgument('mqtt_status_topic', default_value='sitl/patrol/status'),
        tb3_sim_launch,
        scan_delay_node,
        odom_tf_gate_node,
        mission_node,
    ])