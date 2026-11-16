from launch import LaunchDescription
from launch_ros.actions import Node

import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import yaml

from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # Path to vispa package share (move to top so it's available for all nodes)
    vispa_pkg_share = get_package_share_directory('vispa')


    def load_file(package_name, file_path):
        from ament_index_python.packages import get_package_share_directory
        abs_path = os.path.join(get_package_share_directory(package_name), file_path)
        with open(abs_path, 'r') as f:
            return f.read()

    # Launch argument for ros2_control_hardware_type
    ros2_control_hardware_type_arg = DeclareLaunchArgument(
        'ros2_control_hardware_type',
        default_value='isaac',
        description='ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]',
    )

    ros2_control_hardware_type = LaunchConfiguration('ros2_control_hardware_type')

    urdf_xml = load_file('vispa', 'urdf/VISPA_modifiedDH_Ros2.urdf')
    srdf_xml = load_file('vispa', 'config/VISPA.srdf')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Path to the RViz config file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    # Add ros2_control_node (controller manager) - pass robot_description as parameter
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': urdf_xml},
            os.path.join(vispa_pkg_share, 'config', 'ros2_controllers.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
            ('/joint_states', '/vispa/joint_states')
        ]
    )

    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawner for vispa_arm_controller (match Panda naming if needed)
    vispa_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['vispa_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # gripper controller spawner 
    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    def load_yaml(package_name, file_path):
        abs_path = os.path.join(get_package_share_directory(package_name), file_path)
        with open(abs_path, 'r') as f:
            return yaml.safe_load(f)

    controller_yaml = load_yaml('vispa', 'config/moveit_controllers.yaml')
    print("Loaded moveit_controllers.yaml:", controller_yaml)


    # Path to vispa display.launch.py
    vispa_display_launch = os.path.join(vispa_pkg_share, 'launch', 'display.launch.py')

    # Pass URDF and RViz config to display.launch.py
    vispa_display_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vispa_display_launch),
        launch_arguments={
            'use_rviz': 'True',
            'urdf_file': os.path.join(vispa_pkg_share, 'urdf', 'VISPA_modifiedDH_Ros2.urdf'),
            'rviz_config_file': os.path.join(vispa_pkg_share, 'rviz', 'vispa_ros2.rviz'),
            'use_robot_state_pub': 'True',
            'use_joint_state_pub': 'False',
            'robot_description': urdf_xml,
            'robot_description_semantic': srdf_xml,
            'joint_states_topic': '/vispa/joint_states'
        }.items()
    )

    # MoveItConfigsBuilder for VISPA robot
    moveit_config = (
        MoveItConfigsBuilder("vispa", package_name="vispa")
        .robot_description(
            file_path="config/VISPA.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": ros2_control_hardware_type
            },
        )
        .robot_description_semantic(file_path="config/VISPA.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # MoveIt move_group node with remappings for Isaac Sim topics
    # Compose the parameters list for debugging
    move_group_params = [
        moveit_config.to_dict(),
        controller_yaml,
        {'use_sim_time': use_sim_time}
    ]
    # print("\n===== move_group parameters =====\n", move_group_params, "\n===============================\n")
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
            ('/joint_states', '/vispa/joint_states')
        ]
    )

    topic_time_synchronizer_node = Node(
        package='topic_time_synchronizer',
        executable='time_synchronizer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/robot_joint_states', '/robot_joint_states'),
            ('/vispa/tool_ft', '/vispa/tool_ft'),
            ('vispa/space_robot_state', 'vispa/space_robot_state'),
        ]
    )

    isaac_sim_bootstrap = Node(
        package='isaac_sim_ctrl',
        executable='sim_bootstrap',
        output='screen',
        parameters=[
            {'usdfile_uri': 'file:<YOUR-PATH-TO>/SpaceAlign/isaac_sim/assets/space_align_vispa_workcell.usda'}
        ]
    )

    isaac_sim_ctrl = Node(
        package='isaac_sim_ctrl',
        executable='sim_ctrl',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            controller_yaml,
            {'use_sim_time': use_sim_time},
            {'tmp_usdpath': '<YOUR-PATH-TO>/SpaceAlign/isaac_sim/assets/tmp/'},
            # {'tmp_usdpath_win': 'd:/GITHUB/SpaceAlign/isaac_sim/assets/tmp/'}
        ],
        remappings=[
            ('/joint_states', '/robot_joint_states')
        ]
    )
    
    def on_isaac_sim_bootstrap_exit(event, context):
        return_code = event.returncode
        if return_code == 0:
            return [ 
                ros2_control_hardware_type_arg,
                use_sim_time_arg,
                rviz_config_arg,
                controller_manager_node,
                joint_state_broadcaster_spawner,
                vispa_arm_controller_spawner,
                move_group_node,
                vispa_display_launch_include,
                isaac_sim_ctrl,
                topic_time_synchronizer_node,
            ]
        else:
            return [ LogInfo(msg=f'isaac_sim_bootstrap failed! Cannot launch SpaceAlign.') ]

    return LaunchDescription([
        isaac_sim_bootstrap,    # IsaacSim MUST have a simulation loaded and running that publish the /clock topic BEFORE we launch the rest!
                                # Failing to do that will cause the 'controller_manager_node' to die while spawning the controllers !!!
        RegisterEventHandler(
            OnProcessExit(
                target_action=isaac_sim_bootstrap,
                on_exit=on_isaac_sim_bootstrap_exit
            )
        )
    ])
