import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_description = LaunchConfiguration('robot_description')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    robot_description_semantic = LaunchConfiguration('robot_description_semantic')
    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value='',
        description='Robot URDF XML string for MoveIt/RViz')
    declare_robot_description_semantic_cmd = DeclareLaunchArgument(
        'robot_description_semantic',
        default_value='',
        description='Robot SRDF XML string for MoveIt/RViz')
    declare_joint_states_topic_cmd = DeclareLaunchArgument(
        'joint_states_topic',
        default_value='/joint_states', # Default to standard topic
        description='Name of the input joint states topic'
    )
    # Get the launch directory
    bringup_dir = get_package_share_directory('vispa')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_file = LaunchConfiguration('urdf_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'vispa_ros2.rviz'),
        description='Full path to the RVIZ config file to use')  
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='True',
        description='Whether to start the joint state publisher')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(bringup_dir, 'urdf', 'VISPA_modifiedDH_Ros2.urdf'),
        description='Whether to start RVIZ')
 

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        remappings=[
            ('/joint_states', joint_states_topic)
        ]
    )

    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file])
    
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        # VS Code Snap can leak GTK_PATH into launch environments and crash rviz2.
        additional_env={'GTK_PATH': ''},
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            {'robot_description_semantic': ParameterValue(robot_description_semantic, value_type=str)}
        ]
    )
        
  
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_joint_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(declare_robot_description_semantic_cmd)
    ld.add_action(declare_joint_states_topic_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add any conditioned actions
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    return ld   
    
