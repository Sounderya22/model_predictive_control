from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


robot_name = 'ubot_burger.urdf'

# ARGUMENTS = [
#     DeclareLaunchArgument('model', default_value='burger',
#                           choices=['burger', 'waffle', 'waffle-pi'],
#                           description='Turtlebot4 Model'),
#     DeclareLaunchArgument('use_sim_time', default_value='false',
#                           choices=['true', 'false'],
#                           description='use_sim_time'),
#     DeclareLaunchArgument('robot_name', default_value='ubot_burger',
#                           description='Robot name'),
#     DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
#                           description='Robot namespace'),
# ]


def generate_launch_description():
    ld = LaunchDescription()

    description_package_path = FindPackageShare('ubot_description')
    default_model_path = PathJoinSubstitution(['urdf', robot_name])
    default_rviz_config_path = PathJoinSubstitution([description_package_path, 'rviz', 'model.rviz'])

    urdf_path = os.path.join(
        get_package_share_directory('ubot_description'),
        'urdf',
        robot_name)

    print("urdf_file_name : {}".format(urdf_path))

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}


    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time')
    ld.add_action(sim_time_arg)
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))
    

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'ubot_description',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher)

    return ld