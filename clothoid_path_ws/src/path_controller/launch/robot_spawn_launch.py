import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart


import xacro

def generate_launch_description():
    pkg_name = 'path_controller'

    yaml_file_path = os.path.join(get_package_share_directory(pkg_name),'config','my_controllers.yaml')

    ld = LaunchDescription()

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp_launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    ld.add_action(rsp)

    # Declare the world argument
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.world', description='World file name to load in Gazebo'
    )
    ld.add_action(world_arg)
    
    world_file_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'worlds',
        LaunchConfiguration('world')
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={'world': world_file_path}.items()
    )
    ld.add_action(gazebo)

    # Declare the world argument - refer above for spawn points
    spawn_x = DeclareLaunchArgument('spawn_x', default_value="0.0", description='Spawn x coordinate in Gazebo')
    spawn_y = DeclareLaunchArgument('spawn_y', default_value="1.2", description='Spawn y coordinate in Gazebo')
    spawn_z = DeclareLaunchArgument('spawn_z', default_value="0.5", description='Spawn z coordinate in Gazebo')
    roll = DeclareLaunchArgument('roll', default_value="0.0", description='Roll')
    pitch = DeclareLaunchArgument('pitch', default_value="0.0", description='Pitch')
    yaw = DeclareLaunchArgument('yaw', default_value="0.0", description='Yaw')

    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'autonomous_car',
                    '-x', LaunchConfiguration('spawn_x'),
                    '-y', LaunchConfiguration('spawn_y'),
                    '-z', LaunchConfiguration('spawn_z'),
                    '-R', LaunchConfiguration('roll'),
                    '-P', LaunchConfiguration('pitch'),
                    '-Y', LaunchConfiguration('yaw')
                ],
                output='screen'
            )
        ]
    )
    ld.add_action(spawn_entity)


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ld.add_action(rviz)



    # arguments
    
    ld.add_action(spawn_x)
    ld.add_action(spawn_y)
    ld.add_action(spawn_z)
    ld.add_action(roll)
    ld.add_action(pitch)
    ld.add_action(yaw)

    return ld 

