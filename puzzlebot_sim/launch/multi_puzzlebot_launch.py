import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    robots = [
        {'name': 'robot1', 'x_pose': 0.5, 'y_pose': 0.5},
        {'name': 'robot2', 'x_pose': -0.5, 'y_pose': 0.5},
    ]

    urdf_file = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'urdf',
        urdf_file
    )

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    nodes = []

    for robot in robots:
        # Cargar URDF modificado con nombre único
        group = GroupAction([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot['name'],  # Better to use namespace than frame_prefix
                output='screen',
                parameters=[{
                    'robot_description': robot_desc,
                    'frame_prefix': robot['name'] + '/'
                }],
                arguments=[urdf]
            ),

            Node(
                package='puzzlebot_sim',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                namespace=robot['name'],  # Esto establece el namespace
                parameters=[{'robot_name': robot['name']}],  # Pasa el nombre como parámetro
                output='screen',
            ),

            Node(
                package='puzzlebot_sim',
                executable='puzzlebot_kinematic_model',
                name='puzzlebot_kinematic_model',
                namespace=robot['name'],
                parameters=[{'robot_name': robot['name']}],
                output='screen',
            ),
        
            Node(
                package='puzzlebot_sim',
                executable='localisation',
                name='localisation',
                namespace=robot['name'],  # Esto establece el namespace
                parameters=[{'robot_name': robot['name']}],  # Pasa el nombre como parámetro
                output='screen',
            ),
            
            Node(
                package='puzzlebot_sim',
                executable='controller',
                name='controller',
                namespace=robot['name'],  # Esto establece el namespace
                parameters=[{'robot_name': robot['name']}],  # Pasa el nombre como parámetro
                output='screen',
            ),
                    
            Node(
                package='puzzlebot_sim',
                executable='set_point_generator',
                name='set_point_generator',
                namespace=robot['name'],
                parameters=[{'robot_name': robot['name']}],
                output='screen',
            ),

            
        ])
        nodes.append(group)
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('puzzlebot_sim'), 'rviz', 'puzzlebot.rviz')],
        output='screen',
    )

    return LaunchDescription([rviz2_node, *nodes])