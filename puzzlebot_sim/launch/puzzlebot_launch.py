import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    urdf_file = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'urdf',
        urdf_file
    )

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_desc': robot_desc}],
        arguments=[urdf]
    )

    joint_state_pub_node = Node(
        package='puzzlebot_sim',
        executable='joint_state_publisher',
        name='join_state_publisher',
        output='screen',
    )
    
    #rviz2_node = Node(
    #package='rviz2',
    #executable='rviz2',
    #name='rviz2',
    #arguments=['-d', os.path.join(get_package_share_directory('puzzlebot_sim'), 'rviz', 'puzzlebot.rviz')],
    #output='screen',
    #)
	
    puzzlebot_kinematic_model = Node(
        package='puzzlebot_sim',
        executable='puzzlebot_kinematic_model',
        name='puzzlebot_kinematic_model',
        output='screen',
    )
    
    localisation = Node(
        package='puzzlebot_sim',
        executable='localisation',
        name='localisation',
        output='screen',
    )
    
    controller = Node(
        package='puzzlebot_sim',
        executable='controller',
        name='controller',
        output='screen',
        remappings=[
            ('/odom', '/ground_truth')
        ]
    )
    bug2 = Node(
        package='puzzlebot_sim',
        executable='bug2',
        name='bug2',
        output='screen',
        remappings=[
            ('/odom', '/ground_truth')
        ]
    )
   
    set_point_generator = Node(
        package='puzzlebot_sim',
        executable='set_point_generator',
        name='set_point_generator',
        output='screen',
    )
   


    l_d = LaunchDescription([robot_state_pub_node,
                            joint_state_pub_node,
                            puzzlebot_kinematic_model,
                            localisation,
                            controller,
                            #bug2,
                            set_point_generator])

    return l_d
