from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    dof_arg = DeclareLaunchArgument(
        'DOF', default_value='4', description= 'Degrees of freedom for the dobot magician'
    )

    tool_arg = DeclareLaunchArgument(
        'tool', default_value='suction_cup', description= 'Tool attached to dobot magician'
    )

    dobot_bringup_launch= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/home/chichi/magician_ros2_control_system_ws/src/dobot_bringup/launch', 'dobot_magician_control_system.launch.py')
        )
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/home/chichi/magician_ros2_control_system_ws/src/dobot_description/launch', 'display.launch.py')
        ),
        launch_arguments = {
            'DOF': LaunchConfiguration('DOF'),
            'tool': LaunchConfiguration('tool')
        }.items()
    )

    dobot_homing_service_call = ExecuteProcess(
                cmd = ['ros2 ', 'service ', 'call ', '/dobot_homing_service', 'dobot_msgs/srv/ExecuteHomingProcedure'],
                shell=True,
                output='screen')

    pick_and_place_node = Node(
            package='PJT_server',
            executable='pick_and_place',
            name='pick_and_place',
            output='screen',
        )
    seperator_with_yolo_node = Node(
            package='PJT_server',
            executable='seperator_with_yolo',
            name='seperator_with_yolo',
            output='screen',
        )
    data_sending_server_node = Node(
            package='PJT_server',
            executable='data_sending_server',
            name='data_sending_server',
            output='screen',
        )
    
    return LaunchDescription([
        dof_arg,
        tool_arg,
        dobot_bringup_launch,
        TimerAction(period=25.0, actions=[display_launch]),
        TimerAction(period=30.0, actions=[dobot_homing_service_call]),
        # TimerAction(period=51.0, actions=[pick_and_place_node]),
        TimerAction(period=50.0, actions=[data_sending_server_node]),
        TimerAction(period=50.0, actions=[seperator_with_yolo_node]),
    ])