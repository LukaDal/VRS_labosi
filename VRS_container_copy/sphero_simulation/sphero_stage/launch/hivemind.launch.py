from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    hivemind = Node(
        package='sphero_stage',
        executable='hivemind',
        name='hivemind',
        output='screen'
    )

    rqt = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen'
    )

    return LaunchDescription([hivemind,rqt])