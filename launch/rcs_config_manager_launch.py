from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rcs_config_manager',
            executable='rcs_cms_node',
            name='RCS_CMS_Node',
            output='screen',
            emulate_tty=True
        )
    ])