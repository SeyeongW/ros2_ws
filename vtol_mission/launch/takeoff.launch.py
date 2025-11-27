from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            {'fcu_url': '/dev/serial0:57600'}, 
            {'gcs_url': ''},
            {'system_id': 255},
            {'component_id': 240},
            {'plugin_allowlist': ['sys_status', 'local_position', 'command', 'setpoint_raw', 'param', 'rc_io', 'mission', 'ftp']},
        ]
    )
    
    takeoff_land_node = Node(
        package='vtol_mission',
        executable='vtol_mission', 
        name='vtol_mission_node',
        output='screen'
    )

    return LaunchDescription([
        mavros_node,
        takeoff_land_node
    ])