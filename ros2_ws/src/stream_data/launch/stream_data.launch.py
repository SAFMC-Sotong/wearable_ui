from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stream_data',
            executable='stream_data',
            name='stream_data',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'battery_topic': '/mavros/battery',
                    'state_topic': '/mavros/state',
                    'update_rate_ms': 100,
                    'battery_udp_ip': '10.42.0.104',
                    'battery_udp_port': 7881,
                    'state_udp_port': 7882,
                }
            ]
        )
    ])