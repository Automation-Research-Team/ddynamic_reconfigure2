from launch             import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(name='testnode',
             package='ddynamic_reconfigure2',
             executable='testnode',
             output='screen',
             parameters=[{'numeric.param_i64': -2, 'numeric.param_d': 1.6}])
        ])
