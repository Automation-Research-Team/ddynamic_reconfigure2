from launch             import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [Node(name='pytestnode',
              package='ddynamic_reconfigure2',
              executable='pytestnode.py',
              output='screen',
              parameters=[{'numeric.param_i64': -2, 'numeric.param_d': 1.6}]),
         Node(name='rqt_reconfigure', package='rqt_reconfigure',
              executable='rqt_reconfigure', output='screen')])
