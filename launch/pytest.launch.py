from launch                            import LaunchDescription
from launch.substitutions              import (PathJoinSubstitution,
                                               ThisLaunchFileDir)
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    return LaunchDescription(
        [Node(name='pytestnode',
              package='ddynamic_reconfigure2',
              executable='pytestnode.py',
              output='screen',
              parameters=[
                  ParameterFile(
                      PathJoinSubstitution([ThisLaunchFileDir(),
                                            '..', 'config', 'test.yaml']))
              ]),
         Node(name='rqt_reconfigure', package='rqt_reconfigure',
              executable='rqt_reconfigure', output='screen')])
