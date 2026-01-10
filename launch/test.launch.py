from launch                            import LaunchDescription
from launch.substitutions              import (PathJoinSubstitution,
                                               ThisLaunchFileDir)
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    return LaunchDescription(
        [Node(name='testnode',
              package='ddynamic_reconfigure2',
              executable='testnode',
              output='screen',
              parameters=[
                  ParameterFile(
                      PathJoinSubstitution([ThisLaunchFileDir(),
                                            '..', 'config', 'test.yaml']))
              ]),
         Node(name='rqt_reconfigure', package='rqt_reconfigure',
              executable='rqt_reconfigure', output='screen')])
