import os

import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='100'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='vehicle_filter',
            default_value='vehicle.*'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point',
            default_value='None'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town10HD'
        ),
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=get_package_share_directory(
                'carla_spawn_objects') + '/config/objects_town10.json'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'launch/carla_ros_bridge_autopilot.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'passive': launch.substitutions.LaunchConfiguration('passive'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'vehicle_filter': launch.substitutions.LaunchConfiguration('vehicle_filter'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'spawn_point': launch.substitutions.LaunchConfiguration('spawn_point'),
                'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_manual_control'), 'carla_manual_control.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ackermann_control'), 'carla_ackermann_control.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
