import launch
import launch_ros.actions

from os.path import expanduser
import json 

CAMERA_FRAMERATE = 30.0

def generate_launch_description():
    with open(expanduser("~")+'/Formula-Student-Driverless-Simulator/settings.json', 'r') as file:
        settings = json.load(file)

    camera_configs = settings['Vehicles']['FSCar']['Cameras']
    if(not camera_configs):
        print('no cameras configured in ~/Formula-Student-Driverless-Simulator/settings.json')

    camera_nodes = [
        launch_ros.actions.Node(
            package='fsds_ros2_bridge',
            executable='fsds_ros2_bridge_camera',
            namespace="fsds/camera", 
            name=camera_name,
            output='screen',
            parameters=[
                {'camera_name': camera_name},
                {'depthcamera': camera_config["CaptureSettings"][0]["ImageType"] == 2},
                {'framerate': CAMERA_FRAMERATE},
                {'host_ip': launch.substitutions.LaunchConfiguration('host')},
            ]
        ) for camera_name, camera_config in camera_configs.items()]

    ld = launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mission_name',
            default_value='trackdrive'
        ),
        launch.actions.DeclareLaunchArgument(
            name='track_name',
            default_value='A'
        ),
        launch.actions.DeclareLaunchArgument(
            name='competition_mode',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='manual_mode',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='UDP_control',
            default_value='false'
        ),
        *camera_nodes,
        launch_ros.actions.Node(
            package='fsds_ros2_bridge',
            executable='fsds_ros2_bridge',
            name='ros_bridge',
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'update_odom_every_n_sec': 0.004
                },
                {
                    'update_imu_every_n_sec': 0.004
                },
                {
                    'update_gps_every_n_sec': 0.1
                },
                {
                    'update_gss_every_n_sec': 0.01
                },
                {
                    'publish_static_tf_every_n_sec': 1.0
                },
                {
                    'update_lidar_every_n_sec': 0.1
                },
                {
                    'host_ip': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'mission_name': launch.substitutions.LaunchConfiguration('mission_name')
                },
                {
                    'track_name': launch.substitutions.LaunchConfiguration('track_name')
                },
                {
                    'competition_mode': launch.substitutions.LaunchConfiguration('competition_mode')
                },
                {
                    'manual_mode': launch.substitutions.LaunchConfiguration('manual_mode')
                },
                {
                    'UDP_control': launch.substitutions.LaunchConfiguration('UDP_control')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
