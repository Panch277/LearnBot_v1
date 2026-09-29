import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Standalone launch file (no ament package) for the custom web dashboard's
    # ROS-side bridges. Cloned from the pattern in
    # src/learnbot/launch/foxglove.launch.py rather than editing that package:
    # run on the desktop only, never on the Pi. DDS discovery lets it see the
    # Pi's topics with zero changes on the Pi side.

    rosbridge_websocket = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'
        )]),
        launch_arguments={
            'port': '9090',
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Serves any image topic as plain MJPEG-over-HTTP (re-encoding internally)
    # so the web dashboard can just point an <img> tag at it, instead of piping
    # raw image bytes through rosbridge's JSON/base64 encoding (far too slow
    # for live video). Point it at the *base* raw topic, e.g. /camera/image_raw
    # -- not an already-compressed variant, which causes a silent type
    # mismatch and zero frames.
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen',
        parameters=[{
            'port': 8080,
            'use_sim_time': use_sim_time,
        }]
    )

    # Lets the web dashboard start/stop `ros2 launch` processes (SLAM, Nav2)
    # via ROS services instead of a terminal -- see process_manager.py for
    # the process list and start/stop/monitor logic. Not an installed
    # package executable (same "standalone, no ament package" reasoning as
    # this launch file itself), so it's run directly with python3 rather
    # than through launch_ros's Node action.
    process_manager = ExecuteProcess(
        cmd=['python3', os.path.join(os.path.dirname(__file__), 'process_manager.py')],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        rosbridge_websocket,
        web_video_server,
        process_manager,
    ])
