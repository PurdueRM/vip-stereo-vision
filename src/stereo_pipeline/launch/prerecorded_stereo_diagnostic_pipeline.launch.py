from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    left_video = LaunchConfiguration('left_video')
    right_video = LaunchConfiguration('right_video')
    fps = LaunchConfiguration('fps')
    loop = LaunchConfiguration('loop')
    start_frame = LaunchConfiguration('start_frame')

    replay_input = Node(
        package='stereo_pipeline',
        executable='replay_video_file_input_node',
        name='replay_video_file_input_node',
        output='screen',
        parameters=[{
            'left_video_path': left_video,
            'right_video_path': right_video,
            'playback_speed': LaunchConfiguration('playback_speed'),
            'buffer_size': LaunchConfiguration('buffer_size'),
            'start_frame': LaunchConfiguration('start_frame'),
        }],
    )

    stereo_vision = Node(
        package='stereo_pipeline',
        executable='stereo_vision_node',
        name='stereo_vision_node',
        output='screen',
    )

    video_output = Node(
        package='stereo_pipeline',
        executable='write_video_file_output_node',
        name='write_video_file_output_node',
        output='screen',
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'playback_speed',
            default_value='1.0'
        ),

        DeclareLaunchArgument(
            'buffer_size',
            default_value='4'
        ),

        DeclareLaunchArgument(
            'start_frame',
            default_value='0'
        ),

        replay_input,
        stereo_vision,
        video_output,
    ])
