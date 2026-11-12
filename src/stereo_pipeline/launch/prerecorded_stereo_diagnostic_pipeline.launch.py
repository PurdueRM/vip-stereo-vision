
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    replay_input = Node(
        package='stereo_pipeline',
        executable='replay_video_file_input_node',
        name='replay_video_file_input_node',
        output='screen',
    )

    armor_isolation = Node(
        package='stereo_pipeline',
        executable='armor_isolation_node',
        name='armor_isolation_node',
        output='screen',
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
        replay_input,
        armor_isolation,
        stereo_vision,
        video_output,
    ])
