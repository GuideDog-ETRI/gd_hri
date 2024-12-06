from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='gdh_speech_audio',
        #     executable='srv_play_audio_from_msg',
        #     name='msg_to_audio'
        # ),
        # Node(
        #     package='gdh_speech_audio',
        #     executable='pub_command_from_speech',
        #     name='speech_to_cmd'
        # ),
        # Node(
        #     package='gdh_speech_audio',
        #     executable='gdh_node_monitor',
        #     name='gdh_node_monitor'
        # ),      
    ])
