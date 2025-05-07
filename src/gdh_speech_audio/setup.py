from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gdh_speech_audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yochin',
    maintainer_email='yochin47@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
#            'pub_command_from_speech = gdh_speech_audio.speech_to_cmd:main',
            'pub_command_from_speech_by_w = gdh_speech_audio.speech_to_cmd_by_whisper:main',
#            'srv_play_audio_from_msg = gdh_speech_audio.msg_to_audio:main',
            'srv_play_audio_from_msg_by_m = gdh_speech_audio.msg_to_audio_by_melo:main',
            'gdh_node_monitor = gdh_speech_audio.gdh_node_monitor:main'
        ],
    },
)
