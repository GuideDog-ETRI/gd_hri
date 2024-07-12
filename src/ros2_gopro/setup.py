from setuptools import find_packages, setup

package_name = 'ros2_gopro'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'gopro_pub = ros2_gopro.publisher_gopro:main',
            'webcam_pub = ros2_gopro.publisher_video:main',
            'photo_pub = ros2_gopro.publisher_photo:main',
            'video_sub = ros2_gopro.subscriber_video:main',
        ],
    },
)
