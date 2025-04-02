from setuptools import find_packages, setup

package_name = 'gdh_vlm_client'

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
    maintainer='gdh',
    maintainer_email='yochin@etri.re.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run gdh_vlm_client image_ai_node
            'convert_image_node = gdh_vlm_client.convert_image_node:main',
            'image_ai_node = gdh_vlm_client.image_ai_node:main',
            'toggle_flag_node = gdh_vlm_client.toggle_flag_node:main',
            'qwen_console_node = gdh_vlm_client.qwen_console_node:main'
        ],
    },
)
