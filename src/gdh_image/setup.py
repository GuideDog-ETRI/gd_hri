from setuptools import find_packages, setup

package_name = 'gdh_image'

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
            # ros2 run gdh_image convert_image_node
            'image_publisher_node = gdh_image.image_publisher_node:main',
            'image_viewer_node = gdh_image.image_viewer_node:main'
        ],
    },
)
