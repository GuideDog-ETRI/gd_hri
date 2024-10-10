from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gdh_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'service = gdh_package.service_member_function:main',
            'client_start_detect = gdh_package.client_member_function_start_detect:main',
            'client_stop_detect = gdh_package.client_member_function_stop_detect:main',
            'client_display_detect = gdh_package.client_member_function_display_detect:main',
            'client_code_send = gdh_package.client_code_send:main',
            'server_toy_gd_others = gdh_package.server_toyGDOthers:main'
        ],
    },
)
