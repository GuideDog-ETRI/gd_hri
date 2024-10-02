from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gdh_package',
            executable='service',
            name='service_member_function'
        ),
        # Node(
        #     package='gdh_package',
        #     executable='client_det_init',
        #     name='client_member_function_init_det'
        # ),
        # Node(
        #     package='gdh_package',
        #     executable='client_det_term',
        #     name='client_member_function_term_det'
        # ),
        # Node(
        #     package='gdh_package',
        #     executable='client_det_all',
        #     name='client_member_function_det_all'
        # ),
        # Node(
        #     package='gdh_package',
        #     executable='client_code_send',
        #     name='client_code_send'
        # ),
        # Node(
        #     package='gdh_package',
        #     executable='server_toy_gd_others',
        #     name='server_toyGDOthers'
        # ),
    ])
