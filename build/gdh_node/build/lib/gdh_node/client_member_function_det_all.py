import sys

from gdh_interfaces.srv import GDHDetectStaticObjectAll, GDHDetectStaticTargetObject

import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')


        self.cli = self.create_client(GDHDetectStaticObjectAll, 'GDH_detect_all')
        # self.cli = self.create_client(GDHDetectStaticTargetObject, 'GDH_detect_target')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # self.req = GDHDetectStaticTargetObject.Request()
        self.req = GDHDetectStaticObjectAll.Request()

    def send_request(self):
        #self.req.object_name = ['stairs', 'door closed', 'escalator', 'subway entrance', 'door open']
        # self.future = self.cli.call_async(self.req)

        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of service call: %d' % (response.errcode))
                
                print(response)
 
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()