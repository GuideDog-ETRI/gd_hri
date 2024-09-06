import rclpy
from rclpy.node import Node
from gd_ifc_pkg.srv import GDGGuideAssist

class GDOthers(Node):
    def __init__(self):
        super().__init__('gd_others')
        self.srv = self.create_service(GDGGuideAssist, '/gdg_guide_assist', self.handle_sentence_send)

    def handle_sentence_send(self, request, response):
        self.get_logger().info(f'Received cmd and type in GD Others: {request.cmd}, {request.type}')
        
        response.errcode = response.ERR_NONE
        
        return response

def main(args=None):
    rclpy.init(args=args)
    gdother_node = GDOthers()
    rclpy.spin(gdother_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

