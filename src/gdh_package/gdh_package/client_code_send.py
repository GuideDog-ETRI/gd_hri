import sys
import rclpy
from rclpy.node import Node

from gd_ifc_pkg.srv import GDHSpeakCodeID


class CodesendClient(Node):
    def __init__(self):
        super().__init__('codesend_client')
        self.client = self.create_client(GDHSpeakCodeID, '/GDH_speak_codeid')  # Trigger 서비스 타입

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = GDHSpeakCodeID.Request()

    def send_request(self, code_value):
        self.request.code_id = int(code_value)  # code_value is text ('1'), 입력 값을 요청 객체에 설정

        future = self.client.call_async(self.request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Response: errcode={response.errcode}')
        else:
            self.get_logger().error('Service call failed')


def main(args=None):
    rclpy.init(args=args)
    client = CodesendClient()

    try:
        while rclpy.ok():
            # 사용자로부터 입력 값을 받아 요청을 보냄
            code_value = input("Enter a code value (1 ~ 4 in models/code_sentence_table.txt) to send (or 'exit' to quit): ")
            if code_value.lower() == 'exit':
                break
            client.send_request(code_value)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
