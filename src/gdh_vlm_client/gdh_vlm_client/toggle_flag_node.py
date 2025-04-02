#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading
import datetime

class ToggleFlagNode(Node):
    def __init__(self):
        super().__init__('toggle_flag_node')

        # 현재 flag 상태 (처음에는 False)
        self.flag_state = False

        # 퍼블리셔: /convert_flag (std_msgs/Bool)
        self.publisher_ = self.create_publisher(Bool, '/convert_flag', 10)

        # 0.5초 간격으로 flag_state를 퍼블리시
        self.timer = self.create_timer(0.5, self.timer_callback)

        # True로 만든 뒤 일정 시간 후 False로 되돌릴 때 쓸 임시 타이머
        self.revert_timer = None

        self.get_logger().info('ToggleFlagNode started. Publishing False by default.')
        
    def log_info(self, msg: str):
        # 현재 시간을 사람이 읽기 좋은 형식으로 변환
        current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # 커스텀 로그 메시지 구성 및 출력
        self.get_logger().info(f'[{current_time}] {msg}')

    def timer_callback(self):
        """
        0.5초마다 현재 flag_state를 퍼블리시
        """
        msg = Bool()
        msg.data = self.flag_state
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published flag: {msg.data}')
        self.log_info(f'Published flag: {msg.data}')

    def toggle_flag_once(self, duration=0.25):
        """
        flag_state를 True로 바꾸고,
        duration(초) 뒤에 자동으로 False로 복귀.
        """
        if self.revert_timer is not None:
            # 만약 기존에 설정된 타이머가 있으면 취소
            self.revert_timer.cancel()

        # flag_state = True
        self.flag_state = True
        self.get_logger().info('Set flag to True (once)')

        # duration 이후에 False로 되돌리는 타이머 설정
        self.revert_timer = self.create_timer(duration, self.set_flag_to_false)

    def set_flag_to_false(self):
        """
        타이머 콜백으로, flag_state를 False로 되돌린다.
        그리고 자기 자신(이 타이머)도 취소한다.
        """
        # 타이머 한 번만 실행 후 취소
        if self.revert_timer:
            self.revert_timer.cancel()
            self.revert_timer = None

        self.flag_state = False
        self.get_logger().info('Set flag back to False')

def main(args=None):
    rclpy.init(args=args)
    node = ToggleFlagNode()

    def user_input_thread():
        """
        사용자 입력 스레드:
          - 엔터(빈 입력) 입력시 True를 짧게 발행 후 자동 False 복귀
          - "exit" 입력시 노드 종료
        """
        while rclpy.ok():
            user_input = input('Press Enter to send True once, or type "exit" to quit: ')
            if user_input.strip().lower() == 'exit':
                node.get_logger().info('Exiting user input thread...')
                rclpy.shutdown()  # 전체 노드 종료 트리거
                break

            # 엔터이면 True -> 약간의 시간 후 False 복귀
            node.toggle_flag_once(duration=0.2)

    # 사용자 입력을 받는 스레드 시작
    thread = threading.Thread(target=user_input_thread, daemon=True)
    thread.start()

    try:
        # 메인 스레드는 ROS 스핀
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # 스핀 종료
    node.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
