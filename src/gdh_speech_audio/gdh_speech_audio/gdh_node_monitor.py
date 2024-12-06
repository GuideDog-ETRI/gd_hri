import os
import time
import subprocess
import rclpy
from rclpy.node import Node

# yaml
import yaml

class GDHNodeMonitor(Node):
    def __init__(self):
        super().__init__('gdh_node_monitor')

        # load from conf.yaml
        path_to_config = 'models/gdh_config.yaml'
        if os.path.exists(path_to_config):
            with open(path_to_config) as fid:
                conf = yaml.full_load(fid)
        else:
            raise AssertionError(f'No gdh_config file in {path_to_config}.')

        # 감시할 노드 정보: {노드 이름: 재시작 명령어}
        self.nodes_to_monitor = {
            '/msg_to_audio': 'ros2 run gdh_speech_audio srv_play_audio_from_msg',
            '/speech_to_cmd': 'ros2 run gdh_speech_audio pub_command_from_speech',
        }

        # 일정 주기마다 상태 확인
        freq_sec = float(conf['node_monitor']['freq_sec'])
        self.timer = self.create_timer(freq_sec, self.check_and_restart_nodes)

    def check_and_restart_nodes(self):
        try:
            # 활성 노드 목록 가져오기
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
            active_nodes = result.stdout.splitlines()

            for node_name, restart_command in self.nodes_to_monitor.items():
                # 노드가 활성화되어 있는지 확인
                if node_name not in active_nodes:
                    self.get_logger().warn(f"Node '{node_name}' is not running! Restarting...")
                    self.restart_node(node_name, restart_command)
                else:
                    pass
                    # self.get_logger().info(f"Node '{node_name}' is running.")
        except Exception as e:
            self.get_logger().error(f"Error while checking node status: {e}")

    def restart_node(self, node_name, restart_command):
        try:
            # 노드 재시작 명령 실행
            subprocess.Popen(restart_command, shell=True)
            self.get_logger().info(f"Node '{node_name}' restarted successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to restart node '{node_name}': {e}")


def main(args=None):
    rclpy.init(args=args)
    node_monitor = GDHNodeMonitor()
    rclpy.spin(node_monitor)
    node_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
