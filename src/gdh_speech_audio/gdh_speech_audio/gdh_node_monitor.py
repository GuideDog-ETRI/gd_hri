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
#            '/msg_to_audio': 'ros2 run gdh_speech_audio srv_play_audio_from_msg',
            '/speech_to_cmd': 'ros2 run gdh_speech_audio pub_command_from_speech',
        }

        # 일정 주기마다 상태 확인
        freq_sec = float(conf['node_monitor']['freq_sec'])
        self.node_wait_sec = float(conf['node_monitor']['node_wait_sec'])
        self.timer = self.create_timer(freq_sec, self.check_and_restart_nodes)

        # 노드 상태 기록 (노드 재시작 시간 기록)
        self.node_restart_times = {node_name: 0 for node_name in self.nodes_to_monitor}

    def check_and_restart_nodes(self):
        try:
            # 활성 노드 목록 가져오기
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
            active_nodes = result.stdout.splitlines()

            # 중복된 '/' 정리
            active_nodes = [node.lstrip('/') for node in active_nodes]  # 앞의 '//' 제거
            active_nodes = [f"/{node}" for node in active_nodes]  # 절대 경로 형식으로 통일

            current_time = time.time()
            for node_name, restart_command in self.nodes_to_monitor.items():
                # 노드가 활성화되어 있는지 확인
                if node_name not in active_nodes:
                    # 재시작 간격 확인 (5초 이상 지나야 재시작 가능)
                    last_restart = self.node_restart_times[node_name]
                    if current_time - last_restart > self.node_wait_sec:  # 5초 대기
                        self.get_logger().warn(f"Node '{node_name}' is not running! Restarting...")
                        self.restart_node(node_name, restart_command)
                        self.node_restart_times[node_name] = current_time  # 재시작 시간 업데이트
                    else:
                        self.get_logger().info(f"Node '{node_name}' recently restarted. Waiting...")
                else:
                    self.get_logger().info(f"Node '{node_name}' is running.")
        except Exception as e:
            self.get_logger().error(f"Error while checking node status: {e}")

    def restart_node(self, node_name, restart_command):
        try:
            # 노드 재시작 명령 실행
            subprocess.Popen(restart_command, shell=True)
            self.get_logger().info(f"Node '{node_name}' restarted successfully. Waiting for stabilization...")
            time.sleep(self.node_wait_sec)  # 노드 안정화 대기 시간 (필요에 따라 조정)
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
