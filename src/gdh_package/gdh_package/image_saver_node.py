import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml
import cv2
import sys
import threading

class ImageSaverNode(Node):
    def __init__(self, image_topic, output_filename):
        super().__init__('image_saver_node')
        self.bridge = CvBridge()
        self.image_topic = image_topic
        self.output_filename = output_filename
        self.images = []
        self.image_count = 0
        self.running = True

        self.subscriber = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.get_logger().info(f"Subscribed to image topic: {self.image_topic}")

    def image_callback(self, msg):
        if not self.running:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.images.append(cv_image)
            self.image_count += 1
            self.get_logger().info(f"Image received. Total images: {self.image_count}")
            print(f"[Console] Image received. Total images: {self.image_count}")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def save_video(self, fps=30):
        if not self.images:
            self.get_logger().warn("No images to save.")
            return

        height, width, _ = self.images[0].shape
        writer = cv2.VideoWriter(self.output_filename, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

        for img in self.images:
            writer.write(img)
        writer.release()
        self.get_logger().info(f"Video saved as {self.output_filename}")

    def shutdown(self):
        self.get_logger().info("Shutting down, saving video...")
        self.running = False
        self.save_video()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def load_config(path='models/gdh_config.yaml'):
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
    return config['image_saver']['image_topic'], config['image_saver']['output_filename']

def keyboard_listener(node: ImageSaverNode):
    print("Press 'q' then Enter to quit and save video.")
    while True:
        key = input()
        if key.lower() == 'q':
            node.shutdown()
            break

def main():
    rclpy.init()
    image_topic, output_filename = load_config()
    node = ImageSaverNode(image_topic, output_filename)

    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()

if __name__ == '__main__':
    main()
