import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

# Ricoh Theta Z
import py360convert

class BagVideoConverter(Node):
    def __init__(self, bag_file, topic_name, output_video):
        super().__init__('bag_video_converter')
        self.bridge = CvBridge()
        self.bag_file = bag_file
        self.topic_name = topic_name
        self.output_video = output_video
        self.video_writer = None
        self.count = 0

    def process_bag(self):
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

        reader = SequentialReader()
        storage_options = StorageOptions(uri=self.bag_file, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader.open(storage_options, converter_options)

        topics = reader.get_all_topics_and_types()
        if not any(topic.name == self.topic_name for topic in topics):
            self.get_logger().error(f"Topic {self.topic_name} not found in the bag file.")
            return

        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            if topic == self.topic_name:
                try:
                    compressed_image_msg = self.deserialize_compressed_image(data)
                    frame = self.compressed_image_to_cv2(compressed_image_msg)
                    proc_frame = self.process_image(frame)

                    if self.video_writer is None:
                        height, width, _ = proc_frame.shape
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        self.video_writer = cv2.VideoWriter(self.output_video, fourcc, 15, (width, height))

                    self.video_writer.write(proc_frame)
                except Exception as e:
                    self.get_logger().error(f"Error processing frame: {e}")

                self.count += 1
            
            #if self.count == 10:
            #    break

        if self.video_writer:
            self.video_writer.release()

    def deserialize_compressed_image(self, serialized_data):
        from rclpy.serialization import deserialize_message
        return deserialize_message(serialized_data, CompressedImage)

    def compressed_image_to_cv2(self, compressed_image_msg):
        # 압축된 이미지를 OpenCV 형식으로 디코딩
        np_arr = np.frombuffer(compressed_image_msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    def deserialize_image(self, serialized_data):
        from rclpy.serialization import deserialize_message
        return deserialize_message(serialized_data, Image)

    def process_image(self, np_image):
        if np_image is not None:
            # ERP 이미지 펼치기
            u_deg=0
            planarImg = py360convert.e2p(np_image, 
                                        fov_deg=(90, 70), 
                                        u_deg=u_deg, 
                                        v_deg=0, 
                                        out_hw=(480, 640), 
                                        mode='bilinear')

            ih, iw, ic = np_image.shape
            oh, ow, oc = planarImg.shape
            print(f"Py360 from {iw}x{ih} to {ow}x{oh}")

            return planarImg
        else:
            return None

def main():
    rclpy.init()

    # bag_file = "rosbag/rosbag2_2024_11_22-13_32_28"  # ROS2 bag 파일 경로
    # topic_name = "/theta/image_raw/compressed"     # 이미지 토픽 이름
    # output_video = "rosbag_video.mp4"  # 저장할 비디오 파일 이름

    # bag_file = "rosbag/241125_1dong_democourse/rosbag2_2024_11_25-16_31_08"  # ROS2 bag 파일 경로
    # bag_file = "rosbag/241125_1dong_democourse/rosbag2_2024_11_25-16_16_32"
    bag_file = "rosbag/rosbag2_2024_12_09-16_33_40"
    topic_name = "/theta/image_raw/compressed"     # 이미지 토픽 이름
    output_video = "rosbag_video_udeg0_2024_12_09-16_33_40.mp4"  # 저장할 비디오 파일 이름

    converter = BagVideoConverter(bag_file, topic_name, output_video)
    converter.process_bag()

    rclpy.shutdown()

if __name__ == "__main__":
    main()