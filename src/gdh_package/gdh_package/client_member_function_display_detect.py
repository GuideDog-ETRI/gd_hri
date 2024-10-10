import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('GDH_image_subscriber')

        # Create a subscription to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/GDH_detections_img',  # Change this to your image topic
            self.listener_callback,
            10  # QoS profile
        )
        self.subscription  # prevent unused variable warning
        
        # Create a CvBridge object to convert ROS images to OpenCV images
        self.br = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv2_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if os.environ.get('DISPLAY', '') == '':
            # No display available, save image to a file
            print("No display found. Saving the image to a file.")
            
            # Create a directory to save images if it doesn't exist
            save_dir = "saved_images"
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            
            # Define the image file path with a unique name
            img_file = os.path.join(save_dir, "detection_image.jpg")
            
            # Save the image to a file
            cv2.imwrite(img_file, cv2_img)
            
            print(f"Image saved at {img_file}")
        else:
            # Display the image using OpenCV if display is available
            cv2.imshow("Display detections", cv2_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Keep the node spinning to listen to the image topic
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly (optional)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()