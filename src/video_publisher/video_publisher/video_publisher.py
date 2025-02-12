#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class VideoPublisher(Node):
    def __init__(self, video_path):
        super().__init__('video_publisher')
        
        # Create the publisher
        self.publisher = self.create_publisher(Image, 'video_frames', 10)
        
        # Set up CV Bridge
        self.bridge = CvBridge()
        
        # Open the video file
        self.video = cv2.VideoCapture(video_path)
        if not self.video.isOpened():
            self.get_logger().error(f'Could not open video file: {video_path}')
            sys.exit(1)
            
        # Get video properties
        self.fps = self.video.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f'Video FPS: {self.fps}')
        
        # Create timer to publish frames at the video's FPS
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.publish_frame)
        
        self.get_logger().info('Video publisher started')

    def publish_frame(self):
        ret, frame = self.video.read()
        
        if ret:
            # Convert the frame to a ROS Image message
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                # set header correctly
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = 'video_src'
                self.publisher.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f'Error converting frame: {str(e)}')
        else:
            # Video is finished, restart from beginning
            self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.get_logger().info('Restarting video from beginning')

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 2:
        print("Usage: ros2 run <package_name> video_publisher <video_path>")
        sys.exit(1)
        
    video_path = sys.argv[1]
    node = VideoPublisher(video_path)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()