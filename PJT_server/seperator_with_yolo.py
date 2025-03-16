import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import cv2
import numpy as np

import pyrealsense2 as rs

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dobot_msgs.action import PointToPoint

import torch


class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolov5_node')

        # YOLOv5 model_load
        pt_path = '/home/chichi/final_PJT/src/PJT_server/best.pt'
        self.yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', path=pt_path)


        # RealSense camera setup
        self.pipeline = rs.pipeline()
        config = rs.config()    
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.detection_publisher = self.create_publisher(String, 'detection_results', 10)
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)
        
        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.conveyor_connect_status = False
        self.monitoring_connect_status = False

        self.roi_position = (278, 79, 490, 351)

    def get_color_name(self, hsv_color):
        h, s, v = hsv_color
        # print("H, S, V: ", h, s, v)

        if 70 < h < 100 and s < 40 and 150 < v < 180:
            return 'white'
        elif 90 < h < 120 and s > 220 and 80< v < 110:
            return 'blue'
        elif 165 < h < 195 and 205 < s < 235 and 115 < v < 145:
            return 'red'
        return 'unknown'

    def get_color_bgr(self, color_name):
        if color_name == 'white':
            return (255, 255, 255)
        elif color_name == 'red':
            return (0, 0, 255)
        elif color_name == 'blue':
            return (255, 0, 0)
        return (0, 255, 0)  # Default to green for unknown colors

    def get_center_color(self, image):
        height, width = image.shape[:2]
        center_y, center_x = height // 2, width // 2
        sample_size = min(width, height) // 4
        start_x = max(0, center_x - sample_size // 2)
        end_x = min(width, center_x + sample_size // 2)
        start_y = max(0, center_y - sample_size // 2)
        end_y = min(height, center_y + sample_size // 2)
        center_region = image[start_y:end_y, start_x:end_x]
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        average_color = np.mean(hsv_region, axis=(0, 1))
        
        return average_color
    
    def timer_callback(self):
        # Get RealSense frames
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            return

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        roi_x1, roi_y1, roi_x2, roi_y2 = self.roi_position
        cv2.rectangle(color_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)  # 파란색 ROI 사각형

        roi_image = color_image[roi_y1:roi_y2, roi_x1:roi_x2]
                
        # YOLOv5 inference
        results = self.yolo_model(roi_image)
        print(results)

        self.detection_result = String()

        # Draw bounding boxes
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = map(int, result[:6])

            object_roi = color_image[roi_y1 + y1: roi_y1 + y2, roi_x1 + x1: roi_x1 + x2]
            center_color = self.get_center_color(object_roi)

            color_name = self.get_color_name(center_color)
            if color_name == 'red':
                state = 'broken'
            elif color_name == 'unknown' :
                state = 'unknown'
            else:
                state = 'normal'

            print("color_name: ", color_name)
            color_bgr = self.get_color_bgr(color_name)

            label = self.yolo_model.names[class_id]
            
            self.detection_result.data = label+'_'+state
            cv2.rectangle(color_image, (roi_x1+x1, roi_y1+y1), (roi_x1+x2, roi_y1+y2), (0, 255, 0), 2)
            cv2.putText(color_image, f'{label}-{state}', (roi_x1+x1, roi_y1+y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.detection_publisher.publish(self.detection_result) # String
            ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            self.image_publisher.publish(ros_image_message) # image
            
        cv2.imshow('frame', color_image)
        cv2.waitKey(1)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
