import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import socket
import threading

import time
conn_conv = False
conn_robo = False

def start_conveyor_server(host='192.168.110.125', port=44441):
    global conn_conv, conn_robo
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen(2)
        # s.listen()
        print(f'Server is listening on {host}: {port}....')

        conn_conv, addr_conv = s.accept()
        conn_robo, addr_robo = s.accept()
        print(f'Connected by {addr_robo}, {addr_conv}')

class ObjectDetectionSubscriber(Node):
    global conn_conv
    global conn_robo
    def __init__(self):
        super().__init__('objectDetectionsubscriber')

        self.subscription = self.create_subscription(
            String,
            '/detection_results',
            self.listener_callback,
            10
        )
        
        self.detection_buffer = []
        # 방이 20개 방중에 12개가 차면 로직발동
        self.buffer_size = 20
        self.detection_threshold = 12

        self.get_logger().info("Object Detection Subscriber has started.")
        
        # Conveyor Server On
        global conn_conv, conn_robo
        server_thread = threading.Thread(target=start_conveyor_server)
        server_thread.daemon = True
        server_thread.start()

    def listener_callback(self, msg):
        detection_results = msg.data

        detected_objects = self.parse_detection_results(detection_results)

        if 'back_panel_normal' in detected_objects:
            self.detection_buffer.append('back panel')
        elif 'board_panel_normal' in detected_objects:
            self.detection_buffer.append('board panel')
        elif ('board_panel_broken' in detected_objects) or ('back_panel_broken' in detected_objects):
            self.detection_buffer.append('broken')
        elif ('board_panel_unknown' in detected_objects) or ('back_panel_unknown' in detected_objects):
            self.detection_buffer.append('unknown')
        else:
            self.detection_buffer.append('None')
        
        if len(self.detection_buffer) > self.buffer_size:
            self.detection_buffer.pop(0)

        self.check_object_detection()
    
    def check_object_detection(self):
        if self.detection_buffer.count('back panel') >= self.detection_threshold:
            self.perform_task_for_object('back panel')

        elif self.detection_buffer.count('board panel') >= self.detection_threshold:
            self.perform_task_for_object('board panel')

        elif self.detection_buffer.count('broken') >= self.detection_threshold:
            self.perform_task_for_object('broken')

        elif self.detection_buffer.count('unknown') >= self.detection_threshold:
            self.perform_task_for_object('unknown')

    def parse_detection_results(self, detection_results):
        try:
            if not detection_results:
                self.get_logger().warn("Empty detection results recieved.")
                return []
            detected_objects = [obj.strip() for obj in detection_results.split(',')]
            return detected_objects
        
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            return []
        
    def perform_task_for_object(self, object_name):
        self.get_logger().info(f'{object_name} detected! Performing task...')
        # board_panel
        # back_panel
        # broken
        # unknown
        self.wait_for_clients_connection()
        if object_name == 'back panel':
            self.perform_task_back_panel()
        elif object_name == 'board panel':
            self.perform_task_board_panel()
        elif object_name == 'broken':
            self.perform_task_broken_panel()
        elif object_name == 'unknown':
            self.perform_task_unknown_panel()

        self.detection_buffer = []

    def perform_task_broken_panel(self):
        self.get_logger().info(f'Executing task for broken panel')

        if conn_conv:
            command = 'broken'
            conn_conv.sendall(command.encode('utf-8'))
            conn_robo.sendall(command.encode('utf-8'))
            self.get_logger().info(f'Sent command {command} to the client.')
            print(f'Sent command {command} to the client.')

        time.sleep(5)

    def perform_task_back_panel(self):
        self.get_logger().info(f'Executing task for back panel')

        if conn_conv:
            command = 'back_panel'
            conn_conv.sendall(command.encode('utf-8'))
            conn_robo.sendall(command.encode('utf-8'))
            self.get_logger().info(f'Sent command {command} to the client.')
            print(f'Sent command {command} to the client.')

        time.sleep(5)
    
    def perform_task_board_panel(self):
        self.get_logger().info(f'Executing task for board panel')

        if conn_conv:
            command = 'board_panel'
            conn_conv.sendall(command.encode('utf-8'))
            conn_robo.sendall(command.encode('utf-8'))
            self.get_logger().info(f'Sent command {command} to the client.')
            print(f'Sent command {command} to the client.')

        time.sleep(5)

    def perform_task_unknown_panel(self):
        self.get_logger().info(f'Executing task for unknown panel')

        if conn_conv and conn_robo:
            command = 'unknown'
            conn_conv.sendall(command.encode('utf-8'))
            conn_robo.sendall(command.encode('utf-8'))
            self.get_logger().info(f'Sent command {command} to the client.')
            print(f'Sent command {command} to the client.')

        time.sleep(5)

    def wait_for_clients_connection(self, timeout=10):
        start_time = time.time()

        while not conn_conv and not conn_robo and time.time() - start_time < timeout:
            self.get_logger().info("Waiting for server connection...")
            time.sleep(1)

        if not conn_conv and not conn_robo:
            self.get_logger().error("Failed to establish server connection within timeout.")

        else:
            self.get_logger().info("Conveyor Server connection established.")
        
def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetectionSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()