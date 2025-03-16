from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import socket
import PJT_server.stream_stt_keyboard
import configparser
import os
import sys
import time
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QTextEdit, QVBoxLayout, QWidget
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QShortcut, QKeySequence, QPixmap
from PyQt6.QtCore import QThread, pyqtSignal
from openai import OpenAI
import re

gpt_response = ''

# 스레드 클래스 정의
class VoiceRecognitionThread(QThread):
    result_signal = pyqtSignal(str)  # 음성 인식 결과를 전달할 신호

    def __init__(self, voice_client):
        super().__init__()
        self.voice_client = voice_client

    def run(self):
        # 음성 인식 실행
        result = self.voice_client.transcribe_streaming_grpc(trigger=True)
        self.result_signal.emit(result)  # 결과를 신호로 전달

class DobotControlThread(QThread):
    log_signal = pyqtSignal(str)  # 로그 업데이트를 전달할 신호
    global gpt_response
    def __init__(self, dobot_client, command_text):
        super().__init__()
        self.dobot_client = dobot_client
        self.command_text = command_text
        self.gpt_client = OpenAI(api_key='your_key')

    def run(self):
        # 두봇 제어 작업 실행
        # self.log_signal.emit(f"Executing Dobot action for command: {self.command_text}")
        # 여기서 `execute_dobot_action` 호출
        self.execute_dobot_action(self.command_text)

    def execute_dobot_action(self, text):
        if text is None:
            text = ''
        flag = 0
        try:
            response = self.gpt_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": """
                    로봇 명령어를 처리하는 어시스턴트입니다.
                    - 로봇 제어 명령이면 'COMMAND:'를 붙여서 응답.
                    - 로봇 제어 명령의 종류는 1번~ 6번 부품 이동 명령, 춤 명령, 손 명령, 돌아 명령, 홈 명령이 있음. 
                     사용자는 로봇어시스턴트를 로봇과 동일시 할 수 있음. 로봇 명령은 무조건 'COMMAND:"로 시작되어야함.
                    - 'COMMAND: 1번', 'COMMAND: 2번', 'COMMAND: 3번', 'COMMAND: 4번', 'COMMAND: 5번', 
                     'COMMAND: 6번', 'COMMAND: 춤', 'COMMAND: 손', 'COMMAND: 돌아','COMMAND: 홈'으로 각각 응답해야함.
                    - 일반 대화는 일반 텍스트로 응답
                    """},
                    {"role": "user", "content": text}
                ]
            )
            
            gpt_response = response.choices[0].message.content
            self.log_signal.emit(f"Dobot: {gpt_response}")
            =
            # GPT 응답이 명령어인 경우
            if gpt_response.startswith('COMMAND:'):
                print('gptresponce ;', gpt_response)
                text = gpt_response  # 매칭된 부분만 text에 저장
                flag = 1
                time.sleep(1)
        except Exception as e:
            self.log_signal.emit(f"GPT API Error: {str(e)}")

        num = 'nope'
        # print('text: ', text,' end')
        # print(text)
        if text.startswith('COMMAND:'):
            if ('1번' in text) or ('일번' in text) or ('일 번' in text):
                ditto(self.dobot_client)
                num = '1'
            elif ('2번' in text) or ('이번' in text) or ('이 번' in text):
                ditto(self.dobot_client)
                num = '2'
            elif ('3번' in text) or ('삼번' in text) or ('삼 번' in text):
                ditto(self.dobot_client)
                num = '3'
            elif ('4번' in text) or ('사번' in text) or ('사 번' in text):
                ditto(self.dobot_client)
                num = '4'
            elif ('5번' in text) or ('오번' in text) or ('오 번' in text):
                ditto(self.dobot_client)
                num = '5'
            elif ('6번' in text) or ('육번' in text) or ('육 번' in text):
                ditto(self.dobot_client)
                num = '6'

            elif ('손' in text):
                give_hand(self.dobot_client)
            elif ('춤' in text):
                dancing(self.dobot_client)
            elif ('돌' in text):
                turning(self.dobot_client)
            elif ('홈' in text):
                self.dobot_client.send_goal(target = home_j, mode = 4)
            else:
                dori_dori(self.dobot_client)

        if num == '1':
            # pick-way1
            self.dobot_client.send_goal(target = pick_board_red_way, mode = 4)
            time.sleep(2)
            # pick
            self.dobot_client.send_goal(target = pick_board_red, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(True)
            time.sleep(2)
            # pick-way1
            self.dobot_client.send_goal(target = pick_board_red_way, mode = 4)
            time.sleep(2)
            # pick-way2(conv)
            self.dobot_client.send_goal(target = pick_way2, mode = 4)
            time.sleep(2)
            # place-conv
            self.dobot_client.send_goal(target = place_conv, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(False)
            time.sleep(1)
            # pick-way1
            self.dobot_client.send_goal(target = home_j, mode = 4)
            time.sleep(2)

        elif num == '2':
            # pick-way1
            self.dobot_client.send_goal(target = pick_way1, mode = 4)
            time.sleep(2)
            # pick
            self.dobot_client.send_goal(target = pick_back_red, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(True)
            time.sleep(2)
            # pick-way1
            self.dobot_client.send_goal(target = pick_way1, mode = 4)
            time.sleep(2)
            # pick-way2(conv)
            self.dobot_client.send_goal(target = pick_way2, mode = 4)
            time.sleep(2)
            # place-conv
            self.dobot_client.send_goal(target = place_conv, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(False)
            time.sleep(1)
            # pick-way1
            self.dobot_client.send_goal(target = home_j, mode = 4)
            time.sleep(2)

            
        elif num == '3':
            # pick-way1
            self.dobot_client.send_goal(target = pick_board_blue_way, mode = 4)
            time.sleep(2)
            # pick
            self.dobot_client.send_goal(target = pick_board_blue, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(True)
            time.sleep(2)
            # pick-way1
            self.dobot_client.send_goal(target = pick_board_blue_way, mode = 4)
            time.sleep(2)
            # pick-way2(conv)
            self.dobot_client.send_goal(target = pick_way2, mode = 4)
            time.sleep(2)
            # place-conv
            self.dobot_client.send_goal(target = place_conv, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(False)
            time.sleep(1)
            # pick-way1
            self.dobot_client.send_goal(target = home_j, mode = 4)
            time.sleep(2)


        elif num == '4':
            # pick-way1
            self.dobot_client.send_goal(target = pick_way1, mode = 4)
            time.sleep(2)
            # pick  
            self.dobot_client.send_goal(target = pick_back_blue, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(True)
            time.sleep(2)
            # pick-way1
            self.dobot_client.send_goal(target = pick_way1, mode = 4)
            time.sleep(2)
            # pick-way2(conv)
            self.dobot_client.send_goal(target = pick_way2, mode = 4)
            time.sleep(2)
            # place-conv
            self.dobot_client.send_goal(target = place_conv, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(False)
            time.sleep(1)
            # pick-way1
            self.dobot_client.send_goal(target = home_j, mode = 4)
            time.sleep(2)


        elif num == '5':
            # pick-way1
            self.dobot_client.send_goal(target = pick_board_wite_way, mode = 4)
            time.sleep(2)
            # pick
            self.dobot_client.send_goal(target = pick_board_white, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(True)
            time.sleep(2)
            # pick-way1
            self.dobot_client.send_goal(target = pick_board_wite_way, mode = 4)
            time.sleep(2)
            # pick-way2(conv)
            self.dobot_client.send_goal(target = pick_way2, mode = 4)
            time.sleep(2)
            # place-conv
            self.dobot_client.send_goal(target = place_conv, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(False)
            time.sleep(1)
            # pick-way1
            self.dobot_client.send_goal(target = home_j, mode = 4)
            time.sleep(2)

        elif num == '6':
            # pick-way1
            self.dobot_client.send_goal(target = pick_way1, mode = 4)
            time.sleep(2)
            # pick
            self.dobot_client.send_goal(target = pick_back_white, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(True)
            time.sleep(2)
            # pick-way1
            self.dobot_client.send_goal(target = pick_way1, mode = 4)
            time.sleep(2)
            # pick-way2(conv)
            self.dobot_client.send_goal(target = pick_way2, mode = 4)
            time.sleep(2)
            # place-conv
            self.dobot_client.send_goal(target = place_conv, mode = 4)
            time.sleep(2)
            # suction
            self.dobot_client.turn_suction(False)
            time.sleep(1)
            # pick-way1
            self.dobot_client.send_goal(target = home_j, mode = 4)
            time.sleep(2)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dobot 음성 인식 UI")
        self.setGeometry(100, 100, 640, 480)

        env_config = configparser.ConfigParser()
        env_config.read("/home/chichi/final_PJT/src/PJT_server/config.ini")
        self.voice_client = PJT_server.stream_stt_keyboard.RTZROpenAPIClient(
            env_config["DEFAULT"]["CLIENT_ID"], env_config["DEFAULT"]["CLIENT_SECRET"]
        )

        self.dobot_client = PTP_MOVE()

        self.layout = QVBoxLayout()

        # 음성 인식 버튼
        self.start_button = QPushButton("음성 인식 시작 Click or Press 'S'", self)
        self.start_button.clicked.connect(self.start_voice_recognition)
        self.layout.addWidget(self.start_button)

        # 음성 인식 예시 레이블
        self.result_label = QLabel("Ex) Command 1번, Command 춤춰, 기타 일상대화 가능", self)
        self.layout.addWidget(self.result_label)

        # 작업 로그
        self.log_box = QTextEdit(self)
        self.log_box.setReadOnly(True)
        self.log_box.setFixedHeight(200)  # 높이를 200px로 고정
        self.layout.addWidget(self.log_box)


        # # gpt 대화
        # self.gpt_talk = QTextEdit(self)
        # self.gpt_talk.setReadOnly(True)
        # self.layout.addWidget(self.gpt_talk)

        # 키보드 단축키
        self.shortcut = QShortcut(QKeySequence("S"), self)
        self.shortcut.activated.connect(self.start_voice_recognition)

        container = QWidget()
        container.setLayout(self.layout)
        self.setCentralWidget(container)

        # 이미지 추가
        img_label = QLabel(self)
        pixmap = QPixmap('/home/chichi/final_PJT/src/PJT_server/panel.png')
        pixmap = pixmap.scaled(640, 480)

        img_label.setPixmap(pixmap)
        self.layout.addWidget(img_label)

        # 스레드 초기화
        self.voice_thread = None
        self.dobot_thread = None

    def start_voice_recognition(self):
        # 버튼 비활성화
        self.start_button.setEnabled(False)
        self.start_button.setText("음성 인식 중...")
        QApplication.processEvents()

        # 음성 인식 스레드 실행
        self.voice_thread = VoiceRecognitionThread(self.voice_client)
        self.voice_thread.result_signal.connect(self.handle_voice_result)
        self.voice_thread.start()

    def handle_voice_result(self, result):
        self.log_box.append(f"Me: {result}")
        # self.gpt_talk.append(f"GPT: {gpt_response}")
        QApplication.processEvents()

        # 두봇 작업 스레드 실행
        self.dobot_thread = DobotControlThread(self.dobot_client, result)
        self.dobot_thread.log_signal.connect(self.update_log)
        self.dobot_thread.start()

        # 버튼 활성화
        self.start_button.setEnabled(True)
        self.start_button.setText("음성 인식 시작 Click or Press 'S'")

    def update_log(self, log_message):
        self.log_box.append(log_message)
        QApplication.processEvents()


class PTP_MOVE(Node):

    def __init__(self):
        super().__init__('dobot_PTP_client')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self.suction_client = self.create_client(SuctionCupControl, '/dobot_suction_cup_service')
        self.req = SuctionCupControl.Request()

    def cancel_done(self, future): 
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')

        # Start a 0.5 second timer
        # self._timer = self.create_timer(0.5, self.timer_callback)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.current_pose))

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

    def send_goal(self, target, mode):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = target
        goal_msg.motion_type = mode
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # turn_suction
    def turn_suction(self, on_off):
        self.req.enable_suction = on_off
        future = self.suction_client.call_async(self.req)
        future.add_done_callback(self.callback)
    
    def callback(self, future):
        response = future.result()
        self.get_logger().info(f"start Suction {response}")

# 두봇 좌표
home_j = [0.0, 3.0, 13.0, 0.0]
home_down = [0.0, 3.0, 33.0, 0.0]
home_left = [-10.0, 3.0, 13.0, 0.0]
home_right = [10.0, 3.0, 13.0, 0.0]
pick_way1 = [21.4, 7.2, 4.0, -55.7]
pick_way2 = [-18.5, 10.7, -1.3, -55.7]
pick_board_red = [12.0, 48.7, 43.2, 0.0]
pick_board_red_way = [10.8, 33.7, 7.4, -55.7]
pick_back_red = [15.6, 31.4, 61.8, 0.0]
pick_board_blue = [21.4, 54.4, 38.1, 0.0]
pick_board_blue_way = [21.5, 39.5, 8.4, -55.7]
pick_back_blue = [27.9, 37.7, 55.2, 0.0]
pick_board_white = [30.1, 61.5, 30.7, 0.0]
pick_board_wite_way = [32.3, 47.4, 12.0, -55.7]
pick_back_white = [38.6, 44.3, 48.9, 0.0]
place_conv = [-18.0, 29.2, 49.5, -55.7]

def ditto(client):
    client.send_goal(target = home_j, mode = 4)
    time.sleep(1)
    client.send_goal(target = home_down, mode = 4)
    time.sleep(1)
    client.send_goal(target = home_j, mode = 4)
    time.sleep(1)
    client.send_goal(target = home_down, mode = 4)
    time.sleep(1)
    client.send_goal(target = home_j, mode = 4)
    time.sleep(2)

def dori_dori(client):
    client.send_goal(target = home_j, mode = 4)
    time.sleep(1)
    client.send_goal(target = home_left, mode = 4)
    time.sleep(1)
    client.send_goal(target = home_right, mode = 4)
    time.sleep(1)
    client.send_goal(target = home_left, mode = 4)
    time.sleep(1)
    client.send_goal(target = home_right, mode = 4)
    time.sleep(1)
    client.send_goal(target = home_j, mode = 4)
    time.sleep(2)

# give hand
come_hand = [-2.6, 51.3, -1.1, 0.0]
hand_down = [-2.6, 52.2, 7.6, 0.0]

def give_hand(client):
    client.send_goal(target = home_j, mode = 4)
    time.sleep(1)
    client.send_goal(target = come_hand, mode = 4)
    time.sleep(1)
    client.send_goal(target = hand_down, mode = 4)
    time.sleep(1)
    client.send_goal(target = come_hand, mode = 4)
    time.sleep(1)
    client.send_goal(target = hand_down, mode = 4)
    time.sleep(1)
    client.send_goal(target = home_j, mode = 4)
    time.sleep(2)

# turn
turn_way1 = [-63.6, 18.0, 10.6, 0.0]
turn_way2 = [68.0, 18.3, 10.6, 0.0]
def turning(client):
    client.send_goal(target = home_j, mode = 4)
    time.sleep(2)
    for i in range(2):
        client.send_goal(target = turn_way1, mode = 4)
        time.sleep(2)
        client.send_goal(target = turn_way2, mode = 4)
        time.sleep(2)
    client.send_goal(target = home_j, mode = 4)
    time.sleep(2)
    
# dacne
dance_way1 = [0.0, 20.7, 30.3, 0.0]
dance_way2 = [-36.8, 6.3, 0.0, 0.0]
dance_way4 = [24.9, 6.8, -9.0, 0.0]
def dancing(client):
    client.send_goal(target = home_j, mode = 4)
    time.sleep(2)
    for i in range(2):
        client.send_goal(target = dance_way1, mode = 4)
        time.sleep(2)
        client.send_goal(target = dance_way2, mode = 4)
        time.sleep(2)
        client.send_goal(target = dance_way1, mode = 4)
        time.sleep(2)
        client.send_goal(target = dance_way4, mode = 4)
        time.sleep(2)

    client.send_goal(target = home_j, mode = 4)
    time.sleep(2)

def main(args=None):
    try:
        rclpy.init(args=args)
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        sys.exit(app.exec())
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
