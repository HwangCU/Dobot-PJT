# -*- coding: utf-8 -*-
import socket
import gpiod
import time
import threading

# GPIO pin numbers
DIR_PIN = 17
STEP_PIN = 27
ENABLE_PIN = 22
SERVO_PIN = 18

# Open GPIO chip
chip = gpiod.Chip('gpiochip0')

# Get GPIO lines
dir_line = chip.get_line(DIR_PIN)
step_line = chip.get_line(STEP_PIN)
enable_line = chip.get_line(ENABLE_PIN)
servo_line = chip.get_line(SERVO_PIN)

# Set up GPIO lines
dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)
servo_line.request(consumer="servo", type=gpiod.LINE_REQ_DIR_OUT)

# Motor control variables
motor_running = False
motor_direction = 0  # 0 for stop, 1 for CW, -1 for CCW
TargetSpeed = 0.0015  # ���� ���� �ӵ�(�� ����, �������� ����)
InitialSpeed = 0.003  # ������ ���� �ӵ�
Speed = InitialSpeed  
RATIO = 0.00001  # ������ ����(�������� ������ ���� ª��)
isAccelerating = False 
servo_position = 135  # Initial servo position

def step_motor():
    global motor_running, Speed, isAccelerating
    while motor_running:
        if isAccelerating:
            if Speed > TargetSpeed:
                Speed -= RATIO  # Acceleration
            else:
                Speed = TargetSpeed
                isAccelerating = False
        
        step_line.set_value(1)
        time.sleep(Speed)
        step_line.set_value(0)
        time.sleep(Speed)

        if motor_direction == 0 and Speed < InitialSpeed:
            Speed += RATIO  # Deceleration
            if Speed >= InitialSpeed:
                motor_running = False

def set_servo(angle):
    pulse_width = (angle / 270) * (0.0025 - 0.0005) + 0.0005
    
    for _ in range(10):  
        servo_line.set_value(1)
        time.sleep(pulse_width)
        servo_line.set_value(0)
        time.sleep(0.02 - pulse_width)

def print_motor_status():
    print(f"Motor Running: {motor_running}, Direction: {'CW' if motor_direction == 1 else 'CCW' if motor_direction == -1 else 'Stop'}, Speed: {Speed:.6f}")

# Define the server host and port
HOST = '70.12.224.109'  # Server address
PORT = 65432           # Port to connect to the server

def connect_to_server():
    """Tries to connect to the server every 5 seconds until successful."""
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))  # Try to connect to the server
            print(f"Connected to {HOST}:{PORT}")
            return s  # Return the socket object once connected
        except socket.error as e:
            print(f"Connection failed: {e}. Retrying in 5 seconds...")
            s.close()
            time.sleep(5)

try:
    while True:
        # Try to connect to the server
        s = connect_to_server()

        enable_line.set_value(0)  # Enable the motor driver

        while True:
            try:
                # Wait for a message (command) from the server
                data = s.recv(1024)
                if not data:
                    break

                command = data.decode('utf-8')
                
                # Handle the command received from the server
                if command == '1':
                    if motor_direction != 1:
                        print("Starting motor CW")
                        motor_direction = 1
                        dir_line.set_value(0)  # Set direction to CW
                        isAccelerating = True
                        Speed = InitialSpeed
                        if not motor_running:
                            motor_running = True
                            threading.Thread(target=step_motor).start()
                    else:
                        print("Stopping motor")
                        motor_direction = 0
                        
                    print_motor_status()
                
                elif command == '2':
                    print("Stopping the motor immediately")
                    motor_running = False  # Stop the motor
                    # servo_position = 135
                    # set_servo(servo_position)
                    print_motor_status()
    
                elif command == '3':
                    print("Stopping the motor immediately")
                    # motor_running = False  # Stop the motor
                    servo_position = 180
                    set_servo(servo_position)
                    print_motor_status()
                    
                    time.sleep(6)
                    servo_position = 135
                    set_servo(servo_position)
                    print_motor_status()
    
                elif command == '4':
                    print("Stopping the motor immediately")
                    # motor_running = False  # Stop the motor
                    servo_position = 180
                    set_servo(servo_position)
                    print_motor_status()
                    
                    time.sleep(6)
                    servo_position = 135
                    set_servo(servo_position)
                    print_motor_status()
    
                elif command == '5':
                    print("Stopping the motor immediately")
                    # motor_running = False  # Stop the motor
                    servo_position = 90
                    set_servo(servo_position)
                    print_motor_status()
                    
                    time.sleep(6)
                    servo_position = 135
                    set_servo(servo_position)
                    print_motor_status()
                    
                elif command == '6':
                    print("Stopping the motor immediately")
                    # motor_running = False  # Stop the motor
                    servo_position = 135
                    set_servo(servo_position)
                    print_motor_status()
                    
                else:
                    print("Unknown command received.")
    
                    time.sleep(0.1)
    
            except socket.error as e:
              print(f"Socket error: {e}. Reconnecting...")
              break  # Exit inner loop and reconnect
    
        s.close()  # Close the socket after disconnection

except KeyboardInterrupt:
    print("Program terminated")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    # Stop motor and release GPIO lines
    motor_running = False
    time.sleep(0.1)  # Wait for motor to stop
    enable_line.set_value(1)  # Disable the motor driver
    dir_line.release()
    step_line.release()
    enable_line.release()
    servo_line.release()

    print("Resources released and motor stopped.")
