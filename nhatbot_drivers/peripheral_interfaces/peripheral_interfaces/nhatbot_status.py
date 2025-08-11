#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dataclasses import dataclass
import Jetson.GPIO as GPIO
import time
import sounddevice as sd
import soundfile as sf

@dataclass(frozen=True)
class Pins:
    robot_ready: int = 23
    robot_warning: int = 18
    robot_error: int = 35
    status_buzzer: int = 13  
    full_motor_battery: int = 31
    mid_motor_battery: int = 12
    low_motor_battery: int = 29
    full_computer_battery: int = 33
    mid_computer_battery: int = 7
    low_computer_battery: int = 21

class NhatbotStatus(Node):
    def __init__(self):
        super().__init__('nhatbot_status')

        self.pins = Pins()
        self.led_state = False     
        self.blink = False         

        self.setup_gpio()
        self.timer = self.create_timer(0.1, self.timer_callback)  # Nhấp nháy 10Hz

        GPIO.setup(self.pins.robot_warning, GPIO.OUT)

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        for name, pin in vars(self.pins).items():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)  # Khởi tạo tất cả pin ở trạng thái LOW

        # self.active_robot_ready(True)  # Bật LED khi khởi động

    def active_robot_ready(self, on: bool):
        if self.led_state != on:
            GPIO.output(self.pins.robot_ready, GPIO.HIGH if on else GPIO.LOW)
            self.led_state = on

    def timer_callback(self):
        self.blink = not self.blink
        # self.active_robot_ready(self.blink)

    def beep_pattern(self, on_time=0.2, off_time=0.3, repeat=3):
        """
        Bật/tắt còi theo tần suất lặp lại.

        Args:
            on_time (float): thời gian bật (giây)
            off_time (float): thời gian tắt (giây)
            repeat (int): số lần kêu
        """
        for _ in range(repeat):
            GPIO.output(self.pins.status_buzzer, GPIO.HIGH)
            time.sleep(on_time)
            GPIO.output(self.pins.status_buzzer, GPIO.LOW)
            time.sleep(off_time)
    

def main(args=None):
    rclpy.init(args=args)
    node = NhatbotStatus()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()  # Quan trọng để reset trạng thái chân GPIO
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
