#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dataclasses import dataclass, field
import Jetson.GPIO as GPIO
import time
from std_msgs.msg import Bool
from nhatbot_msgs.msg import ZlacStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import os
from nhatbot_msgs.srv import PlayAudio
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
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

@dataclass(frozen=True)
class Battery_Level:
    full_motor_battery: float = field(default=29.8)
    warning_motor_battery: float = field(default=28.0)
    low_motor_battery: float = field(default=25.5)

class NhatbotStatus(Node):
    def __init__(self):
        super().__init__('nhatbot_status')
        self.ReentGroup = ReentrantCallbackGroup()
        print(Battery_Level.full_motor_battery)

        self.declare_parameter('voice_files', ['xin_chao_anh_danh', 'xin_chao_hieu_truong', 'em_chao_dai_ca_nhat', 'vat_can'])
        self.voice_list = self.get_parameter('voice_files').get_parameter_value().string_array_value
        voices_path = Path(get_package_share_directory('peripheral_interfaces')) / 'voices'
        if not voices_path.exists():
            self.get_logger().error(f"Voices dir not found: {voices_path}")
            return

        self.files_dict = {}
        for file_path in voices_path.iterdir():
            if file_path.is_file() and file_path.suffix.lower() == '.mp3':
                self.files_dict[file_path.stem] = str(file_path.resolve())
        if not self.voice_list:
            self.get_logger().error("Parameter voice_files is empty")
            return 


        self.audio_client = self.create_client(PlayAudio, '/play_audio')
        while not self.audio_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
            time.sleep(2.0)

        self.pins = Pins()
        self.led_state = False     
        self.blink = False      
        self.error_blinking = False    

        self.setup_gpio()
        self.timer = self.create_timer(0.05, self.timer_callback, callback_group=self.ReentGroup)  

        qos = QoSProfile(depth=10)
        self.create_subscription(ZlacStatus, '/nhatbot/zlac_status', self.robot_status_callback, qos)

        self.create_subscription(Bool, '/safety_stop', self.safety_stop_callback, 10)

        GPIO.setup(self.pins.robot_warning, GPIO.OUT)


        self.last_time_led = time.time()
        self.last_time_beep = time.time()
        self.last_request_time = time.time()
        self.request_interval = 4

        self.audio_confirm_response = True
       
        self.led_on = False
        self.beep_cycles_left = 0
        self.beep_on = False


        self.robot_status = None 
        self.safety_stop = None


    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        for name, pin in vars(self.pins).items():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)  

    def robot_status_callback(self, msg: ZlacStatus) -> None:
        self.robot_status = msg
    def safety_stop_callback(self, msg) -> None:
        self.safety_stop = msg

    def call_request_audio(self):
        # req = PlayAudio.Request()
        # req.path = str(self.files_dict[self.voice_list[3]])
        # future = self.audio_client.call_async(req)
        # future.add_done_callback(self.responseCallback)


        key = self.voice_list[3]
        path = self.files_dict.get(key)
        if not path:
            self.get_logger().error(f"Voice '{key}' not found. Available: {list(self.files_dict.keys())}")
            self.audio_confirm_response = True
            return 
        req = PlayAudio.Request()
        req.path = str(path)
        future = self.audio_client.call_async(req)
        future.add_done_callback(self.responseCallback)
    def responseCallback(self, future):
        try:
            res = future.result()
            ok = bool(res and getattr(res, "success", False))
            self.audio_confirm_response = True
            if ok:
                self.get_logger().info(f"success={res.success}, msg={res.message}")
            else:
                self.get_logger().warn(f"Audio service returned failure. msg={getattr(res,'message','')}")

        except Exception as e:
            self.audio_confirm_response = True 
            self.get_logger().error(f"Service call failed: {e}")

        
    def active_robot_ready(self, on: bool):
        if self.led_state != on:
            GPIO.output(self.pins.robot_ready, GPIO.HIGH if on else GPIO.LOW)
            self.led_state = on

    def timer_callback(self):
        now = time.time()

        if self.robot_status is None or self.safety_stop is None:
            return 
        
        if Battery_Level.warning_motor_battery <= self.robot_status.battery_voltage < Battery_Level.full_motor_battery:  
            self.led_motor_display('full')
            self.error_blinking = False
        elif Battery_Level.low_motor_battery <= self.robot_status.battery_voltage < Battery_Level.warning_motor_battery:  
            self.led_motor_display('mid')
            self.error_blinking = False
        else:  
            self.led_motor_display('low')
            self.error_blinking = True
        
        print("self.error_blinking: ", self.error_blinking)
        if self.error_blinking:
            self.led_error_blink_pattern(self.pins.robot_error, toggle_time=0.2)
            self.beep_pattern(on_time=0.5, off_time=0.5, repeat=3) 
        else:
            if not self.safety_stop.data:
                self.reset_beep()
        
        if self.safety_stop.data:
            if now - self.last_request_time >= self.request_interval:
                if self.audio_confirm_response:
                    self.audio_confirm_response = False 
                    self.call_request_audio()
                    self.last_request_time = now 
                    print("self.error_blinking2222: ", self.error_blinking)

                    if not self.error_blinking:  
                        self.led_error_blink_pattern(self.pins.robot_error, toggle_time=0.2)
                        self.beep_pattern(on_time=0.5, off_time=0.5, repeat=3) 



    def led_error_blink_pattern(self, pin, toggle_time = 1):
        current_time = time.time()
        if current_time - self.last_time_led >= toggle_time:
            self.led_on = not self.led_on  
            self.last_time_led = current_time
            GPIO.output(pin, GPIO.HIGH if self.led_on else GPIO.LOW)  




    # def beep_pattern(self, on_time, off_time, repeat):
    #     current_time = time.time()
    #     if repeat > 0:
    #         if self.beep_on and current_time - self.last_time_beep >= on_time:
    #             self.beep_on = False
    #             self.last_time_beep = current_time
    #             GPIO.output(self.pins.status_buzzer, GPIO.LOW)  

    #         elif not self.beep_on and current_time - self.last_time_beep >= off_time:
    #             self.beep_on = True
    #             self.last_time_beep = current_time
    #             GPIO.output(self.pins.status_buzzer, GPIO.HIGH)  

    def reset_beep(self):
        # Gọi khi error_blinking chuyển sang False để sẵn sàng cho đợt kế tiếp
        self._beep_armed = False
        self._beep_cycles_left = None
        self.beep_on = False
        GPIO.output(self.pins.status_buzzer, GPIO.LOW)

    def beep_pattern(self, on_time, off_time, repeat):
        now = time.monotonic()  # nên dùng monotonic để tránh ảnh hưởng đổi giờ hệ thống

        # 1) Lần đầu trong một "đợt lỗi": nạp cấu hình và "arm"
        if not getattr(self, "_beep_armed", False):
            self._beep_armed = True
            self._beep_cycles_left = int(max(0, repeat))
            self.beep_on = False
            self.last_time_beep = now
            GPIO.output(self.pins.status_buzzer, GPIO.LOW)

        # Nếu đã hết chu kỳ, giữ im (không kêu lại cho đến khi reset_beep())
        if not self._beep_cycles_left:
            GPIO.output(self.pins.status_buzzer, GPIO.LOW)
            return

        # 2) Máy trạng thái ON/OFF theo thời gian đã trôi
        if self.beep_on:
            # đang kêu → đợi đủ on_time rồi tắt và trừ 1 chu kỳ
            if (now - self.last_time_beep) >= on_time:
                self.beep_on = False
                self.last_time_beep = now
                GPIO.output(self.pins.status_buzzer, GPIO.LOW)
                self._beep_cycles_left -= 1
        else:
            # đang tắt → đợi đủ off_time rồi bật
            if (now - self.last_time_beep) >= off_time:
                self.beep_on = True
                self.last_time_beep = now
                GPIO.output(self.pins.status_buzzer, GPIO.HIGH)


    # def beep_pattern(self, on_time, off_time, repeat):
    #     current_time = time.time()

    #     # Nếu chưa có biến đếm thì khởi tạo
    #     if not hasattr(self, "_beep_cycles_left") or repeat != getattr(self, "_beep_repeat_param", None):
    #         self._beep_cycles_left = repeat
    #         self._beep_repeat_param = repeat
    #         self.beep_on = False
    #         self.last_time_beep = current_time
    #         GPIO.output(self.pins.status_buzzer, GPIO.LOW)

    #     if self._beep_cycles_left <= 0:
    #         GPIO.output(self.pins.status_buzzer, GPIO.LOW)
    #         return

    #     if self.beep_on:
    #         if current_time - self.last_time_beep >= on_time:
    #             self.beep_on = False
    #             self.last_time_beep = current_time
    #             GPIO.output(self.pins.status_buzzer, GPIO.LOW)
    #             self._beep_cycles_left -= 1
    #     else:
    #         if current_time - self.last_time_beep >= off_time:
    #             self.beep_on = True
    #             self.last_time_beep = current_time
    #             GPIO.output(self.pins.status_buzzer, GPIO.HIGH)


            

    def led_motor_display(self, voltage_level: str) -> None:
        GPIO.output(self.pins.full_motor_battery, GPIO.LOW)
        GPIO.output(self.pins.mid_motor_battery, GPIO.LOW)
        GPIO.output(self.pins.low_motor_battery, GPIO.LOW)
        if voltage_level == 'full':
            GPIO.output(self.pins.full_motor_battery, GPIO.HIGH)
        elif voltage_level == 'mid':
            GPIO.output(self.pins.mid_motor_battery, GPIO.HIGH)
        elif voltage_level == 'low':
            GPIO.output(self.pins.low_motor_battery, GPIO.HIGH)


    def robot_status_display(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = NhatbotStatus()
    executor = MultiThreadedExecutor(num_threads=os.cpu_count()) 
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup() 
        executor.shutdown()  
        node.destroy_node()  
        rclpy.shutdown()  

        print("✅ nhatbot_status node shutdown complete")

if __name__ == '__main__':
    main()
