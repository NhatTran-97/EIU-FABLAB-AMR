#!/usr/bin/env python3
import sys, rclpy
from rclpy.node import Node
from nhatbot_msgs.srv import PlayAudio
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

class AudioClient(Node):
    def __init__(self):
        super().__init__('play_audio_client')

        self.declare_parameter('voice_files', ['xin_chao_anh_danh', 'xin_chao_hieu_truong', 'em_chao_dai_ca_nhat'])
        voice_list = self.get_parameter('voice_files').get_parameter_value().string_array_value

        voices_path = Path(get_package_share_directory('peripheral_interfaces')) / 'voices'
        if not voices_path.exists():
            self.get_logger().error(f"Voices dir not found: {voices_path}")
            return

        files_dict = {}
        for file_path in voices_path.iterdir():
            if file_path.is_file() and file_path.suffix.lower() == '.mp3':
                files_dict[file_path.stem] = str(file_path.resolve())
        
        if not voice_list:
            self.get_logger().error("Parameter voice_files is empty")
            return 

        audio_client = self.create_client(PlayAudio, '/play_audio')
        while not audio_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        req = PlayAudio.Request()
        req.path = str(files_dict[voice_list[2]])

        future = audio_client.call_async(req)
        future.add_done_callback(self.responseCallback)
    def responseCallback(self, future):
        try:
            res = future.result()
            if res:
                self.get_logger().info(f"success={res.success}, msg={res.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()
    node = AudioClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
