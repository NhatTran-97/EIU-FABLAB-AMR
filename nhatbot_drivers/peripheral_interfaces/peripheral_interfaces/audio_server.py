#!/usr/bin/env python3

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import sounddevice as sd
import soundfile as sf
import numpy as np
from pathlib import Path
from nhatbot_msgs.srv import PlayAudio


class AudioServer(Node):
    def __init__(self):
        super().__init__('voice_player_server')

        self.declare_parameter('audio_file', '')
        self.declare_parameter('device', -1) 
        self.declare_parameter('target_samplerate', 48000)
        self.declare_parameter('voices_subdir', 'voices') 
        self.declare_parameter('device_keyword', 'USB Audio')


        dev_param = self.get_parameter('device').get_parameter_value().integer_value
        dev_keyword = self.get_parameter('device_keyword').get_parameter_value().string_value

        

        # self.device = None if self.device < 0 else self.device

        if dev_param >= 0:
           self.device =  dev_param
        else:
            self.device = self.find_usb_audio_device(dev_keyword)


        self.target_sr = self.get_parameter('target_samplerate').get_parameter_value().integer_value

        voices_subdir = self.get_parameter('voices_subdir').get_parameter_value().string_value

        self.voices_dir = Path(get_package_share_directory('peripheral_interfaces')) / voices_subdir

        # ---- Interfaces ----
        self.srv = self.create_service(PlayAudio, '/play_audio', self.handle_play)
        self.get_logger().info(f"Audio server ready | voices_dir={self.voices_dir} | device={self.device}")


    def find_usb_audio_device(self, keyword="USB Audio"):
        """Tìm index thiết bị audio theo tên"""
        try:
            devices = sd.query_devices()
            for idx, dev in enumerate(devices):
                name = dev.get('name', '')
                max_out = dev.get('max_output_channels', 0)
                if keyword.lower() in name.lower() and max_out > 0:
                    self.get_logger().info(f"Auto-selected device {idx}: {name}")
                    return idx
            self.get_logger().warn(f"No device found with keyword='{keyword}', fallback to default")
            return None
        except Exception as e:
            self.get_logger().error(f"Error querying devices: {e}")
            return None
    def handle_play(self, req: PlayAudio.Request, resp: PlayAudio.Response):
        path = Path(req.path)
        print("path: ", path)
        if not path.is_absolute():
            path = (self.voices_dir / path).resolve()
        ok, err = self.play_file(path)
        resp.success = bool(ok)
        resp.message = "OK" if ok else f"ERROR: {err}"
        if ok:
            self.get_logger().info(f"Played: {path}")
        else:
            self.get_logger().error(resp.message)
        return resp
    
    def play_file(self, path: Path):
        try:
            if not path.exists():
                return False, f"File not found: {path}"
            
            data, sr = sf.read(str(path), always_2d=True)
            if self.target_sr and sr != self.target_sr:
                ratio = self.target_sr / sr
                new_len = int(data.shape[0] * ratio)
                x_old = np.linspace(0.0, 1.0, data.shape[0], endpoint=False)
                x_new = np.linspace(0.0, 1.0, new_len,      endpoint=False)
                data = np.stack([np.interp(x_new, x_old, data[:, ch]) for ch in range(data.shape[1])], axis=1)
                sr = self.target_sr
            try: 
                sd.check_output_settings(device=self.device, samplerate=sr, channels=data.shape[1])
            except Exception as e:
                self.get_logger().warn(f"check_output_settings failed (device={self.device}): {e}; fallback default")
                self.device = None

            sd.play(data, sr, device=self.device)
            sd.wait()
            return True, None
        
        except Exception as e:
            return False, str(e)
def main():
    rclpy.init()
    node = AudioServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()