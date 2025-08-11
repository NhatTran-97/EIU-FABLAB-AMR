# import pygame
# pygame.mixer.init()
# pygame.mixer.music.load("em_chao_dai_ca_nhat.mp3")
# pygame.mixer.music.play()
# while pygame.mixer.music.get_busy():
#     continue


# from pydub import AudioSegment
# import pyaudio

# def play_mp3(file_path):
#     # Load file MP3 và convert sang raw PCM
#     song = AudioSegment.from_mp3(file_path)
#     song = song.set_channels(2)
#     song = song.set_frame_rate(44100)

#     # Tạo stream phát âm thanh
#     p = pyaudio.PyAudio()
#     stream = p.open(format=p.get_format_from_width(song.sample_width),
#                     channels=song.channels,
#                     rate=song.frame_rate,
#                     output=True)

#     print("🎧 Đang phát:", file_path)
#     stream.write(song.raw_data)

#     stream.stop_stream()
#     stream.close()
#     p.terminate()
#     print("✅ Phát xong.")

# # Test
# play_mp3("xin_chao_anh_danh.mp3")



# from pydub import AudioSegment
# import os

# def play_mp3(mp3_file):
#     print(f"🎵 Đang phát: {mp3_file}")

#     # Chuyển sang WAV tạm thời
#     wav_path = "/tmp/temp.wav"
#     song = AudioSegment.from_mp3(mp3_file)
#     song.export(wav_path, format="wav")

#     # Phát bằng ALSA (card 2, device 0 là USB Audio)
#     os.system(f"aplay -D plughw:2,0 {wav_path}")

# # Gọi hàm phát
# play_mp3("xin_chao_anh_danh.mp3")







# import sounddevice as sd
# import soundfile as sf

# def play_wav_sounddevice(wav_file, device_index=None):
#     data, samplerate = sf.read(wav_file)
#     print(f"🔊 Phát âm thanh bằng sounddevice (device={device_index})")
#     sd.play(data, samplerate=samplerate, device=device_index)
#     sd.wait()

# # Liệt kê thiết bị đầu ra
# import sounddevice as sd
# devices = sd.query_devices()
# for i, d in enumerate(devices):
#     if d['max_output_channels'] > 0:
#         print(f"[{i}] {d['name']}")

# device_index = 22  # Ví dụ: USB Audio Device
# wav_file = "xin_chao_anh_danh.mp3"
# play_wav_sounddevice(wav_file, device_index)


# from pydub import AudioSegment
# import sounddevice as sd
# import soundfile as sf
# import numpy as np
# import tempfile


# print(sd.query_devices())                 # Toàn bộ device với index thật của PortAudio
# print("Default:", sd.default.device)      # (input_idx, output_idx)

# def play_mp3(mp3_path, device_index=None):
#     print(f"🎵 Đang phát: {mp3_path}")

#     # Chuyển mp3 → wav tạm thời
#     song = AudioSegment.from_mp3(mp3_path)

#     # Tạo file wav tạm
#     with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmp_wav:
#         song.export(tmp_wav.name, format="wav")
#         wav_path = tmp_wav.name

#     # Đọc file wav bằng soundfile
#     data, samplerate = sf.read(wav_path, dtype='float32')

#     # Phát âm thanh qua thiết bị mong muốn
#     sd.play(data, samplerate, device=device_index)
#     sd.wait()

# # 🔍 Liệt kê thiết bị
# print("\n📢 Danh sách thiết bị phát âm:")
# devices = sd.query_devices()
# for i, d in enumerate(devices):
#     if d['max_output_channels'] > 0 and 'USB' in d['name']:
#         print(f"[{i}] {d['name']}")

# # 👉 Đặt device index phù hợp với card USB của bạn
# device_index = int(input("\n🔊 Nhập chỉ số thiết bị muốn phát âm: "))

# # ▶️ Gọi hàm
# play_mp3("xin_chao_anh_danh.mp3", device_index)



# import sounddevice as sd
# import soundfile as sf

# # Liệt kê chỉ output devices và tạo map
# all_devs = sd.query_devices()
# out_devs = [(i, d['name']) for i, d in enumerate(all_devs) if d['max_output_channels'] > 0]

# print("Output devices:")
# for k, (idx, name) in enumerate(out_devs):
#     print(f"[{k}] idx={idx}  {name}")

# sel = int(input("Chọn số thứ tự (k): "))
# pa_index = out_devs[sel][0]  # <-- index thật của PortAudio

# # Test setting có hợp lệ không
# def play_mp3(path, device_index):
#     data, sr = sf.read(path, always_2d=True)
#     ch = data.shape[1]
#     sd.check_output_settings(device=device_index, samplerate=sr, channels=ch)
#     sd.play(data, sr, device=device_index)
#     sd.wait()

# play_mp3("xin_chao_anh_danh.mp3", pa_index)

import sounddevice as sd
import soundfile as sf

data, sr = sf.read("xin_chao_anh_danh.mp3", always_2d=True)
sd.play(data, sr) 
sd.wait()