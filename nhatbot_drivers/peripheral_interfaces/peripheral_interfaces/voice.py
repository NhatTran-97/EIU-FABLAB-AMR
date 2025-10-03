
import sounddevice as sd
print(sd.query_devices())


import sounddevice as sd
import soundfile as sf

# File âm thanh cần phát
filename = "../voices/em_chao_dai_ca_nhat.mp3"

# Đọc file
data, fs = sf.read(filename, dtype='float32')

# Chỉ định đúng thiết bị USB Audio (device index = 22)
sd.play(data, samplerate=fs, device=22)
sd.wait()  # chờ phát xong