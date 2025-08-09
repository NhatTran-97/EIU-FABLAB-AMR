


"""LED TABLE
PINS====================FUNCTION
Pin35====================RED Big
Pin33====================Full Led - Right
Pin31====================Full Led - Left: Default: Pull Up
Pin29====================Lowest Led - Left
Pin23====================Blue Big
Pin21====================Lowest Led - Right
Pin15====================
Pin13====================Coi
Pin7====================Mid Led - Right

Thieu mid left: Chan so 2 tu trai
Thieu Mid LED: Chan so 5 tuwf trai sang phai

"""


import Jetson.GPIO as GPIO
import time

# Sử dụng kiểu đánh số BOARD (pin vật lý) hoặc BCM (số logic)
GPIO.setmode(GPIO.BOARD)  # hoặc GPIO.TEGRA_SOC

pin = 15  # ví dụ: chân số 11 (GPIO50)

# Cài đặt chân làm output
GPIO.setup(pin, GPIO.OUT)

# Bật đèn LED (hoặc điều khiển thiết bị)
GPIO.output(pin, GPIO.HIGH)
time.sleep(3)
GPIO.output(pin, GPIO.LOW)
time.sleep(3)
# Dọn dẹp
GPIO.cleanup()
