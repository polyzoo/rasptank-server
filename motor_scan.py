from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
import time

I2C_ADDRESS = 0x5F   # тот же, что в motor.py
PWM_FREQ = 50

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=I2C_ADDRESS)
pca.frequency = PWM_FREQ

def test_pair(ch1: int, ch2: int, throttle: float = 0.5) -> None:
    print(f"\n=== Тест пары каналов {ch1}, {ch2} ===")
    m = motor.DCMotor(pca.channels[ch1], pca.channels[ch2])
    m.decay_mode = motor.SLOW_DECAY
    m.throttle = throttle
    print("  throttle =", throttle, "крутится ли что‑нибудь?")
    time.sleep(2.0)
    m.throttle = 0.0
    print("  стоп")
    time.sleep(1.0)

try:
    # Перебираем пары каналов: (0,1), (2,3), ..., (14,15)
    for ch1 in range(0, 16, 2):
        ch2 = ch1 + 1
        test_pair(ch1, ch2, throttle=0.6)
finally:
    # На всякий случай всё глушим
    for ch in range(16):
        pca.channels[ch].duty_cycle = 0
