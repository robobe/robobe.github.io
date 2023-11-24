import machine
from machine import Pin
led = Pin('LED', Pin.OUT)
led.on()
led.off()
import time
for i in range(10):
    time.sleep_ms(250)
    led.on()
    time.sleep_ms(250)
    led.off()