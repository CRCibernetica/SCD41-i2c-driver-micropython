from machine import Pin,I2C
from time import sleep_ms
from roboboard import RoboBoard
from scd4x import SCD4X

i2c = I2C(0, sda=Pin(21), scl=Pin(22))
sensor = SCD4X(i2c)

sensor.start_periodic_measurement()

while True:
    temp = sensor.temperature
    hum = sensor.relative_humidity
    co2 = sensor.co2
    print(f'Temp: {temp:.1f}C  RH: {hum:.1f}%  CO2(ppm): {co2}')
    sleep_ms(5000)